/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "adt.h"
#include "dapf.h"
#include "dart.h"
#include "firmware.h"
#include "isp.h"
#include "pmgr.h"
#include "soc.h"
#include "utils.h"

#define ISP_ASC_VERSION 0x1800000

#define ISP_VER_T8103 0xb0090
#define ISP_VER_T6000 0xb3091
#define ISP_VER_T8112 0xc1090
#define ISP_VER_T6020 0xc3091
#define ISP_VER_T8122 0xf1001
#define ISP_VER_T603X 0xf3001

// PMGR offset to enable to get the version info to work
#define ISP_PMGR_T8103 0x4018
#define ISP_PMGR_T6000 0x8
#define ISP_PMGR_T6020 0x4008
#define ISP_PMGR_T6031 0x4030

static bool isp_initialized = false;
static u64 heap_phys, heap_iova, heap_size, heap_top;

int isp_get_heap(u64 *phys, u64 *iova, u64 *size)
{
    if (!isp_initialized)
        return -1;

    *phys = heap_phys;
    *iova = heap_iova | isp_iova_base();
    *size = heap_size;
    return 0;
}

u64 isp_iova_base(void)
{
    switch (chip_id) {
        case 0x6020 ... 0x6fff:
        case 0x8122:
            return 0x10000000000;
        default:
            return 0;
    }
}

/*
 * The ISP17a generation (A18 Pro T8140, M5 T8142) is the one the ADT names "isp,isp-generic-ex".
 * Its power is split between a global and a local PMGR aperture and it stays powered for the OS
 * handoff (the firmware iBoot loaded is resident), so it takes its own path. The revision register
 * can't tell it apart before power-up, so the ADT compatible is the key. Ported from aura's isp.c.
 */
bool isp_is_isp17a(void)
{
    int node = adt_path_offset(adt, "/arm-io/isp");
    if (node < 0)
        node = adt_path_offset(adt, "/arm-io/isp0");
    return node >= 0 && adt_is_compatible(adt, node, "isp,isp-generic-ex");
}

/* Global PMGR PS slots from the ADT; virtual devices, so the ADT recursion skips them. */
static const struct {
    const char *name;
    u32 offset;
} isp17a_domains[] = {
    {"ISP_CPU", 0x4000},
    {"ISP_CPU_CORE0", 0x4008},
    {"ISP_CPU_CORE1", 0x4010},
    {"ISP_FE", 0x4018},
};

/*
 * T8142: ADT segment-ranges end at 0x16ec000 (J813 and J815, macOS 26 and 27); the heap runs to
 * the next 2 MiB boundary.
 */
#define ISP_T8142_HEAP_TOP 0x1800000
#define ISP_T8142_SEG_END  0x16ec000

static int isp_init_t8142(void)
{
    const char *isp_path = "/arm-io/isp";
    const char *dart_path = "/arm-io/dart-isp";
    const char *reason;
    struct pmgr_saved_modes adt_modes = {0};
    u8 global_target[ARRAY_SIZE(isp17a_domains)] = {0};
    unsigned int global_attempted = 0;
    u64 isp_base, isp_size, global_base = 0, global_size, local_base, local_size;
    u64 iova = 0;

    int adt_isp_path[8];
    int isp_node = adt_path_offset_trace(adt, isp_path, adt_isp_path);
    if (isp_node < 0)
        return 0;

    reason = "unsupported firmware";
    if (os_firmware.version != V26_6 && os_firmware.version != V26_6_2 &&
        os_firmware.version != V27_0 && os_firmware.version != V27_2)
        goto fail;

    reason = "ISP ADT power gates failed";
    if (pmgr_adt_power_enable_saved(isp_path, &adt_modes) < 0)
        goto fail;

    reason = "invalid apertures";
    if (adt_get_reg(adt, adt_isp_path, "reg", 0, &isp_base, &isp_size) ||
        isp_size < ISP_ASC_VERSION + sizeof(u32) ||
        adt_get_reg(adt, adt_isp_path, "reg", 1, &global_base, &global_size) ||
        adt_get_reg(adt, adt_isp_path, "reg", 2, &local_base, &local_size) ||
        global_size < 0x18000 || local_size != 0x4000)
        goto fail_power;

    for (unsigned int i = 0; i < ARRAY_SIZE(isp17a_domains); i++) {
        uintptr_t addr = global_base + isp17a_domains[i].offset;
        global_target[i] = read32(addr) & 0xf;
        global_attempted++;
        int ret = pmgr_set_mode(addr, PMGR_PS_ACTIVE);
        printf("isp: %s power on at 0x%lx: 0x%x%s\n", isp17a_domains[i].name, addr, read32(addr),
               ret ? " failed" : "");
        /* The ISP_CPU_CORE1 slot ignores writes on T8142 (its target reads back 0). */
        if (ret && isp17a_domains[i].offset == 0x4010 && !(read32(addr) & 0xf)) {
            printf("isp: %s ignores power requests, skipping it\n", isp17a_domains[i].name);
            continue;
        }
        reason = "global ISP PMGR power failed";
        if (ret)
            goto fail_power;
    }

    printf("isp: Version 0x%x\n", read32(isp_base + ISP_ASC_VERSION));

    u32 seg_len = 0;
    const struct adt_segment_ranges *seg = adt_getprop(adt, isp_node, "segment-ranges", &seg_len);
    reason = "invalid segment-ranges";
    if (!seg || !seg_len || seg_len % sizeof(*seg))
        goto fail_power;
    u64 end = 0;
    for (unsigned int i = 0; i < seg_len / sizeof(*seg); i++)
        end = max(end, seg[i].iova + seg[i].size);
    iova = ALIGN_UP(end, SZ_16K);
    reason = "unmeasured segment end";
    if (iova != ISP_T8142_SEG_END)
        goto fail_power;

    reason = "ISP DAPF initialization failed";
    if (dapf_init(dart_path, 5) < 0)
        goto fail_power;

    reason = "heap allocation failed";
    heap_size = ISP_T8142_HEAP_TOP - iova;
    heap_phys = top_of_memory_alloc(heap_size);
    if (!heap_phys)
        goto fail_power;
    heap_iova = iova;
    heap_top = ISP_T8142_HEAP_TOP;

    printf("isp: Heap: 0x%lx..0x%lx (0x%lx @ 0x%lx)\n", heap_iova, heap_top, heap_size,
           heap_phys);
    isp_initialized = true;
    /* The resident firmware stays powered for the OS. */
    return 0;

fail_power:
    while (global_attempted) {
        unsigned int i = --global_attempted;
        pmgr_set_mode(global_base + isp17a_domains[i].offset, global_target[i]);
    }
    pmgr_restore_modes(&adt_modes);
fail:
    printf("isp: disabled: firmware %s, aligned end 0x%lx: %s\n",
           os_firmware.string ? os_firmware.string : "unknown", iova, reason);
    return -1;
}

int isp_init(void)
{
    int err = 0;

    if (chip_id == T8142 && isp_is_isp17a())
        return isp_init_t8142();

    const char *isp_path = "/arm-io/isp";
    const char *dart_path = "/arm-io/dart-isp";

    int adt_path[8], adt_isp_path[8];
    int isp_node = adt_path_offset_trace(adt, isp_path, adt_isp_path);
    int node = adt_path_offset_trace(adt, dart_path, adt_path);
    if (node < 0 || isp_node < 0) {
        isp_path = "/arm-io/isp0";
        dart_path = "/arm-io/dart-isp0";
        isp_node = adt_path_offset_trace(adt, isp_path, adt_isp_path);
        node = adt_path_offset_trace(adt, dart_path, adt_path);
    }
    if (node < 0)
        return 0;

    if (pmgr_adt_power_enable(isp_path) < 0)
        return -1;

    u64 isp_base;
    u64 pmgr_base;
    err = adt_get_reg(adt, adt_isp_path, "reg", 0, &isp_base, NULL);
    if (err)
        return err;

    err = adt_get_reg(adt, adt_isp_path, "reg", 1, &pmgr_base, NULL);
    if (err)
        return err;

    u32 pmgr_off;
    switch (chip_id) {
        case T8103:
        case T8112:
        case T8122:
            pmgr_off = ISP_PMGR_T8103;
            break;
        case T6000 ... T6002:
            pmgr_off = ISP_PMGR_T6000;
            break;
        case T6020 ... T6022:
        case T6030:
            pmgr_off = ISP_PMGR_T6020;
            break;
        case T6031 ... T6034:
            pmgr_off = ISP_PMGR_T6031;
            break;
        default:
            printf("isp: Unsupported SoC\n");
            return -1;
    }

    err = pmgr_set_mode(pmgr_base + pmgr_off, PMGR_PS_ACTIVE);
    if (err) {
        printf("isp: Failed to power on\n");
        return err;
    }

    u32 ver_rev = read32(isp_base + ISP_ASC_VERSION);
    printf("isp: Version 0x%x\n", ver_rev);

    pmgr_set_mode(pmgr_base + pmgr_off, PMGR_PS_PWRGATE);

    /* TODO: confirm versions */
    switch (ver_rev) {
        case ISP_VER_T8103:
        case ISP_VER_T8112:
            switch (os_firmware.version) {
                case V12_3 ... V12_4:
                    heap_top = 0x1800000;
                    break;
                case V13_5:
                    heap_top = 0x1000000;
                    break;
                default:
                    printf("isp: unsupported firmware\n");
                    return -1;
            }
            break;
        case ISP_VER_T6000:
            switch (os_firmware.version) {
                case V12_3:
                    heap_top = 0xe00000;
                    break;
                case V13_5:
                case V13_6_1:
                    heap_top = 0xf00000;
                    break;
                default:
                    printf("isp: unsupported firmware\n");
                    return -1;
            }
            break;
        case ISP_VER_T6020:
            switch (os_firmware.version) {
                case V13_5:
                case V13_6_1:
                    heap_top = 0xf00000;
                    break;
                default:
                    printf("isp: unsupported firmware\n");
                    return -1;
            }
            break;
        case ISP_VER_T603X:
        case ISP_VER_T8122:
            switch (os_firmware.version) {
                case V14_7:
                    heap_top = 0x1000000;
                    break;
                default:
                    printf("isp: unsupported firmware\n");
                    return -1;
            }
            break;
        default:
            printf("isp: unknown revision 0x%x\n", ver_rev);
            return -1;
    }

    const struct adt_segment_ranges *seg;
    u32 segments_len;

    seg = adt_getprop(adt, isp_node, "segment-ranges", &segments_len);
    unsigned int count = segments_len / sizeof(*seg);

    heap_iova = seg[count - 1].iova + seg[count - 1].size;
    heap_size = heap_top - heap_iova;
    heap_phys = top_of_memory_alloc(heap_size);

    printf("isp: Code: 0x%lx..0x%lx (0x%x @ 0x%lx)\n", seg[0].iova, seg[0].iova + seg[0].size,
           seg[0].size, seg[0].phys);
    printf("isp: Data: 0x%lx..0x%lx (0x%x @ 0x%lx)\n", seg[1].iova, seg[1].iova + seg[1].size,
           seg[1].size, seg[1].phys);
    printf("isp: Heap: 0x%lx..0x%lx (0x%lx @ 0x%lx)\n", heap_iova, heap_top, heap_size, heap_phys);

    isp_initialized = true;

    pmgr_adt_power_disable(isp_path);
    return err;
}
