/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __PAYLOAD_H__
#define __PAYLOAD_H__

#include "types.h"

bool payload_logo(void **custom_128, void **custom_256);

int payload_run(void);
#ifdef M5_PACKED_BOOT_PAYLOAD
void payload_set_start(void *start);
#endif

#endif
