/*
 * CHZ Arm Little Cores SoC — RSE multicore firmware distributor interface
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CHZ_MULTICORE_BOOT_H__
#define __CHZ_MULTICORE_BOOT_H__

/* Read core1-4 firmware headers from flash, copy each image into the
 * core's ITCM via its TCM window, then release CPUWAIT staggered.
 * Safe to call when nothing is flashed: cores are just skipped. */
void chz_multicore_boot(void);

#endif /* __CHZ_MULTICORE_BOOT_H__ */
