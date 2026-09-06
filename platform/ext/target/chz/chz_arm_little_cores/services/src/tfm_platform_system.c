/*
 * Copyright (c) 2018-2024, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "platform/include/tfm_platform_system.h"
#include "tfm_hal_device_header.h"

void tfm_platform_hal_system_reset(void)
{
    /* Reset the whole system through SYSCTRL.SYSRST (doc/SYSCTRL.md).
     * NVIC_SystemReset() does not work on this SoC: core0's CPU store to
     * AIRCR is silently dropped inside the TEAL MTX PPB path (verified
     * 2026-09-06 — S and NS AIRCR views both stay at reset value). */
    *(volatile uint32_t *)0x4003000Cu = 0x5A5A55A5u;  /* SYSCTRL KEY */
    *(volatile uint32_t *)0x40030010u = 0x1u;        /* SYSRST pulse */
    for (;;)
        ;
}

enum tfm_platform_err_t tfm_platform_hal_ioctl(tfm_platform_ioctl_req_t request,
                                               psa_invec  *in_vec,
                                               psa_outvec *out_vec)
{
    (void)request;
    (void)in_vec;
    (void)out_vec;

    /* Not needed for this platform */
    return TFM_PLATFORM_ERR_NOT_SUPPORTED;
}

