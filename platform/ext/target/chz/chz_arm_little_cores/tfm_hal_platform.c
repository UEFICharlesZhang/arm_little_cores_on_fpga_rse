/*
 * SPDX-FileCopyrightText: Copyright The TrustedFirmware-M Contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "tfm_hal_device_header.h"
#include "target_cfg.h"
#include "tfm_hal_platform.h"
#include "tfm_plat_defs.h"
#include "uart_stdout.h"
#include "chz_multicore_boot.h"

extern const struct memory_region_limits memory_regions;

/* Minimal fake NS "vector table" + code + stack in ITCM at 0x18000 (SAU
 * region marked NS — tfm_s ends well below 0x18000). The NS DTCM window is
 * deliberately NOT used: stores to DTCM addresses with bit 11 set land one
 * byte off on this SoC (observed on hardware), so everything the NS side
 * touches lives in ITCM. */
#define CHZ_NS_IDLE_BASE   (0x00018000u)
#define CHZ_NS_IDLE_CODE   (CHZ_NS_IDLE_BASE + 0x800u) /* code, even address */
#define CHZ_NS_IDLE_ENTRY  (CHZ_NS_IDLE_CODE | 1u)      /* entry, thumb bit */
#define CHZ_NS_IDLE_STACK  (0x0001BF00u)

FIH_RET_TYPE(enum tfm_hal_status_t) tfm_hal_platform_init(void)
{
    enum tfm_plat_err_t plat_err = TFM_PLAT_ERR_SYSTEM_ERR;
    FIH_DECLARE(fih_rc, FIH_FAILURE);

    plat_err = enable_fault_handlers();
    if (plat_err != TFM_PLAT_ERR_SUCCESS) {
        FIH_RET(TFM_HAL_ERROR_GENERIC);
    }

    plat_err = system_reset_cfg();
    if (plat_err != TFM_PLAT_ERR_SUCCESS) {
        FIH_RET(TFM_HAL_ERROR_GENERIC);
    }

    FIH_CALL(init_debug, fih_rc);
    if (FIH_NOT_EQ(fih_rc, (TFM_PLAT_ERR_SUCCESS))) {
        FIH_RET(TFM_HAL_ERROR_GENERIC);
    }

    __enable_irq();
    stdio_init();

    plat_err = nvic_interrupt_target_state_cfg();
    if (plat_err != TFM_PLAT_ERR_SUCCESS) {
        FIH_RET(TFM_HAL_ERROR_GENERIC);
    }

    plat_err = nvic_interrupt_enable();
    if (plat_err != TFM_PLAT_ERR_SUCCESS) {
        FIH_RET(TFM_HAL_ERROR_GENERIC);
    }

#if defined(TEST_S_FPU) || defined(TEST_NS_FPU)
    /* Set IRQn in secure mode */
    NVIC_ClearTargetState(TFM_FPU_S_TEST_IRQ);

    /* Enable FPU secure test interrupt */
    NVIC_EnableIRQ(TFM_FPU_S_TEST_IRQ);
#endif

#if defined(TEST_NS_FPU)
    /* Set IRQn in non-secure mode */
    NVIC_SetTargetState(TFM_FPU_NS_TEST_IRQ);
#if (TFM_ISOLATION_LEVEL >= 2)
    /* On isolation level 2, FPU test ARoT service runs in unprivileged mode.
     * Set SCB.CCR.USERSETMPEND as 1 to enable FPU test service to access STIR
     * register.
     */
    SCB->CCR |= SCB_CCR_USERSETMPEND_Msk;
#endif
#endif

    /* No NS firmware exists on this SoC (the flash XIP window that used to
     * hold the NS image is gone). Park the SPM's NS launch on a WFI stub
     * placed in the SAU-NS DTCM window, otherwise backend_system_run() jumps
     * to the dead NS alias and takes an INVTRAN SecureFault. All 512 vectors
     * must point at the stub: the core sits in WFI, but any pending IRQ would
     * otherwise vector through a NULL entry and double-fault into lockup. */
    {
        volatile uint32_t *vt = (volatile uint32_t *)CHZ_NS_IDLE_BASE;
        int i;

        for (i = 0; i < 512; i++) {
            vt[i] = CHZ_NS_IDLE_ENTRY;
        }
        vt[0] = CHZ_NS_IDLE_STACK;
        vt[1] = CHZ_NS_IDLE_ENTRY;
        /* wfi; b . — written as one 32-bit store: 16-bit stores take a
         * misaligned byte-lane path on this SoC's custom AHB/TCM wiring. */
        *(volatile uint32_t *)CHZ_NS_IDLE_CODE = 0xE7FDBF30u;
    }

    /* RSE duty on this SoC: distribute core1-4 firmware from flash through
     * their TCM windows and release CPUWAIT (chz_multicore_boot.c). Called
     * last so the console (stdio) is up and all platform state is set. */
    chz_multicore_boot();

    FIH_RET(TFM_HAL_SUCCESS);
}

uint32_t tfm_hal_get_ns_VTOR(void)
{
    return CHZ_NS_IDLE_BASE;
}

uint32_t tfm_hal_get_ns_MSP(void)
{
    return *((uint32_t *)CHZ_NS_IDLE_BASE);
}

uint32_t tfm_hal_get_ns_entry_point(void)
{
    return *((uint32_t *)(CHZ_NS_IDLE_BASE + 4));
}
