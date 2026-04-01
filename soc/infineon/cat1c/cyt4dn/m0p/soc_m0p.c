/*
 * Copyright (c) 2026 Linumiz
 * Copyright (c) 2026 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Infineon CAT1C SOC.
 */

#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <cy_wdt.h>
#include <cy_sysclk.h>

#define M7_0_FREQ DT_PROP(DT_NODELABEL(m7_0), clock_frequency)
#define M7_1_FREQ DT_PROP(DT_NODELABEL(m7_1), clock_frequency)

#define IFX_FAST_CLOCK_DOMAIN_FREQ_MHZ \
	(MAX(M7_0_FREQ, M7_1_FREQ) / 1000000)

void soc_prep_hook(void)
{
	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	SystemCoreClockUpdate();
	Cy_SysLib_SetWaitStates(false, IFX_FAST_CLOCK_DOMAIN_FREQ_MHZ);
}

static int soc_start_cm7(void)
{
#if CONFIG_INFINEON_CAT1C_START_M7_0
	Cy_SysEnableCM7(CORE_CM7_0, DT_REG_ADDR(DT_NODELABEL(flash_m7_0)));
#endif
#if CONFIG_INFINEON_CAT1C_START_M7_1
	Cy_SysEnableCM7(CORE_CM7_1, DT_REG_ADDR(DT_NODELABEL(flash_m7_1)));
#endif
	return 0;
}

SYS_INIT(soc_start_cm7, PRE_KERNEL_2, 0);
