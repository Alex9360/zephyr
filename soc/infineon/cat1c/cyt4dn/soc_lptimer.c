/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>

#include <cy_wdt.h>
#include <cy_sysint.h>
#include <cy_sysclk.h>
#include <cy_syslib.h>

/* ILO0 nominal frequency (32000 Hz for T2G, ±30% accuracy) */
#define WDT_CLK_FREQ		CY_SYSCLK_ILO_FREQ

/* 32-bit free-running counter on WDT version B */
#define WDT_CNT_MAX		0xFFFFFFFFU

/* WDT system interrupt index for CYT4DN */
#define WDT_SYS_INT_IDX	17U

/* CPU interrupt slot to route WDT system interrupt to */
#define WDT_CPU_INT_IDX	3U

/*
 * WDT register writes cross async ILO clock domain boundary.
 * 3 ILO cycles @ 32 kHz = ~94 us. Use 200 us for safety.
 */
#define WDT_PROPAGATION_DELAY_US	200U

#define US_TO_WDT_TICKS(us)	((uint32_t)(((uint64_t)(us) * WDT_CLK_FREQ) / 1000000ULL))
#define WDT_TICKS_TO_US(t)	(((uint64_t)(t) * 1000000ULL) / WDT_CLK_FREQ)

/*
 * WDT ISR — clears the pending interrupt on wakeup.
 * Actual time compensation is done in sys_clock_idle_exit()
 * via z_cms_lptim_hook_on_lpm_exit().
 */
static void wdt_lptimer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	Cy_WDT_ClearInterrupt();
}

/*
 * Called by SysTick driver before entering low-power mode.
 * Programs WDT as a one-shot wakeup timer.
 */
void z_cms_lptim_hook_on_lpm_entry(uint64_t max_lpm_time_us)
{
	uint32_t ticks;

	if (max_lpm_time_us > WDT_TICKS_TO_US(WDT_CNT_MAX - 1U)) {
		ticks = WDT_CNT_MAX - 1U;
	} else {
		ticks = US_TO_WDT_TICKS(max_lpm_time_us);
	}

	if (ticks == 0U) {
		ticks = 1U;
	}

	Cy_WDT_Unlock();
	Cy_WDT_Disable();

	/*
	 * Reset counter while disabled (double write — HW quirk
	 * documented in wdt_ifx_cat1.c line 314).
	 */
	Cy_WDT_ResetCounter();
	Cy_WDT_ResetCounter();

	/* Configure as one-shot LP timer:
	 * - WARN_LIMIT fires interrupt at desired timeout
	 * - UPPER_LIMIT at max, no reset action
	 * - No AUTO_SERVICE so counter preserves value for readback
	 * - Runs in DeepSleep (DPSLP_PAUSE disabled)
	 */
	Cy_WDT_SetLowerLimit(0U);
	Cy_WDT_SetUpperLimit(WDT_CNT_MAX);
	Cy_WDT_SetWarnLimit(ticks);

	Cy_WDT_SetLowerAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
	Cy_WDT_SetUpperAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
	Cy_WDT_SetWarnAction(CY_WDT_WARN_ACTION_INT);
	Cy_WDT_SetAutoService(CY_WDT_DISABLE);
	Cy_WDT_SetDeepSleepPause(CY_WDT_DISABLE);
	Cy_WDT_SetDebugRun(CY_WDT_ENABLE);

	Cy_WDT_ClearInterrupt();
	Cy_WDT_UnmaskInterrupt();

	Cy_WDT_Enable();

	/* Wait for enable + config to propagate across ILO domain */
	Cy_SysLib_DelayUs(WDT_PROPAGATION_DELAY_US);

	Cy_WDT_Lock();
}

/*
 * Called by SysTick driver after exiting low-power mode.
 * Returns elapsed microseconds.
 */
uint64_t z_cms_lptim_hook_on_lpm_exit(void)
{
	uint32_t cnt;
	uint64_t elapsed_us;

	cnt = Cy_WDT_GetCount();
	elapsed_us = WDT_TICKS_TO_US(cnt);

	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	Cy_WDT_ClearInterrupt();
	Cy_WDT_MaskInterrupt();
	Cy_WDT_Lock();

	return elapsed_us;
}

/*
 * One-time init: route WDT system interrupt to CPU using
 * Infineon SysInt mechanism.
 */
static int soc_lptimer_init(void)
{
	enable_sys_int(WDT_SYS_INT_IDX,
		       WDT_CPU_INT_IDX,
		       wdt_lptimer_isr,
		       NULL);

	return 0;
}

SYS_INIT(soc_lptimer_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
