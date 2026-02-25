#if 0
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

#define US_TO_WDT_CYCLES(us)	((uint32_t)(((uint64_t)(us) * WDT_CLK_FREQ) / 1000000ULL))
#define WDT_CYCLES_TO_US(t)	(((uint64_t)(t) * 1000000ULL) / WDT_CLK_FREQ)

/*
 * WDT ISR — clears the pending interrupt on wakeup.
 * Actual time compensation is done in sys_clock_idle_exit()
 * via z_cms_lptim_hook_on_lpm_exit().
 */
static void wdt_lptimer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	printf("isr occured\n");
	Cy_WDT_ClearInterrupt();
	Cy_WDT_MaskInterrupt();
}

/*
 * Called by SysTick driver before entering low-power mode.
 * Programs WDT as a one-shot wakeup timer.
 */
void z_cms_lptim_hook_on_lpm_entry(uint64_t max_lpm_time_us)
{
	printf("entry \n");
	uint32_t ticks;

	if (max_lpm_time_us > WDT_CYCLES_TO_US(WDT_CNT_MAX - 1U)) {
		ticks = WDT_CNT_MAX - 1U;
	} else {
		ticks = US_TO_WDT_CYCLES(max_lpm_time_us);
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
	printf("exit \n");
	uint32_t cnt;
	uint64_t elapsed_us;

	cnt = Cy_WDT_GetCount();
	elapsed_us = WDT_CYCLES_TO_US(cnt);

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
#endif
#if 1
/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTC-based low-power timer hooks for Cortex-M SysTick.
 *
 * Uses PDL for all BACKUP register access. PDL handles:
 *   - RTC_BUSY polling before writes
 *   - WRITE enable/disable with verification
 *   - SyncFromRtc with proper timing
 *
 * XRES hard fault root cause:
 *   CY_RTC_DELAY_WHILE_READING_US = 42000000 / cy_AhbFreqHz
 *   After XRES, SRAM is zeroed -> cy_AhbFreqHz = 0 -> div/0.
 *   Fixed by calling SystemCoreClockUpdate() in init which reads
 *   clock config from HW registers (survive XRES) and sets
 *   cy_AhbFreqHz, SystemCoreClock, cy_delayFreqMhz etc.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/pm/pm.h>

#include <cy_rtc.h>
#include <cy_sysint.h>
#include <cy_sysclk.h>
#include <cy_syslib.h>

#include <soc.h>

#define SECS_PER_MIN	60U
#define SECS_PER_HOUR	3600U
#define SECS_PER_DAY	86400U
#define US_PER_SEC	1000000ULL

#define RTC_SYS_INT_IDX	srss_interrupt_backup_IRQn
#define RTC_CPU_INT_IDX	3U

static uint32_t entry_secs;

static uint32_t time_to_day_secs(const cy_stc_rtc_config_t *t)
{
	return (t->hour * SECS_PER_HOUR) +
	       (t->min * SECS_PER_MIN) +
	        t->sec;
}

static void day_secs_to_hms(uint32_t total, uint32_t *h,
			    uint32_t *m, uint32_t *s)
{
	total %= SECS_PER_DAY;
	*h = total / SECS_PER_HOUR;
	total %= SECS_PER_HOUR;
	*m = total / SECS_PER_MIN;
	*s = total % SECS_PER_MIN;
}

static void rtc_lptimer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
}

void z_cms_lptim_hook_on_lpm_entry(uint64_t max_lpm_time_us)
{
	cy_stc_rtc_config_t now;
	cy_stc_rtc_alarm_t alarm;
	uint32_t timeout_sec;
	uint32_t alarm_total;
	uint32_t ah, am, as;

	Cy_RTC_GetDateAndTime(&now);
	entry_secs = time_to_day_secs(&now);

	timeout_sec = (uint32_t)((max_lpm_time_us + US_PER_SEC - 1U) /
				 US_PER_SEC);
	if (timeout_sec == 0U) {
		timeout_sec = 1U;
	}

	alarm_total = entry_secs + timeout_sec;
	day_secs_to_hms(alarm_total, &ah, &am, &as);

	(void)memset(&alarm, 0, sizeof(alarm));

	alarm.sec         = as;
	alarm.secEn       = CY_RTC_ALARM_ENABLE;
	alarm.min         = am;
	alarm.minEn       = CY_RTC_ALARM_ENABLE;
	alarm.hour        = ah;
	alarm.hourEn      = CY_RTC_ALARM_ENABLE;
	alarm.dayOfWeek   = CY_RTC_SUNDAY;
	alarm.dayOfWeekEn = CY_RTC_ALARM_DISABLE;
	alarm.date        = 1U;
	alarm.dateEn      = CY_RTC_ALARM_DISABLE;
	alarm.month       = CY_RTC_JANUARY;
	alarm.monthEn     = CY_RTC_ALARM_DISABLE;
	alarm.almEn       = CY_RTC_ALARM_ENABLE;

	Cy_RTC_SetAlarmDateAndTime(&alarm, CY_RTC_ALARM_1);

	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
	Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1);
}

uint64_t z_cms_lptim_hook_on_lpm_exit(void)
{
	cy_stc_rtc_config_t now;
	uint32_t exit_secs;
	uint32_t elapsed;

	Cy_RTC_GetDateAndTime(&now);
	exit_secs = time_to_day_secs(&now);

	if (exit_secs >= entry_secs) {
		elapsed = exit_secs - entry_secs;
	} else {
		elapsed = (SECS_PER_DAY - entry_secs) + exit_secs;
	}

	Cy_RTC_SetInterruptMask(0U);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);

	if (elapsed == 0U) {
		elapsed = 1U;
	}

	return (uint64_t)elapsed * US_PER_SEC;
}

/*
 * Ensure SRAM frequency variables are valid.
 *
 * After XRES, .bss is zeroed but clock HW registers survive.
 * SystemCoreClockUpdate() reads HW and sets:
 *   - SystemCoreClock
 *   - cy_AhbFreqHz  (used in CY_RTC_DELAY_WHILE_READING_US)
 *   - cy_delayFreqMhz (used in Cy_SysLib_DelayUs)
 *   - cy_delayFreqKhz
 *
 * Without this, cy_AhbFreqHz=0 causes 42000000/0 -> hard fault
 * on first Cy_RTC_GetDateAndTime() -> Cy_RTC_SyncFromRtc().
 */
static void ensure_clk_vars(void)
{
	extern uint32_t cy_AhbFreqHz;

	if (cy_AhbFreqHz == 0U) {
		SystemCoreClockUpdate();
	}

	/* If SystemCoreClockUpdate still failed, hardcode safe value.
	 * 8 MHz is the IMO default on T2G after reset.
	 * The actual value only affects a ~1 us delay in SyncFromRtc
	 * so being inexact here is harmless.
	 */
	if (cy_AhbFreqHz == 0U) {
		cy_AhbFreqHz = 8000000U;
	}
}

static int soc_lptimer_init(void)
{
	cy_stc_rtc_config_t init_time = {
		.sec       = 0U,
		.min       = 0U,
		.hour      = 12U,
		.amPm      = CY_RTC_AM,
		.hrFormat  = CY_RTC_24_HOURS,
		.dayOfWeek = CY_RTC_SATURDAY,
		.date      = 1U,
		.month     = CY_RTC_FEBRUARY,
		.year      = 25U,
	};

	/* MUST be first - prevents div/0 in PDL after XRES */
	ensure_clk_vars();

	/* Clear stale BACKUP state from previous boot */
	BACKUP_RTC_RW = 0U;
	Cy_RTC_SetInterruptMask(0U);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1 | CY_RTC_INTR_ALARM2);

	Cy_RTC_SelectClockSource(CY_SYSCLK_BAK_IN_ILO);
	Cy_RTC_Init(&init_time);

	enable_sys_int(RTC_SYS_INT_IDX,
		       RTC_CPU_INT_IDX,
		       rtc_lptimer_isr,
		       NULL);

	return 0;
}

SYS_INIT(soc_lptimer_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif
