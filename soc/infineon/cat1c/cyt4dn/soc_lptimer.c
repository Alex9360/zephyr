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
//	printf("isr occured\n");
	Cy_WDT_ClearInterrupt();
	Cy_WDT_MaskInterrupt();
}

/*
 * Called by SysTick driver before entering low-power mode.
 * Programs WDT as a one-shot wakeup timer.
 */
void z_cms_lptim_hook_on_lpm_entry(uint64_t max_lpm_time_us)
{
//	printf("entry \n");
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
//	printf("exit \n");
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
#if 0
/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * RTC-based low-power timer hooks for Cortex-M SysTick companion.
 *
 * BACKUP domain survives XRES. On warm reset, stale alarm and
 * interrupt mask from previous boot are still active. Must clear
 * them before any PM cycle runs, otherwise stale alarm fires
 * immediately and systick compensation math gets elapsed=0 -> div/0.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/pm/pm.h>

#include <cy_rtc.h>
#include <cy_sysint.h>
#include <cy_sysclk.h>

#define SECS_PER_MIN	60U
#define SECS_PER_HOUR	3600U
#define SECS_PER_DAY	86400U
#define US_PER_SEC	1000000ULL

#ifndef CYT4DN_RTC_IRQ
#define CYT4DN_RTC_IRQ	srss_interrupt_backup_IRQn
#endif

#define RTC_IRQ_PRIO	3

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

	alarm.sec   = as;
	alarm.secEn = CY_RTC_ALARM_ENABLE;
	alarm.min   = am;
	alarm.minEn = CY_RTC_ALARM_ENABLE;
	alarm.hour  = ah;
	alarm.hourEn = CY_RTC_ALARM_ENABLE;

	alarm.dayOfWeek   = now.dayOfWeek;
	alarm.dayOfWeekEn = CY_RTC_ALARM_DISABLE;
	alarm.date        = now.date;
	alarm.dateEn      = CY_RTC_ALARM_DISABLE;
	alarm.month       = now.month;
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

	/*
	 * Guard: if elapsed is 0 (stale alarm fired immediately or
	 * RTC sec boundary race), return 1 second minimum.
	 * Returning 0 can cause division by zero in systick
	 * compensation math.
	 */
	if (elapsed == 0U) {
		elapsed = 1U;
	}

	return (uint64_t)elapsed * US_PER_SEC;
}

static int soc_lptimer_init(void)
{
	/*
	 * FIRST: Kill stale BACKUP state from previous boot.
	 *
	 * BACKUP domain survives XRES. Previous boot's alarm config
	 * and interrupt mask are still active. If not cleared, stale
	 * alarm fires immediately on first PM cycle causing
	 * elapsed=0 -> division by zero in systick driver.
	 */
	Cy_RTC_SetInterruptMask(0U);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1 | CY_RTC_INTR_ALARM2);

	/* Init RTC with ILO clock and valid time */
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

	Cy_RTC_SelectClockSource(CY_SYSCLK_BAK_IN_ILO);
	Cy_RTC_Init(&init_time);

	enable_sys_int(CYT4DN_RTC_IRQ,
		       RTC_IRQ_PRIO,
		       rtc_lptimer_isr,
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
 * RTC-based low-power timer hooks for Cortex-M SysTick companion.
 *
 * Uses ALARM1 of the always-on RTC as the wakeup source for
 * Sleep and DeepSleep modes. RTC is clocked from ILO0 and has
 * 1-second resolution.
 *
 * BACKUP domain (RTC, alarm config, BACKUP_RTC_RW, interrupt mask)
 * survives XRES. Stale state from a previous boot MUST be cleared
 * before any PM cycle runs, otherwise:
 *  - BACKUP_RTC_RW.WRITE/READ left set prevents Cy_RTC_Init and
 *    Cy_RTC_GetDateAndTime from working.
 *  - Stale alarm + interrupt mask fires immediately on first PM
 *    cycle causing elapsed=0 in systick compensation math.
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

/* System interrupt index for RTC backup domain */
#define RTC_SYS_INT_IDX	srss_interrupt_backup_IRQn

/* CPU interrupt slot to route the RTC system interrupt */
#define RTC_CPU_INT_IDX	3U

/* Snapshot of day-seconds at LPM entry */
static uint32_t entry_secs;

static uint32_t time_to_day_secs(const cy_stc_rtc_config_t *t)
{
	return (t->hour * SECS_PER_HOUR) +
	       (t->min  * SECS_PER_MIN) +
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
//	printf("isr occurred \n");
	ARG_UNUSED(arg);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
}

void z_cms_lptim_hook_on_lpm_entry(uint64_t max_lpm_time_us)
{
	static bool once;
	if (!once) {
		extern uint32_t cy_delayFreqMhz;
		printk("RTC entry: mhz=%u\n", cy_delayFreqMhz);
		once = true;
	}
//	printf("entry \n");
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
//	printf("exit \n");
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

	/*
	 * Guard: elapsed=0 happens when stale alarm fires immediately
	 * or RTC second boundary race. Return 1 second minimum to
	 * avoid division issues in systick compensation.
	 */
	if (elapsed == 0U) {
		elapsed = 1U;
	}

	return (uint64_t)elapsed * US_PER_SEC;
}

static int soc_lptimer_init(void)
{
	extern uint32_t debug_clk_at_prep;
	printk("prep_hook_clk=%u now_clk=%u\n", debug_clk_at_prep, SystemCoreClock);
	extern uint32_t cy_delayFreqMhz;
	extern uint32_t cy_delayFreqKhz;

	if (cy_delayFreqMhz == 0U) {
	    Cy_SysClk_ImoEnable();
	    Cy_SysLib_DelayUs(0);  // no-op but thats fine
	    SystemCoreClockUpdate();
	    /* if PLL still not up, force safe IMO defaults */
	    if (cy_delayFreqMhz == 0U) {
		cy_delayFreqMhz = 8U;
		cy_delayFreqKhz = 8000U;
	    }
	}

	uint32_t retry = CY_RTC_ACCESS_BUSY_RETRY_COUNT;

	/*
	 * BACKUP domain survives XRES. Force-clear BACKUP_RTC_RW
	 * to kill any stale READ or WRITE state from previous boot.
	 *
	 * If WRITE was stuck set, Cy_RTC_SyncFromRtc refuses to read
	 * (it requires WRITE=0). This causes Cy_RTC_GetDateAndTime and
	 * Cy_RTC_SetAlarmDateAndTime to operate on stale AHB data,
	 * leading to hard fault or bogus alarm timing on next PM cycle.
	 */
	BACKUP_RTC_RW = 0U;
	
	Cy_RTC_SetInterruptMask(0U);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1 | CY_RTC_INTR_ALARM2);

	printk("RTC_RW=0x%x clk=%u mhz=%u khz=%u\n",
	       BACKUP_RTC_RW, SystemCoreClock,
	       cy_delayFreqMhz, cy_delayFreqKhz);

	while ((Cy_RTC_GetSyncStatus() == CY_RTC_BUSY) && (retry != 0U)) {
		retry--;
		Cy_SysLib_DelayUs(CY_RTC_BUSY_RETRY_DELAY_US);
	}
	printk("BACKUP_RTC_RW = 0x%08x\n", BACKUP_RTC_RW);
	BACKUP_RTC_RW = 0U;
	/* Kill stale interrupt state from previous boot */
	Cy_RTC_SetInterruptMask(0U);
	Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1 | CY_RTC_INTR_ALARM2);

	Cy_RTC_SelectClockSource(CY_SYSCLK_BAK_IN_ILO);

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

	Cy_RTC_Init(&init_time);

	enable_sys_int(RTC_SYS_INT_IDX,
		       RTC_CPU_INT_IDX,
		       rtc_lptimer_isr,
		       NULL);

	return 0;
}

SYS_INIT(soc_lptimer_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif
