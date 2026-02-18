#define DT_DRV_COMPAT infineon_lp_timer_wdt

#include <zephyr/init.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <cmsis_core.h>

#include "cy_wdt.h"
#include "cy_syslib.h"

/* ----------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------- */

#ifndef CONFIG_CYT4DN_ILO0_FREQ_HZ
#define CONFIG_CYT4DN_ILO0_FREQ_HZ 32768U
#endif

#define CYC_PER_TICK \
	(CONFIG_CYT4DN_ILO0_FREQ_HZ / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#define COUNTER_MAX 0xFFFFFFFFUL
#define MAX_TICKS   ((COUNTER_MAX / CYC_PER_TICK) - 1U)
#define MAX_CYCLES  (MAX_TICKS * CYC_PER_TICK)
#define MIN_DELAY   4U

/* NVIC IRQ number from devicetree */
#define WDT_CPU_IRQ DT_INST_PROP_BY_IDX(0, system_interrupts, 1)

/*
 * Time to wait for WDT disable/enable to propagate across
 * AHB → ILO0 clock domain.  4 ILO0 cycles at 32768 Hz = ~122µs.
 */
#define WDT_PROPAGATION_DELAY_US 125U

/* ----------------------------------------------------------------
 * State
 * ---------------------------------------------------------------- */

static struct k_spinlock lock;
static uint32_t total_cycles;
static uint32_t announced_cycles;
static uint32_t programmed_limit;

/* ----------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------- */

/*
 * Clear pending WDT interrupt at peripheral AND NVIC.
 * NVIC pending bit is sticky — must be explicitly cleared.
 */
static void clear_all_pending(void)
{
	Cy_WDT_ClearInterrupt();
	(void)SRSS_WDT_INTR;   /* read-back flush */
	__DSB();
	NVIC_ClearPendingIRQ(WDT_CPU_IRQ);
	__DSB();
	__ISB();
}

/*
 * Stop the WDT counter and wait for it to actually stop.
 *
 * WDT_CTL[31] = ENABLE (command), WDT_CTL[0] = ENABLED (status).
 * After clearing ENABLE, we must wait for the disable to propagate
 * across the clock domain before the counter truly stops.
 *
 * WDT must be unlocked before calling this.
 */
static void wdt_stop_and_wait(void)
{
	Cy_WDT_Disable();
	Cy_SysLib_DelayUs(WDT_PROPAGATION_DELAY_US);
}

/* ----------------------------------------------------------------
 * ISR
 *
 * Fired when WDT_CNT == WARN_LIMIT.
 * AUTO_SERVICE already reset WDT_CNT to 0.
 *
 * WDT lock is NOT retained through DeepSleep.
 * ---------------------------------------------------------------- */

static void wdt_isr(const void *arg)
{
	ARG_UNUSED(arg);

	k_spinlock_key_t key = k_spin_lock(&lock);

	Cy_WDT_Unlock();
	Cy_WDT_ClearInterrupt();
	Cy_WDT_Lock();

	/* Exactly programmed_limit ILO0 cycles elapsed */
	total_cycles += programmed_limit;

	uint32_t unannounced = total_cycles - announced_cycles;
	uint32_t dticks      = unannounced / CYC_PER_TICK;

	announced_cycles += dticks * CYC_PER_TICK;

	k_spin_unlock(&lock, key);

	/* Outside spinlock — may re-enter sys_clock_set_timeout */
	sys_clock_announce(dticks);
}

/* ----------------------------------------------------------------
 * sys_clock_set_timeout
 *
 * Reprogram sequence:
 *   1. Disable WDT (counter stops — no match possible)
 *   2. Wait for disable to propagate (~125µs)
 *   3. Read counter, accumulate elapsed cycles
 *   4. Reset counter to 0
 *   5. Set new WARN_LIMIT
 *   6. Clear all pending interrupts (peripheral + NVIC)
 *   7. Re-enable WDT (counter starts from 0)
 *
 * Because the counter is stopped during steps 3-6, there is
 * ZERO race window.  The old WARN_LIMIT cannot match.
 * ---------------------------------------------------------------- */

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	if (ticks == K_TICKS_FOREVER) {
		k_spin_unlock(&lock, key);
		return;
	}

	Cy_WDT_Unlock();

	/* ---- Step 1-2: Stop the counter ---- */
	wdt_stop_and_wait();

	/* ---- Step 3: Read counter, accumulate ---- */
	uint32_t cnt = Cy_WDT_GetCount();

	/*
	 * Check if AUTO_SERVICE completed a full period while we
	 * were stopping.  If INTR is set, the full programmed_limit
	 * elapsed; if not, only a partial count elapsed.
	 */
	if (SRSS_WDT_INTR & _VAL2FLD(WDT_INTR_WDT, 1U)) {
		total_cycles += programmed_limit;
	} else {
		total_cycles += cnt;
	}

	/* ---- Step 4: Reset counter to 0 ---- */
	/*
	 * Cy_WDT_ResetCounter writes WDT_CNT=0 directly.
	 * Only works when WDT is disabled (per PDL docs).
	 */
	Cy_WDT_ResetCounter();

	/* ---- Step 5: Program new WARN_LIMIT ---- */
	uint32_t req   = (uint32_t)CLAMP(ticks, 1, (int32_t)MAX_TICKS);
	uint32_t delay = req * CYC_PER_TICK;
	delay = CLAMP(delay, (uint32_t)MIN_DELAY, (uint32_t)MAX_CYCLES);

	Cy_WDT_SetWarnLimit(delay);
	programmed_limit = delay;

	/* ---- Step 6: Clear ALL pending interrupts ---- */
	clear_all_pending();

	/* ---- Step 7: Re-enable WDT ---- */
	Cy_WDT_Enable();    /* Cy_WDT_Enable also calls ClearInterrupt */

	Cy_WDT_Lock();

	k_spin_unlock(&lock, key);
}

/* ----------------------------------------------------------------
 * sys_clock_elapsed
 * ---------------------------------------------------------------- */

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t now    = total_cycles + Cy_WDT_GetCount();
	uint32_t dticks = (now - announced_cycles) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);

	return dticks;
}

/* ----------------------------------------------------------------
 * sys_clock_cycle_get_32
 * ---------------------------------------------------------------- */

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t ret = total_cycles + Cy_WDT_GetCount();

	k_spin_unlock(&lock, key);

	return ret;
}

/* ----------------------------------------------------------------
 * Init
 * ---------------------------------------------------------------- */

static int sys_clock_driver_init(void)
{
	printk(">>> WDT LP TIMER: CYC_PER_TICK=%u, TICKS/SEC=%u, "
	       "HW_CYC/SEC=%u\n",
	       (unsigned)CYC_PER_TICK,
	       (unsigned)CONFIG_SYS_CLOCK_TICKS_PER_SEC,
	       (unsigned)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	Cy_SysLib_DelayUs(WDT_PROPAGATION_DELAY_US);
	Cy_WDT_ClearInterrupt();

	Cy_WDT_SetLowerLimit(0U);
	Cy_WDT_SetWarnLimit(CYC_PER_TICK);
	Cy_WDT_SetUpperLimit(COUNTER_MAX);

	Cy_WDT_SetLowerAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
	Cy_WDT_SetUpperAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
	Cy_WDT_SetWarnAction(CY_WDT_WARN_ACTION_INT);

	Cy_WDT_SetAutoService(CY_WDT_ENABLE);
	Cy_WDT_SetDeepSleepPause(CY_WDT_DISABLE);
	Cy_WDT_SetHibernatePause(CY_WDT_ENABLE);

	Cy_WDT_UnmaskInterrupt();
	Cy_WDT_ResetCounter();

	total_cycles     = 0U;
	announced_cycles = 0U;
	programmed_limit = CYC_PER_TICK;

	enable_sys_int(DT_INST_PROP_BY_IDX(0, system_interrupts, 0),
		       DT_INST_PROP_BY_IDX(0, system_interrupts, 1),
		       wdt_isr, NULL);

	clear_all_pending();
	Cy_WDT_Enable();
	Cy_WDT_Lock();

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
