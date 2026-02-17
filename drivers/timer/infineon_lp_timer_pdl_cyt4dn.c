#if 0
/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Low Power timer driver for Infineon CYT4DN using WDT
 * 
 * Verified facts:
 * - ILO0 is enabled in PRE_KERNEL_1
 * - Watchdog driver works with same IRQ 17 and enable_sys_int()
 * - CONFIG_SYS_CLOCK_TICKS_PER_SEC must be 1000 (not 10000)
 * - WDT must be unlocked before configuration changes
 */

#define DT_DRV_COMPAT infineon_lp_timer_wdt

#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys_clock.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ifx_lp_timer_wdt, CONFIG_KERNEL_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 1
#error Only one LPTIMER instance should be enabled
#endif

#define LPTIMER_MIN_DELAY       (10U)
#define LPTIMER_MAX_DELAY_TICKS (0xFFFFFFF0UL)

static const uint32_t clock_frequency = DT_PROP(DT_NODELABEL(clk_ilo), clock_frequency);
static uint64_t last_lptimer_value;
static struct k_spinlock lock;
static bool lptimer_enabled;

#include "cy_wdt.h"

static void lptimer_enable_event(bool enable)
{
	if (enable) {
		Cy_WDT_ClearInterrupt();
		Cy_WDT_UnmaskInterrupt();
	} else {
		Cy_WDT_MaskInterrupt();
		Cy_WDT_ClearInterrupt();
	}
	
	lptimer_enabled = enable;
}

static void lptimer_set_delay(uint32_t delay)
{
	uint32_t current_count;
	uint32_t target_count;
	unsigned int key;

	if (delay < LPTIMER_MIN_DELAY) {
		delay = LPTIMER_MIN_DELAY;
	}
	if (delay > LPTIMER_MAX_DELAY_TICKS) {
		delay = LPTIMER_MAX_DELAY_TICKS;
	}

	Cy_WDT_ClearInterrupt();

	key = irq_lock();

	current_count = Cy_WDT_GetCount();
	target_count = current_count + delay;

	Cy_WDT_Unlock();
	Cy_WDT_SetWarnLimit(target_count);
	Cy_WDT_SetWarnAction(CY_WDT_WARN_ACTION_INT);
	Cy_WDT_Lock();

	irq_unlock(key);

	Cy_WDT_UnmaskInterrupt();
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	uint32_t current_cycles;
	uint32_t cycles_per_tick;
	uint32_t delay_cycles;
	uint64_t next_tick_cycles;
	k_spinlock_key_t key;

	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	if (ticks == K_TICKS_FOREVER) {
		lptimer_enable_event(false);
		return;
	}

	lptimer_enable_event(true);

	if (ticks < 1) {
		ticks = 1;
	}

	cycles_per_tick = clock_frequency / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

	key = k_spin_lock(&lock);

	current_cycles = Cy_WDT_GetCount();

	next_tick_cycles = ((current_cycles / cycles_per_tick) + ticks) * cycles_per_tick;

	if (next_tick_cycles < current_cycles + (ticks * cycles_per_tick)) {
		next_tick_cycles += cycles_per_tick;
	}

	delay_cycles = (uint32_t)(next_tick_cycles - current_cycles);

	if (delay_cycles < LPTIMER_MIN_DELAY) {
		next_tick_cycles += cycles_per_tick;
		delay_cycles = (uint32_t)(next_tick_cycles - current_cycles);
	}

	if (delay_cycles > LPTIMER_MAX_DELAY_TICKS) {
		delay_cycles = LPTIMER_MAX_DELAY_TICKS;
	}

	lptimer_set_delay(delay_cycles);

	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	uint32_t current_cycles;
	uint32_t cycles_per_tick;
	uint32_t delta_cycles;
	uint32_t delta_ticks;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	cycles_per_tick = clock_frequency / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

	k_spinlock_key_t key = k_spin_lock(&lock);

	current_cycles = Cy_WDT_GetCount();

	delta_cycles = current_cycles - (uint32_t)last_lptimer_value;

	k_spin_unlock(&lock, key);

	delta_ticks = delta_cycles / cycles_per_tick;

	return delta_ticks;
}

uint32_t sys_clock_cycle_get_32(void)
{
	uint32_t cycles;

	k_spinlock_key_t key = k_spin_lock(&lock);

	cycles = Cy_WDT_GetCount();

	k_spin_unlock(&lock, key);

	return cycles;
}

static void lptimer_isr(void)
{
	Cy_WDT_ClearInterrupt();

	if (lptimer_enabled) {
		uint32_t current_cycles = Cy_WDT_GetCount();
		uint32_t cycles_per_tick = clock_frequency / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		uint64_t delta_ticks;
		k_spinlock_key_t key = k_spin_lock(&lock);

		delta_ticks = (uint64_t)((current_cycles - last_lptimer_value) / cycles_per_tick);

		last_lptimer_value += delta_ticks * cycles_per_tick;

		k_spin_unlock(&lock, key);

		sys_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? delta_ticks
								      : (delta_ticks > 0));
	}
}

static int lptimer_init(void)
{
	lptimer_enabled = false;
	last_lptimer_value = 0;
	
	Cy_WDT_Unlock();
	Cy_WDT_Disable();

	Cy_WDT_SetLowerLimit(0);
	Cy_WDT_SetUpperLimit(0xFFFFUL);
	Cy_WDT_SetWarnLimit(0xFFFFFFFFUL);
	Cy_WDT_SetLowerAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
	Cy_WDT_SetUpperAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
	Cy_WDT_SetWarnAction(CY_WDT_WARN_ACTION_INT);

	Cy_WDT_SetDeepSleepPause(CY_WDT_DISABLE);
	Cy_WDT_SetHibernatePause(CY_WDT_DISABLE);
	Cy_WDT_SetAutoService(CY_WDT_DISABLE);

	enable_sys_int(DT_INST_PROP_BY_IDX(0, system_interrupts, 0),
		       DT_INST_PROP_BY_IDX(0, system_interrupts, 1),
		       (void (*)(const void *))(void *)lptimer_isr,
		       NULL);

	Cy_WDT_Enable();
//	cycles_per_tick = clock_frequency / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
//	initial_count = Cy_WDT_GetCount();
//	initial_target = initial_count + cycles_per_tick;
//	Cy_WDT_SetWarnLimit(initial_target);
	Cy_SysInt_EnableSystemInt(17);
	Cy_WDT_UnmaskInterrupt();
//	printf("status of wdt = %d\n",Cy_WDT_IsEnabled());
	Cy_WDT_Lock();
	lptimer_enabled = true;

	return 0;
}

SYS_INIT(lptimer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
#endif

#if 1
#define DT_DRV_COMPAT infineon_lp_timer_wdt
#include <zephyr/init.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

/* PDL headers provided by the downstream BSP */
#include "cy_wdt.h"

/* -------------------------------------------------------------------------
 * Kconfig knobs
 * -------------------------------------------------------------------------
 * CONFIG_CYT4DN_ILO0_FREQ_HZ   - ILO0 nominal frequency in Hz (default 32000)
 * CONFIG_SYS_CLOCK_TICKS_PER_SEC - standard Zephyr tick rate
 * CONFIG_TICKLESS_KERNEL          - standard Zephyr tickless flag
 * CONFIG_CYT4DN_WDT_TIMER_IRQ     - IRQ number for the WDT/SRSS interrupt
 * CONFIG_CYT4DN_WDT_TIMER_IRQ_PRI - IRQ priority
 * -------------------------------------------------------------------------
 */

#ifndef CONFIG_CYT4DN_ILO0_FREQ_HZ
#define CONFIG_CYT4DN_ILO0_FREQ_HZ  32768U
#endif

/*
 * CYC_PER_TICK: how many ILO0 clock cycles equal one kernel tick.
 *
 * Example: ILO0 = 32000 Hz, ticks/sec = 1000  ->  CYC_PER_TICK = 32
 */
#define CYC_PER_TICK \
        (CONFIG_CYT4DN_ILO0_FREQ_HZ / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

/*
 * The WDT counter is 32-bit. MAX_TICKS is the largest one-shot timeout we
 * can program while leaving one CYC_PER_TICK of headroom before overflow.
 */
#define WDT_COUNTER_MAX  0xFFFFFFFFUL
#define MAX_TICKS        ((WDT_COUNTER_MAX / CYC_PER_TICK) - 1U)
#define MAX_CYCLES       (MAX_TICKS * CYC_PER_TICK)

/*
 * Minimum number of ILO0 cycles we are willing to program as a future
 * timeout. The WDT is asynchronous to the CPU bus clock; the TRM states
 * changes take up to 3 ILO0 cycles to take effect. We add one extra cycle
 * of margin, giving MIN_DELAY = 4 ILO0 cycles.
 */
#define MIN_DELAY  4U

/* -------------------------------------------------------------------------
 * Driver state
 * -------------------------------------------------------------------------
 */
static struct k_spinlock wdt_lock;

/*
 * Total ILO0 cycles elapsed and announced to the kernel respectively.
 * Both are updated only under the spinlock or in the ISR (with interrupts
 * disabled by the CPU exception entry mechanism).
 */
static uint32_t cycle_count;
static uint32_t announced_cycles;

/*
 * The WARN_LIMIT value that was programmed for the current timeout period.
 * Because AUTO_SERVICE resets the counter on each WARN event, cycle_count
 * must be incremented by this amount each time the ISR fires.
 *
 * In steady-state periodic mode this equals CYC_PER_TICK.
 * In one-shot idle mode this equals the programmed idle timeout in cycles.
 */
static uint32_t last_warn_limit;

/* -------------------------------------------------------------------------
 * Low-level WDT helpers
 *
 * All register writes to the WDT require an unlock/lock pair because the
 * lock is not retained through DeepSleep (TRM p.285). The PDL functions
 * Cy_WDT_Unlock() and Cy_WDT_Lock() perform the required two-write unlock
 * sequence and the single-write lock respectively.
 * -------------------------------------------------------------------------
 */

/*
 * wdt_set_warn_limit() - program a new WARN_LIMIT and (re-)enable
 * AUTO_SERVICE so the counter resets automatically on each match.
 *
 * Must be called with WDT already unlocked.
 */
static void wdt_set_warn_limit(uint32_t cycles)
{
        /* Clamp to hardware maximum */
        if (cycles > WDT_COUNTER_MAX) {
                cycles = WDT_COUNTER_MAX;
        }
        /* Enforce minimum safe programming distance */
        if (cycles < MIN_DELAY) {
                cycles = MIN_DELAY;
        }

        Cy_WDT_SetWarnLimit(cycles);
        last_warn_limit = cycles;
}

/*
 * wdt_service() - reset the WDT counter to zero (feed the watchdog).
 * Needed after reprogramming WARN_LIMIT in tickless one-shot mode so
 * the new timeout is measured from zero.
 *
 * Must be called with WDT already unlocked.
 * Note: Cy_WDT_SetService() writes the SERVICE register which resets
 * WDT_CNT to zero (TRM p.284).
 */
static void wdt_service(void)
{
        Cy_WDT_SetService();
}

/* -------------------------------------------------------------------------
 * elapsed()
 *
 * Returns the number of ILO0 cycles that have elapsed since the counter
 * was last reset (i.e. since the last WARN event / AUTO_SERVICE reset, or
 * since init).
 *
 * The WDT is a free-running up-counter, so "elapsed" is simply the live
 * counter value read from WDT_CNT.
 *
 * This function must be called with interrupts disabled (inside the
 * spinlock) to produce a consistent snapshot.
 * -------------------------------------------------------------------------
 */
static uint32_t elapsed(void)
{
        /*
         * Cy_WDT_GetCount() reads WDT_CNT[31:0] directly.
         * No wrap detection is needed here because AUTO_SERVICE resets the
         * counter to zero at each WARN_LIMIT match, so the counter never
         * reaches WARN_LIMIT between ISR invocations in normal operation.
         * In one-shot idle mode the counter is reset by wdt_service() before
         * the idle period starts.
         */
        return Cy_WDT_GetCount();
}
static void cyt4dn_wdt_isr(const void *arg)
{
        ARG_UNUSED(arg);

        /*
         * Step 1+2+3: unlock, clear interrupt, re-lock.
         *
         * The WDT lock is not retained through DeepSleep (TRM p.285).
         * We unconditionally unlock here to handle both the normal-operation
         * case (WDT was still unlocked) and the DeepSleep-wakeup case (WDT
         * is locked after wakeup). Cy_WDT_ClearInterrupt() clears the
         * WDT_INTR[0] bit by writing 1 to it (TRM Table 20-1).
         */
        Cy_WDT_Unlock();
        Cy_WDT_ClearInterrupt();
        Cy_WDT_Lock();

        /*
         * Step 4: accumulate elapsed cycles.
         *
         * last_warn_limit is the number of ILO0 cycles that the counter ran
         * for during this timeout period (either CYC_PER_TICK in steady-state
         * or the programmed idle timeout). AUTO_SERVICE already reset the
         * counter to zero, so the full timeout has expired.
         */
        cycle_count += last_warn_limit;

        /*
         * Step 5: announce elapsed ticks to the kernel.
         *
         * We compute how many complete ticks worth of cycles have accumulated
         * since the last announcement, convert to ticks, advance
         * announced_cycles, and call sys_clock_announce().
         */
        uint32_t dcycles = cycle_count - announced_cycles;
        uint32_t dticks  = dcycles / CYC_PER_TICK;

        announced_cycles += dticks * CYC_PER_TICK;
        sys_clock_announce(dticks);
}

/* -------------------------------------------------------------------------
 * sys_clock_set_timeout()
 *
 * Called by the kernel (with interrupts disabled) to program the next
 * wakeup. In tickless mode, ticks is the number of kernel ticks until the
 * next scheduled event. When idle=true the CPU is about to enter a
 * low-power state.
 *
 * Strategy:
 *   - In non-tickless mode this function is never called; the driver runs
 *     in fixed-period mode (WARN_LIMIT = CYC_PER_TICK, AUTO_SERVICE on).
 *   - In tickless mode we reprogram WARN_LIMIT to match the requested
 *     number of ticks, then reset the counter via SERVICE so the timeout
 *     is measured from now.
 * -------------------------------------------------------------------------
 */
void sys_clock_set_timeout(int32_t ticks, bool idle)
{
        ARG_UNUSED(idle);

        if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
                return;
        }

        k_spinlock_key_t key = k_spin_lock(&wdt_lock);

        /*
         * Snapshot current elapsed cycles before we reprogram. This ensures
         * cycle_count stays monotonically consistent.
         */
        uint32_t current_elapsed = elapsed();

        cycle_count += current_elapsed;

        /*
         * Compute unannounced cycles so we round up to the next tick boundary
         * correctly (mirrors the SysTick driver approach).
         */
        uint32_t unannounced = cycle_count - announced_cycles;

        uint32_t delay;

        if (ticks == K_TICKS_FOREVER) {
                delay = MAX_CYCLES;
        } else {
                /* Clamp to our hardware maximum */
                ticks = CLAMP(ticks, 0, (int32_t)MAX_TICKS);

                /* Requested delay in ILO0 cycles */
                delay = (uint32_t)ticks * CYC_PER_TICK;

                /*
                 * Round up to the next tick boundary relative to the
                 * unannounced cycles already elapsed, then subtract what
                 * has already passed. This prevents us from programming
                 * a timeout that fires slightly before the true tick edge.
                 */
                delay += unannounced;
                delay  = DIV_ROUND_UP(delay, CYC_PER_TICK) * CYC_PER_TICK;
                delay -= unannounced;
        }

        /* Clamp to [MIN_DELAY, MAX_CYCLES] */
        delay = CLAMP(delay, (uint32_t)MIN_DELAY, (uint32_t)MAX_CYCLES);

        /*
         * Reprogram: unlock, set new WARN_LIMIT, service (reset counter to 0),
         * re-lock. The counter restarts from zero so the new delay is measured
         * cleanly from this moment.
         */
        Cy_WDT_Unlock();
        wdt_set_warn_limit(delay);
        wdt_service();          /* reset WDT_CNT to 0 */
        Cy_WDT_Lock();

        k_spin_unlock(&wdt_lock, key);
}
uint32_t sys_clock_elapsed(void)
{
        if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
                return 0;
        }

        k_spinlock_key_t key = k_spin_lock(&wdt_lock);

        /*
         * Total cycles elapsed since init = cycle_count (announced periods)
         * + the live counter value (current partial period).
         */
        uint32_t total    = cycle_count + elapsed();
        uint32_t dcycles  = total - announced_cycles;
        uint32_t dticks   = dcycles / CYC_PER_TICK;

        k_spin_unlock(&wdt_lock, key);

        return dticks;
}

/* -------------------------------------------------------------------------
 * sys_clock_cycle_get_32()
 *
 * Returns the current cycle counter value (ILO0 cycles since boot).
 * Used by the kernel for time measurements at cycle resolution.
 * -------------------------------------------------------------------------
 */
uint32_t sys_clock_cycle_get_32(void)
{
        k_spinlock_key_t key = k_spin_lock(&wdt_lock);
        uint32_t ret = cycle_count + elapsed();

        k_spin_unlock(&wdt_lock, key);
        return ret;
}

/* -------------------------------------------------------------------------
 * sys_clock_driver_init()
 *
 * Initialises the WDT as the kernel tick source.
 *
 * Configuration applied (referencing TRM Table 20-1 and cy_wdt_b.c):
 *   LOWER_ACTION  = NONE  (window mode not used)
 *   UPPER_ACTION  = NONE  (no watchdog reset; we are using this as a timer)
 *   WARN_ACTION   = 1     (interrupt on WARN_LIMIT match)
 *   AUTO_SERVICE  = 1     (counter resets to 0 automatically at WARN_LIMIT)
 *   DPSLP_PAUSE   = 0     (counter keeps running in DeepSleep -- required!)
 *   HIB_PAUSE     = 1     (pause in Hibernate; we do not use WDT for
 *                          Hibernate wakeup in this driver)
 *   WARN_LIMIT    = CYC_PER_TICK (one tick period)
 * -------------------------------------------------------------------------
 */
static int sys_clock_driver_init(void)
{
        /* Unlock WDT registers for configuration */
        Cy_WDT_Unlock();

        /* Disable the WDT before reconfiguring */
        Cy_WDT_Disable();

        /*
         * Clear any stale interrupt that may be pending from a previous
         * configuration or a reset. WDT_INTR[0] is cleared by writing 1
         * (TRM Table 20-1).
         */
        Cy_WDT_ClearInterrupt();

        /*
         * Configure limits:
         *   LOWER_LIMIT = 0  -> LOWER_ACTION = NONE, window mode off
         *   WARN_LIMIT  = CYC_PER_TICK -> fires interrupt every tick
         *   UPPER_LIMIT = 0xFFFFFFFF   -> UPPER_ACTION = NONE, no reset
         *
         * Note: CY_WDT_DEFAULT_UPPER_LIMIT (32000) in cy_wdt_b.c sets
         * UPPER_ACTION = RESET. We explicitly override this to NONE because
         * we are not using the watchdog reset function.
         */
        Cy_WDT_SetLowerLimit(0U);
        Cy_WDT_SetWarnLimit(CYC_PER_TICK);
        Cy_WDT_SetUpperLimit(WDT_COUNTER_MAX);

        last_warn_limit = CYC_PER_TICK;

        /* Actions */
        Cy_WDT_SetLowerAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
        Cy_WDT_SetUpperAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
        Cy_WDT_SetWarnAction(CY_WDT_WARN_ACTION_INT);

        /*
         * AUTO_SERVICE: when the counter reaches WARN_LIMIT it is reset to 0
         * automatically. This allows clean periodic tick generation without
         * requiring a manual SERVICE write in the ISR (TRM Table 20-1,
         * WDT_CONFIG[12]).
         */
        Cy_WDT_SetAutoService(CY_WDT_ENABLE);

        /*
         * DPSLP_PAUSE = 0 (DISABLE): counter keeps counting in DeepSleep.
         * This is mandatory; if we pause the counter we cannot wake up.
         * (TRM Table 20-1, WDT_CONFIG[29])
         */
        Cy_WDT_SetDeepSleepPause(CY_WDT_DISABLE);

        /*
         * HIB_PAUSE = 1 (ENABLE): pause in Hibernate. This driver targets
         * DeepSleep; Hibernate wakeup is handled separately via the
         * PWR_HIBERNATE registers.
         */
        Cy_WDT_SetHibernatePause(CY_WDT_ENABLE);

        /*
         * Unmask the WDT interrupt so it is forwarded to the CPU (and to the
         * WIC in Sleep/DeepSleep modes). WDT_INTR_MASK[0] = 1.
         */
        Cy_WDT_UnmaskInterrupt();

        /* Reset cycle tracking state */
        cycle_count      = 0U;
        announced_cycles = 0U;

	enable_sys_int(DT_INST_PROP_BY_IDX(0, system_interrupts, 0),
                   DT_INST_PROP_BY_IDX(0, system_interrupts, 1),
                   (void (*)(const void *))(void *)cyt4dn_wdt_isr,
                   NULL);
        /* Enable the WDT counter -- counter starts from 0 */
        Cy_WDT_Enable();

        /* Lock the WDT registers for safety */
        Cy_WDT_Lock();

        return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
#endif
