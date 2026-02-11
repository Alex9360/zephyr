/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT infineon_lp_timer_wdt

#include <zephyr/device.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys_clock.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ifx_lp_timer_wdt, CONFIG_KERNEL_LOG_LEVEL);

/* The application only needs one lptimer. Report an error if more than one is selected. */
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 1
#error Only one LPTIMER instance should be enabled
#endif

#define LPTIMER_MIN_DELAY       (3U)              
#define LPTIMER_MAX_DELAY_TICKS (0xFFFFFFF0UL) 

static const uint32_t wdt_base = DT_INST_REG_ADDR(0);
static const uint32_t clock_frequency = DT_PROP(DT_NODELABEL(clk_ilo), clock_frequency);
static uint64_t last_lptimer_value;
static struct k_spinlock lock;

static bool lptimer_enabled;

#include "cy_wdt.h"

/* According to TRM, ILO0 is the clock source for WDT.
 * The WDT takes 4 ILO0 cycles for changes to propagate.
 * At 32768 Hz, this is approximately 122 microseconds.
 */
#define WDT_PROPAGATION_DELAY_US (122U)

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
//	Cy_WDT_MaskInterrupt();

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

	Cy_WDT_Disable();

    	Cy_WDT_SetLowerLimit(CY_WDT_DEFAULT_LOWER_LIMIT);
    	Cy_WDT_SetUpperLimit(0xffffffff);
    	Cy_WDT_SetWarnLimit(CY_WDT_DEFAULT_WARN_LIMIT);
    	Cy_WDT_SetLowerAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
    	Cy_WDT_SetUpperAction(CY_WDT_LOW_UPPER_LIMIT_ACTION_NONE);
    	Cy_WDT_SetWarnAction(CY_WDT_WARN_ACTION_INT);

	Cy_WDT_SetDeepSleepPause(CY_WDT_DISABLE);
	Cy_WDT_SetHibernatePause(CY_WDT_DISABLE);
	Cy_WDT_SetAutoService(CY_WDT_ENABLE);

	 enable_sys_int(DT_INST_PROP_BY_IDX(0, system_interrupts, 0),
                       DT_INST_PROP_BY_IDX(0, system_interrupts, 1),
                       (void (*)(const void *))(void *)lptimer_isr,
                       NULL);

	Cy_WDT_Enable();
	Cy_WDT_Lock();

	return 0;
}

SYS_INIT(lptimer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
