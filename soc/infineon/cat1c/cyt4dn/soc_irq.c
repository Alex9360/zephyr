/*
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cy_device.h"
#include <stdint.h>
#include <zephyr/fatal.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/toolchain.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/arch/arm/arch.h>

#include <cy_sysint.h>

#define SROM_IRQS 3

void enable_sys_int(uint32_t int_num, uint32_t priority, void (*isr)(const void *), const void *arg)
{
	irq_connect_dynamic(int_num, priority, isr, arg, 0);
	irq_enable(int_num);
}

/* Custom interrupt controller */
void z_soc_irq_init()
{
	/* Nothing to initialize */
}

void z_soc_irq_enable(unsigned int irq)
{
	if (irq <= CPUSS_SYSTEM_INT_NR) {
		Cy_SysInt_EnableSystemInt(irq);
	}
}

void z_soc_irq_disable(unsigned int irq)
{
	if (irq <= CPUSS_SYSTEM_INT_NR) {
		Cy_SysInt_DisableSystemInt(irq);
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	if (irq > CPUSS_SYSTEM_INT_NR) {
#ifdef CONFIG_INFINEON_CAT1C_M0PLUS
		return (CPUSS_CM0_SYSTEM_INT_CTL[irq] & CPUSS_CM0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
#elif CONFIG_INFINEON_CAT1C_M7_0
		return (CPUSS_CM7_0_SYSTEM_INT_CTL[irq] & CPUSS_CM7_0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
#else
		return (CPUSS_CM7_1_SYSTEM_INT_CTL[irq] & CPUSS_CM7_1_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
#endif
	}
	return 0;
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags)
{
#if (CONFIG_INFINEON_CAT1C_M0PLUS)
	/* Lower irqs (0-2) are used for SROM, upper IRQs for priority mapping */
	prio = MAX(prio, SROM_IRQS);
#endif
	NVIC_SetPriority(NvicMux0_IRQn + prio, prio);
	NVIC_EnableIRQ(NvicMux0_IRQn + prio);
	Cy_SysInt_SetInterruptSource(prio, irq);
}

void z_soc_irq_eoi(unsigned int irq)
{
	NVIC_ClearPendingIRQ(__get_IPSR() - 16);
}

unsigned int z_soc_irq_get_active(void)
{
	const volatile uint32 *const int_state =
#if IS_ENABLED(CONFIG_INFINEON_CAT1C_M0PLUS)
		&CPUSS_CM0_INT0_STATUS;
#elif IS_ENABLED(CONFIG_INFINEON_CAT1C_M7_0)
		CPUSS_CM7_0_INT_STATUS;
#else
		CPUSS_CM7_1_INT_STATUS;
#endif
	IRQn_Type actirqn = ((int32_t)__get_IPSR()) - 16;

	if (actirqn <= NvicMux7_IRQn &&
	    (int_state[actirqn] & CPUSS_CM0_INT0_STATUS_SYSTEM_INT_VALID_Msk)) {
		return (int_state[actirqn] & CPUSS_CM0_INT0_STATUS_SYSTEM_INT_IDX_Msk) + 16;
	}

#if IS_ENABLED(CONFIG_INFINEON_CAT1C_M7)
	/* Cortex-M7 has support for 8 software IRQn. These are appended to the sw_irq_table
	 * after the system interrupt sources. */
	if (actirqn >= Internal0_IRQn && actirqn <= Internal7_IRQn) {
		return ((CONFIG_NUM_IRQS - 1) - 8 + (actirqn - Internal0_IRQn)) + 16;
	}
#endif
	return (CONFIG_NUM_IRQS - 1) + 16;
}
