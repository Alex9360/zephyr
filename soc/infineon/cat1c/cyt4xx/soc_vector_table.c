/*
 * Copyright (c) 2026 Linumiz
 * Copyright (c) 2026 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>

#define CPU_IRQS 16	/* M0P and M7 has 8 NVIC Lines and 8 Internal IRQs */

#if defined(CONFIG_SOC_CYT4XX_CORE_M0PLUS)
extern void cat1c_srom_syscall_isr(void);
#endif

const uintptr_t __irq_vector_table _irq_vector_table[CPU_IRQS] = {
#if defined(CONFIG_SOC_CYT4XX_CORE_M0PLUS)
	((uintptr_t)0x49),
	((uintptr_t)0x281),
	((uintptr_t)cat1c_srom_syscall_isr),
#else
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
#endif
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
};
