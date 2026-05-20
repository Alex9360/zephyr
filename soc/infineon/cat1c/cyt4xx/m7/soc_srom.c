/*
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/irq.h>

#include <cy_flash_srom.h>
#include <cy_sysint.h>

#define CY_SROM_DR_IPC_REQ_INTR_STRUCT   (0x0UL)
#define CY_SROM_DR_IPC_RESP_INTR_STRUCT   (0x2UL)
#define CY_SROM_RESP_NVIC_MUX	(0x3UL)
#define CY_SROM_RESP_ENCODED_IRQ \
	(IRQ_TO_L2(CY_SROM_DR_IPC_RESP_INTR_STRUCT) | CY_SROM_RESP_NVIC_MUX)

static void (*gp_srom_resp_handler)(void) = NULL;

void cat1c_set_srom_response_handler(cy_srom_handler handler)
{
	gp_srom_resp_handler = handler;
}

static void cat1c_srom_responseip_isr(void *arg)
{
	IPC_INTR_STRUCT_Type *sromRespIntrStr =
		Cy_IPC_Drv_GetIntrBaseAddr(CY_SROM_DR_IPC_RESP_INTR_STRUCT);
	uint32_t masked = Cy_IPC_Drv_GetInterruptStatusMasked(sromRespIntrStr);

	if ((uint32_t)(masked & (uint32_t)(1UL << (uint32_t)CY_IPC_CHAN_SYSCALL)) != 0UL) {
		if (gp_srom_resp_handler != NULL) {
			gp_srom_resp_handler();
		}
	}

	Cy_IPC_Drv_ClearInterrupt(sromRespIntrStr, (masked & 0x0000FFFFUL),
				  (masked & 0xFFFF0000UL) >> 16UL);
}

static int cat1c_srom_init()
{
	/*  Set IPC interrupt mask */
	IPC_INTR_STRUCT_Type *sromRespIntrStr = Cy_IPC_Drv_GetIntrBaseAddr(CY_SROM_DR_IPC_RESP_INTR_STRUCT);
	IPC_INTR_STRUCT_Type *sromReqIntrStr = Cy_IPC_Drv_GetIntrBaseAddr(CY_SROM_DR_IPC_REQ_INTR_STRUCT);

	/* Initialize SROM response interrupt*/
	IRQ_CONNECT(CY_SROM_RESP_ENCODED_IRQ, 2, cat1c_srom_responseip_isr, NULL, 0);
	irq_enable(CY_SROM_RESP_ENCODED_IRQ);

	Cy_IPC_Drv_SetInterruptMask(sromRespIntrStr,
				    (uint32_t)(1UL << (uint32_t)CY_IPC_CHAN_SYSCALL), 0UL);

	Cy_IPC_Drv_SetInterruptMask(sromReqIntrStr, 0,
				    (uint32_t)(1UL << (uint32_t)CY_IPC_CHAN_SYSCALL));

	return 0;
}
SYS_INIT(cat1c_srom_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
