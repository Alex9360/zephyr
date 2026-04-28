/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	uint32_t reset_vec = *(volatile uint32_t *)
                        (DT_REG_ADDR(DT_NODELABEL(flash_m7_1)) + 4);
	printf("reset_vec = %x\n", reset_vec);


	return 0;
}
