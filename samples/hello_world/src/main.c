/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#if 0
#include <stdio.h>

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	return 0;
}
#endif
#if 1
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

static const struct device *uart_dev;

/* Small buffer - DMA interrupt fires only when buffer is completely full */
#define RX_BUF_SIZE 4
static uint8_t rx_buf_a[RX_BUF_SIZE];
static uint8_t rx_buf_b[RX_BUF_SIZE];
static uint8_t tx_buf[] = "DMA TX done test\r\n";
static volatile bool tx_done;

static void uart_async_cb(const struct device *dev,
			   struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		printk("[DMA] TX complete - %d bytes via DMA\n",
		       evt->data.tx.len);
		tx_done = true;
		break;
	case UART_TX_ABORTED:
		printk("[DMA] TX aborted\n");
		break;
	case UART_RX_RDY:
		printk("[DMA] RX complete - %d bytes via DMA: ", evt->data.rx.len);
		for (int i = 0; i < evt->data.rx.len; i++) {
			printk("%c", evt->data.rx.buf[evt->data.rx.offset + i]);
		}
		printk("\n");
		break;
	case UART_RX_BUF_REQUEST:
		printk("[DMA] RX buf request - providing next buffer\n");
		uart_rx_buf_rsp(dev, rx_buf_b, RX_BUF_SIZE);
		break;
	case UART_RX_BUF_RELEASED:
		printk("[DMA] RX buf released\n");
		break;
	case UART_RX_DISABLED:
		printk("[DMA] RX disabled - re-enabling\n");
		uart_rx_enable(dev, rx_buf_a, RX_BUF_SIZE, SYS_FOREVER_US);
		break;
	case UART_RX_STOPPED:
		printk("[DMA] RX stopped reason=%d\n", evt->data.rx_stop.reason);
		break;
	default:
		break;
	}
}

int main(void)
{
	int ret;

	uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	printk("=== UART Async DMA Test (no timeout) ===\n");
	printk("RX buffer = %d bytes\n", RX_BUF_SIZE);
	printk("RX callback fires ONLY when %d bytes received via DMA\n\n",
	       RX_BUF_SIZE);

	uart_callback_set(uart_dev, uart_async_cb, NULL);

	/* Test 1: TX via DMA */
	printk("--- Test 1: TX via DMA ---\n");
	tx_done = false;
	ret = uart_tx(uart_dev, tx_buf, sizeof(tx_buf) - 1, SYS_FOREVER_US);
	printk("uart_tx ret=%d\n", ret);

	/* Wait for TX DMA completion interrupt */
	while (!tx_done) {
		k_sleep(K_MSEC(10));
	}
	printk("TX DMA interrupt confirmed\n\n");

	/* Test 2: RX via DMA */
	printk("--- Test 2: RX via DMA ---\n");
	printk("Type exactly %d characters to trigger DMA completion...\n",
	       RX_BUF_SIZE);
	ret = uart_rx_enable(uart_dev, rx_buf_a, RX_BUF_SIZE, SYS_FOREVER_US);
	printk("uart_rx_enable ret=%d\n", ret);

	while (1) {
		k_sleep(K_FOREVER);
	}
	return 0;
}
#endif
