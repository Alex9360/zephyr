#include<zephyr/kernel.h>
#include"gpio.h"
#define BAUD_RATE 9600
static uint32_t baud()
{
	return (((float)1/BAUD_RATE)*1000000);
}
static void send_data(uint8_t port,uint8_t pin,uint16_t data)
{
      for(uint8_t i=0;i<10;i++)//1<8 changed
      {
              if(data & (1<<i))
	        gpio_set(port,pin);
              else
		 gpio_clear(port,pin);
            k_busy_wait(baud());
      }
}
void uart_init_tx(enum block peripherel,uint8_t port,uint8_t pin)
{
        clock_enable(peripherel,port);
        gpio_mode_set(port,pin,OUT_MODE);
}
void uart_init_rx(enum block peripherel,uint8_t port,uint8_t pin)
{
        clock_enable(peripherel,port);
        gpio_mode_set(port,pin,IN_MODE);
}
static void uart_tx_thread(void*uart_tx_members,void*,void*)
{
	uint8_t port=((struct uart_tx_args*)uart_tx_members)->port;
	uint8_t pin=((struct uart_tx_args*)uart_tx_members)->pin;
	uint8_t* data=((struct uart_tx_args*)uart_tx_members)->data;
	union frame tx;
	while(*data)
	{
           tx.field.start_bit=0;
	   tx.field.data_bits=*data;
 	   tx.field.stop_bit=1;
           send_data(port,pin,tx.data_frame);
	   data++;  
	}
}
static void uart_rx_thread(void * uart_rx_members,void*,void*)
{
	uint8_t port=((struct uart_rx_args*)uart_tx_members)->port;
	uint8_t pin=((struct uart_rx_args*)uart_tx_members)->pin;
	union frame rx;
	while(1)
	{
	rx.data_frame=0;
	while(read_input(port,pin));
	k_busy_wait(1);
	for(uint8_t i=0;i<10;i++)
	{
		rx.data_frame |=(read_input(port,pin)<<i);
                k_busy_wait(baud());
	}
        if(rx.field.start_bit==0 && rx.field.stop_bit==1)
	printf("%c",rx.field.data_bits);
	}
}

K_THREAD_DEFINE(tx_stack,UART_STACK_SIZE);
K_THREAD_DRFINE(rx_stack,UART_STACK_SIZE);
struct k_thread tx_data;
struct k_thread rx_data;
void start_transfer(uint8_t tx_port,uint8_t tx_pin,uint8_t rx_port,uint8_t rx_pin,uint8_t * data)
{
	struct uart_tx_args *uart_tx_members=malloc(sizeof(struct uart_tx_args));
	struct uart_rx_args *uart_rx_members=malloc(sizeof(struct uart_rx_args));
	uart_tx_members->port=tx_port;
	uart_tx_members->pin=tx_pin;
	uart_tx_members->data=data;
	uart_rx_members->port=rx_pin;
	uart_rx_members->pin=rx_pin;
	k_thread_create(&tx_data,tx_stack,UART_STACK_SIZE,uart_tx_thread,uart_tx_members,NULL,NULL,LOW_PRIORITY,0,K_NO_WAIT);
       k_thread_create(&rx_data,rx_stack,UART_STACK_SIZE,uart_rx_thread,uart_rx_members,NULL,NULL,LOW_PRIORITY,0,K_NO_WAIT);
       free(uart_tx_members);
       free(uart_rx_members);
}
