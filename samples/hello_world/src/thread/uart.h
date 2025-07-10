#define UART_H
void uart_init_tx(enum block peripherls,uint8_t port,uint8_t pin);
void uart_init_rx(enum block peripherls,uint8_t port,uint8_t pin);
union frame{
	uint16_t data_frame;
	struct {
            uint16_t  start_bit : 1;
	    uint16_t data_bits : 8;
	    uint16_t stop_bit :1;
	}field;
};
struct uart_tx_args
{
	uint8_t port;
	uint8_t pin;
	uint8_t * data;
};
struct uart_rx_args
{
	uint8_t port;
	uint8_t pin;
};
#endif
