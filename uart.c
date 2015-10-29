#include "uart.h"

#include <avr/interrupt.h>

uint8_t uartbuf[16];
volatile uint8_t uartbuflen = 0;
uint8_t uartbufhead = 0; // start of received

ISR(USART_RX_vect)
{
	if (uartbuflen + 1 < sizeof(uartbuf))
	{
		uartbuf[(uartbufhead + uartbuflen) % sizeof(uartbuf)] = UDR0;
		uartbuflen += 1;
	}
}

void uart_putchar(char c, FILE *stream) {
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}

char uart_getchar(FILE *stream) {
	//*
	while (uartbuflen == 0);
	uint8_t const res = uartbuf[uartbufhead];
	uartbufhead = (uartbufhead + 1) % sizeof(uartbuf);
	uartbuflen -= 1;
	return res;
	/*/
	loop_until_bit_is_set(UCSR0A, RXC0); // Wait until data exists.
	return UDR0;
	//*/
}

uint8_t uart_kbhit(void)
{
	//*
	return uartbuflen;
	/*/
	return UCSR0A & _BV(RXC0);
	//*/
}

FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void setup_uart(void)
{
	#define BAUD 9600 // table 20-6
	UBRR0 = 207;
	UCSR0A |= _BV(U2X0); // 2x speed

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */

	/* Set frame format: 8 data bits */
	UCSR0C |= 1*_BV(UCSZ01) | 1*_BV(UCSZ00);

	/* Enable receiver and transmitter */
	UCSR0B |= _BV(RXEN0) | _BV(TXEN0);

	// enable interrupts
	UCSR0B |= _BV(RXCIE0);

	stdout = &uart_io;
	stdin  = &uart_io;
}


