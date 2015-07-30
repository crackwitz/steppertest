/*
 * _150722_pololu_stepper_antreiben.c
 *
 * Created: 22.07.2015 17:31:32
 *  Author: Chris
 */ 

#include "ramp.h"
#include <stdio.h>
#include <stdbool.h>

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

struct motion {
	uint32_t stepsleft;
	uint32_t rampindex;
	uint32_t rampmax;
	uint32_t rampfactor;
	int32_t position;
} motion = { .position = 0, .stepsleft = 0 };

void uart_putchar(char c, FILE *stream) {
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}
char uart_getchar(FILE *stream) {
	loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
	return UDR0;
}

FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void setup_uart(void)
{
	#define BAUD 115200
	UBRR0 = 8;

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
	
	/* Set frame format: 8 data bits */
	UCSR0C |= 1*_BV(UCSZ01) | 1*_BV(UCSZ00);

	/* Enable receiver and transmitter */
	UCSR0B |= _BV(RXEN0) | _BV(TXEN0);
	
	stdout = &uart_io;
	stdin  = &uart_io;
}

void delay_us(uint16_t n)
{
	while(n--)
		_delay_us(1);
}

inline void set_microstepping(uint8_t usteps)
{
	uint8_t pattern = 0;
	switch(usteps)
	{
		case  1: pattern = 0b000; break;
		case  2: pattern = 0b001; break;
		case  4: pattern = 0b010; break;
		case  8: pattern = 0b011; break;
		case 16: pattern = 0b111; break;
		default:
			fprintf(stderr, "invalid MS3:1");
			return;
	}

	cli();
	uint8_t port = PORTB;
	port &= ~(_BV(PORTB4) | _BV(PORTB3) | _BV(PORTB2));
	if (pattern & 0b001) port |= _BV(PORTB4);
	if (pattern & 0b010) port |= _BV(PORTB3);
	if (pattern & 0b100) port |= _BV(PORTB2);
	PORTB = port;
	sei();
}

inline void set_dir(signed char direction)
{
	if (direction < 0)
		PORTB |= _BV(PORTB0);
	else
		PORTB &= ~_BV(PORTB0);
}

static inline void pulse(void)
{
	PORTB |= _BV(PORTB1);
	_delay_us(1);
	PORTB &= ~_BV(PORTB1);
}

static void setup_timer(void)
{
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	
	TCCR1B |= 1*_BV(WGM13) | 1*_BV(WGM12);
	TCCR1A |= 1*_BV(WGM11) | 0*_BV(WGM10);
	// ICR1 = interval
	//TCCR1A |= 1*_BV(COM1A1) | 0*_BV(COM1A0);

	// enable overflow ISR	
	TIMSK1 |= _BV(TOIE1);

	// prescaler 1
	OCR1A = (2e-6 * F_CPU / 1);
}

static inline void start_timer(void)
{
	TCCR1B |= (0b001 << CS10);
}

static inline void stop_timer(void)
{
	TCCR1B &= ~(0b111 << CS10);
}

ISR(TIMER1_OVF_vect)
{
	// set ICR1
}

static void setup_stepper(void)
{
	PORTB &= ~0b00011111;
	DDRB |= _BV(DDB4); // D12 MS1
	DDRB |= _BV(DDB3); // D11 MS2
	DDRB |= _BV(DDB2); // D10 MS3
	DDRB |= _BV(DDB1); // D09 step
	DDRB |= _BV(DDB0); // D08 dir
	
	set_microstepping(1);
	
	//setup_timer();
}

static void setup(void)
{
	DDRB |= _BV(DDB5); // D13 LED

	setup_uart();

	setup_stepper();

	sei();
}

int main(void)
{
	setup();

	puts("Hello!");
	
	signed char const dir = -1;
	set_dir(dir);
	
	uint16_t rampfactor = 6200;
	uint16_t rampfactor_ = rampfactor;
	uint32_t rampindex = 0;
	uint32_t rampindex_ = 0;
	uint32_t const rampmax = 2400;
	
	uint16_t kshift = 0;
	uint32_t counter = 0;
	bool foo = false;
    while(1)
    {
		pulse();
		
		counter += 1;
		if (!(counter & 0xff))
			PORTB ^= _BV(PORTB5);
		
		//_delay_us(5000);
		if (rampindex_ >= ramplen)
		{
			//printf("kshift %d, rampindex_ %u, rampfactor_ %u\n", kshift, rampindex_, rampfactor_);
			rampindex_ >>= 2;
			rampfactor_ >>= 1;
			kshift += 1;
			putchar('%');
			//printf("kshift %d, rampindex_ %u, rampfactor_ %u\n", kshift, rampindex_, rampfactor_);
		}
		uint32_t const delay = (rampfactor_ * (uint32_t)ramp[rampindex_]) >> 16;
		//printf("delay %lu\n", delay);
		if (delay > 0) delay_us(delay);
		
		if (rampindex < rampmax)
		{
			rampindex += 1;
			uint16_t mod = rampindex & ((1UL << (2*kshift))-1);
			//printf("1<<2k = %u\n", mod);
			if (mod == 0)
			{
				rampindex_ += 1;
				putchar('+');
			}
		}
		else if (!foo)
		{
			//puts("ramp done");
			foo = true;
		}
			
    }
}