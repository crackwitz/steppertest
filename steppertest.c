/*
* _150722_pololu_stepper_antreiben.c
*
* Created: 22.07.2015 17:31:32
*  Author: Chris
*/

#include "ramp.h"
#include <stdio.h>
#include <stdbool.h>

// D10: B2: M0
// D9:  B1: M1
// D8:  B0: M2
// D7:  D7: STEP 2
// D6:  D6: DIR 2
// D5:  D5: STEP 1
// D4:  D4: DIR 1
// // TODO: EN pins to disable drivers while outputs aren't configured yet


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
	} motion1 = { .position = 0, .stepsleft = 0 }, motion2 = { .position = 0, .stepsleft = 0 };

void uart_putchar(char c, FILE *stream) {
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}
char uart_getchar(FILE *stream) {
	loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
	return UDR0;
}

bool uart_kbhit(void)
{
	return UCSR0A & _BV(RXC0);
}

FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void setup_uart(void)
{
	#define BAUD 115200 // table 20-6
	UBRR0 = 16;
	UCSR0A |= _BV(U2X0); // 2x speed

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
		//case 16: pattern = 0b100; break;
		case 16: pattern = 0b111; break;
		case 32: pattern = 0b101; break;
		default:
		fprintf(stderr, "invalid MS3:1");
		return;
	}

	cli();
	uint8_t port = PORTB;
	port &= ~(_BV(PORTB2) | _BV(PORTB1) | _BV(PORTB0));
	if (pattern & 0b001) port |= _BV(PORTB2);
	if (pattern & 0b010) port |= _BV(PORTB1);
	if (pattern & 0b100) port |= _BV(PORTB0);
	PORTB = port;
	sei();
}

inline void set_dir(signed char dir1, signed char dir2)
{
	if (dir1 < 0)
		PORTD |= _BV(PORTD4);
	else if (dir1 > 0)
		PORTD &= ~_BV(PORTD4);

	if (dir2 < 0)
		PORTD |= _BV(PORTD6);
	else if (dir2 > 0)
		PORTD &= ~_BV(PORTD6);
}

static void setup_stepper(void)
{
	uint8_t reg;

	reg = DDRB;
	reg |= _BV(DDB2);
	reg |= _BV(DDB1);
	reg |= _BV(DDB0);
	DDRB = reg;

	reg = DDRD;
	reg |= _BV(DDD7);
	reg |= _BV(DDD6);
	reg |= _BV(DDD5);
	reg |= _BV(DDD4);
	DDRD = reg;

	set_microstepping(1);
}

static inline void pulse1(void)
{
	PORTD |= _BV(PORTD5);
	_delay_us(2.0);
	PORTD &= ~_BV(PORTD5);
}

static inline void pulse2(void)
{
	PORTD |= _BV(PORTD7);
	_delay_us(2.0);
	PORTD &= ~_BV(PORTD7);
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
	ICR1 = 0xff;
	TCCR1B |= (0b001 << CS10);
// 	{0b101, 1024},
// 	{0b100, 256},
// 	{0b011, 64},
// 	{0b010, 8},
// 	{0b001, 1},
// 	{0b000, 0},

}

static inline void stop_timer(void)
{
	TCCR1B &= ~(0b111 << CS10);
}

uint32_t timercounter = 0;
ISR(TIMER1_OVF_vect)
{
	//PORTB ^= _BV(PORTB5);
	timercounter += 1;
	// set ICR1
}

static void setup(void)
{
	DDRB |= _BV(DDB5); // D13 LED

	setup_uart();

	setup_stepper();
	setup_timer();
	start_timer();

	sei();
}

uint16_t getshort(void)
{
	union {
		struct { uint8_t low, high; };
		uint16_t word;
	} res;
	res.low = getchar();
	res.high = getchar();
	return res.word;
}

int main(void)
{
	setup();

	puts("Hello!");

	uint8_t ms = 16;
	set_microstepping(ms);

	if (1)
	{
		float const scale = 0.0001;
		float x = 0, y = 0;
		int16_t vx = 0, vy = 0;
		int16_t x0 = 0, y0 = 0;

		while (true)
		{
			if (uart_kbhit())
			{
				vx = getshort();
				vy = getshort();
				//printf("velocity %+3d %+3d\n", vx, vy);
			}

			PORTB ^= _BV(PORTB5);

			x += vx * scale;
			y += vy * scale;

			int16_t const x1 = x;
			int16_t const y1 = y;

			int8_t const dx = (x1 > x0) - (x1 < x0);
			int8_t const dy = (y1 > y0) - (y1 < y0);

			set_dir(dx, dy);

			if (dx != 0)
			{
				pulse1();
				x0 += dx;
			}

			if (dy != 0)
			{
				pulse2();
				y0 += dy;
			}//*/
		}
	}

	if (1)
	{
		int16_t range = 50;
		int16_t x0 = 0, y0 = 0;
		float timestep = (1.0 * 0x100) / 16e6;
		float freq = 0.1;

		while (true)
		{
			if (uart_kbhit())
			{
				int8_t const vx = getchar();
				int8_t const vy = getchar();
				printf("velocity %+3d %+3d\n", vx, vy);
			}

			PORTB ^= _BV(PORTB5);

			//printf("%f\n", timercounter * timestep);

			//range = 100 - timercounter * timestep;
			//if (range < 0) range = 0;
			float const theta = 2*M_PI * freq * timercounter * timestep;
			float const r = range * sin(theta * 1);
			float const angle = theta;

			//int16_t const x1 = cos(angle) * r;
			int16_t const x1 = sin(angle) * range * 5;
			int16_t const y1 = sin(angle * 2) * range;

			//int16_t x1 = sin(theta) * 1 * range;
			//int16_t y1 = sin(theta * 1) * range;


			int8_t const dx = (x1 > x0) - (x1 < x0);
			int8_t const dy = (y1 > y0) - (y1 < y0);

			//printf("pos %+d dir %+d\n", pos, dir);

			/*
			if (dir > 0 && pos >= range)
			{
			dir = -1;
			}
			else if (dir < 0 && pos <= 0)
			{
			dir = +1;
			}
			//*/

			set_dir(dx, dy);

			if (dx != 0)
			{
				pulse1();
				x0 += dx;
			}

			if (dy != 0)
			{
				pulse2();
				y0 += dy;
			}//*/
		}
	}

	uint32_t rampfactor = 30000;
	uint32_t rampfactor_ = rampfactor;
	uint32_t rampindex = 0;
	uint32_t rampindex_ = 0;
	uint32_t const rampmax = 300; // 4000;

	uint16_t kshift = 0;
	uint32_t counter = 0;
	bool foo = false;
	while(1)
	{
		pulse1();

		counter += 1;
		if (!(counter & 0xff))
		PORTB ^= _BV(PORTB5);

		if (rampindex_ >= ramplen)
		{
			//printf("kshift %d, rampindex_ %u, rampfactor_ %u\n", kshift, rampindex_, rampfactor_);
			rampindex_ >>= 2;
			rampfactor_ >>= 1;
			kshift += 1;
			putchar('%');
			//printf("kshift %d, rampindex_ %u, rampfactor_ %u\n", kshift, rampindex_, rampfactor_);
		}
		uint32_t const delay = (((uint32_t)rampfactor_ * (uint32_t)ramp[rampindex_]) >> 16);
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
				//putchar('+');
			}
		}
		else if (!foo)
		{
			//puts("ramp done");
			foo = true;
		}

	}
}