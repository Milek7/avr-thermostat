#define F_CPU 4800000

#include <inttypes.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define T_MIN 2000
#define T_MAX 4500
#define T_H 100

#define DS_LOW PORTB &= ~(1 << PB3); DDRB |= (1 << PB3)
#define DS_FREE PORTB &= ~(1 << PB3); DDRB &= ~(1 << PB3)
#define DS_PULL PORTB |= (1 << PB3); DDRB |= (1 << PB3)
#define DS_STATE ((PINB & (1 << PB3)) ? 1 : 0)

#define DS_WRITE0 DS_LOW; _delay_us(60); DS_FREE; _delay_us(3)
#define DS_WRITE1 DS_LOW; _delay_us(5); DS_FREE; _delay_us(60)
#define DS_READ DS_LOW; _delay_us(5); DS_FREE; _delay_us(3)

void DSWrite(uint8_t data)
{
	for (int8_t i = 0; i < 8; i++)
		if (data & (1 << i))
			{ DS_WRITE1; }
		else
			{ DS_WRITE0; }
}

uint8_t DSRead()
{
	uint8_t data = 0;
	for (int8_t i = 0; i < 8; i++)
	{
		DS_READ;
		data |= DS_STATE << i;
		_delay_us(60);
	}
	return data;
}

bool DSReset()
{
	DS_LOW;
	_delay_us(1000);
	DS_FREE;
	_delay_us(70);
	DS_READ;
	if (DS_STATE)
		return false;
	_delay_us(480);
	return true;
}

uint8_t CRCStep(uint8_t shift, uint8_t input)
{
	uint8_t tmp = shift & 1;
	shift >>= 1;
	shift |= (tmp ^ input) << 7;
	shift ^= (shift >> 4) & (1 << 3);
	shift ^= (shift >> 5) & (1 << 2);
	return shift;
}

int16_t DSReadT()
{
	if (!DSReset())
		return 20001;
	DSWrite(0xCC);
	DSWrite(0x44);
	DS_PULL;
	_delay_ms(750);
	DS_FREE;
	
	if (!DSReset())
		return 20002;
	DSWrite(0xCC);
	DSWrite(0xBE);
	
	uint8_t crc = 0;
	
	uint8_t lsb = DSRead();
	for (uint8_t i = 0; i < 8; i++)
		crc = CRCStep(crc, (lsb & (1 << i)) ? 1 : 0);
		
	uint8_t msb = DSRead();
	for (uint8_t i = 0; i < 8; i++)
		crc = CRCStep(crc, (msb & (1 << i)) ? 1 : 0);
		
	for (uint8_t i = 0; i < 6; i++)
	{
		uint8_t data = DSRead();
		for (uint8_t i = 0; i < 8; i++)
			crc = CRCStep(crc, (data & (1 << i)) ? 1 : 0);
	}
	
	if (DSRead() != crc)
		return 20003;
		
	uint16_t data = (msb << 8) | lsb;
	
	if (msb >> 7)
		data = ~data + 1;

	return ((msb >> 7) ? -1 : 1) * (((data >> 4) & 0x7F) * 100 + ((data & 0xF) * 625) / 100);
}

void main()
{
	wdt_enable(WDTO_4S);
	DDRB = (1 << PB0);
	PORTB = (1 << PB1) | (1 << PB4);
	ADMUX = (1 << MUX0) | (1 << ADLAR);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);
	DIDR0 = (1 << ADC1D);
	set_sleep_mode(SLEEP_MODE_ADC);
	sei();
	
    while (1)
    {
		wdt_reset();
		
		int16_t current = DSReadT();
		
		sleep_mode();
		int16_t target = T_MIN + ADCH * ((int16_t)(T_MAX - T_MIN) * 10 / 255) / 10;
		
		if (current < target - T_H)
			PORTB |= (1 << PB0);
		if (current > target + T_H)
			PORTB &= ~(1 << PB0);
    }
	
}

EMPTY_INTERRUPT(ADC_vect);
