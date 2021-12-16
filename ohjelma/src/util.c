#include "pins.h"
#include "util.h"
#include "macro.h"

#include <stdbool.h>
#include <avr/interrupt.h>

/* Kellosyklejä per mikrosekunti. */
#define CLK_US (F_CPU/1000000UL) 

/* Kauan ajastimella kestää ylivuotaa laskuri. */
#define OVF_US (64UL*256UL/CLK_US)

#define MS_INC (OVF_US / 1000)
#define US_INC (OVF_US % 1000)

static volatile uint32_t timer0_ovfs;
static volatile uint32_t timer0_ms;
static uint16_t timer0_us;

ISR(TIMER0_OVF_vect) {
	register uint32_t ms = timer0_ms;
	register uint32_t us = timer0_us;

	ms += MS_INC;
	us += US_INC;

	if (us >= 1000) {
		us -= 1000;
		ms += 1;
	}

	timer0_ms = ms;
	timer0_us = us;

	timer0_ovfs++;
}

/* Alusta TIMER0 */
void timer_init()
{
	TCCR0A = TCCR0B = TIMSK0 = 0;

	/* aseta skaalain 64 */
	SET(TCCR0B, CS00);
	SET(TCCR0B, CS01);

	/* ajastin päälle */
	SET(TIMSK0, TOIE0); // OVF

}

/* Arduino tyylinen millis(). */
uint32_t millis()
{
	uint32_t tmp;

	cli();

	tmp = timer0_ms;

	sei();

	return tmp;
}

/* Arduino tyylinen micros(). */
uint32_t micros()
{
	uint32_t tmp;
	uint8_t cnt;

	cli();

	/* monta kertaa ajastin on ylivuotanut */
	tmp = timer0_ovfs;

	/* ajastimen laskurin arvo */
	cnt = TCNT0;

	/* onko ajastin ylivuotanut, mutta keskeytystä
	 * ei ole vielä ehditty suorittamaan
	 */
	if (GET(TIFR0, TOV0) && (cnt < 255))
                tmp++;

	sei();

	/* (tmp << 8) | cnt kertoo kuinka monta kertaa
	 * ajastin on korottanut laskuria.
	 */
	return ((tmp << 8) | cnt)*(64*CLK_US);
}

/* Lue ja lähetä yksi tavu SPI:llä. */
static uint8_t spi_byte(uint8_t v)
{
	/* kirjoitettava arvo */
	SPDR = v;

	/* odota kunnes valmis */
	while (!GET(SPSR, SPIF));

	/* palauta luettu arvo */
	return SPDR;
}

/* SPI alustus AD muuntimelle. */
void adc_init()
{
	SPCR = 0;
	SET(SPCR, SPE);
	SET(SPCR, MSTR);
	SET(SPCR, CPOL);
	SET(SPCR, SPR0);
	WRITE(SPI_SS, HIGH);
	MODE(SPI_SS, OUTPUT);
	MODE(SPI_CLK, OUTPUT);
	MODE(SPI_MOSI, OUTPUT);
	PLUP(SPI_MISO, 0);
}

/* Lue näyte AD muuntimelta. */
uint16_t adc_sample()
{
	uint16_t tmp = 0;

	/* lue näyte AD-muuntimelta */
	WRITE(SPI_SS, LOW);
	(void)spi_byte(0x01);
	tmp |= (uint16_t)spi_byte(0x20) << 8;
	tmp |= spi_byte(0x00);
	WRITE(SPI_SS, HIGH);

	/* näyte on 12 bittiä */
	return tmp & ((1 << 12) - 1);
}

/* I2C alustus EERAM:ille */
void eeram_init()
{
	TWSR = 0;

	/* 500kHz */
	TWBR = 2;
	SET(TWSR, TWPS0);
}

bool eeram_read(uint16_t addr, uint8_t data[], uint8_t len)
{
	return false;
}

bool eeram_write(uint16_t addr, uint8_t data[], uint8_t len)
{
	return false;
}

bool eeram_reg_read(uint8_t *dst)
{
	return false;
}

bool eeram_reg_write(uint8_t src)
{
	return false;
}

/* Nollaa mikrokontrolleri watchdog ajastimella. */
void noreturn reset()
{
	WDTCSR = 0;
	SET(WDTCSR, WDE);
	while (1);
}
