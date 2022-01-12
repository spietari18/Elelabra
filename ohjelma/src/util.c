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

/* Palauttaa true tiettynä ajanjaksona. */
bool interval(uint16_t ms_int, uint32_t *ts)
{
	uint32_t tmp;

	tmp = millis();
	if ((tmp - *ts) > ms_int) {
		*ts = tmp;
		return true;
	}

	return false;
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

#define I2C_MR_STTX 0x08
#define I2C_MR_RETX 0x10
#define I2C_MR_ALST 0x38

#define I2C_MT_WACK 0x18
#define I2C_MT_WNCK 0x20
#define I2C_MT_DACK 0x28
#define I2C_MT_DNCK 0x30

#define I2C_MR_RACK 0x40
#define I2C_MR_RNCK 0x48
#define I2C_MR_DACK 0x50
#define I2C_MR_DNCK 0x58

#define I2C_STAT (TWSR & 0xF8)

/* Varaa I2C väylä. */
static bool i2c_start(uint8_t packet)
{
	/* lähetä START */
	TWCR = 0;
	SET(TWCR, TWINT);
	SET(TWCR, TWSTA);
	SET(TWCR, TWEN);

	while (!GET(TWCR, TWINT)); // odota
	
	/* tarkista, että lähetys onnistui */
	if (I2C_STAT != I2C_MR_STTX)
		return false;

	/* lähetä osoitetavu */
	TWDR = packet;
	TWCR = 0;
	SET(TWCR, TWINT);
	SET(TWCR, TWEN);

	while (!GET(TWCR, TWINT)); // odota

	/* tarkista, että lähetys onnistui */
	if ((I2C_STAT != I2C_MT_WACK) && (I2C_STAT != I2C_MR_RACK))
		return false;

	return true;
}

/* Vapauta I2C väylä. */
static void i2c_stop()
{
	/* lähetä STOP */
	TWCR = 0;
	SET(TWCR, TWINT);
	SET(TWCR, TWSTO);
	SET(TWCR, TWEN);
}

/* Lähetä tavu I2C väylään. */
static bool i2c_tx_byte(uint8_t src, bool ack)
{
	/* lähetä tavu */
	TWDR = src;
	TWCR = 0;
	SET(TWCR, TWINT);
	SET(TWCR, TWEN);
	VAL(TWCR, TWEA, ack);

	while (!GET(TWCR, TWINT)); // odota

	/* tarkista, että lähetys onnistui */
	if (I2C_STAT != I2C_MT_DACK)
		return false;
	
	return true;
}

/* Vastaanota tavu I2C väylästä. */
static bool i2c_rx_byte(uint8_t *dst, bool ack)
{
	/* vastaanota tavu */
	TWCR = 0;
	SET(TWCR, TWINT);
	SET(TWCR, TWEN);
	VAL(TWCR, TWEA, ack);

	while (!GET(TWCR, TWINT)); // odota

	/* tarkista, että vastaanotto onnistui */
	if (I2C_STAT != I2C_MR_DACK)
		return false;

	*dst = TWDR;
	return true;
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
uint16_t adc_sample(uint8_t ch)
{
	uint16_t tmp = 0;

	/* lue näyte AD-muuntimelta */
	WRITE(SPI_SS, LOW);
	(void)spi_byte(0); // aloita
	tmp |= (uint16_t)spi_byte(ch | 32) << 8; // valitse kanava
	tmp |= spi_byte(0); // lue loput näytteen bitit
	WRITE(SPI_SS, HIGH);

	/* näyte on 12 bittiä */
	return tmp & ((1 << 12) - 1);
}

/* I2C alustus EERAM:ille
 * (main() alustaa SDA ja SCL pinnit PULLUP tilaan)
 */
void eeram_init()
{
	TWSR = 0;
	TWBR = 2;
	SET(TWSR, TWPS0); // 500 kHz
}

#define EERAM_SRAM 0x50
#define EERAM_CREG 0x30

#define EERAM_DEVA(A) (((A) >> 0xB) & 0xC)

#define EERAM_R 1
#define EERAM_W 0

#define EERAM_HGH(A) (((A) >> 8) & 0x07)
#define EERAM_LOW(A) ((A) & 0xFF)

static inline bool eeram_tx_addr(uint16_t addr)
{
	/* aloita I2C */
	if (!i2c_start(EERAM_SRAM | EERAM_DEVA(addr) | EERAM_W))
		return false;

	/* lähetä osoite */
	return !(i2c_tx_byte(EERAM_HGH(addr), true)
		&& i2c_tx_byte(EERAM_LOW(addr), true));
}

/* Lue tavu EERAM:ilta. */
bool eeram_read(uint16_t addr, uint8_t data[], uint8_t len)
{
	/* jos ei käytetä nykyistä osoitetta, lähetä osoite */
	if ((addr != EERAM_ADDR) && !eeram_tx_addr(addr))
		return false;
	
	/* aloita I2C */
	if (!i2c_start(EERAM_SRAM | EERAM_DEVA(addr) | EERAM_R))
		return false;

	len--;

	/* vastaanota kaikki tavut */
	for (size_t i = 0; i < len; i++)
		if (!i2c_rx_byte(&data[i], true))
			return false;
	
	/* viimeinen tavu ei saa lähettää ACK bittiä */
	if (!i2c_rx_byte(&data[len], false))
		return false;

	/* lopeta I2C */
	i2c_stop();

	return true;
}

/* Kirjoita tavu EERAM:ille. */
bool eeram_write(uint16_t addr, uint8_t data[], uint8_t len)
{
	/* lähetä osoite */
	if (!eeram_tx_addr(addr))
		return false;

	/* lähetä kaikki tavut */
	for (size_t i = 0; i < len; i++)
		if (!i2c_tx_byte(data[i], true))
			return false;

	/* lopeta I2C */
	i2c_stop();

	return true;
}

bool eeram_reg_read(uint8_t addr, uint8_t *dst)
{
	/* aloita I2C */
	if (!i2c_start(EERAM_CREG | (addr & 3) | EERAM_R))
		return false;

	/* read status register */
	if (!i2c_rx_byte(dst, false))
		return false;

	/* lopeta I2C */
	i2c_stop();

	return true;
}

#define EERAM_TWC  1 // [ms]
#define EERAM_TST 25 // [ms]
#define EERAM_TRC  5 // [ms]

bool eeram_reg_write(uint8_t addr, uint8_t reg, uint8_t src)
{
	/* aloita I2C */
	if (!i2c_start(EERAM_CREG | (addr & 3) | EERAM_W))
		return false;
	
	/* lähetä rekisterin osoite ja tavu */
	if (!(i2c_tx_byte(reg, true) && i2c_tx_byte(src, true)))
		return false;

	/* lopeta I2C */
	i2c_stop();

	/* odota, että operaatio on valmis */
	switch (src) {
	case EERAM_STR:
		_delay_ms(EERAM_TST);
		break;
	case EERAM_REC:
		_delay_ms(EERAM_TRC);
		break;
	default:
		_delay_ms(EERAM_TWC);
		break;
	}

	return true;
}

/* Nollaa mikrokontrolleri watchdog ajastimella. */
void noreturn reset()
{
	WDTCSR = 0;
	SET(WDTCSR, WDE);
	while (1);
}
