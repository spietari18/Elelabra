#include "pins.h"
#include "util.h"
#include "macro.h"

/* SPI alustus AD muuntimelle. */
void spi_init()
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

/* I2C alustus EERAM:ille */
void i2c_init()
{
	TWSR = 0;

	/* 500kHz */
	TWBR = 2;
	SET(TWSR, TWPS0);
}

bool eeram_read(uint16_t addr, uint8_t data[], uint8_t len)
{

}

bool eeram_write(uint16_t addr, uint8_t data[], uint8_t len)
{

}

bool eeram_reg_read(uint8_t *dst)
{

}

bool eeram_reg_write(uint8_t src)
{

}

/* Lue yksi tavu SPI:llä.
 * (SS pitää asettaa itse)
 */
uint8_t spi_byte(uint8_t v)
{
	/* kirjoitettava arvo */
	SPDR = v;

	/* odota kunnes valmis */
	while (!GET(SPSR, SPIF));

	/* palauta luettu arvo */
	return SPDR;
}

/* Nollaa mikrokontrolleri watchdog ajastimella. */
void noreturn reset()
{
	WDTCSR = 0;
	SET(WDTCSR, WDE);
	while (1);
}
