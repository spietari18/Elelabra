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
	// NOP
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
void __attribute__((noreturn)) reset()
{
	WDTCSR = 0;
	SET(WDTCSR, WDE);
	while (1);
}
