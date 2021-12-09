#include "util.h"
#include "macro.h"

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
