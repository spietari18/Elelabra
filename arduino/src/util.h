#ifndef UTIL_H
#define UTIL_H

#include "macro.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* ääretön liukuluku (isfinite() palauttaa 0) */
#define INF (1.0/0.0)

typedef void (*callback_t)();

/* SPI alustus AD muuntimelle. */
void spi_init();

/* I2C alustus EERAM:ille */
void i2c_init();

/* Lue yksi tavu SPI:llä.
 * (SS pitää asettaa itse)
 */
uint8_t spi_byte(uint8_t);

/* Nollaa mikrokontrolleri watchdog ajastimella. */
void noreturn reset();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !UTIL_H
