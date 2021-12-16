#ifndef UTIL_H
#define UTIL_H

#include "macro.h"

#include <stdbool.h>

#define F_CPU 16000000UL // prosessorin kellotaajuus
#include <util/delay.h>

/* ääretön liukuluku (isfinite() palauttaa 0) */
#define INF (1.0/0.0)

typedef void (*callback_t)();

/* Alusta ajastin. */
void timer_init();

/* Arduino tyylinen millis(). */
uint32_t millis();

/* Arduino tyylinen micros() */
uint32_t micros();

/* SPI alustus AD muuntimelle. */
void adc_init();

/* Lue näyte AD Muuntimelta. */
uint16_t adc_sample();

/* I2C alustus EERAM:ille */
void eeram_init();

/* Lue dataa EERAM:ista. */
bool eeram_read(uint16_t addr, uint8_t data[], uint8_t len);

/* Kirjota data EERAM:iin. */
bool eeram_write(uint16_t addr, uint8_t data[], uint8_t len);

/* Lue EERAM:in ohjausrekisteri. */
bool eeram_reg_read(uint8_t *dst);

/* Kirjoita EERAM:in ohjausrekisteriin. */
bool eeram_reg_write(uint8_t src);

/* Nollaa mikrokontrolleri watchdog ajastimella. */
void noreturn reset();

#endif // !UTIL_H
