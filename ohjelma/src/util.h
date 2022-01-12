#ifndef UTIL_H
#define UTIL_H

#include "macro.h"

#include <stdbool.h>

#define F_CPU 16000000UL // prosessorin kellotaajuus
#include <util/delay.h>
#include <avr/pgmspace.h>

#define DEF_PSTR_PTR(name, str) \
	static const char PSTR_PTR_##name[] PROGMEM = (str)

#define REF_PSTR_PTR(name) \
	&(PSTR_PTR_##name)

/* ääretön liukuluku (isfinite() palauttaa 0) */
#define INF (1.0/0.0)

typedef void (*callback_t)();

/* Alusta ajastin. */
void timer_init();

/* Arduino tyylinen millis(). */
uint32_t millis();

/* Arduino tyylinen micros() */
uint32_t micros();

/* Palauttaa true tiettynä ajanjaksona. */
bool interval(uint16_t, uint32_t *);

/* SPI alustus AD muuntimelle. */
void adc_init();

#define ADC_DF0 (0 << 6) // CH0 -> CH1
#define ADC_DF1 (1 << 6) // CH1 -> CH0
#define ADC_CH0 (2 << 6) // CH0 -> GND
#define ADC_CH1 (3 << 6) // CH1 -> GND

/* Lue näyte AD Muuntimelta. */
uint16_t adc_sample(uint8_t);

/* I2C alustus EERAM:ille */
void eeram_init();

/* Kun osoite on tämä, luetaan nykysestä osoitteesta. */
#define EERAM_ADDR (uint16_t)(~0)

/* Lue dataa EERAM:ista. */
bool eeram_read(uint16_t, uint8_t [], uint8_t);

/* Kirjota data EERAM:iin. */
bool eeram_write(uint16_t, uint8_t [], uint8_t);

#define EERAM_REG_STA 0x00 // STATUS rekisteri
#define EERAM_REG_CMD 0x55 // COMMAND rekisteri

/* STATUS rekisterin bitit */
#define EERAM_EDT (1 << 0)
#define EERAM_ASE (1 << 1)
#define EERAM_BP0 (1 << 2)
#define EERAM_BP1 (1 << 3)
#define EERAM_BP2 (1 << 4)
#define EERAM_MOD (1 << 7)

/* COMMAND rekisteri komennot. */
#define EERAM_STR 0x33
#define EERAM_REC 0xDD

/* Lue EERAM:in STATUS rekisteri. */
bool eeram_reg_read(uint8_t, uint8_t *);

/* Kirjoita EERAM:in STATUS tai COMMAND rekisteriin. */
bool eeram_reg_write(uint8_t, uint8_t, uint8_t);

/* Nollaa mikrokontrolleri watchdog ajastimella. */
void noreturn reset();

#endif // !UTIL_H
