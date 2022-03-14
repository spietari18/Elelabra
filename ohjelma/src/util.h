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

#define CALLBACK(X) \
	do { \
		callback_t tmp = (X); \
		if (tmp) \
			tmp(); \
	} while (0)

/* Alusta ajastin. */
void timer_init();

/* Arduino tyylinen millis(). */
uint32_t millis();

/* Arduino tyylinen micros() */
uint32_t micros();

extern uint32_t interval_ts;

/* Palauttaa true tiettynä ajanjaksona. */
bool interval(uint32_t, uint32_t *);

#define INTERVAL(ival) \
	if (interval((ival), &interval_ts))

struct interval_task
{
	callback_t func;
	uint32_t   ival;
	uint32_t   last;
} packed;

#define DEF_TASK(func, ival) \
	{ (callback_t)(func), (ival), 0 }

#define INTERVAL_TASKS(tasks) \
	interval_tasks((tasks), ARRAY_SIZE(tasks))

/* Kutsu interval_task:eja niille määritetyin väliajoin. */
void interval_tasks(struct interval_task *, uint8_t);

/* SPI alustus AD muuntimelle. */
void adc_init();

#define ADC_DF0 0 // CH0 -> CH1
#define ADC_DF1 1 // CH1 -> CH0
#define ADC_CH0 2 // CH0 -> GND
#define ADC_CH1 3 // CH1 -> GND

/* AD muunnin on 12-bittinen. */
#define S_ABS_MIN 0
#define S_ABS_MAX ((1 << 12) - 1)

/* Lue näyte AD muuntimelta. */
uint16_t adc_sample(uint8_t);

/* I2C alustus EERAM:ille */
void eeram_init();

/* Kun osoite on tämä, luetaan nykysestä osoitteesta. */
#define EERAM_ADDR (uint16_t)(~0)

/* Osoiterajat EERAM:ille */
#define EERAM_MIN_ADDR 0
#define EERAM_MAX_ADDR ((1 << 11) - 1)

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

/* Liian alhainen jännite? */
bool undervolt();

#endif // !UTIL_H
