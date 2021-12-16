#ifndef BUTTON_H
#define BUTTON_H

#include "pins.h"

/* Kauan nappien tilaa luetaan ennekuin se lukitaan. */
#define BTN_POLL_TIME 150 // [ms]

/* Kauan nappien tila voi poiketa lukitusta tilasta
 * ennekuin tilan muutos rekisteröidään uutena tapahtumana.
 */
#define BTN_JITTER_TIME 80 // [ms]

/* Kauan nappeja täytyy pitää pohjassa, että
 * HOLD tapahtuma aktivoituu. (ja HLUP UP:in sijasta)
 */
#define BTN_HOLD_TIME 900 // [ms]

/* Kuinka kauan odotetaan UP tai HLUP tapahtuman
 * jälkeen ennenkuin nappien tilaa aletaan lukemaan
 */
#define BTN_INACTIVE_TIME 100 // [ms]

#define BTN_RT_OFF 0
#define BTN_LT_OFF 1

#define RT (1 << BTN_RT_OFF)
#define LT (1 << BTN_LT_OFF)
#define BOTH (RT|LT)

#define DOWN (0 << NUM_KEYS)
#define HOLD (1 << NUM_KEYS)
#define UP   (2 << NUM_KEYS)
#define HLUP (3 << NUM_KEYS)

#define NUM_KEYS 2

#define BTN_POLL_RT (!READ(BTN_RT))
#define BTN_POLL_LT (!READ(BTN_LT))

struct button_state
{
	uint32_t ts_1;
	uint32_t ts_2;
/*  MSB .............................................. LSB
 *    1          2           1          2            2
 * |UNUSED|STATE MACHINE|HOLD FLAG|LOCKED KEYS|POLLED KEYS|
 */
	uint8_t state;
};

uint8_t
button_update(struct button_state *);

#endif // !BUTTON_H
