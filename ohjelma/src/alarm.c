#include "pins.h"
#include "util.h"
#include "macro.h"
#include "alarm.h"

#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define BEEPENBL 0 // äänimerkki päälle
#define BLNKENBL 1 // välkytys päälle
#define BEEPCONT 2 // jatkuva äänimerkki
#define BLNKCONT 3 // jatkuva välkytys
#define SYNCENBL 4 // synkronoi (ei käytetty)
#define BEEPSYNC 5 // äänimerkki synkronoitu
#define BLNKSYNC 6 // välkytys synkronoitu
#define BEEPSTAT 7 // jatkuvan äänimerkin tila

/* Tilakone. */
static volatile uint8_t state;

/* millis() aikaleimat joilla kestoja ohjataan. */
static volatile uint32_t ts_beep;
static volatile uint32_t ts_blink;

/* TIMER2 skaalaimet ja niiden bitit TCCR2B:ssä. */
static const struct {
	uint16_t fact;
	uint8_t  bits;
} packed prescalers[] PROGMEM = {
	{1024, 0b111},
	{ 256, 0b110},
	{ 128, 0b101},
	{  64, 0b100},
	{  32, 0b011},
	{   8, 0b010},
	{   1, 0b001}
};

/* tämä vähentää sotkua alla */
#define fact(i) \
	pgm_read_word(&prescalers[i].fact)
#define bits(i) \
	pgm_read_byte(&prescalers[i].bits)

/* Valitse sopiva skaalain ja OCR arvo
 * ajastimelle halutun taajuuden perusteella.
 */
uint16_t set_prescaler(volatile uint8_t *reg, uint16_t freq, uint16_t max)
{
	uint32_t now, old;
	uint8_t i;

	i = 0;
	old = max;
	while (i < ARRAY_SIZE(prescalers))
	{
		now = F_CPU/((uint32_t)freq*(uint32_t)fact(i)) - 1;
		if (now > max)
			break;
		old = now;
		i++;
	}
	i -= !!i;

	*reg &= ~7;
	*reg |= bits(i);

	return old;
}

/* Ajastimen keskeytys. */
ISR(TIMER2_COMPA_vect)
{
	uint32_t now;

	now =  (!GET(state, BEEPENBL)) | (!GET(state, BLNKENBL) << 1);
	if (unlikely(now)) {
		switch (now) {
		case 1:
			WRITE(BUZZER, LOW);
			break;
		case 2:
			WRITE(LCD_AN, HIGH);
			break;
		case 3:
			WRITE(BUZZER, LOW);
			WRITE(LCD_AN, HIGH);
			break;
		default:
			unreachable;
		}

		// ajastin pois päältä
		CLR(TIMSK2, OCIE2A);

		return;
	}
	
	now = millis();

	/* äänimerkki */
	if (likely(GET(state, BEEPENBL))) {
		/* jatkuva */
		if (GET(state, BEEPCONT)) {
			/* synkronoi */
			if (unlikely(!GET(state, BEEPSYNC))) {
				/* synkronoi välkytyksen kanssa */
				if (unlikely(GET(state, BLNKCONT)
					&& GET(state, BLNKSYNC)))
					ts_beep = ts_blink;

				/* ei välkytystä -> synkronoitu */
				SET(state, BEEPSYNC);
			}

			/* päälle/pois jaksonajan mukaan */
			if (unlikely((now - ts_beep)
				> (COMMON_PERIOD/2))) {
				TGL(state, BEEPSTAT);

				/* IO pinnin tila on LOW kun summeri ei ole
				 * päällä jottei virtaa kulje turhaan.
				 */
				if (!GET(state, BEEPSTAT))
					WRITE(BUZZER, LOW);

				ts_beep = now;
			}

			/* ääni */
			if (GET(state, BEEPSTAT))
				TOGGLE(BUZZER);

		/* yksittäinen */
		} else {
			/* alustettu */
			if (likely(GET(state, BEEPSYNC))) {
				/* aika kulunut */
				if (now >= ts_beep) {
					WRITE(BUZZER, LOW);
					CLR(state, BEEPENBL);
				}

			/* alusta */
			} else {
				ts_beep += now;
				SET(state, BEEPSYNC);
			}

			/* ääni */
			TOGGLE(BUZZER);
		}
	}

	/* välkytys */
	if (likely(GET(state, BLNKENBL))) {
		/* jatkuva */
		if (GET(state, BLNKCONT)) {
			/* synkronoi */
			if (unlikely(!GET(state, BLNKSYNC))) {
				/* synkronoi äänimerkin kanssa */
				if (unlikely(GET(state, BEEPCONT)))
					ts_blink = ts_beep;

				/* ei äänimerkkiä -> synkronoitu */
				SET(state, BLNKSYNC);
			}

			/* päälle/pois jaksonajan mukaan */
			if (unlikely((now - ts_blink)
				> (COMMON_PERIOD/2))) {
				TOGGLE(LCD_AN);
				ts_blink = now;
			}

		/* yksittäinen */
		} else {
			/* alustettu */
			if (likely(GET(state, BLNKSYNC))) {
				/* aika kulunut */
				if (now >= ts_blink) {
					WRITE(LCD_AN, HIGH);
					CLR(state, BLNKENBL);
				}

			/* alusta */
			} else {
				ts_blink += now;
				WRITE(LCD_AN, LOW);
				SET(state, BLNKSYNC);
			}
		}
	}
}

/* Alustus. */
void alarm_init()
{
	cli();

	/* Alusta IO pinnit. */
	WRITE(LCD_AN, HIGH);
	WRITE(BUZZER, LOW);
	MODE(LCD_AN, OUTPUT);
	MODE(BUZZER, OUTPUT);

	/* Alusta ajastin. */
	TCCR2A = TCCR2B = TIMSK2 = 0;
	SET(TCCR2A, WGM21); // CTC
	OCR2A = set_prescaler(&TCCR2B, 2*BEEP_FREQUENCY, (uint8_t)~0);

	sei();
}

/* Nopea äänimerkki. */
void beep_fast()
{
	cli();

	/* onko äänimerkki päällä */
	if (GET(state, BEEPENBL))
		goto exit;

	/* äänimerkki päälle */
	ts_beep = BEEP_FAST_DURATION;
	SET(state, BEEPENBL);
	CLR(state, SYNCENBL);
	CLR(state, BEEPSYNC);
	SET(TIMSK2, OCIE2A);	
exit:
	sei();
}

/* Hidas äänimerkki. */
void beep_slow()
{
	cli();

	/* onko äänimerkki päällä */
	if (GET(state, BEEPENBL))
		goto exit;
	
	/* äänimerkki päälle */
	ts_beep = BEEP_SLOW_DURATION;
	SET(state, BEEPENBL);
	CLR(state, SYNCENBL);
	CLR(state, BEEPSYNC);
	SET(TIMSK2, OCIE2A);	
exit:
	sei();
}

/* Aloita äänimerkkien toisto. */
void beep_begin()
{
	cli();

	/* onko äänimerkki päällä */
	if (GET(state, BEEPENBL))
		goto exit;
	
	/* äänimerkki päälle */
	ts_beep = 0;
	SET(state, BEEPENBL);
	SET(state, BEEPCONT);
	SET(state, SYNCENBL);
	CLR(state, BEEPSYNC);
	SET(state, BEEPSTAT);
	SET(TIMSK2, OCIE2A);	
exit:
	sei();
}

/* Lopeta äänimerkkien toisto. */
void beep_end()
{
	cli();

	/* onko toistuva äänimerkki päällä */
	if (!GET(state, BEEPCONT))
		goto exit;
	
	/* äänimerkki pois päältä */
	CLR(state, BEEPENBL);
	CLR(state, BEEPCONT);
exit:
	sei();
}

/* Väläytä taustavaloa. */
void blink()
{
	cli();

	/* onko välkytys päällä */
	if (GET(state, BLNKENBL))
		goto exit;
	
	/* väläytys päälle */
	ts_blink = BLINK_DURATION;
	SET(state, BLNKENBL);
	CLR(state, SYNCENBL);
	CLR(state, BLNKSYNC);
	SET(TIMSK2, OCIE2A);	
exit:
	sei();
}

/* Aloita taustavalon välkytys. */
void blink_begin()
{
	cli();

	/* onko välkytys päällä */
	if (GET(state, BLNKENBL))
		goto exit;

	/* välkytys päälle */
	ts_blink = 0;
	SET(state, BLNKENBL);
	SET(state, BLNKCONT);
	SET(state, SYNCENBL);
	CLR(state, BLNKSYNC);
	SET(TIMSK2, OCIE2A);	
exit:
	sei();
}

/* Lopeta taustavalon välkytys. */
void blink_end()
{
	cli();

	/* onko toistuva välkytys päällä */
	if (!GET(state, BLNKCONT))
		goto exit;

	/* välkytys pois päältä */
	CLR(state, BLNKENBL);
	CLR(state, BLNKCONT);
exit:
	sei();
}
