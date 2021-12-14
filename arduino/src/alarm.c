#include "pins.h"
#include "macro.h"
#include "alarm.h"

/* Ajastimen maksimiarvo select_prescaler():ille. */
#if (BUZZER_TIMER == 1)
#define TIMER_MAX (uint16_t)(~0)
#else
#define TIMER_MAX (uint8_t)(~0)
#endif

/* Tämä sotku koska C esikääntäjä on paska. */
#define CAT(X, Y) X##Y
#define PASTE2(X, Y) CAT(X, Y)
#define PASTE3(X, Y, Z) PASTE2(X, PASTE2(Y, Z))

/* Ajastimen rekisterit. */
#define TCCRA PASTE3(TCCR, BUZZER_TIMER, A)
#define TCCRB PASTE3(TCCR, BUZZER_TIMER, B)
#define TIMSK PASTE2(TIMSK, BUZZER_TIMER)
#define TIOCR PASTE3(OCR, BUZZER_TIMER, A)

/* Ajastimen OCR ENABLE bitti TIMSK rekisterissä. */
#define ENABLE PASTE3(OCIE, BUZZER_TIMER, A)

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

/* Ajastimien skaalaimet ja niiden bitit TCCRnB:ssä. */
static const struct {
	uint16_t fact;
	uint8_t  bits;
} packed prescalers[] PROGMEM = {
	{1024, 0b101},
	{ 256, 0b100},
	{  64, 0b011},
	{   8, 0b010},
	{   1, 0b001}
};

/* Kellotaajuus (16MHz) */
#define CLOCK_FREQ 16000000UL

/* Valitse sopiva skaalain ja OCR arvo
 * ajastimelle halutun taajuuden perusteella.
 */
uint16_t set_prescaler(volatile uint8_t *reg, uint32_t freq, uint16_t max)
{
	uint32_t res, old;
	uint16_t div;
	uint8_t i;

	i = 0;
	old = max;
	while (i < ARRAY_SIZE(prescalers))
	{
		div = freq*((typeof(&prescalers[0]))
			pgm_read_ptr(&prescalers[i]))->fact;
		res = (CLOCK_FREQ + (div >> 2))/div;
		if (res > max)
			break;
		old = res;
		i++;
	}
	i--;
	*reg &= ~7;
	*reg |= ((typeof(&prescalers[0]))
		pgm_read_ptr(&prescalers[i]))->bits;
	max -= (uint16_t)old;

	return max;
}

/* Ajastimen keskeytys. */
ISR(PASTE3(TIMER, BUZZER_TIMER, _COMPA_vect))
{
	uint32_t now;

	/* jos mikään ei ole päällä, pysäytä ajastin */
	if (unlikely(!(GET(state, BEEPENBL)
		&& GET(state, BLNKENBL)))) {
		CLR(TIMSK, ENABLE);
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
				ts_beep = now;
			}

		/* yksittäinen */
		} else {
			/* alustettu */
			if (likely(GET(state, BLNKSYNC))) {
				/* aika kulunut */
				if (now >= ts_blink) {
					WRITE(LCD_AN, HIGH);
					CLR(state, BEEPENBL);
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
	/* Alusta IO pinnit. */
	WRITE(LCD_AN, HIGH);
	WRITE(BUZZER, LOW);
	MODE(LCD_AN, OUTPUT);
	MODE(BUZZER, OUTPUT);

	/* Alusta ajastin. */
	TCCRA = TCCRB = TIMSK = 0;
	SET(TCCRA, PASTE3(WGM, BUZZER_TIMER, 1)); // CTC
	TIOCR = set_prescaler(&TCCRB, 2*BEEP_FREQUENCY, TIMER_MAX);
}

/* Nopea äänimerkki. */
void beep_fast()
{
	cli();

	/* onko äänimerkki päällä */
	if (GET(state, BEEPENBL))
		return;

	/* äänimerkki päälle */
	ts_beep = BEEP_FAST_DURATION;
	SET(state, BEEPENBL);
	CLR(state, SYNCENBL);
	CLR(state, BEEPSYNC);
	SET(TIMSK, ENABLE);	

	sei();
}

/* Hidas äänimerkki. */
void beep_slow()
{
	cli();

	/* onko äänimerkki päällä */
	if (GET(state, BEEPENBL))
		return;
	
	/* äänimerkki päälle */
	ts_beep = BEEP_SLOW_DURATION;
	SET(state, BEEPENBL);
	CLR(state, SYNCENBL);
	CLR(state, BEEPSYNC);
	SET(TIMSK, ENABLE);	

	sei();
}

/* Aloita äänimerkkien toisto. */
void beep_begin()
{
	cli();

	/* onko äänimerkki päällä */
	if (GET(state, BEEPENBL))
		return;
	
	/* äänimerkki päälle */
	ts_beep = 0;
	SET(state, BEEPENBL);
	SET(state, BEEPCONT);
	SET(state, SYNCENBL);
	CLR(state, BEEPSYNC);
	SET(state, BEEPSTAT);
	SET(TIMSK, ENABLE);

	sei();
}

/* Lopeta äänimerkkien toisto. */
void beep_end()
{
	cli();

	/* onko toistuva äänimerkki päällä */
	if (!GET(state, BEEPCONT))
		return;
	
	/* äänimerkki pois päältä */
	CLR(state, BEEPENBL);
	CLR(state, BEEPCONT);

	sei();
}

/* Väläytä taustavaloa. */
void blink()
{
	cli();

	/* onko välkytys päällä */
	if (GET(state, BLNKENBL))
		return;
	
	/* väläytys päälle */
	ts_blink = BLINK_DURATION;
	SET(state, BLNKENBL);
	CLR(state, SYNCENBL);
	CLR(state, BLNKSYNC);
	SET(TIMSK, ENABLE);

	sei();
}

/* Aloita taustavalon välkytys. */
void blink_begin()
{
	cli();

	/* onko välkytys päällä */
	if (GET(state, BLNKENBL))
		return;

	/* välkytys päälle */
	ts_blink = 0;
	SET(state, BLNKENBL);
	SET(state, BLNKCONT);
	SET(state, SYNCENBL);
	CLR(state, BLNKSYNC);
	SET(TIMSK, ENABLE);

	sei();
}

/* Lopeta taustavalon välkytys. */
void blink_end()
{
	cli();

	/* onko toistuva välkytys päällä */
	if (!GET(state, BLNKCONT))
		return;

	/* välkytys pois päältä */
	CLR(state, BLNKENBL);
	CLR(state, BLNKCONT);

	sei();
}
