#include "pins.h"
#include "util.h"
#include "macro.h"
#include "alarm.h"

#include <stddef.h>

#include <avr/pgmspace.h>
#include <avr/interrupt.h>

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
static uint16_t set_prescaler(volatile uint8_t *reg, uint16_t freq, uint16_t max)
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

callback_t timer_callback;

ISR(TIMER2_COMPA_vect)
{
	/* kutsu takaisinkutsua tai laita pois
	 * päältä jos takaisinkutsua ei ole
	 */
	if (likely(timer_callback != NULL))
		timer_callback();
	else
		CLR(TIMSK2, OCIE2A);
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

	sei();
}

#define BEEPENBL 0 // äänimerkki päälle
#define BLNKENBL 1 // välkytys päälle
#define BEEPCONT 2 // jatkuva äänimerkki
#define BLNKCONT 3 // jatkuva välkytys
#define SYNCENBL 4 // synkronoi (ei käytössä)
#define BEEPSYNC 5 // äänimerkki synkronoitu
#define BLNKSYNC 6 // välkytys synkronoitu
#define BEEPSTAT 7 // jatkuvan äänimerkin tila

/* Tilakone timer_bb_* funktioille. */
static volatile uint8_t state;

/* Aikaleimat, joilla ajastuksia ohjataan.
 * (myös timer_notes() käyttää näitä)
 */
static volatile uint32_t ts_1;
static volatile uint32_t ts_2;

static void timer_bb_single()
{
	uint32_t now;
	uint8_t off = 0;

	now = millis();

	now = millis();

	/* äänimerkki */
	if (likely(GET(state, BEEPENBL))) {
		/* alustettu */
		if (likely(GET(state, BEEPSYNC))) {
			/* aika kulunut */
			if (now >= ts_1) {
				WRITE(BUZZER, LOW);
				CLR(state, BEEPENBL);
			}

		/* alusta */
		} else {
			ts_1 += now;
			SET(state, BEEPSYNC);
		}

		/* ääni */
		TOGGLE(BUZZER);
	
	/* IO pinnin tila tunnetuksi */
	} else {
		WRITE(BUZZER, LOW);
		off++;
	}

	/* välkytys */
	if (likely(GET(state, BLNKENBL))) {
		/* alustettu */
		if (likely(GET(state, BLNKSYNC))) {
			/* aika kulunut */
			if (now >= ts_2) {
				WRITE(LCD_AN, HIGH);
				CLR(state, BLNKENBL);
			}

		/* alusta */
		} else {
			ts_2 += now;
			WRITE(LCD_AN, LOW);
			SET(state, BLNKSYNC);
		}
	
	/* IO pinnin tila tunnetuksi */
	} else {
		WRITE(LCD_AN, HIGH);
		off++;
	}

	/* molemmat pois päältä? */
	if (off >= 2)
		/* ajastin pois päältä */
		CLR(TIMSK2, OCIE2A);
}

static void timer_bb_continous()
{
	uint32_t now;
	uint8_t off = 0;

	now = millis();

	/* äänimerkki */
	if (likely(GET(state, BEEPENBL))) {
		/* synkronoi */
		if (unlikely(!GET(state, BEEPSYNC))) {
			/* synkronoi välkytyksen kanssa */
			if (unlikely(GET(state, BLNKCONT)
				&& GET(state, BLNKSYNC)))
				ts_1 = ts_2;

			/* ei välkytystä -> synkronoitu */
			SET(state, BEEPSYNC);
		}

		/* päälle/pois jaksonajan mukaan */
		if (unlikely((now - ts_1)
			> (COMMON_PERIOD/2))) {
			TGL(state, BEEPSTAT);
			ts_1 = now;
		}

		/* ääni */
		if (GET(state, BEEPSTAT))
			TOGGLE(BUZZER);
	
	/* IO pinnin tila tunnetuksi */
	} else {
		WRITE(BUZZER, LOW);
		off++;
	}

	/* välkytys */
	if (likely(GET(state, BLNKENBL))) {
		/* synkronoi */
		if (unlikely(!GET(state, BLNKSYNC))) {
			/* synkronoi äänimerkin kanssa */
			if (unlikely(GET(state, BEEPCONT)))
				ts_2 = ts_1;

			/* ei äänimerkkiä -> synkronoitu */
			SET(state, BLNKSYNC);
		}

		/* päälle/pois jaksonajan mukaan */
		if (unlikely((now - ts_2)
			> (COMMON_PERIOD/2))) {
			TOGGLE(LCD_AN);
			ts_2 = now;
		}
	
	/* IO pinnin tila tunnetuksi */
	} else {
		WRITE(LCD_AN, HIGH);
		off++;
	}

	/* molemmat pois päältä? */
	if (off >= 2)
		/* ajastin pois päältä */
		CLR(TIMSK2, OCIE2A);
}

/* Nopea äänimerkki. */
void beep_fast()
{
	cli();

	/* onko ajastin päällä */
	if (GET(TIMSK2, OCIE2A)) {
		/* jos on, takaisinkutsun tulee olla oikea
		 * ja äänimerkki ei saa olla päällä
		 */
		if ((timer_callback != &timer_bb_single)
			|| GET(state, BEEPENBL))
			goto exit;
	
	/* ajastin ei päällä, aseta takaisinkutsu ja taajuus */
	} else {
		timer_callback = &timer_bb_single;
		OCR2A = set_prescaler(&TCCR2B, 2*BEEP_FREQUENCY, (uint8_t)~0);
	}

	/* äänimerkki päälle */
	ts_1 = BEEP_FAST_DURATION;
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

	/* onko ajastin päällä */
	if (GET(TIMSK2, OCIE2A)) {
		/* jos on, takaisinkutsun tulee olla oikea
		 * ja äänimerkki ei saa olla päällä
		 */
		if ((timer_callback != &timer_bb_single)
			|| GET(state, BEEPENBL))
			goto exit;
	
	/* ajastin ei päällä, aseta takaisinkutsu ja taajuus */
	} else {
		timer_callback = &timer_bb_single;
		OCR2A = set_prescaler(&TCCR2B, 2*BEEP_FREQUENCY, (uint8_t)~0);
	}
	
	/* äänimerkki päälle */
	ts_1 = BEEP_SLOW_DURATION;
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

	/* onko ajastin päällä */
	if (GET(TIMSK2, OCIE2A)) {
		/* jos on, takaisinkutsun tulee olla oikea
		 * ja äänimerkki ei saa olla päällä
		 */
		if ((timer_callback != &timer_bb_continous)
			|| GET(state, BEEPCONT))
			goto exit;
	
	/* ajastin ei päällä, aseta takaisinkutsu ja taajuus */
	} else {
		timer_callback = &timer_bb_continous;
		OCR2A = set_prescaler(&TCCR2B, 2*BEEP_FREQUENCY, (uint8_t)~0);
	}
	
	/* äänimerkki päälle */
	ts_1 = 0;
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

	/* onko ajastin päällä, onko takaisinkutsu
	 * oikea ja onko äänimerkki päällä
	 */
	if (!GET(TIMSK2, OCIE2A) || (timer_callback != 
		&timer_bb_continous) || !GET(state, BEEPCONT))
		goto exit;
	
	/* äänimerkki pois päältä */
	CLR(state, BEEPENBL);
	CLR(state, BEEPCONT);
	CLR(TIMSK2, OCIE2A);
exit:
	sei();
}

/* Väläytä taustavaloa. */
void blink()
{
	cli();

	/* onko ajastin päällä */
	if (GET(TIMSK2, OCIE2A)) {
		/* jos on, takaisinkutsun tulee olla oikea
		 * ja välkytys ei saa olla päällä
		 */
		if ((timer_callback != &timer_bb_single)
			|| GET(state, BLNKENBL))
			goto exit;
	
	/* ajastin ei päällä, aseta takaisinkutsu ja taajuus */
	} else {
		timer_callback = &timer_bb_single;
		OCR2A = set_prescaler(&TCCR2B, 2*BEEP_FREQUENCY, (uint8_t)~0);
	}
	
	/* väläytys päälle */
	ts_2 = BLINK_DURATION;
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

	/* onko ajastin päällä */
	if (GET(TIMSK2, OCIE2A)) {
		/* jos on, takaisinkutsun tulee olla oikea
		 * ja välkytys ei saa olla päällä
		 */
		if ((timer_callback != &timer_bb_continous)
			|| GET(state, BLNKCONT))
			goto exit;
	
	/* ajastin ei päällä, aseta takaisinkutsu ja taajuus */
	} else {
		timer_callback = &timer_bb_continous;
		OCR2A = set_prescaler(&TCCR2B, 2*BEEP_FREQUENCY, (uint8_t)~0);
	}

	/* välkytys päälle */
	ts_2 = 0;
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

	/* onko ajastin päällä, onko takaisinkutsu
	 * oikea ja onko äänimerkki päällä
	 */
	if (!GET(TIMSK2, OCIE2A) || (timer_callback != 
		&timer_bb_continous) || !GET(state, BLNKCONT))
		goto exit;

	/* välkytys pois päältä */
	CLR(state, BLNKENBL);
	CLR(state, BLNKCONT);
exit:
	sei();
}

static inline uint16_t note_freq(uint16_t note)
{
	uint16_t f;
	int8_t key;

	key = N_PITCH(note);

	//if (key < 3)
	//	key += 12;
	
	key += ((N_OCT(note) - 1)*12) + 1;
	f = 440.0 * pow(2.0, (float)(key - 49)/12.0);

	return f;
}

static volatile uint16_t note_index;
static volatile uint16_t durnow;

static uint16_t note_count;
static uint16_t gap;
static uint16_t dur4th;
static uint16_t *notes;

#include "ui.h"

static void timer_notes()
{
	uint32_t now;
	uint16_t note;

	now = millis();

	/* nuotti ei vielä valmis */
	if (likely((now - ts_1) < durnow))
		goto play;

	/* jos nuotit loppu, ajastin pois päältä */
	if (note_index >= note_count) { 
		WRITE(BUZZER, LOW);
		CLR(TIMSK2, OCIE2A);
		return;
	}

	/* väliä ei ole vielä odotettu */
	if (likely((now - ts_2) < gap)) {
		WRITE(BUZZER, LOW);
		return;
	}

	note = pgm_read_word(&notes[note_index++]);

	/* laske nuotin kesto */
	durnow = 4*dur4th;
	if (N_INV(note))
		durnow *= N_DUR(note);
	else
		durnow /= N_DUR(note);

	/* aseta uusi taajuus */
	OCR2A = set_prescaler(&TCCR2B, 2*note_freq(note), (uint8_t)~0);

	ts_1 = now;
	ts_2 = ts_1 + durnow;
play:
	/* toista nuottia */
	TOGGLE(BUZZER);
}

void play_notes(const struct sheet *s)
{
	cli();

	/* ajastin on päällä */
	if (GET(TIMSK2, OCIE2A))
		goto end;

	/* aseta ajastimen takaisinkutsu */
	timer_callback = &timer_notes;

	/* laske neljännesosanuotin kesto */
	dur4th = 60000/pgm_read_byte(&s->tempo);

	/* nuottien väli (tähän ei ole oikeaa arvoa) */
	gap = dur4th/64;

	/* aseta nuotit ja nuottien määrä */
	notes = pgm_read_ptr(&s->notes);
	note_count = pgm_read_byte(&s->count);
	ts_1 = ts_2 = note_index = durnow = 0;

	/* ei nuotteja */
	if (note_count < 1)
		return;

	/* aseta ajastimen taajuus ensimmäisen nuotin
	 * taajuudeksi. (2x koska ajastimen yksi jakso
	 * vastaa puolikasta kanttiaallon jaksoa)
	 */
	OCR2A = set_prescaler(&TCCR2B,
		2*note_freq(pgm_read_word(&notes[0])), (uint8_t)~0);

	/* aloita ajastin */
	SET(TIMSK2, OCIE2A);
end:
	sei();
}

NOTES(e1m1_notes) = {
	N8(E, 3), N8(E, 3), N8(E, 4), N8(E, 3),
	N8(E, 3), N8(D, 4), N8(E, 3), N8(E, 3),

	N8(C, 4), N8(E, 3), N8(E, 3), N8(As, 3),
	N8(E, 3), N8(E, 3), N8(B, 4), N8(C, 4),
#if 0
	N8(E, 3), N8(E, 3), N8(E, 4), N8(E, 3),
	N8(E, 3), N8(D, 4), N8(E, 3), N8(E, 3),

	N8(C, 4), N8(E, 3), N8(E, 3), N2(As, 3), // (puuttuu 1/8)

	
	N8(E, 3), N8(E, 3), N8(E, 4), N8(E, 3),
	N8(E, 3), N8(D, 4), N8(E, 3), N8(E, 3),

	N16(G, 5), N16(F, 5), N16(E, 5), N16(F, 5),
	N16(G, 5), N16(F, 5), N16(F, 5), N16(G, 5),
	N16(F, 5), N16(E, 5), N16(D, 5), N16(B, 5),
	N16(E, 4), N16(C, 4), N4(B, 4) // (1/8 liikaa)
#endif
};

SHEET(e1m1_sheet, 150, e1m1_notes);

void play_e1m1() { play_notes(&e1m1_sheet); }
