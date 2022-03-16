#ifndef ALARM_H
#define ALARM_H

/* Äänimerkin taajuus. */
#define BEEP_FREQUENCY 750 // [Hz]

/* Nopean äänimerkin kesto. */
#define BEEP_FAST_DURATION 80 // [ms]

/* Hitaan äänimerkin kesto. */
#define BEEP_SLOW_DURATION 200 // [ms]

/* Taustavalon väläytyksen kesto. */
#define BLINK_DURATION 200 // [ms]

/* Äänimerkin ja taustavalon välkkymisen jaksonaika. */
#define COMMON_PERIOD 350 // [ms]

/* Alustus. */
void alarm_init();

/* Nopea äänimerkki. */
void beep_fast();

/* Hidas äänimerkki. */
void beep_slow();

/* Aloita äänimerkkien toisto. */
void beep_begin();

/* Lopeta äänimerkkien toisto. */
void beep_end();

/* Väläytä taustavaloa. */
void blink();

/* Aloita taustavalon välkytys. */
void blink_begin();

/* Lopeta taustavalon välkytys. */
void blink_end();

/* Standardinuotit (siirtona A:han verrattuna) */
#define N_A  0
#define N_As 1
#define N_B  2
#define N_C  3
#define N_Cs 4
#define N_D  5
#define N_Ds 6
#define N_E  7
#define N_F  8
#define N_Fs 9
#define N_G  10
#define N_Gs 11

#define N_NOTE  0
#define N_PAUSE 1

/* ************ NOTE *************
 * 0 .. 1 . 2 . 5 ... 9 . 12 .. 16
 * |type|inv|dur|pitch|oct|unused| */
#define N(pitch, octave, duration, invert) \
	(N_NOTE | (((invert) & 1) << 1) | (((8*sizeof(duration) - \
		__builtin_clz(duration) - 1) & 7) << 2) | \
		(((N_##pitch) & 15) << 5) | (((octave) & 7) << 9))

/* ******* PAUSE *******
 * 0 .. 1 . 2 . 5 ... 16
 * |type|inv|dur|unused|
 */
#define P(duration, invert) \
	(N_PAUSE | (((invert) & 1) << 1) | ((duration) << 2))

#define N_TYPE(n)  ((n) & 1)
#define N_INV(n)   (((n) >> 1) & 1)
#define N_DUR(n)   (1 << (((n) >> 2) & 7))
#define N_PITCH(n) (((n) >> 5) & 15)
#define N_OCT(n)   (((n) >> 9) & 7)

/* Yleiset nuottien kestot. */
#define N1(pitch, oct)  N(pitch, oct, 1, 0)
#define N2(pitch, oct)  N(pitch, oct, 2, 0)
#define N4(pitch, oct)  N(pitch, oct, 4, 0)
#define N8(pitch, oct)  N(pitch, oct, 8, 0)
#define N16(pitch, oct) N(pitch, oct, 16 ,0)
#define N32(pitch, oct) N(pitch, oct, 32, 0)

/* Yleiset taukojen kestot. */
#define P1 P(1, 0)
#define P2 P(2, 0)
#define P4 P(3, 0)
#define P8 P(4, 0)

struct sheet
{
	uint8_t tempo;
	uint8_t count;
	const uint16_t *notes;
};

#define SHEET(name, tempo, notes) \
	const struct sheet name PROGMEM = {tempo, ARRAY_SIZE(notes), notes} 

#define NOTES(name) \
	const uint16_t name[] PROGMEM

/* Soita nuotteja. */
void play_notes(const struct sheet *);

/* Soita DOOM 1:n E1M1 muistista. */
void play_e1m1();

#endif // !ALARM_H
