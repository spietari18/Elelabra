#ifndef ALARM_H
#define ALARM_H

/* Äänimerkin taajuus. */
#define BEEP_FREQUENCY 250 // [Hz]

/* Nopean äänimerkin kesto. */
#define BEEP_FAST_DURATION 500 // [ms]

/* Hitaan äänimerkin kesto. */
#define BEEP_SLOW_DURATION 1000 // [ms]

/* Taustavalon väläytyksen kesto. */
#define BLINK_DURATION 500 // [ms]

/* Äänimerkin ja taustavalon välkkymisen jaksonaika. */
#define COMMON_PERIOD 500 // [ms]

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

#endif // !ALARM_H
