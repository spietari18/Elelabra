#ifndef MACRO_H
#define MACRO_H

#include <avr/io.h>
#include <stdint.h>

/* Ehtolauseiden optimointi. */
#define likely(A)   __builtin_expect((A), 1)
#define unlikely(A) __builtin_expect((A), 0)

/* Kääntöaikana vakiona olevan taulukon koko. */
#define ARRAY_SIZE(A) \
	(sizeof(A)/sizeof(*(A)))

/* Arduinon standardikirjasto määrittää nämä jo. */
#if 0
#define min(A, B) \
	((A < B) ? (A) : (B))
#define max(A, B) \
	((A > B) ? (A) : (B))
#endif

/* Pyöristävä kokonaislukujen jakolasku. */
#define int_div_rnd(A, B) \
	((A) + (B) - 1)/(B)

/* Bittimanipulaatio. */
#define GET(NUM, N) \
	(NUM & (1UL << (N)))
#define SET(NUM, N) \
	((NUM) |= 1UL << (N))
#define CLR(NUM, N) \
	((NUM) &= ~(1UL << (N)))
#define TGL(NUM, N) \
	((NUM) ^= 1UL << (N))
#define VAL(NUM, N, V) \
	((NUM) = ((NUM) & ~(1UL << (N))) | ((!!(V)) << (N)))

/* Tämän avulla voi käyttää suoraan IO rekistereitä
 * IO pinnien manipuloimiseen, joka on huomattavasti
 * nopeampaa kuin Arduinon standardikirjaston funktioiden
 * käyttäminen. Tämä makro määrittää seuraavat muuttujat:
 * <NAME>_REG_PORT
 * <NAME>_REG_DATA
 * <NAME>_REG_PIN 
 * <NAME>_REG_POS
 * Joita muut alla olevat funktiot käyttävit pinnien
 * manipuloimiseen. (Kääntäjän pitäisi optimoida nämä pois)
 * */
#define DEFINE_PIN(NAME, WHICH, POS) \
	static volatile uint8_t *const \
		PIN_##NAME##_REG_PORT = &(PORT##WHICH); \
	static volatile uint8_t *const \
		PIN_##NAME##_REG_DDR = &(DDR##WHICH); \
	static const volatile uint8_t *const \
		PIN_##NAME##_REG_PIN = &(PIN##WHICH); \
	static const uint8_t PIN_##NAME##_REG_POS = (POS)

/* Aseta IO pinnin tila. (READ tai WRITE) */
#define MODE(NAME, MODE) \
	VAL(*(PIN_##NAME##_REG_DDR), PIN_##NAME##_REG_POS, (MODE))

/* Aseta IO pinnin PULLUP tila. */
#define PLUP(NAME, MODE) \
	VAL(*(PIN_##NAME##_REG_PORT), PIN_##NAME##_REG_POS, (MODE))

/* Lue IO pinnin arvo. (LOW tai HIGH) */
#define READ(NAME) \
	((*(PIN_##NAME##_REG_PIN) >> PIN_##NAME##_REG_POS) & 1)

/* Kirjoita IO pinnille arvo. (LOW tai HIGH) */
#define WRITE(NAME, VALUE) \
	VAL(*(PIN_##NAME##_REG_PORT), PIN_##NAME##_REG_POS, (VALUE))

/* Vaihda IO pinnin tilaa. */
#define TOGGLE(NAME) \
	TGL(*(PIN_##NAME##_REG_PORT), PIN_##NAME##_REG_POS);

#endif /* !MACRO_H */
