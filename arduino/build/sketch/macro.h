/* macro.h - useful preprocessor directives */
#ifndef MACRO_H
#define MACRO_H

/* for optimizing conditionals */
#define likely(A)   __builtin_expect((A), 1)
#define unlikely(A) __builtin_expect((A), 0)

/* bit manipulation */
#define GET(NUM, N) \
	(NUM & (1 << (N)))
#define SET(NUM, N) \
	((NUM) |= 1 << (N))
#define CLR(NUM, N) \
	((NUM) &= ~(1 << (N)))
#define TGL(NUM, N) \
	((NUM) ^= 1 << (N))
#define VAL(NUM, N, VAL) \
	((NUM) = ((NUM) & ~(1 << (N))) | ((VAL) << (N)))

/* We use direct register access for IO instead of
 * the Arduino digitalRead/Write instructions for
 * faster IO. To use MODE(), READ() and WRITE() the
 * following variables need to be in place for NAME:
 * <NAME>_REG_PORT
 * <NAME>_REG_DATA
 * <NAME>_REG_PIN 
 * <NAME>_REG_POS
 * These should be created with DEFINE_PIN().
 * (we hope that the compiler optimizes these)
 */
#define DEFINE_PIN(NAME, WHICH, POS) \
	uint8_t *const PIN_##NAME##_REG_PORT = &(PORT##WHICH); \
	uint8_t *const PIN_##NAME##_REG_DDR = &(DDR##WHICH); \
	const uint8_t *const PIN_##NAME##_REG_PIN = &(PIN##WHICH); \
	const uint8_t  PIN_##NAME##_REG_POS = (POS)

/* direct port register pinMode() */
#define MODE(NAME, MODE) \
	VAL(*(PIN_##NAME##_REG_DDR), PIN_##NAME##_REG_POS, (MODE))

/* direct port register digitalRead() */
#define READ(NAME) \
	((*(PIN_##NAME##_REG_PIN) >> PIN_##NAME##_REG_POS) & 1)

/* direct port register digitalWrite() */
#define WRITE(NAME, VALUE) \
	BIT(*(PIN_##NAME##_REG_PORT), PIN_##NAME##_REG_POS, (VALUE))

#endif /* !MACRO_H */
