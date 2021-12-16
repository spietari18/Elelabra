#ifndef ERROR_H
#define ERROR_H

#include "ui.h"
#include <setjmp.h>
#include <avr/pgmspace.h>

#define ERR_OK   0
#define ERR_TEST 1

extern const char *const errstr[] PROGMEM;

#define ERROR(CODE) \
	do { \
		UI_SET_STATE(ERRR); \
		longjmp(error_return, (ERR_##CODE) + 1); \
	} while (0)

/* ERROR():in paluupiste */
extern jmp_buf error_return;
extern uint8_t error_code;

#define ERROR_RETURN \
	error_code = setjmp(error_return) - 1

#define ERROR_CODE \
	error_code

#define ERROR_MSG \
	pgm_read_ptr(&errstr[error_code])

#endif // !ERROR_H
