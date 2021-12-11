#ifndef ERROR_H
#define ERROR_H

#include "ui.h"
#include <setjmp.h>
#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define ERR_OK   0
#define ERR_TEST 1

#define N_ERR 2

extern const char *const errstr[N_ERR] PROGMEM;

#define ERROR(CODE) \
	do { \
		UI_SET_STATE(ERROR); \
		longjmp(error_return, (ERR_##CODE) + 1); \
	} while (0)

/* ERROR():in paluupiste */
extern jmp_buf error_return;
extern uint8_t error_code;

#define ERROR_RETURN \
	error_code = setjmp(error_return)

#define ERROR_CODE \
	error_code

#define ERROR_MSG \
	pgm_read_ptr(&errstr[error_code])

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !ERROR_H
