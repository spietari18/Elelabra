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

extern jmp_buf main_loop;
extern const char *const errstr[N_ERR] PROGMEM;

#define ERROR(CODE) \
	do { \
		UI_SET_STATE(ERROR); \
		longjmp(main_loop, (ERR_##CODE) + 1); \
	} while (0)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !ERROR_H
