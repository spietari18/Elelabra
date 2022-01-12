#include "error.h"

/* jos näiden sijaan haluaisi käyttää util.h:n
 * DEF_PSTR_PTR() ja REF_PSTR_PTR() funktioita
 * errstr jäsenet tulee olle järjestyksessä.
 */
#define ERROR_STR(A, B) \
	static const char ERROR_STR_##A[] PROGMEM = (B)
#define ERROR_PTR(A) \
	[ERR_##A] = ERROR_STR_##A

ERROR_STR(OK, "NO ERROR");
ERROR_STR(TEST, "TEST ERROR");

const char *const errstr[] PROGMEM = {
	ERROR_PTR(OK),
	ERROR_PTR(TEST)
};

jmp_buf error_return;
uint8_t error_code;
