#include "error.h"

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
