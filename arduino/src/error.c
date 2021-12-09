#include "error.h"

#define ERROR_STR(A, B) \
	static const char ERROR_STR_##B[] PROGMEM = (A)

#define ERROR_PTR(A) \
	ERROR_STR_##A

ERROR_STR("NO ERROR", 0);
ERROR_STR("TEST ERROR", 1);

const char *const errstr[N_ERR] PROGMEM = {
	ERROR_PTR(0),
	ERROR_PTR(1)
};
