#ifndef ERROR_H
#define ERROR_H

#include "util.h"

#define ERROR_SHOW 3000 // [ms]
#define ERROR_WAIT 1000 // [ms]

#define error(msg, cb) \
	__error(PSTR(msg), (cb))

#define reboot(msg) \
	error((msg), &__reboot)

void __error(const void *, bool (*)());

bool __reboot();

#endif // !ERROR_H
