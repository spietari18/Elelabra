#include "ui.h"
#include "alarm.h"
#include "error.h"

void __error(const void *msg, bool (*cb)())
{
	beep_begin();

	/* virheviesti */
	LCD_CLEAR;
	lcd_put_P_const("ERROR:", 0, CENTER);
	lcd_put_P(msg, NULLTERM, 1, CENTER);
	lcd_update();

	/* odota */
	_delay_ms(ERROR_SHOW);

	beep_end();

	if (cb)
		while (cb());
	else
		while (1);
}

#define PROG_SEGS 192

noreturn bool __reboot()
{
	/* tulosta uudelleenkäynnistysviesti */
	LCD_CLEAR;
	lcd_put_P_const("REBOOT", 0, CENTER);
	prog_init(PROG_SEGS, PROG_SEGS);
	lcd_update();

loop:
	/* edistymispalkki */
	INTERVAL(ERROR_WAIT/PROG_SEGS) {
		prog_dec();
		lcd_update();
	}

	/* uudelleenkäynnistys */
	if (prog_pos >= PROG_SEGS) {
		_delay_ms(10);
		reset();
	}

	goto loop;

	unreachable;
	return false;
}
