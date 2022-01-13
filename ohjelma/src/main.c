#include "ui.h"
#include "alarm.h"
#include "error.h"
#include "button.h"
#include "temp_util.h"

static uint32_t ts;
#define INTERVAL(F) \
	if (interval((F), &ts))

uint16_t updates;

/* Taustalla kutsuttava päivitysfunktio.
 * Ideana se, että lämpötilaa päivitetään
 * myös valikossa TEMP näkymän lisäksi.
 */
void global_update()
{
	// TESTIKOODI
	INTERVAL(100) {
		lcd_put_P_const("      ", 0, LEFT);
		lcd_put_uint(updates, 6, 0, LEFT);
		lcd_update();
		updates++;
	}
}

/* Valikon teksti. */
DEF_PSTR_PTR(TEMP_1, "TEMPERATURE");
DEF_PSTR_PTR(TEMP_2, "TEMP");
DEF_PSTR_PTR(CLBR_1, "CALIBRATION");
DEF_PSTR_PTR(CLBR_2, "CLBR");
DEF_PSTR_PTR(OPTS_1, "OPTIONS");
DEF_PSTR_PTR(OPTS_2, "OPTS");

void callback() {}

/* Valikon konfiguraatio. */
MENU_CONFIG = {
	MENU_ENTRY(
		REF_PSTR_PTR(TEMP_1),
		REF_PSTR_PTR(TEMP_2),
		TEMP, NULL, NULL, NULL),
	MENU_ENTRY(
		REF_PSTR_PTR(CLBR_1),
		REF_PSTR_PTR(CLBR_2),
		CLBR, NULL, NULL, NULL), 
	MENU_ENTRY(
		REF_PSTR_PTR(OPTS_1),
		REF_PSTR_PTR(OPTS_2),
		OPTS, NULL, NULL, NULL), 
};

/* lämpötila */
static float T;

/* lämpömittarin näkemä minimi- ja maksimilämpötila */
static float obs_min =  INF;
static float obs_max = -INF;

/* asetetut rajat */
static float lim_min = -INF;
static float lim_max =  INF;

bool common_update()
{
	INTERVAL(1000/SAMPLE_RATE) {
		/* päivitä lämpötila */
		T = read_temp();
	
		/* päivitä nähdyt maksimi ja minimi */
		if (unlikely(T > obs_max))
			obs_max = T;
		else if (unlikely(T < obs_min))
			obs_min = T;

		/* päivitä lämpötilat näytöllä */
		if (!in_menu)
			lcd_put_temp(T, 2, 6, 0, CENTER);
	
		// TÄHÄN VÄLIIN HÄLYTYSKOODI

		return false;
	}

	return true;
}


void view_limit_init()
{
	/* näytön staattinen teksti */
	if (!in_menu) {
		lcd_put_P_const("MIN", 0, LEFT);
		lcd_put_P_const("MAX", 0, RIGHT);
	}
}

void view_limit_loop()
{
	/* jos mikään ei päivity, palaa */
	if (common_update())
		return;

	/* näkymäkohtainen info */
	if (!in_menu) {
		lcd_put_temp(obs_min, 1, 5, 1, LEFT);
		lcd_put_temp(obs_max, 1, 5, 1, RIGHT);
		lcd_update();
	}
}

void view_alarm_init()
{
	/* näytön staattinen teksti */
	if (!in_menu) {
		lcd_put_P_const("AL-", 0, LEFT);
		lcd_put_P_const("AL+", 0, RIGHT);
	}
}

void view_alarm_loop()
{
	/* jos mikään ei päivity, palaa */
	if (common_update())
		return;

	/* näkymäkohtainen info */
	if (!in_menu) {
		lcd_put_temp(lim_min, 1, 5, 1, LEFT);
		lcd_put_temp(lim_max, 1, 5, 1, RIGHT);
		lcd_update();
	}
}

DEF_PSTR_PTR(LIMT, "LIMT");
DEF_PSTR_PTR(ALRM, "ALRM");

/* TEMP näkymien takaisinkutsut. */
static const struct {
	const void *name;
	callback_t init;
	callback_t loop;
} packed views[] PROGMEM = {
	{REF_PSTR_PTR(LIMT), &view_limit_init, &view_limit_loop},
	{REF_PSTR_PTR(ALRM), &view_alarm_init, &view_alarm_loop}
};

#define view_name \
	((const char *)pgm_read_ptr(&views[view].name))

#define view_init \
	((callback_t)pgm_read_ptr(&views[view].init))

#define view_loop \
	((callback_t)pgm_read_ptr(&views[view].loop))

/* noreturn ei toimi tässä, mutta viimeistään
 * linkkeri optimoi paluukoodin pois
 */
int main()
{
	struct button_state s = {};
	uint16_t S_now = 0, S_old = 0;

	uint8_t counter;

	/* luku jota napeilla muutetaan */
	//float *target;

	/* mikä näkymä DEFAULT näytössä on päällä */
	uint8_t view = 0;

	/* pitääkö näkymän indikaattori piirtää uudelleen */
	bool view_redraw;

	/* näppäimet lukittu */
	bool is_locked = false;

	/* aseta käytetyt moduulit päälle ja muut pois päältä */
	PRR = ~0;
	CLR(PRR, PRTIM0); // TIMER0 päälle
	CLR(PRR, PRTIM2); // TIMER2 päälle
	CLR(PRR, PRSPI);  // SPI päälle
	CLR(PRR, PRTWI);  // I2C päälle

	/* alusta kaikki IO pinnit INPUT PULLUP tilaan
	 * jotta käyttämättömät pinnit eivät kellu.
	 * aseta myös kaikkien pinnien arvoksi LOW
	 */
	DDRB  = DDRC  = DDRD  =  0; // INPUT
	PORTB = PORTC = PORTD = ~0; // PULLUP
	PINB  = PINC  = PIND  =  0; // LOW

	/* Alusta moduulit/tilat. */
	lcd_init();
	adc_init();
	eeram_init();
	timer_init();
	alarm_init();
	default_points();

	/* käyttöliittymä alkaa splash näytöstä */
	UI_SET_STATE(SPLH);

	/* ERROR() palaa tähän mistä tahansa. */
	ERROR_RETURN;
main_loop:

	switch (UI_GET_STATE) {
	case UI_SETUP(SPLH):

#define SPLASH_1 "L\xE1mp\xEFmittari"
#define SPLASH_2 "V. 1.0"
#define SPLASH_WAIT 1000 // [ms]

		/* splash */
		lcd_put_P_const(SPLASH_1, 0, CENTER);
		lcd_put_P_const(SPLASH_2, 1, CENTER);
		lcd_update();

		beep_slow();

		/* näytä alarivi SPLASH_WAIT millisekuntia */
		_delay_ms(SPLASH_WAIT);

		/* alusta edistymispalkki */
		prog_init(MAX_SAMPLES, 1);

		UI_SETUP_END;     
		break;

	case UI_LOOP(SPLH):
		/* alusta näytepuskuri täyteen */
		INTERVAL(1000/SAMPLE_RATE) {
			/* lue uusi näyte */
			(void)read_sample();

			/* päivitä edistymispalkki */
			prog_inc();
			lcd_update();
		}

		/* Aseta tila oletusnäkymään. */
		if (prog_pos >= (MAX_SAMPLES - 1))
			UI_SET_STATE(TEMP);

		break;
	
	case UI_SETUP(MENU):
		LCD_CLEAR;
		menu_draw();
		UI_SETUP_END;
		break;
	
	case UI_LOOP(MENU):
		menu_update();
		break;

	case UI_SETUP(TEMP):
		/* tyhjennä näyttö */
		LCD_CLEAR;

		/* näkymän alustus */
		view_init();
		view_redraw = true;

		UI_SETUP_END;
		break;

	case UI_LOOP(TEMP):
		/* näkymän päivitys */
		view_loop();

		/* piirrä näkymän indikaattoriteksti */
		if (unlikely(view_redraw)) {
			lcd_put_P(view_name, 4, 1, CENTER);
			view_redraw = false;
			lcd_update();
		}

		/* pois valikosta */
		switch (button_update(&s)) {
		/* näppäinten lukitus */
		case BOTH|HOLD:
			is_locked = !is_locked;

			if (is_locked) {
				lcd_put_P_const("LOCK", 1, CENTER);
				view_redraw = false;
			} else {
				view_redraw = true;
			}

			beep_slow();

			lcd_update();
			break;

		/* näkymä eteen päin */
		case RT|UP:
			if (is_locked)
				break;

			beep_fast();

			INC_MOD(view, ARRAY_SIZE(views));
			view_redraw = true;

			UI_SET_STATE(TEMP);
			break;

		/* näkymä taakse päin */
		case LT|UP:
			if (is_locked)
				break;

			beep_fast();

			DEC_MOD(view, ARRAY_SIZE(views));
			view_redraw = true;

			UI_SET_STATE(TEMP);
			break;

		/* takaisin valikkoon */
		case BOTH|UP:
			if (is_locked)
				break;
			
			menu_enter();
			break;
		}
		break;

	case UI_SETUP(CLBR):
		LCD_CLEAR;

		lcd_put_P_const("<EMPTY>", 0, CENTER);
		lcd_update();

		UI_SETUP_END;
		break;

	case UI_LOOP(CLBR):
		/* lue uusi näyte */
		INTERVAL(1000/SAMPLE_RATE) {
			S_now = (uint16_t)(read_sample() + 0.5);

			/* päivitä näyte */
			if (S_now != S_old) {
				lcd_put_uint(S_now, 4, 1, CENTER);
				lcd_update();

				S_old = S_now;
			}
		}

		switch (button_update(&s)) {
		/* testi */
		case BOTH|HOLD:
			ERROR(TEST);
			break;

		case RT|UP:
		case LT|UP:
			beep_fast();
			break;

		/* takaisin valikkoon */
		case BOTH|UP:
			menu_enter();
			break;
		}
		break;

	case UI_SETUP(OPTS):
		LCD_CLEAR;

		//lcd_put_P_const("<EMPTY>", 0, CENTER);
		//lcd_update();
		
		counter = 0;

		UI_SETUP_END;
		break;

	case UI_LOOP(OPTS):
		INTERVAL(1000) {
			uint8_t tmp;

			if (!eeram_write(0, &counter, 1))
				ERROR(TEST);
			if (!eeram_read(0, &tmp, 1))
				ERROR(TEST);

			lcd_put_P_const("   ", 0, LEFT);
			lcd_put_uint(tmp, 3, 0, LEFT);
			lcd_update();

			counter++;
		}

		switch (button_update(&s)) {
		/* testi */
		case BOTH|HOLD:
			ERROR(TEST);
			break;

		case RT|UP:
		case LT|UP:
			beep_fast();
			break;

		/* takaisin valikkoon */
		case BOTH|UP:
			menu_enter();
			break;
		}
		break;
	
	case UI_SETUP(ERRR):

#define ERROR_SHOW 3000 // [ms]
#define ERROR_WAIT 1000 // [ms]

		LCD_CLEAR;

		beep_begin();

		/* tulosta virhe */
		lcd_put_fmt(LCD_COLS, 0, CENTER, "VIRHE (%u):", ERROR_CODE);
		lcd_put_P(ERROR_MSG, NULLTERM, 1, CENTER);
		lcd_update();

		/* näytä virhe ERROR_SHOW millisekuntia */
		_delay_ms(ERROR_SHOW);

		beep_end();

		LCD_CLEAR;

#define PROG_SEGS 192

		/* tulosta uudelleenkäynnistysviesti */
		lcd_put_P_const("REBOOT", 0, CENTER);
		prog_init(PROG_SEGS, PROG_SEGS);
		lcd_update();

		UI_SETUP_END;
		break;

	case UI_LOOP(ERRR):
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

		break;

	default:
		unreachable;
	}

	/* pääsilmukka on toteutettu goto komennolla, jotta säästytään
	 * turhalta sisennykseltä, eikä silmukasta voi vahingossa poistua.
	 */
	goto main_loop;

	unreachable;
}
