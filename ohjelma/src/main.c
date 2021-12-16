#include "ui.h"
#include "alarm.h"
#include "error.h"
#include "button.h"
#include "temp_util.h"

#include <setjmp.h>

DEFINE_PSTR_PTR(TEMP_1, "TEMPERATURE");
DEFINE_PSTR_PTR(TEMP_2, "TEMP");
DEFINE_PSTR_PTR(CLBR_1, "CALIBRATION");
DEFINE_PSTR_PTR(CLBR_2, "CLBR");
DEFINE_PSTR_PTR(OPTS_1, "OPTIONS");
DEFINE_PSTR_PTR(OPTS_2, "OPTS");

/* Valikon konfiguraatio. */
MENU_CONFIG = {
	MENU_ENTRY(
		REF_PSTR_PTR(TEMP_1),
		REF_PSTR_PTR(TEMP_2),
		TEMP, NULL, NULL),
	MENU_ENTRY(
		REF_PSTR_PTR(CLBR_1),
		REF_PSTR_PTR(CLBR_2),
		CLBR, NULL, NULL), 
	MENU_ENTRY(
		REF_PSTR_PTR(OPTS_1),
		REF_PSTR_PTR(OPTS_2),
		OPTS, NULL, NULL), 
};

/* millis() aikaleimat ajastukseen */
static uint32_t ts_now, ts_old;

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
	/* lue uusi näyte? */
	ts_now = millis();
	if ((ts_now - ts_old) > (1000/SAMPLE_RATE)) {
		/* päivitä lämpötila */
		T = read_temp();
	
		/* päivitä nähdyt maksimi ja minimi */
		if (unlikely(T > obs_max))
			obs_max = T;
		else if (unlikely(T < obs_min))
			obs_min = T;

		/* päivitä lämpötilat näytöllä */
		if (!in_menu)
			lcd_put_temp(T, 2, 6, 0, ALIGN_C);
	
		// TÄHÄN VÄLIIN HÄLYTYSKOODI

		ts_old = ts_now;

		return false;
	}

	return true;
}

void view_limit_init()
{
	/* näytön staattinen teksti */
	if (!in_menu) {
		lcd_put_P_const("MIN", 0, ALIGN_L);
		lcd_put_P_const("MAX", 0, ALIGN_R);
	}
}

void view_limit_loop()
{
	/* jos mikään ei päivity, palaa */
	if (common_update())
		return;

	/* näkymäkohtainen info */
	if (!in_menu) {
		lcd_put_temp(obs_min, 1, 5, 1, ALIGN_L);
		lcd_put_temp(obs_max, 1, 5, 1, ALIGN_R);
		lcd_update();
	}
}

void view_alarm_init()
{
	/* näytön staattinen teksti */
	if (!in_menu) {
		lcd_put_P_const("AL-", 0, ALIGN_L);
		lcd_put_P_const("AL+", 0, ALIGN_R);
	}
}

void view_alarm_loop()
{
	/* jos mikään ei päivity, palaa */
	if (common_update())
		return;

	/* näkymäkohtainen info */
	if (!in_menu) {
		lcd_put_temp(lim_min, 1, 5, 1, ALIGN_L);
		lcd_put_temp(lim_max, 1, 5, 1, ALIGN_R);
		lcd_update();
	}
}

DEFINE_PSTR_PTR(LIMT, "LIMT");
DEFINE_PSTR_PTR(ALRM, "ALRM");

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
	uint32_t tmp;

	uint16_t S_now = 0, S_old = 0;

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
	//eeram_init();
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
		lcd_put_P_const(SPLASH_1, 0, ALIGN_C);
		lcd_put_P_const(SPLASH_2, 1, ALIGN_C);
		lcd_update();

		beep_slow();

		/* näytä alarivi SPLASH_WAIT millisekuntia */
		_delay_ms(SPLASH_WAIT);

		/* alusta SPLASH LOOP tila */
		tmp = 0;
		ts_old = 0;
		prog_init(MAX_SAMPLES, 1);

		UI_SETUP_END;     
		break;

	case UI_LOOP(SPLH):
		/* alusta näytepuskuri täyteen */
		ts_now = millis();
		if ((ts_now - ts_old) > (1000/SAMPLE_RATE)) {
			/* lue uusi näyte */
			(void)read_sample();

			/* päivitä edistymispalkki */
			prog_inc();
			lcd_update();

			tmp++;
			ts_old = ts_now;
		}

		/* menu_enter() alustaa valikon ja siirtyy valikossa
		 * kohtaan 0. (kutsuu MENU_CALLBACKS[0] navigaatio-
		 * takaisinkutsua, joka asettaa tilan DEFAULT)
		 */
		if (tmp >= MAX_SAMPLES)
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
			lcd_put_P(view_name, 4, 1, ALIGN_C);
			view_redraw = false;
			lcd_update();
		}

		/* pois valikosta */
		switch (button_update(&s)) {
		/* näppäinten lukitus */
		case BOTH|HOLD:
			is_locked = !is_locked;

			if (is_locked) {
				lcd_put_P_const("LOCK", 1, ALIGN_C);
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
			
			beep_slow();

			MENU_BACK;
			break;
		}
		break;

	case UI_SETUP(CLBR):
		LCD_CLEAR;

		lcd_put_P_const("<EMPTY>", 0, ALIGN_C);
		lcd_update();


		UI_SETUP_END;
		break;

	case UI_LOOP(CLBR):
		/* lue uusi näyte */
		ts_now = millis();
		if ((ts_now - ts_old) > (1000/SAMPLE_RATE)) {
			S_now = (uint16_t)(read_sample() + 0.5);

			/* päivitä näyte */
			if (S_now != S_old) {
				lcd_put_uint(S_now, 4, 1, ALIGN_C);
				lcd_update();

				S_old = S_now;
			}

			ts_old = ts_now;
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
			beep_slow();
			MENU_BACK;
			break;
		}
		break;

	case UI_SETUP(OPTS):
		LCD_CLEAR;

		lcd_put_P_const("<EMPTY>", 0, ALIGN_C);
		lcd_update();

		UI_SETUP_END;
		break;

	case UI_LOOP(OPTS):

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
			beep_slow();
			MENU_BACK;
			break;
		}
		break;
	
	case UI_SETUP(ERRR):

#define ERROR_SHOW 3000 // [ms]
#define ERROR_WAIT 1000 // [ms]

		LCD_CLEAR;

		beep_begin();

		/* tulosta virhe */
		lcd_put_fmt(LCD_COLS, 0, ALIGN_C, "VIRHE (%u):", ERROR_CODE);
		//lcd_put_P_const("VIRHE", 0, ALIGN_C);
		lcd_put_P(ERROR_MSG, NULLTERM, 1, ALIGN_C);
		lcd_update();

		/* näytä virhe ERROR_SHOW millisekuntia */
		_delay_ms(ERROR_SHOW);

		beep_end();

		LCD_CLEAR;

		/* tulosta uudelleenkäynnistysviesti */
		lcd_put_P_const("REBOOT", 0, ALIGN_C);
		prog_init(200, 200);
		lcd_update();

		ts_old = 0;
		tmp = millis();

		UI_SETUP_END;
		break;

	case UI_LOOP(ERRR):
		ts_now = millis();

		/* edistymispalkki */
		if ((ts_now - ts_old) > (ERROR_WAIT/225)) {
			prog_dec();
			lcd_update();
			ts_old = ts_now;
		}

		/* uudelleenkäynnistys */
		if ((ts_now - tmp) > ERROR_WAIT)
			reset();

		break;
	}

	goto main_loop;

	__builtin_unreachable();
}
