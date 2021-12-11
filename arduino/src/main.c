#include "ui.h"
#include "util.h"
#include "alarm.h"
#include "error.h"
#include "screen.h"
#include "button.h"
#include "temp_util.h"

#include <Arduino.h>
#include <setjmp.h>

#define DEFINE_NAV_CALLBACK(NAME) \
	void callback_##NAME##_nav() {\
		UI_SET_STATE(NAME); \
	}\

#define NAV_CALLBACK(NAME) \
	(&callback_##NAME##_nav)

/* Arduino.h määrittelee tämän jostain syystä. */
#undef DEFAULT

DEFINE_NAV_CALLBACK(DEFAULT);
DEFINE_NAV_CALLBACK(CALIBR);
DEFINE_NAV_CALLBACK(CONFIG1);
DEFINE_NAV_CALLBACK(CONFIG2);

/* Valikon takaisinkutsut. */
MENU_CALLBACKS = {
	MENU_CALLBACK(NAV_CALLBACK(DEFAULT), NULL),
	MENU_CALLBACK(NAV_CALLBACK(CALIBR), NULL),
	MENU_CALLBACK(NAV_CALLBACK(CONFIG1), NULL),
	MENU_CALLBACK(NAV_CALLBACK(CONFIG2), NULL)
};

void __attribute__((noreturn))
entry_point()
{
	struct button_state s = {};
	uint32_t ts_now, ts_old, tmp;
	bool is_locked = false;

	/* lämpömittarin näkemä minimi- ja maksimilämpötila */
	float obs_min = INF, obs_max = -INF;

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
	spi_init();
	i2c_init();
	alarm_init();
	default_points();
	compute_lss_coefs();
	
	/* käyttöliittymä alkaa splash näytöstä */
	UI_SET_STATE(SPLASH);

	/* ERROR() palaa tähän mistä tahansa. */
	ERROR_RETURN;
main_loop:

	switch (UI_GET_STATE) {
	case UI_SETUP(SPLASH):

#define SPLASH_1 "L\xE1mp\xEFmittari"
#define SPLASH_2 "v. 1.0"
#define SPLASH_WAIT 1000 // [ms]

		/* splash */
		lcd_put_P_const(SPLASH_1, 0, ALIGN_C);
		lcd_put_P_const(SPLASH_2, 1, ALIGN_C);
		lcd_update();

		/* näytä alarivi SPLASH_WAIT millisekuntia */
		ts_old = millis();
		while ((millis() - ts_old) < SPLASH_WAIT);

		/* alusta SPLASH LOOP tila */
		tmp = 0;
		ts_old = 0;
		prog_init(MAX_SAMPLES, 1);

		UI_SETUP_END;     
		break;

	case UI_LOOP(SPLASH):
		/* alusta näytepuskuri täyteen */
		ts_now = millis();
		if ((ts_now - ts_old) > (1000/SAMPLE_RATE)) {
			/* päivitä lämpötila */
			(void)temp_update();

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
			menu_enter(0);

		break;
	
	/* oletusnäkymä */
	case UI_SETUP(DEFAULT):
		/* tyhjennä näyttö */
		LCD_CLEAR;

		/* näytön staattinen teksti */
		lcd_put_P_const("MIN", 0, ALIGN_L);
		lcd_put_P_const("MAX", 0, ALIGN_R);

		UI_SETUP_END;
		break;

	case UI_LOOP(DEFAULT):
		/* lue uusi näyte? */
		ts_now = millis();
		if ((ts_now - ts_old) > (1000/SAMPLE_RATE)) {
			/* päivitä lämpötila */
			float T = temp_update();
		
			/* päivitä nähdyt maksimi ja minimi */
			if (unlikely(T > obs_max))
				obs_max = T;
			else if (unlikely(T < obs_min))
				obs_min = T;

			/* päivitä lämpötilat näytöllä */
			lcd_put_temp(T, 2, 6, 0, ALIGN_C);
			lcd_put_temp(obs_min, 1, 5, 1, ALIGN_L);
			lcd_put_temp(obs_max, 1, 5, 1, ALIGN_R);
			lcd_update();

			ts_old = ts_now;
		}

		/* valikossa */
		if (menu())
			break;

		/* pois valikosta */
		switch (button_update(&s)) {
		case BOTH|HOLD:
			is_locked = !is_locked;
			if (is_locked)
				lcd_put_P_const("LOCK", 1, ALIGN_C);
			else
				lcd_put_P_const("    ", 1, ALIGN_C);
			lcd_update();
			break;
		case BOTH|UP:
			if (is_locked)
				break;
			menu_return();
			break;
		}
		break;

	/* kalibrointinäkymä */
	case UI_SETUP(CALIBR):
		LCD_CLEAR;	/* Tässä sellainen vika, että jos käyttäjä
	 * ehtii rämpyttää nappeja tarpeeksi nopeasti
	 * valikko piirretään kaksi kertaa turhaan.
	 */

		lcd_put_P_const("CALIBR", 0, ALIGN_C);

		UI_SETUP_END;
		break;

	case UI_LOOP(CALIBR):
		/* valikossa */
		if (menu())
			break;

		/* pois valikosta */
		switch (button_update(&s)) {
		case BOTH|HOLD:
			ERROR(TEST);
			break;
		case BOTH|UP:
			menu_return();
			break;
		}
		break;

	/* asetusnäkymä 1 */
	case UI_SETUP(CONFIG1):
		LCD_CLEAR;

		lcd_put_P_const("CONFIG1", 0, ALIGN_C);

		UI_SETUP_END;
		break;

	case UI_LOOP(CONFIG1):
		/* valikossa */
		if (menu())
			break;

		/* pois valikosta */
		switch (button_update(&s)) {
		case BOTH|HOLD:
			ERROR(TEST);
			break;
		case BOTH|UP:
			menu_return();
			break;
		}
		break;

	/* asetusnäkymä 2 */
	case UI_SETUP(CONFIG2):
		LCD_CLEAR;

		lcd_put_P_const("CONFIG2", 0, ALIGN_C);

		UI_SETUP_END;
		break;
	
	case UI_LOOP(CONFIG2):
		/* valikossa */
		if (menu())
			break;

		/* pois valikosta */
		switch (button_update(&s)) {
		case BOTH|HOLD:
			ERROR(TEST);
			break;
		case BOTH|UP:
			menu_return();
			break;
		}
		break;
	
	case UI_SETUP(ERROR):
#define ERROR_SHOW 2000
#define ERROR_WAIT 1000
		LCD_CLEAR;

		/* tulosta virhe */
		lcd_put_P_const("VIRHE:", 0, ALIGN_L);
		lcd_put_uint(ERROR_CODE, 2, 0, ALIGN_R);
		lcd_put_P(ERROR_MSG, NULLTERM, 1, ALIGN_C);
		lcd_update();

		beep_begin();

		/* näytä virhe ERROR_SHOW millisekuntia */
		tmp = millis();
		while ((millis() - tmp) < ERROR_SHOW);

		beep_end();

		LCD_CLEAR;

		/* tulosta uudelleenkäynnistysviesti */
		lcd_put_P_const("REBOOT", 0, ALIGN_C);
		prog_init(200, 200);
		lcd_update();

		blink();

		ts_old = 0;
		tmp = millis();

		UI_SETUP_END;
		break;

	case UI_LOOP(ERROR):
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
}
