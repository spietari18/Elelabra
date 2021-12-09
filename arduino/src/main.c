#include "ui.h"
#include "util.h"
#include "error.h"
#include "screen.h"
#include "button.h"
#include "temp_util.h"

#include <Arduino.h>
#include <setjmp.h>

jmp_buf main_loop;

void __attribute__((noreturn))
entry_point()
{
	uint32_t ts_now, ts_old, tmp;
	uint8_t code;

	/* lämpömittarin näkemä minimi- ja maksimilämpötila */
	float obs_min =  1.0/0.0, obs_max = -1.0/0.0;

	/* aseta käytetyt moduulit päälle ja muut pois päältä */
	PRR = ~0;
	CLR(PRR, PRTIM0); // TIMER0 päälle
	//CLR(PRR, PRTIM2); // TIMER2 päälle
	CLR(PRR, PRSPI);  // SPI päälle

	/* alusta kaikki IO pinnit INPUT PULLUP tilaan
	 * jotta käyttämättömät pinnit eivät kellu.
	 * aseta myös kaikkien pinnien arvoksi LOW
	 */
	DDRB  = DDRC  = DDRD  =  0; // INPUT
	PORTB = PORTC = PORTD = ~0; // PULLUP
	PINB  = PINC  = PIND  =  0; // LOW

	/* alusta SPI (tarpeellinen tehdä vain kerran koska
	 * AD muunnin on ainoa kytketty SPI laite)
	 */
	SPCR = 0;
	SET(SPCR, SPE);
	SET(SPCR, MSTR);
	SET(SPCR, CPOL);
	SET(SPCR, SPR0);
	WRITE(SPI_SS, HIGH);
	MODE(SPI_SS, OUTPUT);
	MODE(SPI_CLK, OUTPUT);
	MODE(SPI_MOSI, OUTPUT);
	PLUP(SPI_MISO, 0);

	/* alusta näyttö (LiquidCrystal kutsuu
	 * pinMode():a vasta tässä vaiheessa)
	 */
	lcd_init();

	/* alusta muut pinnit, napeille voidaan käyttää
	 * sisäisiä ylosvetovastuksia joten niiden
	 * modeihin ei tarvitse puuttua.
	 */
	WRITE(LCD_AN, HIGH);
	MODE(LCD_AN, OUTPUT);

	//tone(BUZZER, 1000);

	UI_SET_STATE(SPLASH);

	/* alusta PNS malli */
	default_points();
	compute_lss_coefs();

	/* paluu keskeytyksistä tähän */
	code = setjmp(main_loop);
main_loop:

	switch (UI_GET_STATE) {
	case UI_SETUP(SPLASH):

#define SPLASH_1 "L\xE1mp\xEFmittari"
#define SPLASH_2 "v. 1.0"
#define SPLASH_WAIT 1000 // [ms]

		/* splash */
		lcd_put_P_const(SPLASH_1, 0, ALIGN_C);
		lcd_put_P_const(SPLASH_2, 0, ALIGN_C);
		lcd_update();

		/* näytä alarivi SPLASH_WAIT millisekuntia */
		ts_old = millis();
		while ((millis() - ts_old) < SPLASH_WAIT);

		/* alusta SPLASH LOOP tila */
		tmp = 0;
		ts_old = 0;
		prog_init(MAX_SAMPLES, 0);

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

		/* menu_update() asettaa UI tilan DEFAULT
		 * ensimmäisellä kutsuntakerralla
		 */
		if (tmp >= MAX_SAMPLES)
			menu_update();

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

		menu_update();
		break;

	case UI_SETUP(CALIBR):
		LCD_CLEAR;
		lcd_put_P_const("CALIBR", 0, ALIGN_C);
		lcd_update();
		UI_SETUP_END;
		break;

	case UI_LOOP(CALIBR):
		menu_update();
		break;
	
	case UI_SETUP(CONFIG1):
		LCD_CLEAR;
		lcd_put_P_const("CONFIG1", 0, ALIGN_C);
		lcd_update();
		UI_SETUP_END;
		break;

	case UI_LOOP(CONFIG1):
		menu_update();
		break;
	
	case UI_SETUP(CONFIG2):
		LCD_CLEAR;
		lcd_put_P_const("CONFIG2", 0, ALIGN_C);
		lcd_update();
		UI_SETUP_END;
		break;
	
	case UI_LOOP(CONFIG2):
		menu_update();
		break;
	
	case UI_SETUP(ERROR):
#define ERROR_WAIT 2000
		LCD_CLEAR_ALL;

		lcd_put_P_const("ERROR", 0, ALIGN_C);
		lcd_put_P(errstr[code], NULLTERM, 1, ALIGN_C);
		ts_old = millis();

		UI_SETUP_END;
		break;

	case UI_LOOP(ERROR):
		ts_now = millis();
		if ((ts_now - ts_old) > ERROR_WAIT)
			reset();
		break;

	default:
		UI_SET_STATE(ERROR);
	}

	goto main_loop;
}
