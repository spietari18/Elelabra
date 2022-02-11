#include "ui.h"
#include "alarm.h"
#include "error.h"
#include "button.h"
#include "temp_util.h"

static struct button_state s;

/* Valikon teksti. */
DEF_PSTR_PTR(TEMP_1, "TEMPERATURE");
DEF_PSTR_PTR(TEMP_2, "TEMP");
DEF_PSTR_PTR(CLBR_1, "CALIBRATION");
DEF_PSTR_PTR(CLBR_2, "CLBR");
DEF_PSTR_PTR(OPTS_1, "OPTIONS");
DEF_PSTR_PTR(OPTS_2, "OPTS");

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

static float T;

static uint16_t S;

static bool T_change;
static bool S_change;

static float T_obs_max = -INF;
static float T_obs_min =  INF;

static float T_lim_max =  20.0;
static float T_lim_min = -20.0;

static bool alarm_on;
static bool alarm_enabled = false;
static bool buzzer_enabled = true;
static bool flash_backlight = true;

void task_temperature()
{
	float old = T; 

	/* PNS sovitus tarvitsee vähintään 2 pistettä */
	if (unlikely(n_data_points < 2)) {
		/* tämän voi tarkastaa isfinite():llä */
		T = INF;

	} else {
		/* päivitä lämpötila */
		T = read_temp();

		/* päivitä nähdyt maksimi ja minimi */
		if (unlikely(T > T_obs_max))
			T_obs_max = T;
		else if (unlikely(T < T_obs_min))
			T_obs_min = T;
	}

	if (T != old)
		T_change = true;
}

void task_sample()
{
	uint16_t old = S;

	/* lue näyte AD-muuntimelta */
	S = adc_sample(ADC_CH0);

	if (S != old)
		S_change = true;
}

void task_alarm()
{
	if (!alarm_enabled) {
		if (unlikely(alarm_on))
			goto alarm_off;

		return;
	}

	if (unlikely((T < T_lim_min) || (T > T_lim_max))) {
		blink_begin();
		beep_begin();

		alarm_on = true;
	} else {
alarm_off:
		blink_end();
		beep_end();

		alarm_on = false;
	}
}

void task_voltage()
{
	if (unlikely(undervolt()))
		ERROR(TEST);
}

//uint8_t count;

//void test_task()
//{
//	lcd_put_uint(count++, 3, 0, CENTER);
//	lcd_update();
//}

/* globaalit taskit */
static struct interval_task global_tasks[] = {
	DEF_TASK(&task_voltage, 1000),
	//DEF_TASK(&test_task, 500)
};

/* nämä on pakko laittaa RAM muistiin, koska
 * interval_tasks() kirjoittaa structeihin
 */
#define DEF_TASKS(state) \
	static struct interval_task tasks_##state[]

#define REF_TASKS(state) \
	[state] = tasks_##state

#define CNT_TASKS(state) \
	ARRAY_SIZE(tasks_##state)

#define SAMPLE_INTERVAL (1000/SAMPLE_RATE)
#define ALARM_INTERVAL  1000

DEF_TASKS(UI_SPLH) = {};

DEF_TASKS(UI_MENU) = {
	DEF_TASK(&task_temperature, SAMPLE_INTERVAL),
	DEF_TASK(&task_alarm, ALARM_INTERVAL)
};

DEF_TASKS(UI_TEMP) = {
	DEF_TASK(&task_temperature, SAMPLE_INTERVAL),
	DEF_TASK(&task_alarm, ALARM_INTERVAL)
};

DEF_TASKS(UI_CLBR) = {
	DEF_TASK(&task_sample, SAMPLE_INTERVAL)
};

DEF_TASKS(UI_OPTS) = {
	DEF_TASK(&task_temperature, SAMPLE_INTERVAL),
	DEF_TASK(&task_alarm, ALARM_INTERVAL)
};

DEF_TASKS(UI_ERRR) = {};

static struct interval_task *const ui_tasks[UI_STATE_COUNT] PROGMEM = {
	REF_TASKS(UI_SPLH),
	REF_TASKS(UI_MENU),
	REF_TASKS(UI_TEMP),
	REF_TASKS(UI_CLBR),
	REF_TASKS(UI_OPTS),
	REF_TASKS(UI_ERRR)
};

static const uint8_t ui_task_count[UI_STATE_COUNT] PROGMEM = {
	CNT_TASKS(UI_SPLH),
	CNT_TASKS(UI_MENU),
	CNT_TASKS(UI_TEMP),
	CNT_TASKS(UI_CLBR),
	CNT_TASKS(UI_OPTS),
	CNT_TASKS(UI_ERRR)
};

static callback_t menu_loop;

static bool v_common_loop()
{
	if (T_change) {
		/* lämpötila näytölle */
		if (likely(isfinite(T)))
			lcd_put_temp(T, 2, 6, 0, CENTER);

		/* ei tarpeeksi pisteitä PNS sovitukseen */
		else
			lcd_put_P_const("PTS<2!", 0, CENTER);

		lcd_update();

		T_change = false;
		return false;
	}

	return true;
}

static void v_limit_loop()
{
	/* jos lämpötila ei muutu, niin luetut
	 * maksimi ja minimikään eivät muutu
	 */
	if (v_common_loop())
		return;

	lcd_put_temp(T_obs_min, 1, 5, 1, LEFT);
	lcd_put_temp(T_obs_max, 1, 5, 1, RIGHT);
	lcd_update();
}

static void v_limit_init()
{
	LCD_CLEAR;

	/* staattinen teksti */
	lcd_put_P_const("MIN", 0, LEFT);
	lcd_put_P_const("MAX", 0, RIGHT);

	menu_loop = &v_limit_loop;

	T_change = true;
}

static void v_alarm_init()
{
	LCD_CLEAR;

	/* staattinen teksti, rajat eivät
	 * voi muuttua TEMP näkymässä
	 */
	lcd_put_P_const("AL-", 0, LEFT);
	lcd_put_P_const("AL+", 0, RIGHT);
	lcd_put_temp(T_lim_min, 1, 5, 1, LEFT);
	lcd_put_temp(T_lim_max, 1, 5, 1, RIGHT);

	menu_loop = &v_common_loop;

	T_change = true;
}

static void v_menu_loop() { menu_loop(); }

static void a_create()
{
	float temp;

	if (n_data_points >= MAX_POINTS) {
		// virhe
		return;
	}

	select_float_P_const("TEMPERATURE", &temp, T_ABS_MIN, T_ABS_MAX, 0.01);	

	data_points[n_data_points][0]   = S;
	data_points[n_data_points++][1] = temp;
}

static void a_delete()
{
	uint16_t point;

	select_uint_P_const("SELECT POINT", &point, 1, n_data_points, 1);

	if (point == n_data_points)
		n_data_points--;
	else
		(void)memcpy(&data_points[point - 1],
				&data_points[point], n_data_points - point);
}

static void a_edit()
{
	float (*target)[2];
	float temp;
	uint16_t sample;

	select_uint_P_const("SELECT POINT", &sample, 1, n_data_points, 1);

	target = &data_points[sample];

	select_uint_P_const("SET SAMPLE", &sample, 0, (1 << 12) - 1, 1);
	select_float_P_const("SET TEMP", &temp, T_ABS_MIN, T_ABS_MAX, 0.01);
	
	*target[0] = sample;
	*target[1] = temp;
}

static void a_empty()
{
	if (yesno_P_const("CLEAR POINTS"))
		/* näitä ei tarvitse tämän kummemmin poistaa */
		n_data_points = 0;
}

static void a_restore()
{
	if (yesno_P_const("RESTORE BUILTINS"))
		/* ROM:issa olevat PNS pisteet. */
		default_points();
}

static void o_alarm_high()
{
	select_float_P_const("ALARM HIGH", &T_lim_max, T_lim_min, 30.0, 0.1);
}

static void o_alarm_low()
{
	select_float_P_const("ALARM LOW", &T_lim_min, -30.0, T_lim_max, 0.1);
}

static void o_alarm_enable()
{
	select_bool_P_const("ENABLE ALARM", &alarm_enabled);
}

static void o_buzzer()
{
	// ei tee mitään
	select_bool_P_const("ENABLE BUZZER", &buzzer_enabled);
}

static void o_backlight()
{
	// ei tee mitään
	select_bool_P_const("FLASH BACKLIGHT", &flash_backlight);
}

static void menu_enter_and_commit()
{
	/* laske PNS kertoimet uudelleen */
	compute_lss_coefs();

	// tässä välissä uudet pisteet voi
	// kirjoittaa EERAM:iin
	
	/* valikkoon */
	menu_enter();
}

DEF_PSTR_PTR(CREA, "CREATE");
DEF_PSTR_PTR(DELE, "DELETE");
DEF_PSTR_PTR(EDIT, "MODIFY");
DEF_PSTR_PTR(EMPT, "REMALL");
DEF_PSTR_PTR(REST, "BLTINS");
DEF_PSTR_PTR(AHGH, "ALARM+");
DEF_PSTR_PTR(ALOW, "ALARM-");
DEF_PSTR_PTR(ALEN, "ALRMEN");
DEF_PSTR_PTR(BEEP, "BUZZER");
DEF_PSTR_PTR(BKLT, "BACKLT");
DEF_PSTR_PTR(MENU, " MENU ");
DEF_PSTR_PTR(LIMT, " LIMT ");
DEF_PSTR_PTR(ALRM, " ALRM ");

DEF_SUBMENU(views) = {
	SUBMENU_ENTRY(REF_PSTR_PTR(LIMT), &v_limit_init, &v_limit_loop, NULL),
	SUBMENU_ENTRY(REF_PSTR_PTR(ALRM), &v_alarm_init, &v_common_loop, NULL),
	SUBMENU_ENTRY(REF_PSTR_PTR(MENU), NULL, &v_menu_loop, &menu_enter)
};

DEF_SUBMENU(actions) = {
	SUBMENU_ENTRY(REF_PSTR_PTR(CREA), NULL, NULL, &a_create),
	SUBMENU_ENTRY(REF_PSTR_PTR(DELE), NULL, NULL, &a_delete),
	SUBMENU_ENTRY(REF_PSTR_PTR(EDIT), NULL, NULL, &a_edit),
	SUBMENU_ENTRY(REF_PSTR_PTR(EMPT), NULL, NULL, &a_empty),
	SUBMENU_ENTRY(REF_PSTR_PTR(REST), NULL, NULL, &a_restore),
	SUBMENU_ENTRY(REF_PSTR_PTR(MENU), NULL, NULL, &menu_enter_and_commit),
};

DEF_SUBMENU(options) = {
	SUBMENU_ENTRY(REF_PSTR_PTR(AHGH), NULL, NULL, &o_alarm_high),
	SUBMENU_ENTRY(REF_PSTR_PTR(ALOW), NULL, NULL, &o_alarm_low),
	SUBMENU_ENTRY(REF_PSTR_PTR(ALEN), NULL, NULL, &o_alarm_enable),
	SUBMENU_ENTRY(REF_PSTR_PTR(BEEP), NULL, NULL, &o_buzzer),
	SUBMENU_ENTRY(REF_PSTR_PTR(BKLT), NULL, NULL, &o_backlight),
	SUBMENU_ENTRY(REF_PSTR_PTR(MENU), NULL, NULL, &menu_enter)
};

/* noreturn ei toimi tässä, mutta viimeistään
 * linkkeri optimoi paluukoodin pois
 */
int main()
{
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
	/* valikkotilakohtaiset taskit */
	interval_tasks(
		pgm_read_ptr(&ui_tasks[UI_GET_STATE & UI_MASK_NOW]),
		pgm_read_byte(&ui_task_count[UI_GET_STATE & UI_MASK_NOW]));

	/* globaalit taskit */
	INTERVAL_TASKS(global_tasks);

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

		SUBMENU_INIT(views);

		UI_SETUP_END;
		break;

	case UI_LOOP(TEMP):
		/* jos näppäimet ovat lukossa, pitää kutsua
		 * loop takaisunkutsua manuaalisesti, koska
		 * SUBMENU_POLL() katsoo näppäinten tilan.
		 */
		if (is_locked)
			submenu_docb(views, SM_LOOP);
		else
			SUBMENU_POLL(views);

		switch (button_update(&s)) {
		/* näppäinten lukitus */
		case BOTH|HOLD:
			if ((is_locked = !is_locked))
				lcd_put_P_const("LOCK", 1, CENTER);
			else
				submenu_text(views);
			lcd_update();

			beep_slow();
			break;
		}
		break;

	case UI_SETUP(CLBR):
		LCD_CLEAR;

		S_change = true;

		SUBMENU_INIT(actions);

		UI_SETUP_END;
		break;

	case UI_LOOP(CLBR):
		if (unlikely(S_change)) {
			lcd_put_uint(S, 4, 0, CENTER);
			lcd_update();

			S_change = false;
		}

		SUBMENU_POLL(actions);
		break;

	case UI_SETUP(OPTS):
		LCD_CLEAR;

		SUBMENU_INIT(options);
		
		UI_SETUP_END;
		break;

	case UI_LOOP(OPTS):
		SUBMENU_POLL(options);
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
