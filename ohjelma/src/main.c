#include "ui.h"
#include "alarm.h"
#include "error.h"
#include "button.h"
#include "temp_util.h"

#include <stdio.h>

#define OPTS_ADDR EERAM_MIN_ADDR
#define SPTS_ADDR (EERAM_MAX_ADDR - (1 + sizeof(data_points)))

/* asetukset */
struct {
	float alarm_low;
	float alarm_high;

	bool alarm_enabled;
	bool buzzer_enabled;
	bool flash_backlight;

/* tähän oletukset, kun EERAM ei toimi */
} opts = {
	-20.0, 20.0,
	false, true, true
};

bool get_options()
{
	uint8_t buf[sizeof(opts)];
	bool ret;

	if ((ret = eeram_read(OPTS_ADDR, buf, sizeof(buf))))
		(void)memcpy(&opts, buf, sizeof(buf));

	return !ret;
}

bool put_options()
{
	return !eeram_write(OPTS_ADDR, (uint8_t *)&opts, sizeof(opts));
}

bool get_data_points()
{
	if (!eeram_read(SPTS_ADDR, &n_data_points, 1)
		|| (n_data_points < MAX_POINTS))
		return true;

	/* tämä käyttää 47C16:n sisäistä muistiosoitinta */
	if (!eeram_read(EERAM_ADDR,
		(uint8_t *)data_points,
			n_data_points*sizeof(*data_points))) {
		default_points();
		return true;
	}

	compute_lss_coefs();

	return false;
}

bool put_data_points()
{
	return !(eeram_write(SPTS_ADDR, &n_data_points, 1)
		&& eeram_write(SPTS_ADDR + 1,
			(uint8_t *)data_points,
				n_data_points*sizeof(*data_points)));
}

/* main.c:n lokaali nappitila */
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

static bool alarm_on;

void task_temperature()
{
	float old_T = T; 
	uint16_t old_S = S;

	/* PNS sovitus tarvitsee vähintään 2 pistettä */
	if (unlikely(n_data_points < 2)) {
		/* tämän voi tarkastaa isfinite():llä */
		S = read_sample();
		T = INF;

	} else {
		/* päivitä lämpötila ja näyte */
		S = read_sample();
		T = calc_temp(S);

		/* päivitä nähdyt maksimi ja minimi */
		if (unlikely(T > T_obs_max))
			T_obs_max = T;
		else if (unlikely(T < T_obs_min))
			T_obs_min = T;
	}

	if (unlikely(T != old_T))
		T_change = true;
	
	if (unlikely(S != old_S))
		S_change = true;
}

void task_sample()
{
	uint16_t old = S;

	/* lue näyte AD-muuntimelta */
	S = read_sample();

	if (S != old)
		S_change = true;
}

void task_alarm()
{
	if (!opts.alarm_enabled) {
		if (unlikely(alarm_on))
			goto alarm_off;

		return;
	}

	if (unlikely(((T < opts.alarm_low) ||
		(T > opts.alarm_high)) && isfinite(T))) {
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

bool undervolt_recover()
{
	/* tässä kannattaisi ehkä mieluummin käynnistää koko
	 * laite uudestaan reboot() komennolla, koska sisäinen
	 * tila voi olla korruptoitunut alhaisesta jännitteestä
	 */ 
	_delay_ms(1000);
	return undervolt();
}

void task_voltage()
{
	if (unlikely(undervolt()))
		error("LOW VOLTAGE", &undervolt_recover);
}

/* globaalit taskit */
static struct interval_task global_tasks[] = {
	DEF_TASK(&task_voltage, 1000)
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

static struct interval_task *const ui_tasks[UI_STATE_COUNT] PROGMEM = {
	REF_TASKS(UI_SPLH),
	REF_TASKS(UI_MENU),
	REF_TASKS(UI_TEMP),
	REF_TASKS(UI_CLBR),
	REF_TASKS(UI_OPTS)
};

static const uint8_t ui_task_count[UI_STATE_COUNT] PROGMEM = {
	CNT_TASKS(UI_SPLH),
	CNT_TASKS(UI_MENU),
	CNT_TASKS(UI_TEMP),
	CNT_TASKS(UI_CLBR),
	CNT_TASKS(UI_OPTS)
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
	lcd_put_temp(opts.alarm_low, 1, 5, 1, LEFT);
	lcd_put_temp(opts.alarm_high, 1, 5, 1, RIGHT);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
	menu_loop = &v_common_loop;
#pragma GCC diagnostic pop

	T_change = true;
}

static void v_menu_loop() { menu_loop(); }

static float *select_point()
{
	float *target = NULL;
	uint16_t point;
	char msg[13]; // sprintf() kirjoittaa nollaterminaattorin

	if (n_data_points < 1) {
		msg_P_const("CAN'T SELECT\nNO POINTS");
		goto exit;
	}

	(void)snprintf_P(msg, sizeof(msg),
		PSTR("WHICH (1-%-2u)"), n_data_points);
	
	point = 0;
	select_uint_const(msg, &point, 0, n_data_points, 1);

	if (point > 0)
		target = (float *)&data_points[point - 1];
exit:
	return target;
}

static void a_create()
{
	float *target;
	uint16_t tmp;

	if (n_data_points >= MAX_POINTS) {
		msg_P_const("CAN'T CREATE\nPOINTS FULL");
		return;
	}

	if (!yesno_P_const("CREATE POINT?"))
		return;

	target = (float *)&data_points[n_data_points++];

	/* käytä nykyisiä arvoja alkuarvoina */
	target[0] = S;
	target[1] = T;

	select_float_P_const("TEMPERATURE",
		&target[1], T_ABS_MIN, T_ABS_MAX, 0.01);	
	tmp = target[0];
	select_uint_P_const("SAMPLE",
		&tmp, S_ABS_MIN, S_ABS_MAX, 1);
	target[0] = tmp;

	compute_lss_coefs();
}

static void a_delete()
{
	float *target;
	uint8_t index;

	if (!(target = select_point()))
		return;

	index = target - (float *)data_points;

	if (index == (n_data_points - 1)) {
		n_data_points--;
	} else {
		(void)memcpy(target, target + 1,
			(n_data_points - index)*sizeof(*data_points));
	}

	compute_lss_coefs();
}

static void a_edit()
{
	float *target;
	uint16_t sample;

	if (!(target = select_point()))
		return;

	select_float_P_const("TEMPERATURE",
		&target[1], T_ABS_MIN, T_ABS_MAX, 0.01);
	sample = target[0];
	select_uint_P_const("SAMPLE",
		&sample, S_ABS_MIN, S_ABS_MAX, 1);
	target[0] = sample;

	compute_lss_coefs();
}

static void a_show()
{
	float *target;
	char buf[LCD_ROWS*LCD_COLS];

	if (!(target = select_point()))
		return;
	
	(void)memcpy(buf, lcd_buffer, LCD_ROWS*LCD_COLS);
	LCD_CLEAR;

	/* tulosta lämpötila ja näyte */
	lcd_put_P_const("SAMP ->", 0, LEFT);
	lcd_put_P_const("TEMP ->", 1, LEFT);
	lcd_put_uint(target[0], 4, 0, RIGHT);
	lcd_put_temp(target[1], 2, 6, 1, RIGHT);

	lcd_update();

	user_wait();

	(void)memcpy(lcd_buffer, buf, LCD_ROWS*LCD_COLS);
	lcd_update();
}

static void a_empty()
{
	char msg[11]; // snprintf() kirjoittaa nollaterminaattorin

	if (n_data_points < 1) {
		msg_P_const("CAN'T CLEAR\nNO POINTS");
		return;
	}

	(void)snprintf_P(msg, sizeof(msg), PSTR("CLEAR (%2u)"), n_data_points);

	if (yesno_const(msg))
		/* näitä ei tarvitse tämän kummemmin poistaa */
		n_data_points = 0;

	compute_lss_coefs();
}

static void a_restore()
{
	if (yesno_P_const("RESTORE BUILTINS"))
		/* ROM:issa olevat PNS pisteet. */
		default_points();
}

static void o_alarm_high()
{
	select_float_P_const("ALARM HIGH",
		&opts.alarm_high, opts.alarm_low, T_ABS_MAX, 0.1);
}

static void o_alarm_low()
{
	select_float_P_const("ALARM LOW",
		&opts.alarm_low, T_ABS_MIN, opts.alarm_high, 0.1);
}

static void o_alarm_enable()
{
	select_bool_P_const("ENABLE ALARM", &opts.alarm_enabled);
}

static void o_buzzer()
{
	/* jos flash_backlight on false, tätä ei voi poistaa päältä */
	if (!opts.flash_backlight) {
		msg_P_const("CAN'T DISABLE\nBLIGHT DISABLED");
		return;
	}

	select_bool_P_const("ENABLE BUZZER", &opts.buzzer_enabled);
}

static void o_backlight()
{
	/* jos buzzer_enabled on false, tätä ei voi poistaa päältä */
	if (!opts.buzzer_enabled) {
		msg_P_const("CAN'T DISABLE\nBUZZER DISABLED");
		return;
	}

	select_bool_P_const("FLASH BACKLIGHT", &opts.flash_backlight);
}

static void o_memtest()
{
	uint8_t screen[LCD_ROWS*LCD_COLS];
	uint8_t buf[32];
	uint8_t read;
	uint16_t addr;

	if (!yesno_P_const("MEMTEST?"))
		return;
	
	(void)memcpy(screen, lcd_buffer, LCD_ROWS*LCD_COLS);
	LCD_CLEAR;

	addr = EERAM_MIN_ADDR;
	while (addr < EERAM_MAX_ADDR)
	{
		read = MIN(sizeof(buf), EERAM_MAX_ADDR - addr);

		lcd_put_P_const("           ", 0, RIGHT);
		lcd_put_fmt(11, 0, RIGHT, "%u-%u", addr, addr + read);

		lcd_put_P_const("READ ", 0, LEFT);
		lcd_put_P_const("....", 1, CENTER);
		lcd_update();

		if (!eeram_read(addr, buf, sizeof(buf))) {
			lcd_put_P_const("FAIL", 1, CENTER);
			lcd_update();

			_delay_ms(1000);

			break;
		}

		lcd_put_P_const(" OK ", 1, CENTER);
		lcd_update();

		_delay_ms(500);

		lcd_put_P_const("WRITE", 0, LEFT);
		lcd_put_P_const("....", 1, CENTER);
		lcd_update();

		if (!eeram_write(addr, buf, sizeof(buf))) {
			lcd_put_P_const("FAIL", 1, CENTER);
			lcd_update();

			_delay_ms(1000);

			break;
		}

		lcd_put_P_const(" OK ", 1, CENTER);
		lcd_update();

		_delay_ms(500);

		addr += read;
	}

	(void)memcpy(lcd_buffer, screen, LCD_ROWS*LCD_COLS);
	lcd_update();
}

static void menu_commit_acts()
{
	if (put_data_points())
		msg_P_const("SAVE DATE FAIL\nNOT SAVED");
	menu_enter();
}

static void menu_commit_opts()
{
	if (put_options())
		msg_P_const("SAVE OPTS FAIL\nNOT SAVED");
	menu_enter();
}

DEF_PSTR_PTR(CREA, "CREATE");
DEF_PSTR_PTR(DELE, "DELETE");
DEF_PSTR_PTR(EDIT, "MODIFY");
DEF_PSTR_PTR(SHOW, " SHOW ");
DEF_PSTR_PTR(EMPT, "REMALL");
DEF_PSTR_PTR(REST, "BLTINS");
DEF_PSTR_PTR(AHGH, "ALARM+");
DEF_PSTR_PTR(ALOW, "ALARM-");
DEF_PSTR_PTR(ALEN, "ALRMEN");
DEF_PSTR_PTR(BEEP, "BUZZER");
DEF_PSTR_PTR(BKLT, "BACKLT");
DEF_PSTR_PTR(MTST, "MEMTST");
DEF_PSTR_PTR(MENU, " MENU ");
DEF_PSTR_PTR(LIMT, " LIMT ");
DEF_PSTR_PTR(ALRM, " ALRM ");

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
DEF_SUBMENU(views) = {
	SUBMENU_ENTRY(REF_PSTR_PTR(LIMT), &v_limit_init, &v_limit_loop, NULL),
	SUBMENU_ENTRY(REF_PSTR_PTR(ALRM), &v_alarm_init, &v_common_loop, NULL),
	SUBMENU_ENTRY(REF_PSTR_PTR(MENU), NULL, &v_menu_loop, &menu_enter)
};

DEF_SUBMENU(actions) = {
	SUBMENU_ENTRY(REF_PSTR_PTR(CREA), NULL, NULL, &a_create),
	SUBMENU_ENTRY(REF_PSTR_PTR(DELE), NULL, NULL, &a_delete),
	SUBMENU_ENTRY(REF_PSTR_PTR(EDIT), NULL, NULL, &a_edit),
	SUBMENU_ENTRY(REF_PSTR_PTR(SHOW), NULL, NULL, &a_show),
	SUBMENU_ENTRY(REF_PSTR_PTR(EMPT), NULL, NULL, &a_empty),
	SUBMENU_ENTRY(REF_PSTR_PTR(REST), NULL, NULL, &a_restore),
	SUBMENU_ENTRY(REF_PSTR_PTR(MENU), NULL, NULL, &menu_commit_acts),
};

DEF_SUBMENU(options) = {
	SUBMENU_ENTRY(REF_PSTR_PTR(AHGH), NULL, NULL, &o_alarm_high),
	SUBMENU_ENTRY(REF_PSTR_PTR(ALOW), NULL, NULL, &o_alarm_low),
	SUBMENU_ENTRY(REF_PSTR_PTR(ALEN), NULL, NULL, &o_alarm_enable),
	SUBMENU_ENTRY(REF_PSTR_PTR(BEEP), NULL, NULL, &o_buzzer),
	SUBMENU_ENTRY(REF_PSTR_PTR(BKLT), NULL, NULL, &o_backlight),
	SUBMENU_ENTRY(REF_PSTR_PTR(MTST), NULL, NULL, &o_memtest),
	SUBMENU_ENTRY(REF_PSTR_PTR(MENU), NULL, NULL, &menu_commit_opts)
};
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclobbered"

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

	/* Lue asetukset ja datapisteet EERAM:ista */
	if (get_options())
		msg_P_const("READ OPTS FAIL\nFALLBACK TO DEFS");
	if (get_data_points())
		msg_P_const("READ DATA FAIL\nFALLBACK TO DEFS");
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

		SUBMENU_INIT(views, SM_NONE);

		UI_SETUP_END;
		break;

	case UI_LOOP(TEMP):
		/* jos näppäimet ovat lukossa, pitää kutsua
		 * loop takaisunkutsua manuaalisesti, koska
		 * submenu_poll() katsoo näppäinten tilan.
		 */
		if (is_locked)
			submenu_docb(SM_LOOP);
		else
			submenu_poll();

		switch (button_update(&s)) {
		/* näppäinten lukitus */
		case BOTH|HOLD:
			if ((is_locked = !is_locked))
				lcd_put_P_const("LOCK", 1, CENTER);
			else
				submenu_text();
			lcd_update();

			beep_slow();
			break;
		}
		break;

	case UI_SETUP(CLBR):
		LCD_CLEAR;

		/* mahdollinen hälytys pois päältä task_alarm()
		 * laittaa sen takaisin jos tarvitsee kun tästä
		 * näkymästä poistutaan
		 */
		blink_end();
		beep_end();

		S_change = true;

		SUBMENU_INIT(actions, SM_1ROW);

		lcd_put_P_const("->", 0, CENTER);

		UI_SETUP_END;
		break;

	case UI_LOOP(CLBR):
		if (unlikely(S_change)) {
			(void)memset(&lcd_buffer[0][0], ' ', 4);
			lcd_put_uint(S, 4, 0, LEFT);
			lcd_put_temp(T, 2, 6, 0, RIGHT);
			lcd_update();

			S_change = false;
		}

		submenu_poll();
		break;

	case UI_SETUP(OPTS):
		LCD_CLEAR;

		SUBMENU_INIT(options, SM_2ROW);
		
		UI_SETUP_END;
		break;

	case UI_LOOP(OPTS):
		submenu_poll();
		break;

	default:
		unreachable;
	}

	/* pääsilmukka on toteutettu goto komennolla, jotta säästytään
	 * turhalta sisennykseltä, eikä silmukasta voi vahingossa poistua.
	 * (tosin -mendup-at=main ei anna pääohjelman palata)
	 */
	goto main_loop;

	unreachable;
}

#pragma GCC diagnostic pop
