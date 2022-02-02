#include "ui.h"
#include "alarm.h"
#include "button.h"

/* Käyttöliittymän tilakone. */
uint8_t ui_state;

/* Valikon tila. */
static struct button_state s;
static uint8_t entry_now;
static uint8_t entry_old;
static uint8_t entry_last = ~0;
bool in_menu;

/* Piirrä valikko. */
void menu_draw()
{
	char *dst = &lcd_buffer[1][1];
	uint8_t i = 0;

	LCD_CLEAR;
	
	/* Kirjoita otsikko. */
	lcd_put_P(pgm_read_ptr(&menu_config[entry_now]
		.text_title), NULLTERM, 0, CENTER);
	
	/* Kirjoita napit. */
	for (uint8_t j = 0; j < MENU_ENTRIES; j++)
	{
		if (unlikely(j == entry_now)) {
			uint8_t tmp;

			dst[i++] = '>';
			(void)memset(&dst[i], ' ', EXPANDED_LEN);
			tmp = strnlen_P(pgm_read_ptr(
				&menu_config[j].text_menu), EXPANDED_LEN);
			(void)memcpy_P(&dst[i + (EXPANDED_LEN- tmp)/2],
				pgm_read_ptr(&menu_config[j].text_menu), tmp);
			i += EXPANDED_LEN;
			dst[i++] = '<';
		} else {
			dst[i++] = '<';
			dst[i++] = pgm_read_byte(pgm_read_ptr(
				&menu_config[j].text_menu));
			dst[i++] = '>';
		}

		dst[i++] = ' ';
	}

	lcd_update();
}

/* Päivitä valikko. */
void menu_update()
{
	switch (button_update(&s)) {
	/* valikossa eteen päin */
	case RT|UP:
		beep_fast();
		INC_MOD(entry_now, MENU_ENTRIES);
		break;
	
	/* valikossa taakse piän */
	case LT|UP:
		beep_fast();
		DEC_MOD(entry_now, MENU_ENTRIES);
		break;
	
	/* poistu valikosta */
	case BOTH|UP:
		beep_slow();

		/* siirtymisen takaisinkutsu */
		//CALLBACK((callback_t)pgm_read_ptr(
		//	&menu_config[entry_now].enter));

		/* aseta tila */
		__UI_SET_STATE(pgm_read_word(
			&menu_config[entry_now].state));
		in_menu = false;

		return;
	}

	/* jos mikään ei muutu älä päivitä */
	if (likely(entry_old == entry_now))
		return;

	menu_draw();

	/* taustatilan päivitystakaisinkutsu */
	//CALLBACK((callback_t)pgm_read_ptr(
	//	&menu_config[entry_now].update));

	entry_old = entry_now;
}

/* Siirry valikkoon. */
void menu_enter()
{
	/* poistumisen takaisinkutsu */
	//if (likely(entry_old != (uint8_t)~0))
	//	CALLBACK((callback_t)pgm_read_ptr(
	//		&menu_config[entry_last].exit));

	beep_slow();

	/* aseta tila */
	in_menu = true;
	UI_SET_STATE(MENU);
}

#define PROG_CHARS \
	(sizeof(PROG_INIT) - 3)

/* Edistymispalkin paikka. */
uint8_t prog_pos;

/* Sisäinen tila. */
static uint8_t old;
static uint8_t num;

/* Mistä kohtaa puskurissa edistymispalkki alkaa. */
static char* const dst = &lcd_buffer[PROG_ROW][(LCD_COLS - PROG_CHARS)/2];

static const char prg[4] PROGMEM = {'-', '=', ' ', '-'};

/* Alusta edistymispalkki. (määrittää "määrän"
 * jota prog_dec() ja prog_inc() käsittelevät)
 */
void prog_init(uint8_t n, uint8_t i)
{
	uint8_t j, k;

	/* päivitä globaali tila */
	num = n;
	old = i - 1;
	prog_pos = i - 1;

	/* kopioi palkin alkutila */
	lcd_put_P_const(PROG_INIT, PROG_ROW, CENTER);

	/* missä kohtaa palkkia mennään */
	j = INT_DIV_RND((2*PROG_CHARS - 1)*i, n);

	/* j == 0 tarkoittaa, että palkki on tyhjä */
	if (j == 0)
		return;
	
	/* täytä palkki */
	k = j/2;
	dst[k] = pgm_read_byte(&prg[j & 1]);
	while (k--)
		dst[k] = pgm_read_byte(&prg[1]);
}

/* Pienennä edistymispalkkia yhdellä. */
void prog_dec()
{
	uint8_t i, j, k;

	if (old == 0)
		return;

	/* askeleen alku- ja loppuindeksi */
	i = INT_DIV_RND((2*PROG_CHARS - 1)*old, num)/2;
	old = prog_pos;
	j = INT_DIV_RND((2*PROG_CHARS - 1)*prog_pos, num);
	prog_pos--;

	/* täytä askeleen pää */
	k = j/2;
	dst[k] = pgm_read_byte(&prg[(j & 1) + 2]);

	/* onko tyhjennys tarpeellinen? */
	if ((i - k) < 1)
		return;
	i--;

	/* tyhjennä skipatut kirjaimet */
	while (k < i)
		dst[k++] = ' ';
}

/* Suurenna edistymispalkkia yhdellä. */
void prog_inc()
{
	uint8_t i, j, k;

	if (prog_pos >= (num - 1)) {
		prog_pos = num - 1;
		return;
	}

	/* askeleen alku- ja loppuindeksi */
	i = INT_DIV_RND((2*PROG_CHARS - 1)*old, num)/2;
	old = prog_pos;
	j = INT_DIV_RND((2*PROG_CHARS - 1)*prog_pos, num);
	prog_pos++;

	/* täytä askeleen pää */
	k = j/2;
	dst[k] = pgm_read_byte(&prg[j & 1]);
	
	/* onko täyttö tarpeellinen? */
	if ((k - i) < 1)
		return;
	k--;

	/* täytä skipatut kirjaimet */
	while (i < k)
		dst[i++] = pgm_read_byte(&prg[1]);
}

static uint8_t submenu_now;
static uint8_t submenu_old;

void submenu_init(const __unused struct submenu_entry *entries,
	__unused uint8_t n_entries)
{
	submenu_now =  0;
	submenu_old = ~0;
}

void submenu_poll(const struct submenu_entry *entries, uint8_t n_entries)
{
	switch (button_update(&s)) {
	case RT|UP:
		beep_fast();
		INC_MOD(submenu_now, n_entries);
		break;
	
	case LT|UP:
		beep_fast();
		DEC_MOD(submenu_now, n_entries);
		break;
	
	case BOTH|UP:
		beep_slow();
		CALLBACK(pgm_read_ptr(&entries[submenu_now].click));
		return;
	}

	if (unlikely(submenu_now != submenu_old)) {
		CALLBACK(pgm_read_ptr(&entries[submenu_now].init));

		lcd_put_P(pgm_read_ptr(
			&entries[submenu_now].name), NULLTERM, 1, CENTER);

		lcd_update();

		submenu_old = submenu_now;
	}

	CALLBACK(pgm_read_ptr(&entries[submenu_now].loop));
}

void submenu_docb(const struct submenu_entry *entries, uint8_t cb)
{
	CALLBACK(pgm_read_ptr(&((callback_t *)&entries[entry_now].init)[cb]));
}

void submenu_text(const struct submenu_entry *entries)
{
	lcd_put_P(pgm_read_ptr(
			&entries[submenu_now].name), NULLTERM, 1, CENTER);
}

typedef bool (*click_t)(void *);
typedef void (*hold_t)(void *);

#define US_LT (1 << 0)
#define US_RT (1 << 1)
#define US_BT (1 << 2)

static void user_select(void *target, const char *msg, const char *left,
	const char *right, hold_t value, click_t click_left, click_t click_right,
	click_t click_both, hold_t hold_left, hold_t hold_right)
{
	char tmp[LCD_ROWS][LCD_COLS];
	uint8_t hold = 0;

	/* tallenna vanha näyttö */
	(void)memcpy(tmp, lcd_buffer, LCD_ROWS*LCD_COLS);
	LCD_CLEAR;
	
	/* näppäimet ja otsikko */
	lcd_put_P(msg, NULLTERM, 0, CENTER);
	lcd_put_P(left, NULLTERM, 1, LEFT);
	lcd_put_P(right, NULLTERM, 1, RIGHT);

	/* muuttujan alkuarvo */
	if (value)
		value(target);

	lcd_update();

	while (1)
	{
		if (unlikely(hold_left && GET(hold, US_LT)))
			hold_left(target);

		else if (unlikely(hold_right && GET(hold, US_RT)))
			hold_right(target);

		switch (button_update(&s)) {
		case RT|UP:
			if (!click_right)
				break;

			beep_fast();
			if (click_right(target))
				goto _break;
			if (value)
				value(target);

			break;
		case LT|UP:
			if (!click_left)
				break;

			beep_fast();
			if (click_left(target))
				goto _break;
			if (value)
				value(target);

			break;

		case BOTH|UP:
			if (!click_both)
				break;

			beep_fast();
			if (click_both(target))
				goto _break;
			if (value)
				value(target);

			break;

		case RT|HOLD:
			beep_slow();
			SET(hold, US_RT);
			break;

		case LT|HOLD:
			beep_slow();
			SET(hold, US_LT);
			break;

		case RT|HLUP:
			beep_fast();
			if (value)
				value(target);
			CLR(hold, US_RT);
			break;

		case LT|HLUP:
			beep_fast();
			if (value)
				value(target);
			CLR(hold, US_LT);
			break;
		}
	}
_break:
	/* palauta vanha näyttö */
	(void)memcpy(lcd_buffer, tmp, LCD_ROWS*LCD_COLS);
	lcd_update();
}

DEF_PSTR_PTR(YS, "[Y]  ");
DEF_PSTR_PTR(NO, "  [N]");
DEF_PSTR_PTR(IN, "[+] ");
DEF_PSTR_PTR(DC, " [-]");

static bool yn_left(void *target)
{
	*(bool *)target = false;
	return true;
}

static bool yn_right(void *target)
{
	*(bool *)target = true;
	return true;
}

bool __yesno(const char *msg)
{
	bool target;

	user_select(&target, msg, *REF_PSTR_PTR(NO), *REF_PSTR_PTR(YS),
		NULL, &yn_left, &yn_right, NULL, NULL, NULL);

	return target;
}

static bool s_done(__unused void *target)
{
	return true;
}

static void s_bool_value(bool *target)
{
	if (*target)
		lcd_put_P_const("ENABLE", 1, CENTER);
	else
		lcd_put_P_const("DISBLE", 1, CENTER);
	lcd_update();
}

static bool s_bool_left(bool *target)
{
	*target = false;
	return false;
}

static bool s_bool_right(bool *target)
{
	*target = true;
	return false;
}

void __select_bool(const char *msg, bool *target)
{
	user_select(target, msg, *REF_PSTR_PTR(DC), *REF_PSTR_PTR(IN),
		&s_bool_value, &s_bool_left, &s_bool_right, &s_done, NULL, NULL);
}

struct sfarg
{
	float value;
	float min;
	float max;
	float step;
};

static void s_float_value(struct sfarg *target)
{
	lcd_put_float(target->value, 2, true, 6, 1, CENTER);
	lcd_update();
}

static bool s_float_left(struct sfarg *target)
{
	float tmp;

	tmp = target->value - target->step;
	target->value = MAX(tmp, target->min);

	return false;
}

static bool s_float_right(struct sfarg *target)
{

	float tmp;

	tmp = target->value + target->step;
	target->value = MIN(tmp, target->max);

	return false;
}

static void s_float_hleft(struct sfarg *target)
{
	(void)s_float_left(target);
	_delay_ms(HOLD_DELAY);
	s_float_value(target);
}

static void s_float_hright(struct sfarg *target)
{
	(void)s_float_right(target);
	_delay_ms(HOLD_DELAY);
	s_float_value(target);
}

void __select_float(const char *msg,
	float *target, float min, float max, float step)
{
	struct sfarg arg = {*target, min, max, step};

	user_select(&arg, msg, *REF_PSTR_PTR(DC), *REF_PSTR_PTR(IN),
		&s_float_value, &s_float_left, &s_float_right, &s_done,
		&s_float_hleft, &s_float_hright);

	*target = arg.value;
}

struct siarg
{
	uint16_t value;
	uint16_t min;
	uint16_t max;
	uint16_t step;
};

static void s_uint_value(struct siarg *target)
{
	lcd_put_P_const("      ", 1, CENTER);
	lcd_put_uint(target->value, 6, 1, CENTER);
	lcd_update();
}

static bool s_uint_left(struct siarg *target)
{
	uint16_t tmp;

	/* tämä alivuotaa muuten */
	if (target->value < 1)
		return false;

	tmp = target->value - (int16_t)(target->step & 0x7FFF);
	target->value = MAX(tmp, target->min);

	return false;
}

static bool s_uint_right(struct siarg *target)
{
	uint16_t tmp;

	/* tämä ylivuotaa muuten */
	if (target->value == 0xFFFF)
		return false;

	tmp = target->value + (int16_t)(target->step & 0x7FFF);
	target->value = MIN(tmp, target->max);

	return false;
}

static void s_uint_hleft(struct sfarg *target)
{
	(void)s_uint_left(target);
	_delay_ms(HOLD_DELAY);
	s_uint_value(target);
}

static void s_uint_hright(struct sfarg *target)
{
	(void)s_uint_right(target);
	_delay_ms(HOLD_DELAY);
	s_uint_value(target);
}

void __select_uint(const char *msg, uint16_t *target,
	uint16_t min, uint16_t max, uint16_t step)
{
	struct siarg arg = {*target, min, max, step};

	user_select(&arg, msg, *REF_PSTR_PTR(DC), *REF_PSTR_PTR(IN),
		&s_uint_value, &s_uint_left, &s_uint_right, &s_done,
		&s_uint_hleft, &s_uint_hright);

	*target = arg.value;
}
