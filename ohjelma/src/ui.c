#include "ui.h"
#include "alarm.h"
#include "button.h"

/* Käyttöliittymän tilakone. */
uint8_t ui_state;

/* Valikon tila. */
static struct button_state s;
static uint8_t entry_now;
static uint8_t entry_old;
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
		CALLBACK(pgm_read_ptr(&menu_config[entry_now].enter));

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
	CALLBACK(pgm_read_ptr(&menu_config[entry_now].update));

	entry_old = entry_now;
}

/* Siirry valikkoon. */
void menu_enter()
{
	/* poistumisen takaisinkutsu */
	if (likely(entry_old != (uint8_t)~0))
		CALLBACK(pgm_read_ptr(&menu_config[entry_old].exit));

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

static uint8_t sm_now;
static uint8_t sm_old;

static const struct submenu_entry *sm_entries;
static uint8_t sm_n_entries;
static uint8_t sm_mode;

void submenu_init(const struct submenu_entry *entries,
	uint8_t n_entries, uint8_t mode)
{
	sm_now =  0;
	sm_old = ~0;

	switch (mode) {
	/* ei ylimääräistä tekstiä */
	case SM_NONE:
		break;

	case SM_2ROW:
		lcd_put_P_const("<>", 0, CENTER);
		fallthrough;
		
	case SM_1ROW:
		lcd_put_P_const("  ->", 1, LEFT);
		lcd_put_P_const("->  ", 1, RIGHT);

		break;

	default:
		unreachable;
	}

	sm_entries = entries;
	sm_n_entries = n_entries;
	sm_mode = mode;
}

void submenu_poll()
{
	switch (button_update(&s)) {
	case RT|UP:
		beep_fast();
		INC_MOD(sm_now, sm_n_entries);
		break;
	
	case LT|UP:
		beep_fast();
		DEC_MOD(sm_now, sm_n_entries);
		break;
	
	case BOTH|UP:
		beep_slow();
		CALLBACK(pgm_read_ptr(&sm_entries[sm_now].click));
		return;
	}

	if (unlikely(sm_now != sm_old)) {
		CALLBACK(pgm_read_ptr(&sm_entries[sm_now].init));

		lcd_put_P(pgm_read_ptr(
			&sm_entries[sm_now].name), NULLTERM, 1, CENTER);

		switch (sm_mode) {
		case SM_NONE:
			break;

		case SM_2ROW:
			//(void)memset(&lcd_buffer[0][6], ' ', 6);
			//(void)memset(&lcd_buffer[0][11], ' ', 6);
			lcd_put_P(pgm_read_ptr(&sm_entries[(sm_now
				+ sm_n_entries - 1) % sm_n_entries].name),
				NULLTERM, 0, LEFT);
			lcd_put_P(pgm_read_ptr(
				&sm_entries[(sm_now + 1)
				% sm_n_entries].name), NULLTERM, 0, RIGHT);

			fallthrough;
			
		case SM_1ROW:
			(void)memset(&lcd_buffer[1][0], ' ', 2);
			(void)memset(&lcd_buffer[1][14], ' ', 2);
			lcd_put_uint(sm_now + 1, 2, 1, LEFT);
			lcd_put_uint(sm_n_entries, 2, 1, RIGHT);

			break;

		default:
			unreachable;
		}

		lcd_update();

		sm_old = sm_now;
	}

	CALLBACK(pgm_read_ptr(&sm_entries[sm_now].loop));
}

void submenu_docb(uint8_t cb)
{
	CALLBACK(pgm_read_ptr(
		&((callback_t *)&sm_entries[sm_now].init)[cb]));
}

void submenu_text()
{
	lcd_put_P(pgm_read_ptr(
		&sm_entries[sm_now].name), NULLTERM, 1, CENTER);
}

typedef bool (*click_t)(void *);
typedef void (*hold_t)(void *);

#define US_LT (1 << 0)
#define US_RT (1 << 1)
#define US_BT (1 << 2)

static void user_select(void *target, const void *msg, uint8_t size,
	const void *left, const void *right, hold_t value,
	click_t click_left, click_t click_right, click_t click_both,
	hold_t hold_left, hold_t hold_right, 
	void *(*cpy)(void *, const void *, size_t),
	size_t (*len)(const char *))
{
	char tmp[LCD_ROWS][LCD_COLS];
	uint8_t hold = 0;

	/* tallenna vanha näyttö */
	(void)memcpy(tmp, lcd_buffer, LCD_ROWS*LCD_COLS);
	LCD_CLEAR;
	
	/* näppäimet ja otsikko */
	__lcd_put(msg, size, 0, CENTER, cpy, len);
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

bool __yesno(const void *msg, uint8_t size,
	void *(*cpy)(void *, const void *, size_t),
	size_t (*len)(const char *))
{
	bool target;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
	user_select(&target, msg, size, *REF_PSTR_PTR(NO), *REF_PSTR_PTR(YS),
		NULL, &yn_left, &yn_right, NULL, NULL, NULL, cpy, len);
#pragma GCC diagnostic pop

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

void __select_bool(const void *msg, uint8_t size, bool *target,
	void *(*cpy)(void *, const void *, size_t),
	size_t (*len)(const char *))
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
	user_select(target, msg, size, *REF_PSTR_PTR(DC), *REF_PSTR_PTR(IN),
		&s_bool_value, &s_bool_left, &s_bool_right, &s_done,
		NULL, NULL, cpy, len);
#pragma GCC diagnostic pop
}

struct sfarg
{
	float value;
	float min;
	float max;
	float step;
	float sact;
	float smul;
};

static void __float_put_value(struct sfarg *target)
{
	lcd_put_float(target->value, 2, true, 6, 1, CENTER);
	lcd_update();
}

static void s_float_value(struct sfarg *target)
{
	target->sact = target->step;
	__float_put_value(target);
}

static bool s_float_left(struct sfarg *target)
{
	float tmp;

	tmp = target->value - target->sact;
	target->value = MAX(tmp, target->min);

	return false;
}

static bool s_float_right(struct sfarg *target)
{
	float tmp;

	tmp = target->value + target->sact;
	target->value = MIN(tmp, target->max);

	return false;
}

static void s_float_hleft(struct sfarg *target)
{
	(void)s_float_left(target);
	_delay_ms(HOLD_DELAY);
	__float_put_value(target);
	target->sact = NEAREST(
		target->sact*target->smul, target->step);
}

static void s_float_hright(struct sfarg *target)
{
	(void)s_float_right(target);
	_delay_ms(HOLD_DELAY);
	__float_put_value(target);
	target->sact = NEAREST(
		target->sact*target->smul, target->step);
}

void __select_float(const void *msg, uint8_t size,
	float *target, float min, float max, float step,
	void *(*cpy)(void *, const void *, size_t),
	size_t (*len)(const char *))
{
	struct sfarg arg = {*target, min, max, step, step, 1.05};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
	user_select(&arg, msg, size, *REF_PSTR_PTR(DC), *REF_PSTR_PTR(IN),
		&s_float_value, &s_float_left, &s_float_right, &s_done,
		&s_float_hleft, &s_float_hright, cpy, len);
#pragma GCC diagnostic pop

	*target = arg.value;
}

struct siarg
{
	uint16_t value;
	uint16_t min;
	uint16_t max;
	uint16_t step;
	float sact;
	float smul;
};

static void __uint_put_value(struct siarg *target)
{
	lcd_put_P_const("      ", 1, CENTER);
	lcd_put_uint(target->value, 6, 1, CENTER);
	lcd_update();
}

static void s_uint_value(struct siarg *target)
{
	target->sact = target->step;
	__uint_put_value(target);
}

static bool s_uint_left(struct siarg *target)
{
	uint16_t tmp;

	tmp = target->value - ((uint16_t)target->sact & 0x7FFF);

	/* alivuoto */
	if (tmp > target->value)
		target->value = target->min;
	else 
		target->value = MAX(tmp, target->min);

	return false;
}

static bool s_uint_right(struct siarg *target)
{
	uint16_t tmp;

	tmp = target->value + ((uint16_t)target->sact & 0x7FFF);

	/* ylivuoto */
	if (tmp < target->value)
		target->value = target->max;
	else
		target->value = MIN(tmp, target->max);

	return false;
}

static void s_uint_hleft(struct siarg *target)
{
	(void)s_uint_left(target);
	_delay_ms(HOLD_DELAY);
	__uint_put_value(target);
	target->sact = NEAREST(
		((float)target->sact)*target->smul, (float)target->step);
}

static void s_uint_hright(struct siarg *target)
{
	(void)s_uint_right(target);
	_delay_ms(HOLD_DELAY);
	__uint_put_value(target);
	target->sact = NEAREST(
		target->sact*target->smul, target->step);
}

void __select_uint(const void *msg, uint8_t size, uint16_t *target,
	uint16_t min, uint16_t max, uint16_t step,
	void *(*cpy)(void *, const void *, size_t),
	size_t (*len)(const char *))
{
	struct siarg arg = {*target, min, max, step, step, 1.02};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
	user_select(&arg, msg, size, *REF_PSTR_PTR(DC), *REF_PSTR_PTR(IN),
		&s_uint_value, &s_uint_left, &s_uint_right, &s_done,
		&s_uint_hleft, &s_uint_hright, cpy, len);
#pragma GCC diagnostic pop

	*target = arg.value;
}

void __msg(const void *msg, uint8_t size, enum text_align align,
	void *(*cpy)(void *, const void *, size_t),
	size_t (*len)(const char *))
{
	/* puskuri */
	char tmp[LCD_ROWS][LCD_COLS];

	/* onko merkkijono nollaterminoitu? */
	if (size == NULLTERM)
		size = len(msg);
	
	/* tallenna vanha näyttö */
	(void)memcpy(tmp, lcd_buffer, LCD_ROWS*LCD_COLS);

	/* tyhjennä näyttö */
	LCD_CLEAR;

	for (uint8_t row = 0; row < 2; row++)
	{
		uint8_t i = 0;

		/* etsi rivin loppu */
		for (; i < size; i++)
		{
			uint8_t c;

			/* tämä koska ei tiedetä onko merkkijono
			 * RAM vai ROM muistissa (hidas kuin mikäkin)
			 */
			(void)cpy(&c, &((const char *)msg)[i], 1);

			/* uusi rivi */
			if (c == '\n')
				break;
		}

		/* kirjoita rivi (jos ei tyhjä) */
		if (likely(i > 0))
			__lcd_put(msg,
				MIN(LCD_COLS, i), row, align, cpy, NULL);

		/* koko ei saa alivuotaa */
		if (likely(i < size))
			i++;

		msg += i;
		size -= i;
	}

	lcd_update();

	beep_slow();

	/* odota */
	_delay_ms(MSG_WAIT);

	/* palauta vanha näyttö */
	(void)memcpy(lcd_buffer, tmp, LCD_ROWS*LCD_COLS);
	lcd_update();
}

void user_wait()
{
	while (1)
	{
		switch (button_update(&s))
		{
		case RT|UP:
		case LT|UP:
		case BOTH|UP:
		case RT|HLUP:
		case LT|HLUP:
		case BOTH|HLUP:
			beep_fast();
			_delay_ms(100);
			return;
		}
	}
}
