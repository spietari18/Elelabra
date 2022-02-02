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
			&entries[submenu_now].name), 4, 1, CENTER);

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
			&entries[submenu_now].name), 4, 1, CENTER);
}
