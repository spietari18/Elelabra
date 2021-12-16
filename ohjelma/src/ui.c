#include "ui.h"
#include "alarm.h"
#include "button.h"

/* Käyttöliittymän tilakone. */
uint8_t ui_state;

/* Valikon tila. */
static struct button_state s;
static uint8_t entry_now;
static uint8_t entry_old;
static bool redraw;

/* tämä on globaali, jotta koodi jonka
 * tarvitsee tietää onko valikko päällä
 * voi hakea arvon nopeasti.
 */
bool in_menu;

/* Piirrä valikko. */
void menu_draw()
{
	char *dst = &lcd_buffer[1][1];
	uint8_t i = 0;

	LCD_CLEAR;
	
	lcd_put_P(pgm_read_ptr(&menu_config[entry_now]
		.text_title), NULLTERM, 0, ALIGN_C);
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
	callback_t callback;

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

		/* poistumisen takaisinkutsu */
		callback = pgm_read_ptr(
			&menu_config[entry_now].enter);
		if (callback)
			callback();

		__UI_SET_STATE(pgm_read_word(
			&menu_config[entry_now].state));
		in_menu = false;

		return;
	}

	/* jos mikään ei muutu tai päivitystä
	 * ei pakoteta, älä päivitä
	 */
	if (likely(entry_old == entry_now))
		return;

	menu_draw();

	/* taustatilan päivitystakaisinkutsu */
	callback = pgm_read_ptr(
		&menu_config[entry_now].update);
	if (callback)
		callback();

	entry_old = entry_now;
}

#define PROG_CHARS \
	(sizeof(PROG_INIT) - 3)

static uint8_t old;
static uint8_t pos;
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
	pos = i - 1;

	/* kopioi palkin alkutila */
	lcd_put_P_const(PROG_INIT, PROG_ROW, ALIGN_C);

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
	old = pos;
	j = INT_DIV_RND((2*PROG_CHARS - 1)*pos, num);
	pos--;

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

	if (pos >= (num - 1)) {
		pos = num - 1;
		return;
	}

	/* askeleen alku- ja loppuindeksi */
	i = INT_DIV_RND((2*PROG_CHARS - 1)*old, num)/2;
	old = pos;
	j = INT_DIV_RND((2*PROG_CHARS - 1)*pos, num);
	pos++;

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
