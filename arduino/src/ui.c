#include "ui.h"
#include "screen.h"
#include "button.h"

/* Käyttöliittymän tilakone. */
uint8_t ui_state;

/* Valikon tila. */
static struct button_state s;
static uint8_t entry_now;
static uint8_t entry_old = ~0;
static bool in_menu;
static bool redraw;

#define MENU_EXIT 0
#define MENU_NOCH 1
#define MENU_MOVE 2

/* Piirrä valikko. */
static void menu_draw(bool clear)
{
	char menu_str[MENU_ENTRIES];

	if (unlikely(clear)) {
		(void)memset(menu_str, ' ', MENU_ENTRIES);
	} else {
		for (uint8_t i = 0; i < MENU_ENTRIES; i++)
		{
			if (unlikely(i == entry_now))
				menu_str[i] = 0xFF;
			else
				menu_str[i] = i + '1';
		}
	}

	lcd_put(menu_str, MENU_ENTRIES, MENU_ROW, MENU_ALIGN);	
	lcd_update();
}

/* päivitä valikko */
static uint8_t menu_update(bool force)
{
	/* ohita napit */
	if (force)
		goto skip_buttons;

	switch (button_update(&s)) {
	case RT|UP:
		entry_now = (entry_now + 1) % MENU_ENTRIES;
		break;
	case LT|UP:
		entry_now = (entry_now + MENU_ENTRIES - 1) % MENU_ENTRIES;
		break;
	case BOTH|UP:
		/* poistumisen takaisinkutsu */
		callback_t callback = pgm_read_ptr(
			&menu_callbacks[entry_now][1]);
		if (callback)
			callback();

		return MENU_EXIT;
	}

	/* jos mikään ei muutu tai päivitystä
	 * ei pakoteta, älä päivitä
	 */
	if (likely(entry_old == entry_now))
		return MENU_NOCH;

skip_buttons:
	/* navigaation takaisinkutsu */
	callback_t callback = pgm_read_ptr(
		&menu_callbacks[entry_now][0]);
	if (callback)
		callback();

	entry_old = entry_now;

	return MENU_MOVE;
}

/* Valikon tila. */
bool menu()
{
	if (in_menu) {
		switch (menu_update(false)) {
		case MENU_EXIT:
			/* tyhjennä valikko */
			menu_draw(true);
			in_menu = false;
			return false;
		case MENU_NOCH:
			/* piirrä valikko */
			if (redraw) {
				menu_draw(false);
				redraw = false;
			}
			return true;
		case MENU_MOVE:
			/* piirrä valikko seuraavalla
			 * menu() kutsulla
			 */
			redraw = true;
			return true;
		}
	} 
	
	return false;
}

/* Siirry valikkoon. */
void menu_enter(uint8_t entry)
{
	in_menu = true;
	redraw = false;
	entry_now = entry;
	(void)menu_update(true);
}

/* Palaa valikkoon. */
void menu_return()
{
	in_menu = redraw = true;
	(void)menu_update(true);
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
	j = int_div_rnd((2*PROG_CHARS - 1)*i, n);

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
	i = int_div_rnd((2*PROG_CHARS - 1)*old, num)/2;
	old = pos;
	j = int_div_rnd((2*PROG_CHARS - 1)*pos, num);
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
	i = int_div_rnd((2*PROG_CHARS - 1)*old, num)/2;
	old = pos;
	j = int_div_rnd((2*PROG_CHARS - 1)*pos, num);
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
