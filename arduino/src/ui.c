#include "ui.h"
#include "screen.h"
#include "button.h"

/* valikko tarvitsee oman nappitilansa
 * jotta kutsut button_update():lle muualla
 * eivät vaikuta nappien tilaan täällä.
 */
static struct button_state s;
static uint8_t menu_entry = ~0;

/* valikon tilakone */
uint8_t ui_state;

bool menu_force_update;

/* päivitä valikko */
bool menu_update()
{
	char *dst = &lcd_buffer[1][(LCD_COLS - MENU_CHARS)/2];
	uint8_t old = menu_entry;

	/* alustus */
	if (unlikely(menu_entry == (uint8_t)~0)) {
		menu_entry = 0;
		goto render_menu;
	}

	switch (button_update(&s)) {
	case RT|UP:
		menu_entry = (menu_entry + 1) % MENU_CHARS;
		break;
	case LT|UP:
		menu_entry = (menu_entry + MENU_CHARS - 1) % MENU_CHARS;
		break;
	case BOTH|UP:
		(void)memset(dst, ' ', MENU_CHARS);
		return true;
		break;
	}

	/* pakotettu valikon päivitys */
	if (unlikely(menu_force_update))
		menu_force_update = false;

	/* jos mikään ei muutu, älä päivitä */
	else if (likely(old == menu_entry))
		return false;

render_menu:
	for (uint8_t i = 0; i < MENU_CHARS; i++)
	{
		if (unlikely(i == menu_entry))
			dst[i] = 0xFF;
		else
			dst[i] = i + '1';
	}

	lcd_update();

	switch (menu_entry) {
	case 0:
		UI_SET_STATE(DEFAULT);
		break;
	case 1:
		UI_SET_STATE(CALIBR);
		break;
	case 2:
		UI_SET_STATE(CONFIG1);
		break;
	case 3:
		UI_SET_STATE(CONFIG2);
		break;
	}

	return false;
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
