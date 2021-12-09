#include "ui.h"
#include "screen.h"
#include "button.h"

#if 0
void (*menu_callback)(void)[MENU_CHARS] = {

};
#endif

/* valikko tarvitsee oman nappitilansa
 * jotta kutsut button_update():lle muualla
 * eivät vaikuta nappien tilaan täällä.
 */
static struct button_state state;
static uint8_t menu_entry = ~0;

/* valikon tilakone */
uint8_t ui_state;

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

	switch (button_update(&state)) {
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

	if (likely(old == menu_entry))
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

#if 0
	menu_callback[menu_entry]();
#else
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
#endif

	return false;
}

#define PROG_CHARS \
	(sizeof(PROG_INIT) - 3)

static uint8_t pos;
static uint8_t num;

/* Mistä kohtaa puskurissa edistymispalkki alkaa. */
static char* const dst = &lcd_buffer[PROG_ROW][(LCD_COLS - PROG_CHARS)/2];

static const char prg[2] PROGMEM = {'=', '-'};

/* Alusta edistymispalkki. (määrittää "määrän"
 * jota prog_dec() ja prog_inc() käsittelevät)
 */
void prog_init(uint8_t n, uint8_t i)
{
	uint8_t j, k;

	/* päivitä globaali tila */
	num = n;
	pos = i;

	/* kopioi palkin alkutila */
	lcd_put_P_const(PROG_INIT, PROG_ROW, ALIGN_C);

	/* missä kohtaa palkkia mennään */
	j = int_div_rnd(PROG_CHARS*(PROG_CHARS - 1)*i, i);

	/* j == 0 tarkoittaa, että palkki on tyhjä */
	if (j == 0)
		return;
	
	/* täytä palkki */
	k = j/2;
	dst[k] = pgm_read_byte(&prg[j & 1]);
	while (k--)
		dst[k] = pgm_read_byte(&prg[0]);
	
}

/* Pienennä edistymispalkkia yhdellä. */
void prog_dec()
{
	uint8_t i, j, k;

	if (pos == 0)
		return;

	/* askeleen alku- ja loppuindeksi */
	i = int_div_rnd(PROG_CHARS*(PROG_CHARS - 1)*pos, num)/2;
	pos--;
	j = int_div_rnd(PROG_CHARS*(PROG_CHARS - 1)*pos, num);

	/* täytä askeleen pää */
	k = j/2;
	if (j == 0)
		dst[k] = ' ';
	else
		dst[k] = pgm_read_byte(&prg[j & 1]);

	/* onko tyhjennys tarpeellinen? */
	if ((i - k) < 2)
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
	i = int_div_rnd(PROG_CHARS*(PROG_CHARS - 1)*pos, num)/2;
	pos++;
	j = int_div_rnd(PROG_CHARS*(PROG_CHARS - 1)*pos, num);

	/* täytä askeleen pää */
	k = j/2;
	dst[k] = pgm_read_byte(&prg[j & 1]);
	
	/* onko täyttö tarpeellinen? */
	if ((k - i) < 2)
		return;
	k--;

	/* täytä skipatut kirjaimet */
	while (i < k)
		dst[i++] = pgm_read_byte(&prg[0]);
}
