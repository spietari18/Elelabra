#ifndef UI_H
#define UI_H

#include "util.h"
#include "screen.h"

#include <stdint.h>
#include <stdbool.h>

/* Käyttöliittymän tilat. */
#define UI_SPLH 0
#define UI_MENU 1
#define UI_TEMP 2
#define UI_CLBR 3
#define UI_OPTS 4
#define UI_ERRR 5

/* Monta bittiä tilassa on. */
#define UI_BITS \
	(8*sizeof(int) - __builtin_clz(UI_ERRR))

#define UI_MASK_NOW \
	((1 << UI_BITS) - 1)
#define UI_MASK_OLD \
	(((1 << UI_BITS) - 1) << UI_BITS)

/* Määrittää onko tila SETUP vai LOOP. */
#define UI_BIT (1 << (UI_BITS << 1))

#define __UI_SET_STATE(state) \
	(ui_state = ((ui_state & UI_MASK_OLD) \
		<< UI_BITS) | (state & UI_MASK_NOW))

/* Vaihda käyttöliittymän tilaa. (UIT_BIT = 0) */
#define UI_SET_STATE(state) \
	__UI_SET_STATE(UI_##state)

/* Mikä on käyttöliittymän tila. */
#define UI_GET_STATE \
	(ui_state & (~UI_MASK_OLD))

/* Tämä täytyy olla UI_SETUP() lohkon lopussa. */
#define UI_SETUP_END \
	(ui_state |= UI_BIT)

#define UI_SETUP(state) \
	(UI_##state & UI_MASK_NOW)

#define UI_LOOP(state) \
	(UI_BIT | (UI_##state & UI_MASK_NOW))

/* Monta painiketta valikossa on. */
#define MENU_ENTRIES 3

/* Laajennetun tekstin maksimipituus. */
#define EXPANDED_LEN \
	(LCD_COLS - 3*(MENU_ENTRIES + 1))

/* Käyttöliittymän tilakone. */
extern uint8_t ui_state;

struct menu_config
{
	const void *text_title;
	const void *text_menu;
	uint8_t state;
	callback_t update;
	callback_t enter;
	callback_t exit;
};

#define __MENU_CONFIG \
	const struct menu_config menu_config[MENU_ENTRIES]

#define MENU_CONFIG \
	__MENU_CONFIG PROGMEM

#define MENU_ENTRY(text_title, text_menu, state, update, enter, exit) \
	{(text_title), (text_menu), (UI_##state), (update), (enter), (exit)}

extern bool in_menu;

/* Tämä tulee määritellä pääohjelmassa. */
extern __MENU_CONFIG;

/* Piirrä valikko. */
void menu_draw();

/* Päivitä valikko. */
void menu_update();

/* Siirry valikkoon. */
void menu_enter();

/* Mille riville edistymispalkki piirretään. */
#define PROG_ROW 1

/* Edistymispalkin koko ja reunat. */
#define PROG_INIT "[            ]"

/* Edistymispalkin paikka. */
extern uint8_t prog_pos;

/* Alusta edistymispalkki. (määrittää "määrän"
 * jota prog_dec() ja prog_inc() käsittelevät)
 */
void prog_init(uint8_t, uint8_t);

/* Pienennä edistymispalkkia yhdellä. */
void prog_dec();

/* Suurenna edistymispalkkia yhdellä. */
void prog_inc();

#endif // !UI_H
