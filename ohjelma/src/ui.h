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

#define UI_STATE_COUNT 5

/* Monta bittiä tilassa on. */
#define UI_BITS \
	(8*sizeof(int) - __builtin_clz(UI_OPTS))

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

struct submenu_entry
{
	const void *name;
	callback_t init;
	callback_t loop;
	callback_t click;
};

#define SM_INIT 0
#define SM_LOOP 1
#define SM_CLCK 2

#define SM_NONE 0
#define SM_1ROW 1
#define SM_2ROW 2

void submenu_init(const struct submenu_entry *, uint8_t, uint8_t);
void submenu_poll();
void submenu_docb(uint8_t);
void submenu_text();

#define DEF_SUBMENU(name) \
	static const struct submenu_entry name[] PROGMEM

#define SUBMENU_ENTRY(name, init, loop, click) \
	{(void *)(name), (init), (loop), (click)}

#define SUBMENU_INIT(name, mode) \
	submenu_init((name), ARRAY_SIZE(name), (mode))

#define HOLD_DELAY 50 // [ms]

bool __yesno(const void *, uint8_t, void *(*)(void *,
	const void *, size_t), size_t (*)(const char *));

void __select_bool(const void *, uint8_t, bool *,
	void *(*)(void *, const void *, size_t),
	size_t (*)(const char *));

void __select_float(const void *, uint8_t,
	float *, float, float, float, void *(*)(void *,
	const void *, size_t), size_t (*)(const char *));

void __select_uint(const void *, uint8_t,
	uint16_t *, uint16_t, uint16_t, uint16_t,
	void *(*)(void *, const void *, size_t),
	size_t (*)(const char *));

#define yesno(msg, size) \
	__yesno((msg), (size), &memcpy, &strlen)

#define yesno_P(msg, size) \
	__yesno((msg), (size), &memcpy_P, &strlen_P)

#define yesno_const(msg) \
	__yesno((msg), sizeof(msg) - 1, &memcpy, NULL)

#define yesno_P_const(msg) \
	__yesno(PSTR(msg), sizeof(msg) - 1, &memcpy_P, NULL)

#define select_bool(msg, size, target) \
	__select_bool((msg), (size), (target), &memcpy, &strlen)

#define select_bool_P(msg, size, target) \
	__select_bool(PSTR(msg), (size), (target), &memcpy_P, &strlen_P)

#define select_bool_const(msg, target) \
	__select_bool((msg), sizeof(msg) - 1, (target), &memcpy, NULL)

#define select_bool_P_const(msg, target) \
	__select_bool(PSTR(msg), sizeof(msg) - 1, (target), &memcpy_P, NULL)

#define select_float(msg, size, target, min, max, step) \
	__select_float((msg), (size), (target), (min), (max), (step), \
		&memcpy, &strlen)

#define select_float_P(msg, size, target, min, max, step) \
	__select_float((msg), (size), (target), (min), (max), (step), \
		&memcpy_P, &strlen_P)

#define select_float_const(msg, target, min, max, step) \
	__select_float((msg), sizeof(msg) - 1 (target), (min), (max), \
		(step), &memcpy, NULL)

#define select_float_P_const(msg, target, min, max, step) \
	__select_float(PSTR(msg), sizeof(msg) - 1, (target), (min), \
		(max), (step), &memcpy_P, NULL)

#define select_uint(msg, size, target, min, max, step) \
	__select_uint((msg), (size), (target), (min), (max), (step), \
		&memcpy, &strlen)

#define select_uint_P(msg, size, target, min, max, step) \
	__select_uint((msg), (size), (target), (min), (max), (step), \
		&memcpy_P, &strlen_P)

#define select_uint_const(msg, target, min, max, step) \
	__select_uint((msg), sizeof(msg) - 1, (target), (min), (max), \
		(step), &memcpy, NULL)

#define select_uint_P_const(msg, target, min, max, step) \
	__select_uint(PSTR(msg), sizeof(msg) - 1, (target), (min), \
		(max), (step), &memcpy_P, NULL)

#define MSG_WAIT 1800 // [ms]

void __msg(const void *, uint8_t, enum text_align,
	void *(*)(void *, const void *, size_t), size_t (*)(const char *));

#define msg(msg, size) \
	__msg((msg), (size), CENTER, &memcpy, &strlen)

#define msg_P(msg, size) \
	__msg((msg), (size), CENTER, &memcpy_P, &strlen_P)

#define msg_const(msg) \
	__msg((msg), sizeof(msg) - 1, CENTER, &memcpy, NULL)

#define msg_P_const(msg) \
	__msg(PSTR(msg), sizeof(msg) - 1, CENTER, &memcpy_P, NULL)

#endif // !UI_H
