#ifndef UI_H
#define UI_H

#include "util.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Käyttöliittymän tilat. */
#define UI_SPLASH  0 
#define UI_DEFAULT 1
#define UI_CONFIG1 2
#define UI_CONFIG2 3
#define UI_CALIBR  4
#define UI_ERROR   5

/* Monta bittiä tilassa on. */
#define UI_BITS \
	(8*sizeof(int) - __builtin_clz(UI_ERROR))

#define UI_MASK_NOW \
	((1 << UI_BITS) - 1)
#define UI_MASK_OLD \
	(((1 << UI_BITS) - 1) << UI_BITS)

/* Määrittää onko tila SETUP vai LOOP. */
#define UI_BIT (1 << (UI_BITS << 1))

/* Vaihda käyttöliittymän tilaa. (UIT_BIT = 0) */
#define UI_SET_STATE(state) \
	(ui_state = ((ui_state & UI_MASK_OLD) \
		<< UI_BITS) | (UI_##state & UI_MASK_NOW))

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

/* Valikon asetukset. */
#define MENU_ALIGN ALIGN_C
#define MENU_ROW 1
#define MENU_ENTRIES 4

/* Käyttöliittymän tilakone. */
extern uint8_t ui_state;

#define __MENU_CALLBACKS \
	const callback_t menu_callbacks[MENU_ENTRIES][2]

#define MENU_CALLBACKS \
	__MENU_CALLBACKS PROGMEM

#define MENU_CALLBACK(ENTER, EXIT) \
	{(ENTER), (EXIT)}

/* Tämä tulee määritellä pääohjelmassa. */
extern __MENU_CALLBACKS;

/* Valikon tila. (päivittää valikon ja palauttaa
 * true kun käyttäjä poistuu valikosta)
 */
bool menu();

/* Siirry valikkoon. */
void menu_enter(uint8_t);

/* Palaa valikkoon. */
void menu_return();

/* Mille riville edistymispalkki piirretään. */
#define PROG_ROW 1

/* Edistymispalkin koko ja reunat. */
#define PROG_INIT "[            ]"

/* Alusta edistymispalkki. (määrittää "määrän"
 * jota prog_dec() ja prog_inc() käsittelevät)
 */
void prog_init(uint8_t, uint8_t);

/* Pienennä edistymispalkkia yhdellä. */
void prog_dec();

/* Suurenna edistymispalkkia yhdellä. */
void prog_inc();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !UI_H