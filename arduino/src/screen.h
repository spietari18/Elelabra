#ifndef SCREEN_H
#define SCREEN_H

#include <stddef.h>
#include <string.h>
#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define LCD_ROWS 2  // näytön rivit
#define LCD_COLS 16 // näytön sarakkeet

/* tyhjennä koko näyttö */
#define LCD_CLEAR \
	(void)memset(lcd_buffer, ' ', LCD_ROWS*LCD_COLS)

#define ALIGN_L 0 // kohdista vasemmalle
#define ALIGN_R 1 // kohdista oikealle
#define ALIGN_C 2 // keskitä

/* arvo len parametrille lcd_put_* funktioissa jos haluaa
 * että merkkijonon pituus selvitetään strlen() funktiolla.
 */
#define NULLTERM ((uint8_t)~0)

/* näytön puskuri */
extern char lcd_buffer[2*LCD_ROWS][LCD_COLS];

/* alusta näyttö */
void lcd_init();

/* päivitä näyttö vastaamaan puskurin sisältöä */
void lcd_update();

/* kirjoita merkkijono näytölle (älä kutsu suoraan) */
void __lcd_put(const char *, uint8_t, uint8_t, uint8_t,
	void *(*)(void *, const void *, size_t), size_t (*)(const char *));

/* kirjoita liukuluku näytölle */
void lcd_put_float(float, uint8_t, bool, uint8_t, uint8_t, uint8_t);

/* kirjoita uint16_t tai uint8_t näytölle */
void lcd_put_uint(uint16_t, uint8_t, uint8_t, uint8_t);

/* kirjoita lämpötila näytölle (huomioi absoluuttiset rajat) */
void lcd_put_temp(float, uint8_t, uint8_t, uint8_t, uint8_t);

/* printf() näytölle (älä kutsu suoraan) */
void __lcd_put_fmt(uint8_t, uint8_t, uint8_t, void *, ...);

/* kirjoita merkkijono joka on kääntöaikana vakio
 * näytölle (varastoituu RAM muistiin)
 */
#define lcd_put_const(msg, row, align) \
	__lcd_put((msg), sizeof(msg) - 1, (row), (align), &memcpy, NULL)

/* kirjoita merkkijono joka on kääntöaikana vakio
 * näytölle (varastoituu ROM muistiin)
 */
#define lcd_put_P_const(msg, row, align) \
	__lcd_put(PSTR(msg), sizeof(msg) - 1, (row), (align), &memcpy_P, NULL)

/* kirjoita merkkijono RAM muistista näytölle */
#define lcd_put(msg, len, row, align) \
	__lcd_put((msg), (len), (row), (align), &memcpy, &strlen)

/* kirjoita merkkijono ROM muistista näytölle */
#define lcd_put_P(msg, len, row, align) \
	__lcd_put((msg), (len), (row), (align), &memcpy_P, &strlen_P)

/* printf() näytölle */
#define lcd_put_fmt(lim, row, align, fmt, ...) \
	__lcd_put_fmt((lim), (row), (align), PSTR(fmt), ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !SCREEN_H
