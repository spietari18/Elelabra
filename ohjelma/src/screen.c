#include "lcd.h"
#include "util.h"
#include "pins.h"
#include "screen.h"
#include "temp_util.h"

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#define DECL_WRITER(pin) \
	void pin##_writer(bool value) { WRITE(pin, value); }

#define REF_WRITER(pin) (&pin##_writer)

DECL_WRITER(_LCD_RS);
DECL_WRITER(_LCD_EN);
DECL_WRITER(_LCD_B5);
DECL_WRITER(_LCD_B6);
DECL_WRITER(_LCD_B7);
DECL_WRITER(_LCD_B8);

/* näyttö */
static LCD(lcd);

/* näytön puskuri */
char lcd_buffer[2*LCD_ROWS][LCD_COLS];

void lcd_init()
{
	/* laita pinnit OUTPUT modeen */
	MODE(_LCD_RS, OUTPUT);
	MODE(_LCD_EN, OUTPUT);
	MODE(_LCD_B5, OUTPUT);
	MODE(_LCD_B6, OUTPUT);
	MODE(_LCD_B7, OUTPUT);
	MODE(_LCD_B8, OUTPUT);

	/* pinnit */
	lcd.pins[LCD_RS] = REF_WRITER(_LCD_RS);
	lcd.pins[LCD_EN] = REF_WRITER(_LCD_EN);
	lcd.pins[LCD_B5] = REF_WRITER(_LCD_B5);
	lcd.pins[LCD_B6] = REF_WRITER(_LCD_B6);
	lcd.pins[LCD_B7] = REF_WRITER(_LCD_B7);
	lcd.pins[LCD_B8] = REF_WRITER(_LCD_B8);

	/* alusta näyttö */
	lcd_mode(&lcd,
		LCD_4BIT, LCD_2LIN, LCD_8DOT,
		LCD_ENLT, LCD_ESDC, LCD_DPON,
		LCD_CROF, LCD_BLOF, LCD_CRMV, LCD_MVRT, 0);
	lcd_clear(&lcd);

	/* tyhjennä puskuri */	
	(void)memset(lcd_buffer, ' ', 2*LCD_ROWS*LCD_COLS);
}

/* päivitä näyttö vastaamaan puskurin sisältöä */
void lcd_update()
{
	/* näitä ei tiedetä ennenkun kutsutaan
	 * lcd_cursor():ia kerran
	 */
	uint8_t lcd_col = -1, lcd_row = -1;

	for (uint8_t row = 0; row < LCD_ROWS; row++)
	{
		for (uint8_t col = 0; col < LCD_COLS; col++)
		{
		 	char *src, *dst;
			src = &lcd_buffer[row][col],
			dst = &lcd_buffer[row + LCD_ROWS][col];

			/* ei muutosta */
			if (likely(*src == *dst))
				continue;

			/* LCD kursoria pitää siirtää */
			if (unlikely((lcd_col != col)
			    || (lcd_row != row))) {
				lcd_col = col;
				lcd_row = row;
				lcd_cursor(&lcd, row, col);
			}

			/* kirjoita muuttunut kirjain
			 * näytölle ja muistiin
			 */
			(void)lcd_write(&lcd, *src);
			*dst = *src;

			/* kursori siirtyy sarakkeen eteenpäin mutta
			 * ei siirry itse seuraavalle riville.
			 */
			lcd_col++;
		}
	}
}

/* kirjoita merkkijono näytölle (älä kutsu suoraan) */
void __lcd_put(const char *msg, uint8_t len, uint8_t row,
	uint8_t align, void * (*copy)(void *, const void *,
	size_t), size_t (*length)(const char *))
{
	void *dst = NULL;

	if (unlikely(len == NULLTERM))
		len = length(msg);

	switch (align) {
	case ALIGN_L:
		dst = &lcd_buffer[row][0];
	break;
	case ALIGN_R:
		dst = &lcd_buffer[row][LCD_COLS - len];
	break;
	case ALIGN_C:
		dst = &lcd_buffer[row][(LCD_COLS - len + 1)/2];
	break;
	}

	(void)copy(dst, msg, len);
}

static const char itoa_chars[16] PROGMEM = {
	'0', '1', '2', '3', 
	'4', '5', '6', '7', 
	'8', '9', 'A', 'B', 
	'C', 'D', 'E', 'F' 
};

/* uint16_t tai uint8_t merkkijononksi */
static uint8_t custom_itoa(char *dst, uint16_t val,
	uint8_t lim, uint8_t base, bool trunc_low)
{
	uint8_t i = 0, j = 0, k;

	do {
		if (unlikely(i >= lim)) {
			if (likely(trunc_low)) {
				(void)memmove(&dst[0],
					&dst[1], lim - 1);
				i = lim - 1;
			} else {
				break;
			}
		}

		dst[i++] = pgm_read_byte(&itoa_chars[val % base]);
		val /= base;
	} while (val);

	k = MIN(i, lim)/2;
	while (j < k)
	{
		char c = dst[j];
		dst[j] = dst[i - j - 1];
		dst[i - j - 1] = c;
		j++;
	}

	return i;
}

static const char sgn[2] PROGMEM = {'+', '-'};

/* kirjoita liukuluku näytölle */
void lcd_put_float(float V, uint8_t p, bool fill,
	uint8_t lim, uint8_t row, uint8_t align)
{
	uint8_t i = 1, j;
	int16_t v;
	float rnd = 0.5;
	char buf[lim];

	/* erota kokonais ja desimaaliosa */
	v = (int16_t)V;
	V -= v;

	if (!isfinite(V)) {
		buf[0] = sgn[V < 0];
#define X "INFINITY"
		i = MIN(lim, sizeof(X));
		(void)memcpy_P(&buf[1], PSTR(X), i - 1);
#undef X
		goto print;
	} else {
		buf[0] = pgm_read_byte(&sgn[v < 0]);
		/* Poista V:n etumerkkibitti. (V = abs(V)) */
		uint32_t *const may_alias tmp = (uint32_t *)&V;
		CLR(*tmp, 31);
	}

	/* kokonaisosa */
	i += custom_itoa(&buf[1], abs(v), lim - 1, 10, false);
	if (unlikely(i == lim))
		goto print;

	/* desimaalipiste */
	buf[i++] = '.';
	if (unlikely(i == lim))
		goto print;

	/* pyöristä desimaaliosa */
	j = p;
	while (j--)
		rnd *= 0.1;
	V += rnd;	

	/* muuta kokonaisluvuksi */
	j = p;
	while (j--)
		V *= 10.0;

	/* desimaaliosa */
	j = custom_itoa(&buf[i], (uint16_t)V, lim - i, 10, true);
	i += j;
	if (unlikely(i == lim))
		goto print;
	
	/* lisää nollia, jotta numeron pituus on lim */
	if (fill) {
		i += (p - j);
		while (j < p)
			buf[i - j++] = '0';
		if (likely(i == lim))
			goto print;
		j = lim - i + 1;
		(void)memmove(&buf[j], &buf[1], i);
		i = 1;
		while (i < j)
			buf[i++] = '0';
		i = lim;
	}
print:
	lcd_put(buf, i, row, align);
}

/* kirjoita uint16_t tai uint8_t näytölle */
void lcd_put_uint(uint16_t val, uint8_t lim, uint8_t row, uint8_t align)
{
	char buf[6];

	lcd_put(buf, custom_itoa(buf, val, lim, 10, false), row, align);
}

/* kirjoita lämpötila näytölle (huomioi absoluuttiset rajat) */
void lcd_put_temp(float T, uint8_t p, uint8_t lim, uint8_t row, uint8_t align)
{
	char lim_str[6];

	if (unlikely(T < T_ABS_MIN)) {
		lim_str[0] = '-';
		(void)memcpy_P(&lim_str[1], PSTR("LIMIT"), 5);
		lcd_put(lim_str, MIN(6, lim), row, align);
	} else if (unlikely(T > T_ABS_MAX)) {
		lim_str[0] = '+';
		(void)memcpy_P(&lim_str[1], PSTR("LIMIT"), 5);
		lcd_put(lim_str, MIN(6, lim), row, align);
	} else {
		lcd_put_float(T, p, true, lim, row, align);
	}
}

/* printf() näytölle (älä kutsu suoraan) */
void __lcd_put_fmt(uint8_t lim, uint8_t row, uint8_t align, const char *fmt, ...)
{
	char buf[lim + 1];
	uint8_t len;
	va_list args;

	va_start(args, fmt);

	len = vsnprintf_P(buf, lim, fmt, args);
	len = MIN(len, LCD_COLS);

	va_end(args);

	lcd_put(buf, len, row, align);
}
