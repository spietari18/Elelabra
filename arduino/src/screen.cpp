#include "pins.h"
#include "screen.h"
#include "temp_util.h"
#include <LiquidCrystal.h>

/* näytön puskuri (näyttö alkaa tyhjänä) */
char lcd_buffer[2*LCD_ROWS][LCD_COLS] = {
	/* nykyinen tila */
	{"                "},
	{"                "},
	/* tila viimeisen lcd_update():en jälkeen */
	{"                "},
	{"                "}
};

static LiquidCrystal lcd(LCD_RS, LCD_CL, LCD_B4, LCD_B5, LCD_B6, LCD_B7);

/* alusta näyttö. tämä omana funktionaan, koska tämä on C++ tiedosto,
 * mutta muut C tiedostoja. (C ei pytsy kutsumaan C++ olion funktioita
 * suoraan)
 */
void lcd_init()
{
	lcd.begin(LCD_COLS, LCD_ROWS);
}

/* päivitä näyttö vastaamaan puskurin sisältöä */
void lcd_update()
{
	/* näitä ei tiedetä ennenkun kutsutaan
	 * lcd.setCursor():ia kerran
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
				lcd.setCursor(col, row);
			}

			/* kirjoita muuttunut kirjain
			 * näytölle ja muistiin
			 */
			(void)lcd.write(*src);
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
	void *dst;

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
static uint8_t itoa(char *dst, uint16_t val,
	uint8_t lim, uint8_t base, bool trunc_low)
{
	uint8_t i = 0, j = 0, k;

	while (val)
	{
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
	}

	k = min(i, lim)/2;
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
		i = min(lim, sizeof(X));
		(void)memcpy_P(&buf[1], PSTR(X), i - 1);
#undef X
		goto print;
	} else {
		buf[0] = pgm_read_byte(&sgn[v < 0]);
		CLR(*(uint32_t *)&V, 31); // V = abs(V)
	}

	/* kokonaisosa */
	i += itoa(&buf[1], abs(v), lim - 1, 10, false);
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
	j = itoa(&buf[i], (uint16_t)V, lim - i, 10, true);
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

	lcd_put(buf, itoa(buf, val, lim, 10, false), row, align);
}

/* kirjoita lämpötila näytölle (huomioi absoluuttiset rajat) */
void lcd_put_temp(float T, uint8_t p, uint8_t lim, uint8_t row, uint8_t align)
{
	char lim_str[6];

	if (unlikely(T < T_ABS_MIN)) {
		lim_str[0] = '-';
		(void)memcpy_P(&lim_str[1], PSTR("LIMIT"), 5);
		lcd_put(lim_str, min(6, lim), row, align);
	} else if (unlikely(T > T_ABS_MAX)) {
		lim_str[0] = '+';
		(void)memcpy_P(&lim_str[1], PSTR("LIMIT"), 5);
		lcd_put(lim_str, min(6, lim), row, align);
	} else {
		lcd_put_float(T, p, true, lim, row, align);
	}
}

