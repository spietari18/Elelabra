/* LiquidCrystal kirjasto portattu C:ksi ja käyttämään tämän
 * kansion sisältämää koodia Arduinon standardikirjaston sijasta.
 */
#include "lcd.h"
#include "util.h"

#include <stdarg.h>

static void pulse(pin_t pin)
{
	pin(0);
	_delay_us(1);
	pin(1);
	_delay_us(1);
	pin(0);
	_delay_us(100);
}

static void put4(struct lcd *lcd, uint8_t data)
{
	for (uint8_t i = 0; i < 4; i++)
		lcd->pins[LCD_B5 + i]((data >> i) & 1);

	pulse(lcd->pins[LCD_EN]);
}

static void put8(struct lcd *lcd, uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
		lcd->pins[LCD_B1 + i]((data >> i) & 1);

	pulse(lcd->pins[LCD_EN]);
}

static void send(struct lcd *lcd, uint8_t data, bool cmd)
{
	//if (lcd->pins[LCD_RW])
	//	lcd->pins[LCD_RW](0);

	lcd->pins[LCD_RS](!cmd);

	if (lcd->mode & LCD_8BIT) {
		put8(lcd, data);
	} else {
		put4(lcd, data >> 4);
		put4(lcd, data);
	}
}

/* tyhjennä näyttö */
void lcd_clear(struct lcd *lcd)
{
	send(lcd, LCD_CLDP, true);
	_delay_us(2000);
}

/* siirrä kursori yläkulmaan */
void lcd_home(struct lcd *lcd)
{
	send(lcd, LCD_HOME, true);
	_delay_us(2000);
}

/* aseta näytön tila */
void lcd_mode(struct lcd *lcd, ...)
{
	va_list args;
	uint16_t new;
	uint16_t tmp;

	va_start(args, lcd);

	new = lcd->mode;
	while ((tmp = (uint16_t)va_arg(args, int)))
	{
		/* komplementti */
		if (tmp & 0x8000)
			new &= ~tmp;

		/* indikaattori */
		else
			new |= tmp;
	}

	tmp = new & FS_MSK;
	if ((lcd->mode & LCD_INIT) || (tmp != (lcd->mode & FS_MSK))) {
		uint8_t cmd = LCD_FCNS | (tmp >> FS_OFF);

		/* tämä koska put4() ei käytä send() funktiota */
		lcd->pins[LCD_RS](0);
		lcd->pins[LCD_EN](0);

		_delay_us(5000);

		/* alustus tapahtuu näin datalehden mukaan */
		if (tmp & LCD_8BIT) {
			send(lcd, cmd, true);
			_delay_us(4500);
			send(lcd, cmd, true);
			_delay_us(150);
			send(lcd, cmd, true);
		} else {
			put4(lcd, 0x03);
			_delay_us(4500);
			put4(lcd, 0x03);
			_delay_us(4500);
			put4(lcd, 0x03);
			_delay_us(150);
			put4(lcd, 0x02);
		}

		send(lcd, cmd, true);
	}

	tmp = new & DM_MSK;
	if (tmp != (lcd->mode & DM_MSK))
		send(lcd, LCD_EDMS | (tmp >> DM_OFF), true);

	tmp = new & DC_MSK;
	if (tmp != (lcd->mode & DC_MSK))
		send(lcd, LCD_DPCT | (tmp >> DC_OFF), true);

	tmp = new & CS_MSK;
	if (tmp != (lcd->mode & CS_MSK))
		send(lcd, LCD_CRST | (tmp >> CS_OFF), true);

	lcd->mode = new & ~LCD_INIT;

	va_end(args);
}

/* lisää kirjain */
void lcd_mkchar(struct lcd *lcd, uint8_t pos, uint8_t map[])
{
	pos &= 0x7;
  	send(lcd, LCD_CRAS | (pos << 3), true);

	for (uint8_t i = 0; i < 8; i++)
		lcd_write(lcd, map[i]);
}

/* aseta kursorin paikka */
void lcd_cursor(struct lcd *lcd, uint8_t row, uint8_t col)
{
	send(lcd, LCD_DRAS | ((col % 40) + 0x40*(row & 1)), true);
}

/* kirjoita näytölle */
void lcd_write(struct lcd *lcd, uint8_t data)
{
	send(lcd, data, false);
}
