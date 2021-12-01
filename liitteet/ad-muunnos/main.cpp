#include <Arduino.h>
#include <LiquidCrystal.h>

#include <setjmp.h>
#include "macro.h"

#define LCD_RS A0
#define LCD_CL A1
#define LCD_B4 A2
#define LCD_B5 A3
#define LCD_B6 A4
#define LCD_B7 A5

DEFINE_PIN(BTN_LT, D, 0);
DEFINE_PIN(BTN_RT, D, 1);

jmp_buf main_loop;

#define LCD_ROWS 2
#define LCD_COLS 16

char lcd_buffer[2*LCD_ROWS][LCD_COLS] = {
	/* nykyinen tila */
	{"  SAMPLE  TEXT  "},
	{"                "},
	/* tila viimeisen lcd_update():en jälkeen */
	{"                "},
	{"                "}
#if 0
	{"MIN +27.5*C  MAX"}
	{"-50.0*C   +80.0C"}

	{"    +27.5*C     "}

	{"(ADJUST) [C] [O]"}
	{"[A] (CALIBR) [O]"}
	{"[A] [C] (OPTION)"}
#endif
};

LiquidCrystal lcd(LCD_RS, LCD_CL, LCD_B4, LCD_B5, LCD_B6, LCD_B7);

void
lcd_update()
{
	uint8_t lcd_col = -1, lcd_row = -1;


	for (uint8_t row = 0; row < LCD_ROWS; row++)
	{
		for (uint8_t col = 0; col < LCD_COLS; col++)
		{
		 	char *src, *dst;
			src = &lcd_buffer[row][col],
			dst = &lcd_buffer[row + LCD_ROWS][col];

			if (likely(*src == *dst))
				continue;

			if (unlikely((lcd_col != col)
			    || (lcd_row != row))) {
				lcd_col = col;
				lcd_row = row;
				lcd.setCursor(col, row);
			}

			(void)lcd.write(*src);
			*dst = *src;

			lcd_col++;
		}
	}
}

#define BTN_POLL_TIME     250 // [ms]
#define BTN_JITTER_TIME   150 // [ms]
#define BTN_HOLD_TIME     900 // [ms]
#define BTN_INACTIVE_TIME 200 // [ms]

#define BTN_RT_OFF 0
#define BTN_LT_OFF 1

#define RT (1 << BTN_RT_OFF)
#define LT (1 << BTN_LT_OFF)

#define BOTH (RT|LT)

#define DOWN (0 << NUM_KEYS)
#define HOLD (1 << NUM_KEYS)
#define UP   (2 << NUM_KEYS)
#define HLUP (3 << NUM_KEYS)

#define NUM_KEYS 2

#define KEY_MASK \
	((1 << NUM_KEYS) - 1)

#define LOCK_MASK \
	(KEY_MASK << NUM_KEYS)

#define HOLD_BIT \
	(1 << (NUM_KEYS << 1))

#define RESET_MASK \
	(KEY_MASK | LOCK_MASK | HOLD_BIT)

#define STATE_OFF \
	(1 + (NUM_KEYS << 1))

#define STATE_MASK \
	(3 << STATE_OFF)

#define UPDATE_KEYS \
	(button_state |= (!READ(BTN_RT) << BTN_RT_OFF) \
		| (!READ(BTN_LT) << BTN_LT_OFF))

#define GET_KEYS \
	(button_state & KEY_MASK)

#define CLEAR_KEYS \
	(button_state &= ~KEY_MASK)

#define LOCK_KEYS \
	(button_state |= (button_state & KEY_MASK) << NUM_KEYS)

#define GET_LOCKED \
	((button_state & LOCK_MASK) >> NUM_KEYS)

#define GET_HOLD_FLAG \
	(!!(button_state & HOLD_BIT))

#define SET_HOLD_FLAG \
	(button_state |= HOLD_BIT)

#define RESET_STATE \
	(button_state &= ~RESET_MASK)

#define GET_STATE \
	(button_state & STATE_MASK)

#define SET_STATE(STATE) \
	(button_state = (button_state \
		& ~STATE_MASK) | (STATE & STATE_MASK))

#define GET_STATE \
		(button_state & STATE_MASK)

#define STATE_WAIT (0 << STATE_OFF)
#define STATE_POLL (1 << STATE_OFF)
#define STATE_LOCK (2 << STATE_OFF)
#define STATE_IACT (3 << STATE_OFF)

/* MSB ................................................ LSB
 *    1          2           1          2            2
 * |UNUSED|STATE MACHINE|HOLD FLAG|LOCKED KEYS|POLLED KEYS|
 */
uint8_t button_state;

uint32_t ts_1;
uint32_t ts_2;

uint8_t
button_update()
{
	unsigned long now, td_1, td_2;

	unsigned char ret = 0;

	now = millis();
	td_1 = now - ts_1;
	td_2 = now - ts_2;
	UPDATE_KEYS;
	switch (GET_STATE) {
	case STATE_WAIT:
		if (GET_KEYS) {
			ts_1 = now;
			SET_STATE(STATE_POLL);
		}
		break;
	case STATE_POLL:
		if (td_1 > BTN_POLL_TIME) {
			LOCK_KEYS;
			CLEAR_KEYS;
			SET_STATE(STATE_LOCK);
			ts_1 = ts_2 = now;
			ret |= DOWN | GET_LOCKED;
		}
		break;
	case STATE_LOCK:
		switch ((GET_HOLD_FLAG << 3)
			| ((td_1 > BTN_HOLD_TIME) << 2) 
			| ((td_2 > BTN_JITTER_TIME) << 1)
			| (GET_LOCKED != GET_KEYS)) {
		case 0x5:
		case 0x6:
		case 0x9:
			ts_2 = now;
			/* fallthrough */
		case 0x4:
			SET_HOLD_FLAG;
			ret |= HOLD | GET_LOCKED;
			break;
		case 0x0:
		case 0x2:
		case 0x8:
		case 0xA:
		case 0xC:
		case 0xE:
			ts_2 = now;
			break;
		case 0x7:
		case 0xB:
		case 0xF:
			ret |= HLUP;
			goto skip;
		case 0x3:
			ret |= UP;
		skip:
			ret |= GET_LOCKED;
			SET_STATE(STATE_IACT);
			ts_1 = now;
		case 0x1:
		case 0xD:
			break;
		}
		CLEAR_KEYS;
		break;
	case STATE_IACT:
		RESET_STATE;
		if (td_1 > BTN_INACTIVE_TIME)
			SET_STATE(STATE_WAIT);
		break;
	}

	return ret;
}

#define DISPLAY_FOR 2000
uint32_t last_msg;
bool clear;

/* tulosta napinpainalluksien tuottamat tapahtumat näytölle */
void
button_test()
{
#define Z 9
	uint32_t tmp;
	char *target = &lcd_buffer[1][LCD_COLS - Z];
	uint8_t count = -1;

	tmp = millis();
	switch (button_update()) {
#define X(Y) \
	case Y:\
		(void)memcpy_P(target, PSTR(#Y), sizeof(#Y) - 1); \
		count = sizeof(#Y) - 1; \
		clear = true; \
		break
	X(RT|DOWN);
	X(RT|HOLD);
	X(RT|UP);
	X(RT|HLUP);
	X(LT|DOWN);
	X(LT|HOLD);
	X(LT|UP);
	X(LT|HLUP);
	X(BOTH|DOWN);
	X(BOTH|HOLD);
	X(BOTH|UP);
	X(BOTH|HLUP);
#undef X
	default:
		if (clear && ((tmp - last_msg) > DISPLAY_FOR)) {
			count = 0;
			clear = false;
		}
	}

	if (count != (uint8_t)-1) {
		if (count < Z) 
			(void)memset(&target[count], ' ', Z - count);
		last_msg = tmp;
		lcd_update();
	}
#undef Z
}

void __attribute__((noreturn))
reset()
{
	WDTCSR = 0;
	SET(WDTCSR, WDE);
	while (1);
}

void __attribute__((noreturn))
entry_point()
{
	PRR = ~0;
	CLR(PRR, PRTIM0);

	DDRB  = DDRC  = DDRD  =  0; // INPUT
	PORTB = PORTC = PORTD = ~0; // PULLUP
	PINB  = PINC  = PIND  =  0; // LOW

	lcd.begin(LCD_COLS, LCD_ROWS);
	lcd_update();

	unsigned int i = 0;
	(void)setjmp(main_loop);
main_loop:
	button_test();
	goto main_loop;
}
