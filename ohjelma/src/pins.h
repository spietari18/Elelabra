#ifndef PINS_H
#define PINS_H

#include "macro.h"

/* SPI pinnit */
DEFINE_PIN(SPI_SS,   B, 2); // PB2
DEFINE_PIN(SPI_CLK,  B, 5); // PB5
DEFINE_PIN(SPI_MOSI, B, 3); // PB3
DEFINE_PIN(SPI_MISO, B, 4); // PB4

/* I2C pinnit */
DEFINE_PIN(I2C_SDA, C, 4); // PC4
DEFINE_PIN(I2C_SCL, C, 5); // PC5

#if 0
/* LCD näyttö. */
DEFINE_PIN(_LCD_RS, D, 7); // PD7
DEFINE_PIN(_LCD_EN, B, 0); // PB0
DEFINE_PIN(_LCD_B5, C, 0); // PC0
DEFINE_PIN(_LCD_B6, C, 1); // PC1
DEFINE_PIN(_LCD_B7, C, 2); // PC2
DEFINE_PIN(_LCD_B8, C, 3); // PC3

/* Summeri. */
#define BUZZER_TIMER 2
DEFINE_PIN(BUZZER, D, 3); // PD3

/* Muut pinnit. */
DEFINE_PIN(LCD_AN, D, 4); // PD4
DEFINE_PIN(BTN_LT, D, 1); // PD1, (vasen)
DEFINE_PIN(BTN_RT, D, 0); // PD1, (oikea)
#else
/* LCD näyttö. */
DEFINE_PIN(_LCD_RS, C, 0); // PD7
DEFINE_PIN(_LCD_EN, C, 1); // PB0
DEFINE_PIN(_LCD_B5, C, 2); // PC0
DEFINE_PIN(_LCD_B6, C, 3); // PC1
DEFINE_PIN(_LCD_B7, C, 4); // PC2
DEFINE_PIN(_LCD_B8, C, 5); // PC3

/* Summeri. */
#define BUZZER_TIMER 2
DEFINE_PIN(BUZZER, D, 3); // PD3

/* Muut pinnit. */
DEFINE_PIN(LCD_AN, D, 2); // PD4
DEFINE_PIN(BTN_LT, D, 1); // PD1, (vasen)
DEFINE_PIN(BTN_RT, D, 0); // PD1, (oikea)
#endif

#endif // !PINS_H
