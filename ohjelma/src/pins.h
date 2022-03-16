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
DEFINE_PIN(_LCD_RS, D, 6); // PD6
DEFINE_PIN(_LCD_EN, D, 5); // PD5
DEFINE_PIN(_LCD_B5, C, 0); // PC0
DEFINE_PIN(_LCD_B6, C, 1); // PC1
DEFINE_PIN(_LCD_B7, C, 2); // PC2
DEFINE_PIN(_LCD_B8, C, 3); // PC3

/* Summeri. */
DEFINE_PIN(BUZZER, D, 4); // PD4

/* LCD:n taustavalon anodi. */
DEFINE_PIN(LCD_AN, D, 3); // PD3

/* Napit. */
DEFINE_PIN(BTN_LT, D, 1); // PD1, (vasen)
DEFINE_PIN(BTN_RT, D, 0); // PD1, (oikea)
#else
/* LCD näyttö. */
DEFINE_PIN(_LCD_RS, C, 0); // PC0, (A0)
DEFINE_PIN(_LCD_EN, C, 1); // PC1, (A1)
DEFINE_PIN(_LCD_B5, C, 2); // PC2, (A2)
DEFINE_PIN(_LCD_B6, C, 3); // PC3, (A3)
DEFINE_PIN(_LCD_B7, D, 4); // PC4, (A4)
DEFINE_PIN(_LCD_B8, D, 5); // PC5, (A5)

/* Summeri. */
DEFINE_PIN(BUZZER, D, 3); // PD3, (D3)

/* LCD:n taustavalon anodi. */
DEFINE_PIN(LCD_AN, D, 2); // PD2, (D2)

/* Napit. */
DEFINE_PIN(BTN_RT, D, 1); // PD1, (D1) (vasen)
DEFINE_PIN(BTN_LT, D, 0); // PD0, (D0) (oikea)
#endif

#endif // !PINS_H
