#ifndef PINS_H
#define PINS_H

/* tässä ei extern "C":tä sen takia, että ei halutakkaan,
 * että linkkeri yrittää tehdä DEFINE_PIN():llä määritetyistä
 * pinneistä globaaleja muuttujia.
 */

#include "macro.h"

/* SPI pinnit */
DEFINE_PIN(SPI_SS,   B, 2); // PB2
DEFINE_PIN(SPI_CLK,  B, 5); // PB5
DEFINE_PIN(SPI_MOSI, B, 3); // PB3
DEFINE_PIN(SPI_MISO, B, 4); // PB4

/* Arduinon LiquidCrystal kirjasto ohjaa näitä. */
#define LCD_RS 7  // PD7
#define LCD_CL 8  // PB0
#define LCD_B4 A0 // PC0
#define LCD_B5 A1 // PC1
#define LCD_B6 A2 // PC2
#define LCD_B7 A3 // PD3

/* tone() ohjaa tätä */
#define BUZZER 3 // PD3

/* Muut pinnit */
DEFINE_PIN(LCD_AN, D, 4); // PD4
DEFINE_PIN(BTN_LT, D, 1); // PD1, (vasen)
DEFINE_PIN(BTN_RT, D, 0); // PD1, (oikea)

#endif // !PINS_H
