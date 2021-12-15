#if 0
#ifndef LiquidCrystal_h
#define LiquidCrystal_h

#include <inttypes.h>
#include "Print.h"

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

class LiquidCrystal : public Print {
public:
  LiquidCrystal(uint8_t rs, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
		uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
  LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
		uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
  LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
  LiquidCrystal(uint8_t rs, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);

  void init(uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
	    uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
	    uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
    
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);

  void clear();
  void home();

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void autoscroll();
  void noAutoscroll();

  void setRowOffsets(int row1, int row2, int row3, int row4);
  void createChar(uint8_t, uint8_t[]);
  void setCursor(uint8_t, uint8_t); 
  virtual size_t write(uint8_t);
  void command(uint8_t);
  
  using Print::write;
private:
  void send(uint8_t, uint8_t);
  void write4bits(uint8_t);
  void write8bits(uint8_t);
  void pulseEnable();

  uint8_t _rs_pin; // LOW: command.  HIGH: character.
  uint8_t _rw_pin; // LOW: write to LCD.  HIGH: read from LCD.
  uint8_t _enable_pin; // activated by a HIGH pulse.
  uint8_t _data_pins[8];

  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _initialized;

  uint8_t _numlines;
  uint8_t _row_offsets[4];
};

#endif
#endif

/* LiquidCrystal kirjasto portattu C:ksi ja käyttämään tämän
 * kansion sisältämää koodia Arduinon standardikirjaston sijasta.
 */
#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include <stdbool.h>

/* takaisinkutsujen määrittämiseen */
#define LCD_RS 0x0
#define LCD_RW 0x1
#define LCD_EN 0x2
#define LCD_B1 0x3
#define LCD_B2 0x4
#define LCD_B3 0x5
#define LCD_B4 0x6
#define LCD_B5 0x7
#define LCD_B6 0x8
#define LCD_B7 0x9
#define LCD_B8 0xA

typedef void (*pin_t)(bool);

struct lcd
{
	/* takaisinkutsut pinneille kirjoittamiseen */
	pin_t pins[11]; 

	/* kaikkien komentojen tilat
	 * yhdessä bittivektorissa
	 */
	uint16_t mode;
};

/* komennot */
#define LCD_CLDP 0x01 // tyhjennä näyttö
#define LCD_HOME 0x02 // siirrä kursori yläkulmaan
#define LCD_EDMS 0x04 // aseta kirjoitustila
#define LCD_DPCT 0x08 // aseta tekstin ominaisuudet
#define LCD_CRST 0x10 // aseta kursorin ominaisuudet
#define LCD_FCNS 0x20 // aseta näytön ominaisuudet
#define LCD_CRAS 0x40 // aseta CGRAM osoite
#define LCD_DRAS 0x80 // aseta DDRAM osoite

#define OFF(A) \
	(8*sizeof(int) - __builtin_clz(A))
#define MSK(A, B) \
	(((1 << (A)) - 1) << (B))

#define DM_OFF OFF(0)
#define DM_MSK MSK(2, DM_OFF)
#define DC_OFF OFF(DM_MSK)
#define DC_MSK MSK(3, DC_OFF)
#define CS_OFF OFF(DC_MSK)
#define CS_MSK MSK(4, CS_OFF)
#define FS_OFF OFF(CS_MSK)
#define FS_MSK MSK(5, FS_OFF)
#define IN_OFF OFF(FS_MSK)
#define IN_MSK MSK(1, IN_OFF)

/* indikaattori ja sen komplementti */
#define FLAG(A, B) ((A) << (B))
#define COMP(A, B) (FLAG(A, B) | 0x8000)

/* indikaattorit kirjoitustilalle */
#define LCD_ETRT COMP(0x02, DM_OFF) // oikealta vasemmalle
#define LCD_ENLT FLAG(0x02, DM_OFF) // vasemmalta oikealle
#define LCD_ESIC FLAG(0x01, DM_OFF) // siirrä kursoria oikealle
#define LCD_ESDC COMP(0x01, DM_OFF) // siirrä kursoria vasemmalle

/* indikaattorit tekstille */
#define LCD_DPON FLAG(0x04, DC_OFF) // teksti päälle
#define LCD_DPOF COMP(0x04, DC_OFF) // teksti pois päältä
#define LCD_CRON FLAG(0x02, DC_OFF) // kursori päälle
#define LCD_CROF COMP(0x02, DC_OFF) // kursori pois päältä
#define LCD_BLON FLAG(0x01, DC_OFF) // välkytä kursoria
#define LCD_BLOF COMP(0x01, DC_OFF) // älä välkytä kursoria

/* indikaattorit kursorille */
#define LCD_DPMV FLAG(0x08, CS_OFF) // siirrä tekstiä
#define LCD_CRMV COMP(0x08, CS_OFF) // siirrä kursoria
#define LCD_MVRT FLAG(0x04, CS_OFF) // siirrä oikealle
#define LCD_MVLT COMP(0x04, CS_OFF) // siirrä vasemmalle

/* indikaattorit näytölle */
#define LCD_8BIT FLAG(0x10, FS_OFF) // 8-bitin tila
#define LCD_4BIT COMP(0x10, FS_OFF) // 4-bitin tila
#define LCD_2LIN FLAG(0x08, FS_OFF) // 2 rivin tila
#define LCD_1LIN COMP(0x08, FS_OFF) // 1 rivin tila
#define LCD_XDOT FLAG(0x04, FS_OFF) // 5x10 kirjaimet
#define LCD_8DOT COMP(0x04, FS_OFF) // 5x8 kirjaimet

#define LCD_INIT FLAG(0x01, IN_OFF) // pakota alustus

#define LCD(var) \
	struct lcd var = {{0}, LCD_INIT}

/* tyhjennä näyttö */
void lcd_clear(struct lcd *);

/* siirrä kursori yläkulmaan */
void lcd_home(struct lcd *);

/* aseta näytön tila */
void lcd_mode(struct lcd *, ...);

/* lisää kirjain (vaatii RS pinnin) */
void lcd_mkchar(struct lcd *, uint8_t, uint8_t []);

/* aseta kursorin paikka */
void lcd_cursor(struct lcd *, uint8_t, uint8_t); 

/* kirjoita näytölle */
void lcd_write(struct lcd *, uint8_t);

#endif // !LCD_H
