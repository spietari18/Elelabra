#include <Arduino.h>
#include <LiquidCrystal.h>

#include <setjmp.h>
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

/* Muut pinnit */
DEFINE_PIN(LCD_AN, B, 4); // PD4
DEFINE_PIN(BUZZER, B, 1); // PB1
DEFINE_PIN(BTN_LT, D, 1); // PD1
DEFINE_PIN(BTN_RT, D, 2); // PD2

/* paluupiste pääsilmukkaan */
jmp_buf jmp_loop;

/* kuinka monta näytepistettä
 * voi olla maksimissaan muistissa
 */
#define MAX_POINTS 32

/* näytepisteet */
uint8_t num_points;
float points[MAX_POINTS][2];

/* sisäänrakennetut kalibrointipisteet
 * (Laitettu ROM:iin koska säästää RAM muistia)
 */
const float points_builtin[][2] PROGMEM = {
	{-51.70,  684},
	{ 79.76, 2496}
};

/* PNS kertoimet */
float lss_coefs[2];

#define MAX_SAMPLES 32  // monta näytettä muistissa kerralla
#define SAMPLE_RATE 128 // näytteistystaajuus [Hz]

/* AD muuntimelta luetut näytteet */
uint8_t sample_pos;
float samples[MAX_SAMPLES];

#define LCD_ROWS 2  // näytön rivit
#define LCD_COLS 16 // näytön sarakkeet

/* näytön puskuri (näyttö alkaa tyhjänä) */
char lcd_buffer[2*LCD_ROWS][LCD_COLS] = {
	/* nykyinen tila */
	{"                "},
	{"                "},
	/* tila viimeisen lcd_update():en jälkeen */
	{"                "},
	{"                "}
};

/* näyttö */
LiquidCrystal lcd(LCD_RS, LCD_CL, LCD_B4, LCD_B5, LCD_B6, LCD_B7);

/* asteta sisäänrakennetut
 * oletusarvot pistejoukolle
 */
void
default_points()
{
	num_points = sizeof(points_builtin)/sizeof(points_builtin[0][0]);
	(void)memcpy_P(points, points_builtin, sizeof(points_builtin));
}

/* laske PNS kertoimet pistejoukolle */
void
compute_lss_coefs()
{
	float Sx = 0, Sy = 0, Sxx = 0, Sxy = 0, Syy = 0;

	for (uint8_t i = 0; i < num_points; i++)
	{
		Sx  += points[i][0];
		Sy  += points[i][1];
		Sxy += points[i][0]*points[i][1];
		Sxx += points[i][0]*points[i][0];
		Syy += points[i][1]*points[i][1];
	}

	lss_coefs[0] = (Sxx*Sy - Sx*Sxy)/(num_points*Sxx - Sx*Sx);
	lss_coefs[1] = (num_points*Sxy - Sx*Sy)/(num_points*Sxx - Sx*Sx);
}

/* muuta AD muuntimen näyte lämpötilaksi
 * lineaarisella PNS sovituksella
 */
float
compute_temp(float s)
{
	return lss_coefs[0] + lss_coefs[1]*s;
}

/* lue yksi tavu SPI:llä */
uint8_t
spi_byte(uint8_t v)
{
	/* kirjoitettava arvo */
	SPDR = v;

	/* odota kunnes valmis */
	while (!GET(SPSR, SPIF));

	/* palauta luettu arvo */
	return SPDR;
}

/* päivitä lämpötilalukema */
float
temp_update()
{
	uint16_t tmp = 0;
	float res = 0.0;

	/* lue näyte AD-muuntimelta */
	WRITE(SPI_SS, LOW);
	(void)spi_byte(0b00000001);
	tmp |= (uint16_t)spi_byte(0b00100000) << 8;
	tmp |= spi_byte(0);
	WRITE(SPI_SS, HIGH);

	/* näyte on 12 bittiä */
	tmp &= (1 << 12) - 1;

	/* lisää kiertopuskuriin */
	samples[sample_pos] = tmp;
	sample_pos = (sample_pos + 1) % MAX_SAMPLES;
	
	/* laske puskurin keskiarvo */
	for (uint8_t i = 0; i < MAX_SAMPLES; i++)
		res += samples[i];
	res /= MAX_SAMPLES;

	return compute_temp(res);
}

/* päivitä näyttö vastaamaan puskurin sisältöä */
void
lcd_update()
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

uint32_t last_sample;

void __attribute__((noreturn))
entry_point()
{
	uint32_t tmp;

	/* aseta käytetyt moduulit päälle ja muut pois päältä */
	PRR = ~0;
	CLR(PRR, PRTIM0); /* TIMER0 päälle */
	//CLR(PRR, PRTIM1); /* TIMER1 päälle */
	CLR(PRR, PRSPI);  /* SPI päälle */

	/* alusta kaikki IO pinnit INPUT PULLUP tilaan
	 * jotta käyttämättömät pinnit eivät kellu.
	 * aseta myös kaikkien pinnien arvoksi LOW
	 */
	DDRB  = DDRC  = DDRD  =  0; // INPUT
	PORTB = PORTC = PORTD = ~0; // PULLUP
	PINB  = PINC  = PIND  =  0; // LOW

	/* alusta SPI (tarpeellinen tehdä vain kerran koska
	 * AD muunnin on ainoa kytketty SPI laite)
	 */
	SPCR = 0;
	SET(SPCR, SPE);
	SET(SPCR, MSTR);
	SET(SPCR, CPOL);
	SET(SPCR, SPR0);
	WRITE(SPI_SS, HIGH);
	MODE(SPI_SS, OUTPUT);
	MODE(SPI_CLK, OUTPUT);
	MODE(SPI_MOSI, OUTPUT);
	PLUP(SPI_MISO, 0);

	/* alusta näyttö (LiquidCrystal kutsuu
	 * pinMode():a vasta tässä vaiheessa)
	 */
	lcd.begin(LCD_COLS, LCD_ROWS);

	/* alusta muut pinnit, napeille voidaan käyttää
	 * sisäisiä ylosvetovastuksia joten niiden
	 * modeihin ei tarvitse puuttua.
	 */
	MODE(BUZZER, OUTPUT);
	WRITE(LCD_AN, HIGH); // taustavalo päälle
	MODE(LCD_AN, OUTPUT);

	/* alusta PNS malli */
	default_points();
	compute_lss_coefs();

	/* näytön staattinen teksti */
	(void)memcpy(&lcd_buffer[0][0], "T=", 2);
	(void)memcpy(&lcd_buffer[1][0], "S=", 2);

	/* paluu keskeytyksistä tähän */
	(void)setjmp(jmp_loop);
main_loop:
	/* lue uusi näyte? */
	tmp = millis();
	if ((tmp - last_sample) > 1000/SAMPLE_RATE) {
		float V;

		/* päivitä lämpötila */
		V = temp_update();
		(void)sprintf_P(&lcd_buffer[0][2],
			(const char *)PSTR("%.1f"), V);

		/* päivitä näytearvo */
		V = samples[(MAX_SAMPLES - 1
			+ sample_pos) % MAX_SAMPLES];
		(void)sprintf_P(&lcd_buffer[1][2],
			(const char *)PSTR("%u"), (unsigned int)V);

		lcd_update();

		last_sample = tmp;
	}

	goto main_loop;
}
