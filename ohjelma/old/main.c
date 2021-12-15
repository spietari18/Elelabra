#include <setjmp.h>
#include <LiquidCrystal.h>
#include "macro.h"

struct data_point
{
	float T; /* lämpötila */
	float S; /* näytearvo */
} __attribute__((packed));

#define POINTS 2
struct data_point data[POINTS] = {
	{-51.7, 684},
	{79.76, 2496}
};

inline float pns_temp(uint16_t s)
{
	float A, B, Sx, Sy, Sxx, Sxy, Syy;

	Sx = Sy = Sxx = Sxy = Syy = 0;
	for (size_t i = 0; i < POINTS; i++)
	{
		Sx  += data[i].S;
		Sy  += data[i].T;
		Sxy += data[i].S*data[i].T;
		Sxx += data[i].S*data[i].S;
		Syy += data[i].T*data[i].T;
	}

	A = (Sxx*Sy - Sx*Sxy)/(POINTS*Sxx - Sx*Sx);
	B = (POINTS*Sxy - Sx*Sy)/(POINTS*Sxx - Sx*Sx);

	return A + B*s;
}

inline uint8_t spi_byte(uint8_t v)
{
	/* kirjoitettava arvo */
	SPDR = v;

	/* odota kunnes valmis */
	while(!GET(SPSR, SPIF));

	/* palauta luettu arvo */
	return SPDR;
}

/* arvo joka näytöllä on */
int val_now =  0;
int val_old = -1;

jmp_buf jmp;

/* nappien debounce softassa: odotetaan BTN_WAIT
 * millisekuntia kunnes uusi painallus on mahdollista
 * rekisteröidä.
 */
#define BTN_WAIT 100
uint32_t last_poll = 0;

/* lisää 1 arvoon */
void inc()
{
	uint32_t tmp;

	tmp = millis();
	if (last_poll > (tmp - BTN_WAIT))
		return;
	last_poll = tmp;

	val_now++;
	longjmp(jmp, 0);
}

/* vähennä 1 arvosta */
void dec()
{
	uint32_t tmp;

	tmp = millis();
	if (last_poll > (tmp - BTN_WAIT))
		return;
	last_poll = tmp;

	val_now--;
	longjmp(jmp, 0);
}

#define LCD_RS 7  /* PD7 */
#define LCD_CL 8  /* PB0 */
#define LCD_B4 A0 /* PC0 */
#define LCD_B5 A1 /* PC1 */
#define LCD_B6 A2 /* PC2 */
#define LCD_B7 A3 /* PD3 */
#define LCD_AN 4  /* PD4 */
#define BUZZER 9  /* PB1 */

#define SPI_SS   10 /* PB2 */ 
#define SPI_CLK  13 /* PB5 */ 
#define SPI_MOSI 11 /* PB3 */
#define SPI_MISO 12 /* PB4 */

#define BUF_SIZE 8
float sample_buffer[BUF_SIZE];
uint8_t buffer_pos;

uint16_t old_S;
float old_T;

/* Tässä ei muuta eroa normaaliin Arduino alustan pohjakoodiin kun että
 * setup() ja loop() on muutettu yhdeksi funktioksi entry_point() johon
 * pitää kirjoittaa oma while(1) silmukka. Tämä sen takia, että setjmp.h
 * funktioita voi käyttää. (ei toimi koska jos kutsuu setjmp() loop())
 * funktiossa, konteksti häviää kun loop() palaa Arduinon main() funktioon.
 */
void entry_point()
{
	/* setup() BEGIN */

	/* tämä asettaa pinmodet itse */
	LiquidCrystal lcd(LCD_RS, LCD_CL, LCD_B4, LCD_B5, LCD_B6, LCD_B7);

	/* alusta näyttö */
	lcd.begin(16, 2);

	/* muut pinmodet */
#if 0
	pinMode(2, INPUT);
	pinMode(3, INPUT);
#endif
	pinMode(LCD_AN, OUTPUT);
	pinMode(BUZZER, OUTPUT);

	/* SPI pinmodet */
	pinMode(SPI_SS, OUTPUT);
	pinMode(SPI_CLK, OUTPUT);
	pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_MISO, INPUT);

	/* ATMega328P:n kaksi pinnikeskeytystä */
#if 0
	attachInterrupt(0, inc, FALLING); /* nappi 1 */
	attachInterrupt(1, dec, FALLING); /* nappi 2 */
#endif
	/* taustavalo päälle */
	digitalWrite(LCD_AN, HIGH);

	/* SS pois päältä */
	digitalWrite(SPI_SS, HIGH);

	/* alusta SPI */
	SPCR = 0;
	SET(SPCR, SPE);
	SET(SPCR, MSTR);
	SET(SPCR, CPOL);
	SET(SPCR, SPR0);
	//SET(SPCR, CPHA);

	/* näytön staattinen teksti */
	lcd.setCursor(0, 0);
	lcd.print("T = ");
	lcd.setCursor(0, 1);
	lcd.print("S = ");

	tone(BUZZER, 1000);

	/* paluupaikka keskeytyksistä, ei haluta että
	 * keskeytys palaa johonkin silmukan keskelle
	 */
	(void)setjmp(jmp);

	/* setup() END */

	while (1)
	{
		uint8_t tmp;
		uint16_t S;
		float T, V;

		/* loop() BEGIN */

		/* lue näyte AD-muuntimelta */
		digitalWrite(SPI_SS, LOW);
		(void)spi_byte(0b00000001);
		tmp = spi_byte(0b00100000);
		S = (uint16_t)(tmp & 0b00001111) << 8;
		tmp = spi_byte(0);
		S |= tmp;
		digitalWrite(SPI_SS, HIGH);

		delay(10);

		sample_buffer[buffer_pos] = S;
		buffer_pos = (buffer_pos + 1) % BUF_SIZE;

		V = 0;
		for (uint8_t i = 0; i < BUF_SIZE; i++)
			V += sample_buffer[i];
		V /= BUF_SIZE;

		/* muutos lämpötilaksi */
		T = pns_temp(V);

		if (T != old_T) {
			lcd.setCursor(4, 0);
			lcd.print("      ");
			lcd.setCursor(4, 0);
			lcd.print(T);
			old_T = T;
		}
		if (S != old_S) {
			lcd.setCursor(4, 1);
			lcd.print("      ");
			lcd.setCursor(4, 1);
			lcd.print(S);
			old_S = S;
		}
#if 0
		/* tulosta arvo näytölle kun se muuttuu */
		if (val_now != val_old) {
			lcd.clear();
			lcd.print(val_now);
			val_old = val_now;
		}
#endif


		/* loop() END */

#ifdef ENABLE_SERIAL	
		/* tämä on tarpeellinen vain jos
		 * käytetään seriaalimonitoria
		 */
		if (serialEventRun)
			serialEventRun();
#endif
	}
}

