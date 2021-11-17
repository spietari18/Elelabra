#include <setjmp.h>
#include <LiquidCrystal.h>

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

/* Tässä ei muuta eroa normaaliin Arduino alustan pohjakoodiin kun että
 * setup() ja loop() on muutettu yhdeksi funktioksi entry_point() johon
 * pitää kirjoittaa oma while(1) silmukka. Tämä sen takia, että setjmp.h
 * funktioita voi käyttää. (ei toimi normaalisti, koska jos kutsuu setjmp() loop()
 * funktiossa, konteksti häviää kun loop() palaa Arduinon main() funktioon.)
 */
void entry_point()
{
	/* setup() BEGIN */

	/* tämä asettaa pinmodet itse */
	LiquidCrystal lcd(LCD_RS, LCD_CL, LCD_B4, LCD_B5, LCD_B6, LCD_B7);

	/* alusta näyttö */
	lcd.begin(16, 2);

	/* muut pinmodet */
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pinMode(LCD_AN, OUTPUT);

	/* ATMega328P:n kaksi pinnikeskeytystä */
	attachInterrupt(0, inc, FALLING); /* nappi 1 */
	attachInterrupt(1, dec, FALLING); /* nappi 2 */

	/* taustavalo päälle */
	digitalWrite(LCD_AN, HIGH);

	/* paluupaikka keskeytyksistä, ei haluta että
	 * keskeytys palaa johonkin silmukan keskelle
	 */
	(void)setjmp(jmp);

	/* setup() END */

	while (1)
	{
		/* loop() BEGIN */

		/* tulosta arvo näytölle kun se muuttuu */
		if (val_now != val_old) {
			lcd.clear();
			lcd.print(val_now);
			val_old = val_now;
		}

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
