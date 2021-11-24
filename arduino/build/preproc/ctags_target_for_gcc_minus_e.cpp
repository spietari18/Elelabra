# 1 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
# 1 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
# 2 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 2
# 3 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 2
# 4 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 2

struct data_point
{
 float T; /* lämpötila */
 float S; /* näytearvo */
} __attribute__((packed));


struct data_point data[2] = {
 {-51.7, 684},
 {79.76, 2496}
};

inline float pns_temp(uint16_t s)
{
 float A, B, Sx, Sy, Sxx, Sxy, Syy;

 Sx = Sy = Sxx = Sxy = Syy = 0;
 for (size_t i = 0; i < 2; i++)
 {
  Sx += data[i].S;
  Sy += data[i].T;
  Sxy += data[i].S*data[i].T;
  Sxx += data[i].S*data[i].S;
  Syy += data[i].T*data[i].T;
 }

 A = (Sxx*Sy - Sx*Sxy)/(2*Sxx - Sx*Sx);
 B = (2*Sxy - Sx*Sy)/(2*Sxx - Sx*Sx);

 return A + B*s;
}

inline uint8_t spi_byte(uint8_t v)
{
 /* kirjoitettava arvo */
 
# 40 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
(*(volatile uint8_t *)((0x2E) + 0x20)) 
# 40 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
     = v;

 /* odota kunnes valmis */
 while(!(
# 43 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
       (*(volatile uint8_t *)((0x2D) + 0x20)) 
# 43 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
       & (1 << (
# 43 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
       7
# 43 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
       ))));

 /* palauta luettu arvo */
 return 
# 46 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
       (*(volatile uint8_t *)((0x2E) + 0x20))
# 46 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
           ;
}

/* arvo joka näytöllä on */
int val_now = 0;
int val_old = -1;

jmp_buf jmp;

/* nappien debounce softassa: odotetaan BTN_WAIT
 * millisekuntia kunnes uusi painallus on mahdollista
 * rekisteröidä.
 */

uint32_t last_poll = 0;

/* lisää 1 arvoon */
void inc()
{
 uint32_t tmp;

 tmp = millis();
 if (last_poll > (tmp - 100))
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
 if (last_poll > (tmp - 100))
  return;
 last_poll = tmp;

 val_now--;
 longjmp(jmp, 0);
}
# 105 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
float sample_buffer[8];
uint8_t buffer_pos;

uint16_t old_S;
float old_T;


uint32_t switch_time;
bool state;

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
 LiquidCrystal lcd(7 /* PD7 */, 8 /* PB0 */, A0 /* PC0 */, A1 /* PC1 */, A2 /* PC2 */, A3 /* PD3 */);

 /* alusta näyttö */
 lcd.begin(16, 2);

 /* muut pinmodet */




 pinMode(4 /* PD4 */, 0x1);
 pinMode(9 /* PB1 */, 0x1);

 /* SPI pinmodet */
 pinMode(10 /* PB2 */, 0x1);
 pinMode(13 /* PB5 */, 0x1);
 pinMode(11 /* PB3 */, 0x1);
 pinMode(12 /* PB4 */, 0x0);

 /* ATMega328P:n kaksi pinnikeskeytystä */




 /* taustavalo päälle */
 digitalWrite(4 /* PD4 */, 0x1);

 digitalWrite(9 /* PB1 */, 0x0);

 /* SS pois päältä */
 digitalWrite(10 /* PB2 */, 0x1);

 /* alusta SPI */
 
# 159 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20)) 
# 159 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
     = 0;
 ((
# 160 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 160 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
) |= 1 << (
# 160 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
6
# 160 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
));
 ((
# 161 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 161 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
) |= 1 << (
# 161 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
4
# 161 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
));
 ((
# 162 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 162 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
) |= 1 << (
# 162 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
3
# 162 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
));
 ((
# 163 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 163 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
) |= 1 << (
# 163 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c" 3
0
# 163 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
));
 //SET(SPCR, CPHA);

 /* näytön staattinen teksti */
 lcd.setCursor(0, 0);
 lcd.print("T = ");
 lcd.setCursor(0, 1);
 lcd.print("S = ");

 tone(9 /* PB1 */, 1000);

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
  digitalWrite(10 /* PB2 */, 0x0);
  (void)spi_byte(0b00000001);
  tmp = spi_byte(0b00100000);
  S = (uint16_t)(tmp & 0b00001111) << 8;
  tmp = spi_byte(0);
  S |= tmp;
  digitalWrite(10 /* PB2 */, 0x1);

  delay(10);

  sample_buffer[buffer_pos] = S;
  buffer_pos = (buffer_pos + 1) % 8;

  V = 0;
  for (uint8_t i = 0; i < 8; i++)
   V += sample_buffer[i];
  V /= 8;

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
# 234 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
  /*
		uint32_t t = millis();
		if ((t - switch_time) > INTERVAL) {
			if (state)
				tone(BUZZER, 1000);
			else
				noTone(BUZZER);
			state = !state;
			t = switch_time;
		}
		*/

  /* loop() END */
# 255 "/home/jovaska/Desktop/ELATY-GIT/arduino/src/main.c"
 }
}
