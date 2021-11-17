# 1 "/home/jovaska/arduino_project/src/main.c"
# 1 "/home/jovaska/arduino_project/src/main.c"
# 2 "/home/jovaska/arduino_project/src/main.c" 2
# 3 "/home/jovaska/arduino_project/src/main.c" 2
# 4 "/home/jovaska/arduino_project/src/main.c" 2

struct data_point
{
 float T; /* lämpötila */
 uint16_t S; /* näytearvo */
} __attribute__((packed));


struct data_point data[2] = {
 {-50.5, 600},
 {79.8, 2905}
};

inline float pns_temp(uint16_t s)
{
 float A, B, Sx, Sy, Sxx, Sxy, Syy;

 Sx = Sy = Sxx = Sxy = Syy = 0;
 for (size_t i = 0; i < 2; i++)
 {
  float tmp;

  tmp = (float)data[i].S;

  Sx += tmp;
  Sy += data[i].T;
  Sxy += tmp*data[i].T;
  Sxx += tmp*tmp;
  Syy += data[i].T*data[i].T;
 }

 A = (Sxx*Sy - Sx*Sxy)/(2*Sxx - Sx*Sx);
 B = (2*Sxy - Sx*Sy)/(2*Sxx - Sx*Sx);

 return A + B*s;
}

inline uint8_t spi_txrx(uint8_t v)
{
 /* kirjoitettava arvo */
 
# 44 "/home/jovaska/arduino_project/src/main.c" 3
(*(volatile uint8_t *)((0x2E) + 0x20)) 
# 44 "/home/jovaska/arduino_project/src/main.c"
     = v;

 /* odota kunnes valmis */
 while(!(
# 47 "/home/jovaska/arduino_project/src/main.c" 3
       (*(volatile uint8_t *)((0x2D) + 0x20)) 
# 47 "/home/jovaska/arduino_project/src/main.c"
       & (1 << (
# 47 "/home/jovaska/arduino_project/src/main.c" 3
       7
# 47 "/home/jovaska/arduino_project/src/main.c"
       ))));

 /* palauta luettu arvo */
 return 
# 50 "/home/jovaska/arduino_project/src/main.c" 3
       (*(volatile uint8_t *)((0x2E) + 0x20))
# 50 "/home/jovaska/arduino_project/src/main.c"
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
# 107 "/home/jovaska/arduino_project/src/main.c"
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

 /* SPI pinmodet */
 pinMode(10 /* PB2 */, 0x1);
 pinMode(13 /* PB5 */, 0x1);
 pinMode(11 /* PB3 */, 0x1);
 pinMode(12 /* PB4 */, 0x0);

 /* ATMega328P:n kaksi pinnikeskeytystä */




 /* taustavalo päälle */
 digitalWrite(4 /* PD4 */, 0x1);

 /* SS pois päältä */
 digitalWrite(10 /* PB2 */, 0x1);

 /* alusta SPI */
 
# 148 "/home/jovaska/arduino_project/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20)) 
# 148 "/home/jovaska/arduino_project/src/main.c"
     = 0;
 ((
# 149 "/home/jovaska/arduino_project/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 149 "/home/jovaska/arduino_project/src/main.c"
) |= 1 << (
# 149 "/home/jovaska/arduino_project/src/main.c" 3
6
# 149 "/home/jovaska/arduino_project/src/main.c"
));
 ((
# 150 "/home/jovaska/arduino_project/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 150 "/home/jovaska/arduino_project/src/main.c"
) |= 1 << (
# 150 "/home/jovaska/arduino_project/src/main.c" 3
4
# 150 "/home/jovaska/arduino_project/src/main.c"
));
 ((
# 151 "/home/jovaska/arduino_project/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 151 "/home/jovaska/arduino_project/src/main.c"
) |= 1 << (
# 151 "/home/jovaska/arduino_project/src/main.c" 3
3
# 151 "/home/jovaska/arduino_project/src/main.c"
));
 ((
# 152 "/home/jovaska/arduino_project/src/main.c" 3
(*(volatile uint8_t *)((0x2C) + 0x20))
# 152 "/home/jovaska/arduino_project/src/main.c"
) |= 1 << (
# 152 "/home/jovaska/arduino_project/src/main.c" 3
0
# 152 "/home/jovaska/arduino_project/src/main.c"
));
 //SET(SPCR, CPHA);

 /* paluupaikka keskeytyksistä, ei haluta että
	 * keskeytys palaa johonkin silmukan keskelle
	 */
 (void)setjmp(jmp);

 /* setup() END */

 while (1)
 {
  uint8_t tmp;
  uint16_t res;
  float T;

  /* loop() BEGIN */

  /* lue näyte AD-muuntimelta */
  digitalWrite(10 /* PB2 */, 0x0);
  (void)spi_txrx(0b00000001);
  tmp = spi_txrx(0b00100000);
  res = (uint16_t)(tmp & 0b00001111) << 8;
  tmp = spi_txrx(0);
  res |= tmp;
  digitalWrite(10 /* PB2 */, 0x1);

  delay(250);

  /* muutos lämpötilaksi */
  T = pns_temp(res);

  lcd.clear();
  lcd.print(T);
# 194 "/home/jovaska/arduino_project/src/main.c"
  /* loop() END */
# 203 "/home/jovaska/arduino_project/src/main.c"
 }
}
