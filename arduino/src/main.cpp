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

/* tone() ohjaa tätä */
#define BUZZER 3 // PD3

/* Muut pinnit */
DEFINE_PIN(LCD_AN, D, 4); // PD4
DEFINE_PIN(BTN_LT, D, 0); // PD0, (kauempi)
DEFINE_PIN(BTN_RT, D, 1); // PD1, (lähempi)

/* paluupiste pääsilmukkaan */
jmp_buf jmp_loop;

/* kuinka monta näytepistettä
 * voi olla maksimissaan muistissa
 */
#define MAX_POINTS 32

/* näytepisteet */
uint8_t num_points;
float points[MAX_POINTS][2];

#define T_ABS_MIN -50.0
#define T_ABS_MAX  80.0

/* sisäänrakennetut kalibrointipisteet
 * (Laitettu ROM:iin koska säästää RAM muistia)
 */
const float points_builtin[][2] PROGMEM = {
	/* S, T */
	{684.0, -51.70},
	{2496.0, 79.76}
};

/* PNS kertoimet */
float lss_coefs[2];

#define MAX_SAMPLES 128 // monta näytettä muistissa kerralla
#define SAMPLE_RATE 64  // näytteistystaajuus [Hz]

#define SAMPLE_MAX_DELTA 4.0

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
	num_points = sizeof(points_builtin)/sizeof(points_builtin[0]);
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

int
float_cmp(const void *a, const void *b)
{
	return *(float *)a - *(float *)b;
}

/* päivitä lämpötilalukema */
float
temp_update()
{
	float buffer[MAX_SAMPLES];
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

	(void)memcpy(buffer, samples, MAX_SAMPLES*sizeof(samples[0]));

	/* järjestä puskuri numerojärkestykseen */
	qsort(buffer, MAX_SAMPLES, sizeof(buffer[0]), float_cmp);

	/* laske kiertopuskurin moodi */
	float mode;
#if (MAX_SAMPLES & 1)
	mode = buffer[MAX_SAMPLES/2];
#else
	mode = (buffer[MAX_SAMPLES/2] + buffer[MAX_SAMPLES/2 + 1])/2;
#endif

#if 0
	char buf[8];
	uint8_t len;
        len = sprintf_P(buf, PSTR("%.1f"), (float)mode);
	(void)memset(&lcd_buffer[1][7], ' ', 8);
	(void)memcpy(&lcd_buffer[1][LCD_COLS - len], buf, len);
#endif

	/* suodata pois arvot, jotka ovat moodia SAMPLE_MAX_DELTA
	 * suurempia tai pienempiä. (poistaa äkilliset vaihtelut)
	 */
	uint8_t beg = 0;
	uint8_t end = MAX_SAMPLES - 1;
	while ((beg < MAX_SAMPLES/2) &&
		(abs(mode - buffer[beg]) > SAMPLE_MAX_DELTA))
		beg++;
	while ((end > MAX_SAMPLES/2) &&
		(abs(mode - buffer[end]) > SAMPLE_MAX_DELTA))
		end--;

	/* laske puskurin keskiarvo */
	for (uint8_t i = beg; i <= end; i++)
		res += buffer[i];
	res /= end - beg + 1;

#if 0
	len = sprintf_P(buf, PSTR("%u"), (uint16_t)res);
	(void)memset(&lcd_buffer[0][7], ' ', 8);
	(void)memcpy(&lcd_buffer[0][LCD_COLS - len], buf, len);
#endif

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

/* Kauan nappien tilaa luetaan ennekuin se lukitaan. */
#define BTN_POLL_TIME 250 // [ms]

/* Kauan nappien tila voi poiketa lukitusta tilasta
 * ennekuin tilan muutos rekisteröidään uutena tapahtumana.
 */
#define BTN_JITTER_TIME 150 // [ms]

/* Kauan nappeja täytyy pitää pohjassa, että
 * HOLD tapahtuma aktivoituu. (ja HLUP UP:in sijasta)
 */
#define BTN_HOLD_TIME 900 // [ms]

/* Kuinka kauan odotetaan UP tai HLUP tapahtuman
 * jälkeen ennenkuin nappien tilaa aletaan lukemaan
 */
#define BTN_INACTIVE_TIME 200 // [ms]

#define BTN_RT_OFF 0
#define BTN_LT_OFF 1

#define RT (1 << BTN_RT_OFF)
#define LT (1 << BTN_LT_OFF)

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

/*  MSB .............................................. LSB
 *    1          2           1          2            2
 * |UNUSED|STATE MACHINE|HOLD FLAG|LOCKED KEYS|POLLED KEYS|
 */
uint8_t button_state;

uint32_t ts_1;
uint32_t ts_2;

/* Päivitä nappien tila ja palauta bittivektori joka kuvaa onko
 * nappi/nappien yhdistelmä painettu alas (DOWN), sitä pidetään
 * alhaalla (HOLD), se on vapautettu nopeasti (UP) tai se
 * vapautetaan HOLD tapahtuman jälkeen (HLUP). Koodissa ei ole
 * kommentteja, mutta alla pseudokoodi (englanniksi).
 * 
 * START:
 * WAIT FOR INITIAL KEYPRESS
 * WAIT BTN_POLL_TIME WHILE OR'ING THE POLLED STATE TO THE INITIAL STATE
 * LOCK STATE
 * DOWN EVENT
 * IF POLLED STATE CHANGES FROM LOCKED STATE FOR LONGER THAN BTN_POLL_TIME
 * 	UP EVENT
 * 	GOTO END
 * WAIT FOR BTN_HOLD_TIME
 * SET HOLD FLAG
 * HOLD EVENT
 * IF POLLED STATE CHANGES FROM LOCKED STATE FOR LONGER THAN BTN_POLL_TIME
 * 	HLUP EVENT
 * END:
 * WAIT FOR BTN_IACT_TIME
 * RESET LOCKED STATE
 * GOTO START
 */
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

#define DISPLAY_FOR 1000
uint32_t last_msg;

void
button_test()
{
	uint32_t tmp;

	tmp = millis();
	switch (button_update())
	{
#define Z 9
#define X(Y) \
	case Y:\
		(void)memset(&lcd_buffer[1][LCD_COLS - Z], ' ', Z); \
		(void)memcpy_P(&lcd_buffer[1][LCD_COLS - Z], \
			PSTR(#Y), sizeof(#Y) - 1); \
		lcd_update(); \
		last_msg = tmp; \
		break
#define BOTH RT|LT
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
#undef BOTH
#undef X
	default:
		if ((tmp - last_msg) > DISPLAY_FOR) {
			(void)memset(&lcd_buffer[1][LCD_COLS - Z], ' ', Z);
			last_msg = tmp;
		}
	}
#undef Z
}

/* nollaa mikrokontrolleri watchdog ajastimella */
void __attribute__((noreturn))
reset()
{
	WDTCSR = 0;
	SET(WDTCSR, WDE);
	while (1);
}

/* käyttöliittymän tilat */
#define UI_SPLASH  0 
#define UI_DEFAULT 1
#define UI_CONFIG  2
#define UI_CALIBR  3
#define UI_ERROR   4

/* monta bittiä tilassa on */
#define UI_BITS \
	(8*sizeof(int) - __builtin_clz(UI_ERROR))

#define UI_MASK_NOW \
	((1 << UI_BITS) - 1)
#define UI_MASK_OLD \
	(((1 << UI_BITS) - 1) << UI_BITS)

/* määrittää onko tila SETUP vai LOOP */
#define UI_BIT (1 << (UI_BITS << 1))

/* vaihda käyttöliittymän tilaa (UIT_BIT = 0) */
#define UI_SET_STATE(state) \
	(ui_state = ((ui_state & UI_MASK_OLD) << UI_BITS) | (UI_##state & UI_MASK_NOW))

/* palauta käyttöliittymän tila */
#define UI_GET_STATE \
	(ui_state & (~UI_MASK_OLD))

#define UI_SETUP_END \
	(ui_state |= UI_BIT)

#define UI_SETUP(state) \
	(UI_##state & UI_MASK_NOW)

#define UI_LOOP(state) \
	(UI_BIT | (UI_##state & UI_MASK_NOW))

#define SPLASH_WAIT 3000 // [ms]

#define DISPLAY_WAIT 3000 // [ms]

void __attribute__((noreturn))
entry_point()
{
	uint32_t last_sample;
	uint32_t init;
	uint32_t tmp;
	char buf[8];
	uint8_t len;
	float V;

	uint8_t ui_state = 0;

	float obs_min =  1.0/0.0;
	float obs_max = -1.0/0.0;


	/* aseta käytetyt moduulit päälle ja muut pois päältä */
	PRR = ~0;
	CLR(PRR, PRTIM0); // TIMER0 päälle
	CLR(PRR, PRTIM2); // TIMER2 päälle
	CLR(PRR, PRSPI);  // SPI päälle

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
	WRITE(LCD_AN, HIGH);
	MODE(LCD_AN, OUTPUT);

	//tone(BUZZER, 1000);

	UI_SET_STATE(SPLASH);

	/* alusta PNS malli */
	default_points();
	compute_lss_coefs();

#if 0
	/* näytön staattinen teksti */
	(void)memcpy_P(&lcd_buffer[0][0], PSTR("T="), 2);
	(void)memcpy_P(&lcd_buffer[1][0], PSTR("S="), 2);

	len = sprintf_P(buf, PSTR("%u"), (unsigned int)num_points);
	(void)memset(&lcd_buffer[1][7], ' ', 8);
	(void)memcpy(&lcd_buffer[1][LCD_COLS - len], buf, len);
#endif

	/* paluu keskeytyksistä tähän */
	(void)setjmp(jmp_loop);
main_loop:
	
	switch (UI_GET_STATE) {
	case UI_SETUP(SPLASH):

#define SPLASH_1 "L\xE1mp\xEFmittari"
#define SPLASH_2 "v. 1.0"

		(void)memcpy_P(&lcd_buffer[0][(LCD_COLS - sizeof(SPLASH_1) + 2)/2],
				PSTR(SPLASH_1), sizeof(SPLASH_1) - 1);
		(void)memcpy_P(&lcd_buffer[1][(LCD_COLS - sizeof(SPLASH_2) + 2)/2],
				PSTR(SPLASH_2), sizeof(SPLASH_2) - 1);

		lcd_update();

		UI_SETUP_END;     
		break;

	case UI_LOOP(SPLASH):
		if (SPLASH_WAIT < millis())
			UI_SET_STATE(DEFAULT);
		break;
	
	/* oletusnäkymä */
	case UI_SETUP(DEFAULT):
		/* tyhjennä näyttö */
		(void)memset(lcd_buffer, ' ', LCD_ROWS*LCD_COLS);

		/* näytön staattinen teksti */
		(void)memcpy_P(&lcd_buffer[0][0], PSTR("MIN"), 3);
		(void)memcpy_P(&lcd_buffer[0][13], PSTR("MAX"), 3);

		init = millis();

		UI_SETUP_END;
		break;

	case UI_LOOP(DEFAULT):
		/* lue uusi näyte? */
		tmp = millis();
		if ((tmp - last_sample) > 1000/SAMPLE_RATE) {
			/* päivitä lämpötila */
			V = temp_update();

			if ((millis() - init) > DISPLAY_WAIT) {
				if (V > obs_max)
					obs_max = V;
				else if (V < obs_min)
					obs_min = V;

				len = sprintf_P(buf,
					(const char *)PSTR("%+.1f"), obs_min);
				(void)memcpy(&lcd_buffer[1][0], buf, len);

				len = sprintf_P(buf,
					(const char *)PSTR("%+.1f"), obs_max);
				(void)memcpy(&lcd_buffer[1][LCD_COLS
					- len], buf, len);

			}

			(void)memset(&lcd_buffer[0][5], ' ', 6);

			char *val;
			if (V < T_ABS_MIN) {
				(void)memcpy_P(&lcd_buffer[0][5], 
					PSTR("-LIMIT"), 6);
			} else if (V > T_ABS_MAX) {
				(void)memcpy_P(&lcd_buffer[0][5], 
					PSTR("+LIMIT"), 6);
			} else {
				len = sprintf_P(buf,
					(const char *)PSTR("%+.2f"), V);
				(void)memcpy(&lcd_buffer[0][(LCD_COLS
					- len)/2], buf, len);
			}
#if 0
			/* päivitä näytearvo */
			V = samples[(MAX_SAMPLES - 1
				+ sample_pos) % MAX_SAMPLES];
			len = sprintf_P(buf,
				(const char *)PSTR("%u"), (unsigned int)V);
			(void)memcpy(&lcd_buffer[1][2], buf, len);
#endif

			lcd_update();

			last_sample = tmp;
		}
#if 0
		len = sprintf_P(buf, PSTR("%lu"), millis());
		(void)memset(&lcd_buffer[0][0], ' ', 8);
		(void)memcpy(&lcd_buffer[0][0], buf, len);
		lcd_update();
#endif		
		break;
	
	case UI_SETUP(CALIBR):
		UI_SETUP_END;
		break;

	case UI_LOOP(CALIBR):
		break;
	
	case UI_SETUP(CONFIG):
		UI_SETUP_END;
		break;

	case UI_LOOP(CONFIG):
		break;
	
	case UI_SETUP(ERROR):
		// näytä virhe
		reset();
	default:
		UI_SET_STATE(ERROR);
	}

	goto main_loop;
}
