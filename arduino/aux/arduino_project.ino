/* Tarvittavat kirjastot. */
#include <SD.h>
#include <SPI.h>

/* Vianetsintäviestit sarjamonitoriin, hidastaa toiston taajuutta,
 * jotta TIMER1:sen ISR antaa muunkin koodin suorittaa. 
 */
//#define ENABLE_SERIAL

#define PLY_PIN A4 /* A4  PLAY nappi */
#define NXT_PIN A3 /* A3  NEXT nappi */
#define PRV_PIN A5 /* A5  PREV nappi*/
#define OUT_PIN 5  /* D5  PWM ulostulo */
#define SDC_PIN 10 /* D10 SD kortinlukija chip select */
#define CDT_PIN 2  /* D9  SD kortinlukija card detect */
#define LD1_PIN 1  /* D1  LED indikaattori (sininen) */
#define LD2_PIN 0  /* D0  LED indikaattori (punainen) */

/* SPI 3,4,6,7 */

#define _F(I) \
	((1UL) << (I))

/* Indikaattorit. */
#define F_EOF _F(0) /* Tiedosto loppu. */
#define F_CRD _F(1) /* Onko SD kortti kiinnitetty. */
#define F_RDY _F(2) /* Valmis toistamaan. */
#define F_PLY _F(3) /* Onko toisto käynnissä. */
#define F_LED _F(4) /* LEDin tila. */

/* Globaali tila. */
volatile uint16_t flags = 0;

/* Aseta indikaattori. */
#define F_SET(F) \
	(flags |= (F))

/* Hae indikaattorin arvo. */
#define F_GET(F) \
	(flags & (F))

/* Poista indikaattori. */
#define F_CLR(F) \
	(flags &= ~(F))

/* Vaihda indikaattorin arvoa. */
#define F_TGL(F) \
	(flags ^= (F))

/* Puskureiden koko, tämä on tarkoituksella 2^8, jotta voimme
 * laskuissa hyödyntää ylivuotoa ja täten yksinkertaistaa
 * monia koodissa hyödynnnettyjä operaatioita.
 */
#define BUFFER_SIZE 256

/* Paljonko näytepuskurissa pitää olla enintään näytteitä jäljellä
 * ennenkuin uusia näytteita luetaan näytepuskuriin.
 */
#define FILL_THRESHOLD 128

uint8_t sample_buffer[BUFFER_SIZE]; /* Näytepuskuri. */
uint8_t read_buffer[BUFFER_SIZE];   /* Lukupuskuri. */

volatile uint8_t  buf_pos;  /* Paikka puskurissa. */
volatile uint16_t buf_left; /* Monta näytettä puskurissa on jäljellä. */

/* Kuinka kauan odotetaan ensimmäisestä napinpainalluksesta, että muut
 * napinpainallukset rekisteröidään tällä ajanjaksolla tapahtuvaksi
 * samalla ajanhetkellä. (Kahta nappia voidaan pitää yhtenä)
 */
#define BTN_WAITTIME 500 /* [ms] */

/* Kuinka kauan nappeja tarvitsee pitää pohjassa, että napin tila 
 * rekisteröityy uutena napinpainalluksena. (nollaa vanhan tilan)
 */
#define BTN_RESETTIME 2000 /* [ms] */

#define MILLIS_CONVERSION 16

/* Nappien paikat bittivektorissa. */
#define BTN_PLY_OFF 0
#define BTN_NXT_OFF 1
#define BTN_PRV_OFF 2

/* Nappien bittimaskit. */
#define BTN_PLY _F(BTN_PLY_OFF)
#define BTN_NXT _F(BTN_NXT_OFF)
#define BTN_PRV _F(BTN_PRV_OFF)

/* Paljonko bittimaskia pitää siirtää, että päästään tilan ohi. */
#define BTN_SHIFT 3

/* Bittimaski tilalle. */
#define BTN_MSK \
	(BTN_PLY | BTN_NXT | BTN_PRV)

/* Muuttuiko nappien tila. */
#define BTN_DELTA \
	(((button_state >> BTN_SHIFT) & BTN_MSK) != (button_state & BTN_MSK))

/* Nykyinen tila. */
#define BTN_STATE \
	(button_state & BTN_MSK)

/* Vanha tila. */
#define BTN_STATE_OLD \
	((button_state >> BTN_SHIFT) & BTN_MSK)

/* Aseta vanha tila nykyiseksi tilaksi ja nollaa nykyinen tila */
#define BTN_CLR \
	do { \
		button_state &= ~(BTN_MSK << BTN_SHIFT); \
		button_state |= (button_state & BTN_MSK) << BTN_SHIFT; \
		button_state &= ~BTN_MSK; \
	} while (0)

/* Nollaa vanha tila. */
#define BTN_RST \
	(button_state &= ~(BTN_MSK << BTN_SHIFT))

/* Lue nappien tilat */
#define BTN_READ \
	do { \
		button_state |= (!digitalRead(PLY_PIN) << BTN_PLY_OFF) \
			| (!digitalRead(NXT_PIN) << BTN_NXT_OFF) \
			| (!digitalRead(PRV_PIN) << BTN_PRV_OFF); \
	} while(0)

uint16_t button_state; /* Nappien tila bittivektorina. */
uint32_t button_ptime; /* Milloin nykyinen napinpainallus aloitettiin. */

uint16_t timer1_start; /* TIMER1 aloitusarvo. (16bit) */
uint8_t  timer2_start; /* TIMER2 aloitusarvo. (8bit) */

/* TIMER2 on liian nopea, joten käytämme laskuria hidastamaan
 * LEDIN välkkymistaajuuden 256 kertaa pienemmäksi.
 */
uint8_t timer2_counter;

/* Yksi tiedosto vie sizeof(void*) + MAX_FILENAME + 1 tavua muistia, sillä
 * varastoidaan tiedostonimi ja tiedostonimen osoitin listaan. Tästä
 * saadaan ehto: MUISTI >= MAX_FILES * (sizeof(void*) + MAX_FILENAME + 1)
 * MAX_FILES tulee määrittää tämän mukaan, jotta muisti ei lopu kesken.
 * HUOM: Muistia pitää jäädä myös funktioiden muuttujille!
 */
#define MAX_FILES 8

/* SD kirjaston tiedostonimen maksimipituus on 12 tavua. */
#define MAX_FILENAME 13

/* Varataan kaikki muisti staattisesti, jotta ei tarvitse
 * käyttää malloc():ia ja free():tä dynaamisesti turhaan.
 */
char file_lst[MAX_FILES][MAX_FILENAME];

uint16_t file_blk; /* Nykyinen tiedostoblockki. */
uint16_t file_tot; /* Tiedostojen määrä kortilla */
uint16_t file_cnt; /* Tiedostojen määrä blockissa. */
uint16_t file_off; /* Nykyisen tiedoston paikka. */
File     file_cur; /* Toistettava tiedosto */
File     root;

/* Ajastimien skaalaimet. */
const struct {
	uint16_t fact;
	uint8_t  bits;
} prescalers[] = {
	{1024, 0b101},
	{ 256, 0b100},
	{  64, 0b011},
	{   8, 0b010},
	{   1, 0b001}
};

#define ARRAY_SIZE(A) \
	(sizeof(A)/sizeof(*(A)))

/* Arduinon kellotaajuus. */
#define CLOCK_FREQ 16000000.0f /* [Hz] */

/* Valitse paras skaalain taajuudelle. Suosii pieniä skaalaimia sillä mitä pienempi
 * skaalain on, sitä tarkemmin taajuus voidaan määritellä ajastimen laskurilla.
 */
uint16_t select_prescaler(float frequency, uint16_t timer_max, uint16_t *timer_min)
{
	uint16_t i;
	float j;
	float k;

	i = k = 0;
	while (i < ARRAY_SIZE(prescalers))
	{
		j = CLOCK_FREQ/(frequency*prescalers[i].fact);
		if (j >= timer_max)
			goto _break;
		k = j;
		i++;
	}

	i--;
	if (j >= 1)
		goto _break;
	k = 1;
_break:
	*timer_min = (timer_max - (uint16_t)k) & timer_max;
	return i;
}

/* Merkittömän kokonaislukutyypin maksimiarvo, koska
 * binaariluvut esitetään kahden komplementtimuodossa.
 */
#define I_MAX(T) \
	((T)(-1))

/* Alusta ajastinrekisterit halutuille taajuuksille. */
void setup_timers(uint16_t sample_rate, uint16_t blink_rate)
{
	uint16_t i, j;
	
	noInterrupts();

	/* Nollaa kaikki ajastinrekisterit. */
	TCCR0A = TCCR0B = TCCR1A = TCCR1B = TCCR2A = TCCR2B = 0;

	/* TIMER0 Fast PWM, säätää PWM:n taajuutta. Taajuuden tulee
	 * olla korkeampi kuin musiikin näytteistaajuus. Fast PWM:n
	 * maksimitaajuus on ~60kHz, musiikin tällä hetkellä 10kHz
	 */
	TCCR0A = 0b10100011;
	TCCR0B = 0b00000001;

	/* Pysäytä ajastimet. */
	TIMSK1 &= ~(1 << TOIE1);
	TIMSK2 &= ~(1 << TOIE2); 

	/* TIMER1 Ääni */
	i = select_prescaler(sample_rate, I_MAX(uint16_t), &j);
	TCCR1B |= prescalers[i].bits;
	timer1_start = j;

	/* TIMER2 LED */
	i = select_prescaler(blink_rate, I_MAX(uint8_t), &j);
	TCCR2B |= prescalers[i].bits;
	timer2_start = j;

	interrupts();
}

/* Muuta ledin tilaa porttirekisterillä, koska digitalWrite() ei toiminut. */
#define LED_SET(S) \
	(PORTD = (PORTD & ~(1 << LED_PIN)) | ((S) << LED_PIN))

/* Sytytä LED */
void led_on()
{
	noInterrupts();
	
	TIMSK2 &= ~(1 << TOIE2);
	F_SET(F_LED);
	LED_SET(1);

	interrupts();
}

/* Sammuta LED */
void led_off()
{
	noInterrupts();

	TIMSK2 &= ~(1 << TOIE2);
	F_CLR(F_LED);
	LED_SET(0);

	interrupts();
}

/* Välkytä LEDiä. */
void led_blink()
{
	noInterrupts();

	timer2_counter = 0;
	TIMSK2 |= (1 << TOIE2);

	interrupts();
}

/* Jatka äänentoistoa. */
void play()
{
	noInterrupts();

	F_SET(F_PLY);
	TIMSK1 |= (1 << TOIE1);

	interrupts();

	led_on();
}

/* Lopeta äänentoisto. */
void stop()
{
	noInterrupts();

	F_CLR(F_PLY);
	TIMSK1 &= ~(1 << TOIE1);

	interrupts();

	led_blink();
}

/* Musiikintoisto, TIMER1 */
ISR(TIMER1_OVF_vect)
{
	/* Kirjoita uusi näyte ulostulopinnin rekisteriin. */
	OCR0B = sample_buffer[buf_pos++];

	/* Onko puskurissa näytteitä jäljellä? */
	if (buf_left < 1) {
		/* Tiedosto loppu? */
		if (F_GET(F_EOF))
			stop();
		/* Jos tiedosto ei ole loppu ja puskurissa ei ole näytteitä Arduinon
		 * kellotaajuus on liian pieni ISR:n taajuuteen verrattuna, jolloin
		 * loop() funktiota ei suoriteta ja tiedostoon ei lueta mitään.
		 * Tällöin puskuri vuotaa yli ja samoja BUFFER_SIZE toistetaan niin
		 * kauan kunnes puskuriin luetaan lisää näytteitä.
		 */
	} else {
		/* Kuluta näyte. */
		buf_left -= 1;
	}

	/* Nollaa ajastimen laskentarekisteri. */
	TCNT1 = timer1_start;
}

/* LEDin välkkyminen, TIMER2 */
ISR(TIMER2_OVF_vect)
{
	/* Vaihda LEDin tilaa. */
	if (!timer2_counter++)
		LED_SET(!(F_TGL(F_LED) & F_LED));

	/* Nolla ajastimen laskentarekisteri. */
	TCNT2 = timer2_start;
}

/* Modulo joka toimii negatiivisilla luvuilla. */
#define MOD(A,B) \
	(((A) + (B)) % (B))

/* Vaihtaa aktiivista tiedostoa x:n yksikön verran. */
void file_change(int16_t x)
{
	file_off = MOD(file_off + x, file_cnt);

	/* Sulje vanha tiedosto ja avaa uusi. */
	if (file_cur)  file_cur.close();
	file_cur = SD.open(file_lst[file_off]);
#ifdef ENABLE_SERIAL
	Serial.println(file_lst[file_off]);
	Serial.println(file_cur.name());
#endif	
	F_CLR(F_EOF);
}

const char pcm_ext[] = ".pcm";

/* Määrittää onko annettu tiedosto PCM tiedosto. */
bool is_pcm(File file)
{
	char *ext, *name;
	uint16_t i;

	/* Hakemistot eivät voi olla PCM tiedostoja. */
	if (file.isDirectory())
		return false;

	name = file.name();
	
	/* Nimen tulee olla pidempi kuin tiedostopääte. */
	i = strlen(name);
	if (i < sizeof(pcm_ext))
		return false;

	/* Tiedostopäätteen tule olla oikea. */
	name += i - sizeof(pcm_ext) + 1;
	ext = &pcm_ext[0];
	while (*name)
	{
		/* Isoilla ja pienillä kirjaimilla ei ole merkitystä. */
		if (tolower(*name++) != *ext++)
			return false;
	}

	return true;
}

/* Vaihda toistettavaa tiedostolohkoa. */
void block_change(int16_t x)
{
	uint16_t i;

	root.rewindDirectory();
	file_off = 0;

	/* Laske mones tiedostoblockki halutaan. */
	i = (file_tot + (MAX_FILES - 1)) / MAX_FILES;
	file_blk = MOD(file_blk + x, i);

#ifdef ENABLE_SERIAL
	Serial.print("BLOCK: ");
	Serial.println(file_blk);
#endif

	/* Skippaa tiedostoblockit ennen haluttua. */
	if (file_blk > 0) {
		i = file_blk * MAX_FILES;
		while (i) {
			file_cur = root.openNextFile();
			if (is_pcm(file_cur))
				i--;
#ifdef ENABLE_SERIAL
			Serial.print("SKIP: ");
			Serial.println(file_cur.name());
#endif
			file_cur.close();	
		}
	}

	/* Lue tiedostonimet ja lue maksimissaan
	 * MAX_FILES tiedostonimeä.
	 */
	file_cnt = 0;
	while ((file_cnt < MAX_FILES) && (file_cur = root.openNextFile()))
	{
		if (is_pcm(file_cur)) {
#ifdef ENABLE_SERIAL
			Serial.println(file_cur.name());
#endif
			strcpy(file_lst[file_cnt], file_cur.name());
			file_cnt++;
		}
		file_cur.close();
	}
#ifdef ENABLE_SERIAL
	Serial.println(file_cnt);
#endif

	/* Avaa ensimmäinen tiedosto. */
	file_change(0);
}

/* JOS MUISTI LOPPUU KESKEN:
 *	SD kirjaston tiedostonimi on maksimissaan 13 tavua. Jos arduinon muisti
 *	ei riitä kaikkien tiedostokahvojen pitämiseen muistissa, ota talteen
 *	tiedostonimet tiedostokahvojen sijasta.
 */

/* SD kortin ISR */
void card_delta()
{
	/* Kortti kiinni? */
	if (F_TGL(F_CRD) & F_CRD) {
		/* SD kirjaston alustus. */
		SD.begin(SDC_PIN);

		root = SD.open("/");
		file_tot = file_blk = 0;
		while ((file_cur = root.openNextFile()))
		{
#ifdef ENABLE_SERIAL
			Serial.println(file_cur.name());
#endif
			if (is_pcm(file_cur))
				file_tot++;
			file_cur.close();
		}
#ifdef ENABLE_SERIAL
		Serial.print("FILES: ");
		Serial.println(file_tot);
#endif
		/* Lue ensimmäinen tiedostolohko. */
		block_change(0);

		led_blink();
	/* Kortti irti? */
	} else {
		/* Lopeta toisto. */
		stop();

		root.close();

		led_off();
	}
}

/* Näytteistaajuuden voisi määritellä dynaamisesti. */
#ifdef ENABLE_SERIAL
# define SAMPLE_RATE 10 /* [Hz] */
#else
# define SAMPLE_RATE 10500 /* [Hz] */
#endif

/* Oikea taajuus hertseinä on 256 kertaa pienempi. */
#define BLINK_RATE 100 /* [Hz/256] */

/* Alustus. */
void setup()
{
#ifdef ENABLE_SERIAL
	Serial.begin(9600);
#endif

	/* pinmodet */
	pinMode(PLY_PIN, INPUT);
	pinMode(NXT_PIN, INPUT);
	pinMode(PRV_PIN, INPUT);
	pinMode(CDT_PIN, INPUT);
	pinMode(LED_PIN, OUTPUT);
	pinMode(OUT_PIN, OUTPUT);

	/* Kortinlukijan ISR */
	attachInterrupt(digitalPinToInterrupt(CDT_PIN), card_delta, CHANGE);

	/* Alusta ajastimet */
	setup_timers(SAMPLE_RATE, BLINK_RATE);

	/* Alusta kortin alkutila */
	digitalRead(CDT_PIN) ? F_CLR(F_CRD) : F_SET(F_CRD);
	card_delta();
}

/* Hoitaa napit ja tiedoston lukemisen/vaihtamisen. */
void loop()
{
	uint32_t a;
	uint16_t b;

	/* Odottaa kunnes kortti jolla on PCM tiedostoja kiinnitetään.*/
	while (!F_GET(F_CRD) && (file_cnt < 1));

#if 1
	/* Lue nappien tila. Koska nappien tila tällä hetkellä lisätään
	 * button_state:een OR operaatiolla, ei ole välilä vaikka nappi
	 * pomppii, sillä button_state:n nykyinen tila nollaantuu vasta
	 * kun BTN_WAITTIME millisekuntia on kulunut ensimmäisestä
	 * napinpainalluksesta. Tämän avulla voidaan tehdä kahden napin
	 * "yhtäaikaisesta" painamisesta erillinen funktio ohjelmaan.
	 * Samaa napinpainallusta ei rekisteröidä kahdesti, sillä jos
	 * nappien tila pysyy samana, alla oleva koodi huomaa sen.
	 */
	BTN_READ;
	a = millis() / MILLIS_CONVERSION;
	if (BTN_STATE) {
		/* Rekisteröidäänkö nykyinen painallus? */
		if ((a - button_ptime) >= BTN_WAITTIME) {
			/* Jos tila ei muuttunut viimeisestä */
			if (!BTN_DELTA) {
				/* Jos ei ole kulunut BTN_RESETTIME millisekuntia, emme
				 * nollaa ajastinta vielä vaan nollaamme nappien tilan ja
				 * odotamme muuttuuko tilanne seuraavalla kerralla.
				 */
				if ((a - button_ptime) < BTN_RESETTIME)
					goto reset_state;
				/* Muuten tämä rekisteröidään uutena napinpainalluksena. */	
			}

			switch (BTN_STATE) {
			/* PLAY: toisto/tauko */
			case BTN_PLY:
#ifdef ENABLE_SERIAL
				Serial.println("PLAY");
#endif
				if (F_GET(F_PLY))
					stop();
				else
					play();
				break;
			/* NEXT: seuraava */
			case BTN_NXT:
#ifdef ENABLE_SERIAL
				Serial.println("NEXT");
#endif
				stop();
				file_change(1);
				play();
				break;
			/* PREV: edellinen */
			case BTN_PRV:
#ifdef ENABLE_SERIAL
				Serial.println("PREV");
#endif
				stop();
				file_change(-1);
				play();
				break;
			/* NEXT & PREV: Valitse satunnainen. */
			case BTN_NXT|BTN_PRV:
#ifdef ENABLE_SERIAL
				Serial.println("NEXT|PREV");
#endif
				stop();
				b = file_cnt >> 1;
				file_change(random(-b, b));
				play();
				break;
			/* PLAY & NEXT: Seuraava tiedostoblockki. */
			case BTN_PLY|BTN_NXT:
#ifdef ENABLE_SERIAL
				Serial.println("PLAY|NEXT");
#endif
				stop();
				block_change(1);
				play();
				break;
			/* PLAY & PREV: Edellinen tiedostoblockki. */
			case BTN_PLY|BTN_PRV:
#ifdef ENABLE_SERIAL
				Serial.println("PLAY|PREV");
#endif
				stop();
				block_change(-1);
				play();
				break;
			}

			/* Nollaa ajastin. */
			button_ptime = a;
reset_state:
			/* Nollaa nappien tila. */
			BTN_CLR;
		}
	/* Muuten nollaa ajastin, koska mitään ei painettu. */
	} else {
		/* Nollaa vanha tila jos mitään ei paineta. */
		if (BTN_STATE_OLD && ((a - button_ptime) >= BTN_RESETTIME)) {
			BTN_RST;
			button_ptime = a;
		}
	}
#endif

#if 1
	/* Jos nykyinen tiedosto on loppu, avaa seuraava tiedosto. */
	if (!F_GET(F_PLY) && F_GET(F_EOF)) {
		file_change(1);
		play();
	}

	/* Pitääkö puskuriin lukea lisää näytteitä? */
	if (F_GET(F_PLY) && (buf_left < FILL_THRESHOLD))
	{
		uint16_t a, b;
		uint8_t  c;

		/* Paljonko pitää lukea, että näytepuskuri täyttyy. */
		a = BUFFER_SIZE - buf_left;

		/* Virhetilanteessa lopeta toisto
		 * ja aloita uusi tiedosto.
		 */
		if ((b = file_cur.read(read_buffer, a)) == -1) {
			stop();
			F_SET(F_EOF);
		}

		/* Jos luettiin vähemmän kuin haluttiin
		 * tiedosto on loppu.
		 */
		if (a != b)
			F_SET(F_EOF);

		/* Puskuriin tuli b näytettä. */
		buf_left += b;

		/* Kopioi näytteet lukupuskurista näytepuskuriin.
		 * Käyttää hyväksi 8 bittistä ylivuotoa.
		 */
		c = buf_pos + buf_left;
		a = 0;
		do {
			sample_buffer[c++] = read_buffer[a++];
		} while (a < b);
	}
#endif
}
