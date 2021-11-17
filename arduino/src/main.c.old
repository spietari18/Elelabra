#include <setjmp.h>
#include <stdarg.h>

#include <SD.h>
#include <LiquidCrystal.h>

/* This enables serial monitor debugging output. */
//#define ENABLE_SERIAL

/* When serial debug is enabled, the sample rate is
 * scaled down by this factor to prevent the TIMER1
 * overflow ISR from consuming all CPU time.
 */
#define SLOWDOWN_FACTOR 1000

/* Should we write DOS newlines or UNIX newlines */
//#define NEWLINES_DOS

#ifdef ENABLE_SERIAL
#define DEBUG(FMT, ...) (void) printf_P(PSTR(FMT), ##__VA_ARGS__)
#else
#define DEBUG(FMT, ...)
#endif

/* How fast should we skip audio. */
#define SKIP_RATE 5000 /* [ms/s] */

/* How fast should the blink timer update. [Hz] */
#define BLINK_RATE 100

/* Modulo that works with negative numbers. */
#define MOD(A, B) \
	(((A) % (B) + (B)) % (B))

/* Fast increment A modulo B. */
#define INC(A, B) \
	((A) = ((A) + 1 == (B)) ? 0 : ((A) + 1))

/* for optimizing conditionals */
#define likely(A)   __builtin_expect((A), 1)
#define unlikely(A) __builtin_expect((A), 0)

/* We use direct register access for IO instead of
 * the Arduino digitalRead/Write instructions for
 * faster IO. To use MODE(), READ() and WRITE() the
 * following variables need to be in place for NAME:
 * <NAME>_REG_PORT
 * <NAME>_REG_DATA
 * <NAME>_REG_PIN 
 * <NAME>_REG_POS
 * These should be created with DEFINE_PIN().
 * (we hope that the compiler optimizes these)
 */
#define DEFINE_PIN(NAME, WHICH, POS) \
	uint8_t *const PIN_##NAME##_REG_PORT = &(PORT##WHICH); \
	uint8_t *const PIN_##NAME##_REG_DDR = &(DDR##WHICH); \
	const uint8_t *const PIN_##NAME##_REG_PIN = &(PIN##WHICH); \
	const uint8_t  PIN_##NAME##_REG_POS = (POS) \

/* set the Nth bit of NUM to VAL*/
#define NTH_BIT(NUM, N, VAL) ((NUM) = ((NUM) & ~(1 << (N))) | ((VAL) << (N)))

/* You can use INTPUT, OUTPUT for MODE and HIGH, LOW for VALUE. */
#define MODE(NAME, MODE) \
	NTH_BIT(*(PIN_##NAME##_REG_DDR), PIN_##NAME##_REG_POS, (MODE))
#define READ(NAME) \
	((*(PIN_##NAME##_REG_PIN) >> PIN_##NAME##_REG_POS) & 1)
#define WRITE(NAME, VALUE) \
	NTH_BIT(*(PIN_##NAME##_REG_PORT), PIN_##NAME##_REG_POS, (VALUE))

/* button pins */
DEFINE_PIN(NEXT, C, 0); /* A0 */
DEFINE_PIN(PLAY, C, 1); /* A1 */
DEFINE_PIN(PREV, C, 2); /* A2 */
#define CRD_INTR_NO 0 /* Should be 0 for pin 2, 1 for pin 3 */
DEFINE_PIN(DTCT, D, 2); /* D2 */
DEFINE_PIN(OUT1, D, 5); /* D5 */
DEFINE_PIN(OUT2, D, 6); /* D6 */

/* error messages, this mess is necessary
 * because compiler retardation and because
 * we want the error strings in ROM
 */
#define ERROR_STR(A, B) \
	const char ERROR_STR_##B[] PROGMEM = (A)

#define ERROR_PTR(A) \
	ERROR_STR_##A

ERROR_STR("NO ERROR", 0);
ERROR_STR("SD INIT FAIL", 1);
ERROR_STR("SD OPEN FAIL", 2);
ERROR_STR("SD IO FAIL", 3);
ERROR_STR("INVALID FORMAT", 4);
ERROR_STR("USUPPORTED FORMAT", 5);
ERROR_STR("CAN'T KEEP UP", 6);
ERROR_STR("NO AUDIO FILES", 7);

const char *const errstr[] PROGMEM = {
	ERROR_PTR(0),
	ERROR_PTR(1),
	ERROR_PTR(2),
	ERROR_PTR(3),
	ERROR_PTR(4),
	ERROR_PTR(5),
	ERROR_PTR(6),
	ERROR_PTR(7)
};

/* Global state, mark anything that can be changed by
 * and ISR as volatile to avoid cryptic issues.
 * This should only contain variables that can change
 * at runtime. Constants should be global or #defines
 */
struct {
	/* A state machine describing the current state
	 * of operation. The macros below should be used
	 * to control the state with the #defined flags.
	 */
#define FLAG_RDY (1 << 0)  /* Ready for playback */
#define FLAG_CRD (1 << 1)  /* Card is connected */
#define FLAG_EOF (1 << 2)  /* End of the current file */
#define FLAG_BST (1 << 3)  /* Blink state */
#define FLAG_BMD (1 << 4)  /* Blink mode */
#define FLAG_JMP (1 << 5)  /* setjmp.h buffer initialized */
#define FLAG_2CH (1 << 6)  /* Audio has two channels */
#define FLAG_EMP (1 << 7)  /* Card has no audio files */
#define FLAG_SKP (1 << 8)  /* Skipping */
#define FLAG_SDR (1 << 9)  /* Skipping direction */
#define FLAG_PLY (1 << 10) /* Playback state */
#define FLAG_ERR (1 << 14) /* Error occurred */
#define FLAG_FTL (1 << 15) /* Error is fatal */

#define FLAG_GET(F) (state.flags &   (FLAG_##F))
#define FLAG_SET(F) (state.flags |=  (FLAG_##F))
#define FLAG_CLR(F) (state.flags &= ~(FLAG_##F))
#define FLAG_TGL(F) (state.flags ^=  (FLAG_##F))
	volatile uint16_t flags;

	/* If an error occurs, the error code is stored here.
	 * The following #defines should be used for the codes.
	 */
#define ERR_OK       0
#define ERR_SD_INIT  1
#define ERR_SD_OPEN  2
#define ERR_SD_IO    3 
#define ERR_FMT_IVAL 4
#define ERR_FMT_USUP 5
#define ERR_PBK_SLOW 6
#define ERR_PBK_EMPT 7

#define ERROR(CODE, FATAL) \
	do { \
		if (FATAL) \
			FLAG_SET(FTL); \
		FLAG_SET(ERR); \
		state.code = (ERR_##CODE); \
		LOOP_RESET; \
	} while (0)
	volatile uint8_t code;


/* How long do we poll for the initial state. */
#define BTN_POLL_TIME 250 /* [ms] */

/* How long can the state be something other than
 * the locked state before we reset.
 */
#define BTN_JITTER_TIME 150 /* [ms] */

/* How long should the locked state be held down
 * before we set the hold flag.
 */
#define BTN_HOLD_TIME 900 /* [ms] */

/* How long do we treat the buttons as inactive
 * after we reset.
 */
#define BTN_INACTIVE_TIME 200 /* [ms] */

/* We could use some insane macro tricks to simplify
 * this, but it's probably just simpler to define
 * all of this shit manually.
 */
#define BTN_NEXT_OFF 0
#define BTN_PLAY_OFF 1
#define BTN_PREV_OFF 2

/* use these with button_event() */
#define NEXT (1 << BTN_NEXT_OFF)
#define PLAY (1 << BTN_PLAY_OFF)
#define PREV (1 << BTN_PREV_OFF)

/* event types for button_event() */
#define DOWN (0 << NUM_KEYS)
#define HOLD (1 << NUM_KEYS)
#define UP   (2 << NUM_KEYS)
#define HLUP (3 << NUM_KEYS)

#define NUM_KEYS 3

/* mask polled keys from state */
#define KEY_MASK \
	((1 << NUM_KEYS) - 1)

/* mask locked keys from state */
#define LOCK_MASK \
	(KEY_MASK << NUM_KEYS)

/* mask for the hold flag */
#define HOLD_BIT \
	(1 << (NUM_KEYS << 1))

/* mask for everything but the state machine */
#define RESET_MASK \
	(KEY_MASK | LOCK_MASK | HOLD_BIT)

#define STATE_OFF \
	(1 + (NUM_KEYS << 1))

/* mask for the state machine */
#define STATE_MASK \
	(3 << STATE_OFF)

/* poll the key state */
#define UPDATE_KEYS \
	(state.button_state |= (!READ(NEXT) << BTN_NEXT_OFF) \
		| (!READ(PLAY) << BTN_PLAY_OFF) \
		| (!READ(PREV) << BTN_PREV_OFF))

#define GET_KEYS \
	(state.button_state & KEY_MASK)

#define CLEAR_KEYS \
	(state.button_state &= ~KEY_MASK)

#define LOCK_KEYS \
	(state.button_state |= (state.button_state & KEY_MASK) << NUM_KEYS)

#define GET_LOCKED \
	((state.button_state & LOCK_MASK) >> NUM_KEYS)

#define GET_HOLD_FLAG \
	(!!(state.button_state & HOLD_BIT))

#define SET_HOLD_FLAG \
	(state.button_state |= HOLD_BIT)

#define RESET_STATE \
	(state.button_state &= ~RESET_MASK)

#define GET_STATE \
	(state.button_state & STATE_MASK)

#define SET_STATE(STATE) \
	(state.button_state = (state.button_state \
		& ~STATE_MASK) | (STATE & STATE_MASK))

#define GET_STATE \
	(state.button_state & STATE_MASK)

#define STATE_WAIT (0 << STATE_OFF)
#define STATE_POLL (1 << STATE_OFF)
#define STATE_LOCK (2 << STATE_OFF)
#define STATE_IACT (3 << STATE_OFF)

	/*  MSB .............................................. LSB
	 *    7          2           1          3            3
	 * |UNUSED|STATE MACHINE|HOLD FLAG|LOCKED KEYS|POLLED KEYS|
	 */
	uint16_t button_state;

	/* Timestamps used by button_update(). They have different
	 * meanings depending on the state machine state.
	 */
	uint16_t ts_1;
	uint16_t ts_2;

	uint16_t ts_now;
	uint16_t ts_old;
	uint16_t dt;

	uint16_t timer1_start;
	uint8_t timer2_start;
	uint8_t timer2_count;

	/* We use setjmp() and longjmp() to return to the
	 * beginning of loop() in order to avoid issues
	 * with state corruption which may happen when the
	 * SD card is unplugged and we attempt to read
	 * for example. This should be initialized in loop()
	 */
#define LOOP_RESET longjmp(state.jump_target, 0)
	jmp_buf jump_target;

	/* SD library objects */
	File root; /* root directory */
	File file; /* current file we're reading */

	/* These are needed for navigation. */
	uint8_t audio_files;
	uint8_t audio_index;

	/* these are read from the fmt chunk */
	uint32_t data_size;
	uint32_t data_read;
	uint32_t sample_rate;

#ifdef ENABLE_SERIAL
	FILE stream;
#endif

	/* This buffer stores our audio samples that are
	 * read from the SD card and played back with PWM.
	 */
#define BUFFER_SIZE 200
#define FILL_THRESH 200
	uint16_t buf_pos;
	uint16_t buf_left;
	uint8_t buffer[BUFFER_SIZE];
	/* make sure buffer is at the end */	
} __attribute__((packed)) state;

/* TIMER1 controls the audio playback */
ISR(TIMER1_OVF_vect)
{
	uint8_t samples;

	/* how many samples will we consume */
	//samples = !!FLAG_GET(2CH) + 1;
	samples = 1;
	
	/* if we're out of samples */
	if (unlikely(state.buf_left < samples)) {
		/* on EOF we stop playback */
		if (FLAG_GET(EOF))
			stop();

		/* otherwise the reading code can't keep up */
		else
			ERROR(PBK_SLOW, 0);
	} else {
		/* consume appropriate amount of samples */
		state.buf_left -= samples;
	}

	/* if we have two channels, the samples
	 * should be adjacent in memory.
	 */
	if (samples > 1) {
		OCR0A = state.buffer[state.buf_pos];
		INC(state.buf_pos, BUFFER_SIZE);
		OCR0B = state.buffer[state.buf_pos];
	/* with a single channel we use the same sample
	 * for both channel outputs.
	 */
	} else {
		OCR0A = OCR0B = state.buffer[state.buf_pos];
	}
	INC(state.buf_pos, BUFFER_SIZE);

	/* reset timer counter */
	TCNT1 = state.timer1_start;
}

/* TIMER2 controls the blink state */
ISR(TIMER2_OVF_vect)
{
	/* NOP */

	/* reset timer counter */
	TCNT2 = state.timer2_start;
}

/* This ISR should be fired when the SD card
 * card detect pin changes state. We shouldn't
 * read anything here but handle that in loop()
 * instead.
 */
void ISR_card()
{
	/* card attached */
	if (READ(DTCT)) {
		if (unlikely(FLAG_GET(CRD)))
			goto end;
		FLAG_SET(CRD);
	/* card removed? */
	} else {
		if (unlikely(!FLAG_GET(CRD)))
			goto end;
		FLAG_CLR(CRD);
		FLAG_CLR(RDY);
	}

	/* restart loop() if we've set the longjump buffer */
	if (likely(FLAG_GET(JMP)))
		LOOP_RESET;

	/* Otherwise returning execution shouldn't be able
	 * to cause corruption issues. We also do this if
	 * an interrupt is fired when the card state doesn't
	 * actually change for some reason.
	 */
end:
	return;
}

/* Start audio playback and enable PWM output. */
void play()
{
	cli();

	FLAG_SET(PLY);

	TIMSK1 |= (1 << TOIE1);
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1);

	sei();
}

/* Stop audio playback and disable PWM output. */
void stop()
{
	cli();

	FLAG_CLR(PLY);

	TIMSK1 &= ~(1 << TOIE1);
	TCCR0A &= ~((1 << COM0A1) | (1 << COM0B1));

	sei();
}

/* This function handles polling the buttons and returns
 * button events when appropriate.
 */
uint8_t button_event()
{
	uint16_t now;
	uint16_t td_1;
	uint16_t td_2;
	uint8_t ret = 0;

	now = (uint16_t)millis();

	/* handle overflow (this gives correct time delta if we
	 * assume that multiple overflows can't occur between calls)
	 */
	if (unlikely(now < state.ts_1))
		td_1 = now + ((uint16_t)(~0) - state.ts_1);
	else
		td_1 = now - state.ts_1;
	if (unlikely(now < state.ts_2))
		td_2 = now + ((uint16_t)(~0) - state.ts_2);
	else
		td_2 = now - state.ts_2;

	UPDATE_KEYS;
	switch (GET_STATE) {
	case STATE_WAIT:
		if (GET_KEYS) {
			state.ts_1 = now;
			SET_STATE(STATE_POLL);
		}
		break;
	case STATE_POLL:
		if (td_1 > BTN_POLL_TIME) {
			LOCK_KEYS;
			CLEAR_KEYS;
			SET_STATE(STATE_LOCK);
			state.ts_1 = state.ts_2 = now;
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
			state.ts_2 = now;
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
			state.ts_2 = now;
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
			state.ts_1 = now;
		case 0x1:
		case 0xD:
			/* NOP */
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

/* Timer prescalers and their correspoding bits in TCCRnB */
const struct {
	uint16_t fact;
	uint8_t  bits;
} __attribute__((packed)) prescalers[] = {
	{1024, 0b101},
	{ 256, 0b100},
	{  64, 0b011},
	{   8, 0b010},
	{   1, 0b001}
};

#define ARRAY_SIZE(A) \
	(sizeof(A)/sizeof(*(A)))

#define CLOCK_FREQ 16000000UL

/* Returns the initial value for the timer count register
 * which needs to be set in an overflow vector to get the
 * requested frequency. The prescaler bits are set in the 
 * timer register appropriately.
 */
uint16_t set_prescaler(uint8_t *reg, uint32_t freq, uint16_t max)
{
	uint32_t res, old;
	uint16_t div;
	uint8_t i;

	i = 0;
	old = max;
	while (i < ARRAY_SIZE(prescalers))
	{
		div = freq*prescalers[i].fact;
		res = (CLOCK_FREQ + (div >> 2))/div;
		if (res > max)
			break;
		old = res;
		i++;
	}
	i--;
_break:
	*reg &= ~7;
	*reg |= prescalers[i].bits;
	max -= (uint16_t)old;

	DEBUG("SET PRESCALER %u, START %u FOR %p\n",
		prescalers[i].fact, max, reg);

	return max;
}

/* Attempt to read a RIFF master chunk of a WAVE file */
uint32_t wave_master_chunk(File file)
{
	uint32_t tmp;
	uint8_t buf[12];

	/* The file can't be a directory */
	if (file.isDirectory())
		return 0;

	/* RIFF master chunk header is 12 bytes */
	if (file.available() < 12)
		return 0;
	
	(void) file.read(buf, 12);

	/* Check that the valid identifiers exist */
	if (memcmp_P(&buf[0], PSTR("RIFF"), 4)
		|| memcmp_P(&buf[8], PSTR("WAVE"), 4))
		return 0;
	
	/* subchunk size */
	tmp = *(uint32_t *)&buf[4];

	/* Make sure the size is valid. */
	if (tmp < 4)
		return 0;

	/* return the size of the subchunk minus the WAVE identifier */
	return tmp - 4;
}

/* Reads the header of a WAVE file into the global state
 * and seeks state.file into the right position for playback.
 * Will throw an error if the WAVE file isn't PCM8
 */
void wave_fmt_chunk(uint32_t expected_size)
{
	uint8_t buf[4];
	uint32_t size;
	uint16_t tmp;

	/* read the chunk ID */
	(void) state.file.read(&buf[0], 4);

	/* make sure the chunk ID is valid */
	if (memcmp_P(&buf[0], PSTR("fmt "), 4))
		ERROR(FMT_IVAL, 0);

	/* read the chunk size */
	(void) state.file.read(&size, 4);

	/* make sure the chunk size is valid */
	if ((size < 16) || (size > 40))
		ERROR(FMT_IVAL, 0);
	
	expected_size -= size;

	/* read the format tag and channel count */
	(void) state.file.read(&buf[0], 4);
	size -= 4;

	/* we only support PCM audio */
	if (*(uint16_t *)&buf[0] != 1)
		ERROR(FMT_USUP, 0);

	tmp = *(uint16_t *)&buf[2];

	/* we only support 1 or 2 channels */
	if ((tmp != 1) && (tmp != 2))
		ERROR(FMT_USUP, 0);
	
	/* flag that we have 2 channels */
	if (tmp == 2)
		FLAG_SET(2CH);	

	/* read sample rate */	
	(void) state.file.read(&state.sample_rate, 4);
	size -= 4;

	/* skip the data rate and alignment */
	(void) state.file.seek(state.file.position() + 6);
	size -= 6;

	/* read the bits per sample value */
	(void) state.file.read(&tmp, 2);
	size -= 2;

	/* we only support PCM8 */
	if (tmp != 8)
		ERROR(FMT_USUP, 0);

	/* skip the possible extension */
	(void) state.file.seek(state.file.position() + size);

	/* the rest of the data is PCM8 audio */
	state.data_size = expected_size;
}

void open_file(int16_t offset)
{
	uint32_t tmp;
	uint8_t new_index;

	new_index = MOD((int16_t)state.audio_index 
		+ offset, state.audio_files);
	
	DEBUG("DELTA %i, OLD INDEX %u, NEW INDEX %u\n",
		offset, state.audio_index, new_index);

	/* handle reopening the same file */
	if (unlikely(state.audio_index == new_index)) {
		/* we can safely skip the master chunk */
		state.file.seek(12);
		goto read_fmt_chunk;
	}

	/* skip to the new file */
	while (state.audio_index != new_index)
	{
		state.file.close();
		state.file = state.root.openNextFile();

		/* end of directory */
		if (!state.file) {
			state.root.rewindDirectory();
			continue;
		}

		/* if this is a WAVE file */
		if ((tmp = wave_master_chunk(state.file)))
			INC(state.audio_index, state.audio_files);
	}
read_fmt_chunk:
	wave_fmt_chunk(tmp);

	/* slow down sample rate */
#ifdef ENABLE_SERIAL
	tmp = state.sample_rate;
	state.sample_rate /= SLOWDOWN_FACTOR;
	DEBUG("SLOWING DOWN SAMPLE RATE %lu (%lu)\n", state.sample_rate, tmp);
#endif

	/* set up TIMER1 for the new sample rate */
	state.timer1_start = set_prescaler(
		&TCCR1B, state.sample_rate, (uint16_t)~(0));

	/* reset file state */
	state.buf_pos = 0;
	state.buf_left = 0;
	state.data_read = 0;

	FLAG_CLR(EOF);

	DEBUG("OPENED %s:\n  SIZE %lu BYTES\n"
		"  SAMPLE RATE %lu\n  CHANNELS %u\n",
		state.file.name(), state.data_size,
		state.sample_rate, 2 - !FLAG_GET(2CH));
}

#ifdef ENABLE_SERIAL
static int serial_putchar(char c, FILE *s)
{
	if (c == '\n') {
#ifdef NEWLINES_DOS
		Serial.write('\r');
#endif
		Serial.write('\n');
		Serial.flush();
	} else {
		Serial.write(c);
   	}

	return 0;
}
#endif

/* We do this to avoid having to call setjmp() every time
 * loop() is called due to the loss of context.
 */
void entry_point()
{

	/* setup() */
	/* initialize state.ts_old here so we get some
	 * delta time on the first loop iteration
	 */
	state.ts_old = (uint16_t)millis();

#ifdef ENABLE_SERIAL
	/* make stdout print to serial */
	fdev_setup_stream(&state.stream,
		serial_putchar, NULL, _FDEV_SETUP_WRITE);
	stdout = &state.stream;
	Serial.begin(9600);
#endif
	cli();

	/* pinmodes */
	MODE(PLAY, INPUT);
	MODE(NEXT, INPUT);
	MODE(PREV, INPUT);
	MODE(DTCT, INPUT);
	MODE(OUT1, OUTPUT);
	MODE(OUT2, OUTPUT);

	/* reset all timer control registers */
	TCCR0A = TCCR0B = TCCR1A = TCCR1B = TCCR2A = TCCR2B = 0;

	/* disable timer interrupts for TIMER1 and TIMER2 */
	TIMSK1 = TIMSK2 = 0;

	/* TIMER0 fast PWM mode */
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << CS00); /* no prescaler */

	/* Set up TIMER2 */
	state.timer2_start = set_prescaler(
		&TCCR2B, BLINK_RATE, (uint8_t)(~0));

	/* We have to use the Arduino library here. */
	attachInterrupt(CRD_INTR_NO, ISR_card, CHANGE);
	ISR_card(); /* initialize the card state */

	sei();
	/* end of setup() */

	/* initialize the environment buffer */
	FLAG_SET(JMP);
	(void) setjmp(state.jump_target);

	while (1)
	{
		/* loop() */

		state.ts_now = (uint16_t)millis();

		/* handle overflow and set delta time */
		if (unlikely(state.ts_now < state.ts_old))
			state.dt = state.ts_now
				+ ((uint16_t)(~0) - state.ts_old);
		else
			state.dt = state.ts_now - state.ts_old;

		state.ts_old = state.ts_now;

		/* Check the error state */
		if (unlikely(FLAG_GET(ERR))) {
			if (FLAG_GET(FTL))
				DEBUG("FATAL ");
			DEBUG("ERROR: %S\n",
				pgm_read_ptr(&errstr[state.code]));
	
			/* make sure to stop playback on error */
			stop();

			/* Fatal errors can't be cleared */
			if (FLAG_GET(FTL))
				while(1);

			/* Tiny delay before reset */
			delay(5000);

			/* Clear the error */
			FLAG_CLR(ERR);
			state.code = ERR_OK;
		}

		/* Handle card disconnect. */
		if (unlikely(!FLAG_GET(CRD))) {
			DEBUG("CARD DISCONNECTED\n");

			/* close open files */
			if (state.root)
				state.root.close();
			if (state.file)
				state.file.close();
		}

		/* If the card isn't connected, we busy wait. */
		while (!FLAG_GET(CRD));

		/* Handle card connect. */
		if (unlikely(!FLAG_GET(RDY))) {
			DEBUG("CARD CONNECTED\n");

			/* initialize the SD library */
			if (unlikely(!SD.begin()))
				ERROR(SD_INIT, 0);

			/* open the root folder */
			if (unlikely(!(state.root = SD.open("/"))))
				ERROR(SD_OPEN, 0);

			state.audio_files = 0;

			/* scan all the files */
			while (1)
			{
				/* We only care about files in the root directory. */
				File tmp = state.root.openNextFile();

				/* no more files */
				if (!tmp)
					break;

				DEBUG("FILE %s", tmp.name());

				if (wave_master_chunk(tmp)) {
					state.audio_files++;
					DEBUG(" AUDIO\n");
				} else {
					DEBUG(" IGNORED\n");
				}

				tmp.close();			
			}

			DEBUG("%u AUDIO FILES\n", state.audio_files);

			/* Using these initial values, we open the
			 * first file without any extra logic in
			 * open_file()
			 */
			state.audio_index = state.audio_files - 1;

			FLAG_SET(RDY);

			/* set the empty flag if necessary */
			if (state.audio_files < 1) {
				FLAG_SET(EMP);
				ERROR(PBK_EMPT, 0);

			/* otherwise open the first file */
			} else {
				open_file(1);
			}
		}

		/* we can't play anything if the card is empty */
		while (FLAG_GET(EMP));

		switch (button_event()) {
		/* toggle playback */
		case PLAY|UP:
			DEBUG("PLAY/PAUSE\n");
			if (FLAG_GET(PLY))
				stop();
			else
				play();
			break;
		
		/* next track */
		case NEXT|UP:
			DEBUG("NEXT TRACK\n");
			open_file(1);
			break;
		
		/* previous track */
		case PREV|UP:
			DEBUG("PREV TRACK\n");
			open_file(-1);
			break;
		
		/* random track */
		case PLAY|HOLD:
			DEBUG("RANDOM TRACK\n");
			open_file(random(1, state.audio_files));
			break;
		
		/* start skipping forward */
		case NEXT|HOLD:
			DEBUG("SKIP FORWARD BEGIN\n");
			FLAG_SET(SKP);
			FLAG_SET(SDR);
			break;
		
		/* stop skipping forward */
		case NEXT|HLUP:
			DEBUG("SKIP FORWARD END\n");
			FLAG_CLR(SKP);
			break;
		
		/* start skipping backward */
		case PREV|HOLD:
			DEBUG("SKIP BACKWARD BEGIN\n");
			FLAG_SET(SKP);
			FLAG_CLR(SDR);
			break;
		
		/* stop skipping backward */
		case PREV|HLUP:
			DEBUG("SKIP BACKWARD END\n");
			FLAG_CLR(SKP);
			break;
		}

		/* handle skipping */
		if (unlikely(FLAG_GET(SKP))) {

		}

		/* read samples */
		if (unlikely(!FLAG_GET(EOF)
			&& (state.buf_left <= FILL_THRESH))) {
			/* SD is fucking shit */
			uint16_t i, j;

			j = BUFFER_SIZE - state.buf_left;

			i = state.file.available();
			/* nothing available */
			if (unlikely(i == -1)) {
				FLAG_SET(EOF);
				goto read_end;

			/* data available and then empty */
			} else if (unlikely(i < j)) {
				FLAG_SET(EOF);
				j = i;
			}

#ifdef ENABLE_SERIAL
			uint16_t total = 0;
#endif

			/* do we need to split the read */
			if ((state.buf_pos + j) > BUFFER_SIZE) {
				i = BUFFER_SIZE - state.buf_pos;
				DEBUG("READ1(%u, %u)\n",state.buf_pos, i);
				(void) state.file.read(
					&state.buffer[state.buf_pos], i);
#ifdef ENABLE_SERIAL
				total += i;
#endif
				j -= i;
				i  = 0;
			} else {
				i = state.buf_pos;
			}

			DEBUG("READ2(%u, %u)\n", i, j);
			(void) state.file.read(
				&state.buffer[i], j);
#ifdef ENABLE_SERIAL
			total += j;
			DEBUG("READ %hu BYTES\n", total);
#endif

			state.buf_left = BUFFER_SIZE;
		}
read_end:
		/* end of loop() */

		/* needs to be called at the end of loop() */
		if (unlikely(serialEventRun))
			serialEventRun();
	}
}
