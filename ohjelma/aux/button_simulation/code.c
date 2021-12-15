#include <time.h>
#include <stdio.h>

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

#define UPDATE_KEYS \
	(button_state |= bits & 7)

#define GET_KEYS \
	(button_state & 7)

#define CLEAR_KEYS \
	(button_state &= ~7)

#define LOCK_KEYS \
	(button_state |= (button_state & 7) << 3)

#define GET_LOCKED \
	((button_state >> 3) & 7)

#define GET_HOLD_FLAG \
	((button_state >> 6) & 1)

#define SET_HOLD_FLAG \
	((button_state) |= 1 << 6)

#define RESET_STATE \
	(button_state &= ~((1 << 7) - 1))

#define SET_STATE(WHAT) \
	(button_state = (button_state & ~(3 << 7)) | (WHAT & (3 << 7)))

#define GET_STATE \
	(button_state & (3 << 7))

/* state machine */
#define STATE_WAIT (0 << 7) /* waiting for inital keypress */
#define STATE_POLL (1 << 7) /* polling for the locked state */
#define STATE_LOCK (2 << 7) /* state locked and waiting */
#define STATE_IACT (3 << 7) /* buttons inactive */

#define DOWN (0 << 3)
#define HOLD (1 << 3)
#define UP   (2 << 3)
#define HLUP (3 << 3)

#define PREV (1 << 0)
#define PLAY (1 << 1)
#define NEXT (1 << 2)

unsigned int ts_1, ts_2; /* meaning depends on the state machine */

/*  MSB .............................................. LSB
 *    7          2           1          3            3
 * |UNUSED|STATE MACHINE|HOLD FLAG|LOCKED KEYS|POLLED KEYS|
 */
unsigned short button_state;

// PSEUDOCODE
// 
// START:
// WAIT FOR INITIAL KEYPRESS
// WAIT BTN_POLL_TIME WHILE OR'ING THE POLLED STATE TO THE INITIAL STATE
// LOCK STATE
// DOWN EVENT
// IF POLLED STATE CHANGES FROM LOCKED STATE FOR LONGER THAN BTN_POLL_TIME
// 	UP EVENT
// 	GOTO END
// WAIT FOR BTN_HOLD_TIME
// SET HOLD FLAG
// HOLD EVENT
// IF POLLED STATE CHANGES FROM LOCKED STATE FOR LONGER THAN BTN_POLL_TIME
// 	HLUP EVENT
// END:
// WAIT FOR BTN_IACT_TIME
// RESET LOCKED STATE
// GOTO START

/* arduino code has millisecond precision */
unsigned long millis()
{
	struct timespec ts;

	(void) clock_gettime(CLOCK_MONOTONIC, &ts);

	return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

unsigned char button_update(unsigned int bits)
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

/* put code here */
void code(unsigned int bits)
{
	switch (button_update(bits)) {
	/* toggle playback */
	case PLAY|UP:
		printf("PLAY/PAUSE\n");
		break;
	
	/* next track */
	case NEXT|UP:
		printf("NEXT TRACK\n");
		break;
	
	/* previous track */
	case PREV|UP:
		printf("PREV TRACK\n");
		break;
	
	/* random track */
	case PLAY|HOLD:
		printf("RANDOM TRACK\n");
		break;
	
	/* start skipping forward */
	case NEXT|HOLD:
		printf("SKIP FORWARD BEGIN\n");
		break;
	
	/* stop skipping forward */
	case NEXT|HLUP:
		printf("SKIP FORWARD END\n");
		break;
	
	/* start skipping backward */
	case PREV|HOLD:
		printf("SKIP BACKWARD BEGIN\n");
		break;
	
	/* stop skipping backward */
	case PREV|HLUP:
		printf("SKIP BACKWARD END\n");
		break;
	}
}
