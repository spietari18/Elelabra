#include "button.h"
#include <Arduino.h>

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
	(s->state |= (BTN_POLL_RT << BTN_RT_OFF) \
		| (BTN_POLL_LT << BTN_LT_OFF))

#define GET_KEYS \
	(s->state & KEY_MASK)

#define CLEAR_KEYS \
	(s->state &= ~KEY_MASK)

#define LOCK_KEYS \
	(s->state |= (s->state & KEY_MASK) << NUM_KEYS)

#define GET_LOCKED \
	((s->state & LOCK_MASK) >> NUM_KEYS)

#define GET_HOLD_FLAG \
	(!!(s->state & HOLD_BIT))

#define SET_HOLD_FLAG \
	(s->state |= HOLD_BIT)

#define RESET_STATE \
	(s->state &= ~RESET_MASK)

#define GET_STATE \
	(s->state & STATE_MASK)

#define SET_STATE(STATE) \
	(s->state = (s->state \
		& ~STATE_MASK) | (STATE & STATE_MASK))

#define GET_STATE \
		(s->state & STATE_MASK)

#define STATE_WAIT (0 << STATE_OFF)
#define STATE_POLL (1 << STATE_OFF)
#define STATE_LOCK (2 << STATE_OFF)
#define STATE_IACT (3 << STATE_OFF)

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
button_update(struct button_state *s)
{
	uint32_t now, td_1, td_2;
	uint8_t ret = 0;

	now = millis();
	td_1 = now - s->ts_1;
	td_2 = now - s->ts_2;
	UPDATE_KEYS;
	switch (GET_STATE) {
	case STATE_WAIT:
		if (GET_KEYS) {
			s->ts_1 = now;
			SET_STATE(STATE_POLL);
		}
		break;
	case STATE_POLL:
		if (td_1 > BTN_POLL_TIME) {
			LOCK_KEYS;
			CLEAR_KEYS;
			SET_STATE(STATE_LOCK);
			s->ts_1 = s->ts_2 = now;
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
			s->ts_2 = now;
			/* fallthrough */
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
			s->ts_2 = now;
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
			s->ts_1 = now;
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

