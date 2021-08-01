/* Copyright 2020 Qurn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	If not, see <http://www.gnu.org/licenses/>.
 */

/***************************
 * Usual defines
 **************************/

#include QMK_KEYBOARD_H

#define _MAIN 0
#define _LOWER 1
#define _RAISE 2
#define _SYMBOL 3
#define _GAMING 4
#define _ADJUST 5


enum custom_keycodes {
    KC_BSPC_LCTL= SAFE_RANGE,
    KC_CPI_DOWN,
    KC_CPI_STD,
    KC_CPI_UP,
    KC_SMO_SC,
    RAISE,
    LOWER,
    SYMBOL,
    GAMING,
    MACSLEEP,
    ENC_PLAY,
};

// Gaming
#define SCRSHT LSFT(KC_P5)
#define STATS MEH(KC_8)
#define RAISE MO(_RAISE)
#define LOWER MO(_LOWER)

// #define SYMBOL LT(_SYMBOL,KC_ESC)

/***************************
 * Trackball related defines
 **************************/

uint8_t track_mode = 0; // 0 Mousecursor; 1 arrowkeys/carret; 2 scrollwheel; 3 sound/brightness
#define cursor_mode 0
#define carret_mode 1
#define scroll_mode 2
uint8_t prev_track_mode = 0;
bool integration_mode = false;
int16_t cum_x = 0;
int16_t cum_y = 0;
int16_t sensor_x = 0;
int16_t sensor_y = 0;

// Thresholds help to move only horizontal or vertical. When accumulated distance reaches threshold, only move one discrete value in direction with bigger delta.
uint8_t	carret_threshold = 24;		 // higher means slower
uint16_t carret_threshold_inte = 340; // in integration mode higher threshold

#define regular_smoothscroll_factor 8
bool smooth_scroll = true;
uint8_t	scroll_threshold = 200 / regular_smoothscroll_factor;	// divide if started smooth
uint16_t scroll_threshold_inte = 1000 / regular_smoothscroll_factor;

uint16_t cursor_multiplier = 250;	// adjust cursor speed
uint16_t cursor_multiplier_inte = 20;
#define CPI_STEP 20

int16_t cur_factor;

/***************************
 * Mouse pressed
 **************************/

void on_mouse_button(uint8_t mouse_button, bool pressed) {
	report_mouse_t report = pointing_device_get_report();

	if(pressed)
		report.buttons |= mouse_button;
	else
		report.buttons &= ~mouse_button;
	pointing_device_set_report(report);
	pointing_device_send();
}

/***************************
 * Combos
 **************************/

enum combos_events {
	RS_MOUSE,
	ST_MOUSE,
	RT_MOUSE
};

const uint16_t PROGMEM rsm_combo[] = {KC_R, KC_S, COMBO_END};
const uint16_t PROGMEM stm_combo[] = {KC_S, KC_T, COMBO_END};
const uint16_t PROGMEM rtm_combo[] = {KC_R, KC_T, COMBO_END};

combo_t key_combos[3] = {
	[RS_MOUSE] = COMBO_ACTION(rsm_combo),
	[ST_MOUSE] = COMBO_ACTION(stm_combo),
	[RT_MOUSE] = COMBO_ACTION(rtm_combo),
};

void process_combo_event(uint16_t combo_index, bool pressed) {
	switch(combo_index) {
	case RS_MOUSE:
		on_mouse_button(MOUSE_BTN2, pressed);
		break;
	case ST_MOUSE:
		on_mouse_button(MOUSE_BTN1, pressed);
		break;
	case RT_MOUSE:
		on_mouse_button(MOUSE_BTN3, pressed);
		break;
	}
}

/***************************
 * Layers
 **************************/

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_MAIN] = LAYOUT_RHEUMATOID(
KC_TAB,     KC_Q,       KC_W,       KC_E,       KC_R,       KC_T,       KC_EQL,         ENC_PLAY,   KC_Y,       KC_U,       KC_I,       KC_O,       KC_P,      KC_GRV,
SYMBOL,     KC_A,       KC_S,       KC_D,       KC_F,       KC_G,       KC_MINS,        MO(_ADJUST),KC_H,       KC_J,       KC_K,       KC_L,       KC_SCLN,   KC_QUOT,
KC_LSPO,    KC_Z,       KC_X,       KC_C,       KC_V,       KC_B,                                   KC_N,       KC_M,       KC_COMM,    KC_DOT,     KC_SLSH,   KC_RSPC,
                        KC_ESC,     KC_LALT,    KC_LGUI,    KC_SPC,     KC_ENT,                                 KC_BSPC,    LALT(KC_3), _______,
                                                KC_LCTL,    LOWER,      RAISE,          _______,    KC_DEL
),

[_LOWER] = LAYOUT_RHEUMATOID(
KC_TILD,    KC_EXLM,    KC_AT,      KC_HASH,    KC_DLR,     KC_PERC,    _______,        _______,    KC_CIRC,    KC_AMPR,    KC_ASTR,    KC_LPRN,    KC_RPRN,    KC_BSLS,
_______,    KC_1,       KC_2,       KC_3,       KC_4,       KC_5,       _______,        _______,    KC_6,       KC_7,       KC_8,       KC_9,       KC_0,       LALT(KC_3),
_______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
                        _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
                                                _______,    _______,    _______,        _______,    _______
),

[_RAISE] = LAYOUT_RHEUMATOID(
_______,    KC_F1,      KC_F3,      KC_F3,      KC_F4,      KC_F5,      _______,        _______,    KC_F6,      KC_F7,      KC_F8,      KC_F9,      KC_F10,     _______,
_______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    KC_BTN1,    KC_BTN2,    KC_BTN3,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
                        _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
                                                _______,    _______,    _______,        _______,    _______
),

[_SYMBOL] = LAYOUT_RHEUMATOID(
_______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    KC_EXLM,    KC_LBRC,    KC_RBRC,    KC_DLR,     KC_PERC,    KC_F11,
_______,    _______,    _______,    _______,    _______,    _______,    _______,        KC_UNDS,    KC_PIPE,    KC_LCBR,    KC_RCBR,    KC_EQL,     KC_COLN,    KC_F12,
_______,    _______,    _______,    _______,    _______,    _______,                                KC_AMPR,    KC_LT,      KC_GT,      KC_MINS,    KC_SLSH,    _______,
                        _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
                                                _______,    _______,    _______,        _______,    _______
),

[_GAMING] = LAYOUT_RHEUMATOID(
KC_TAB,     _______,    _______,    _______,    _______,    _______,    KC_Y,           _______,    _______,    _______,    _______,    _______,    _______,    _______,
KC_CAPS,    _______,    _______,    _______,    _______,    _______,    KC_H,           _______,    _______,    _______,    _______,    _______,    _______,    _______,
KC_LSFT,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    KC_RSFT,
                        _______,    _______,    _______,    _______,    _______,                                _______,    SCRSHT,     STATS,
                                                _______,    _______,    _______,        _______,    _______
),

[_ADJUST] = LAYOUT_RHEUMATOID(
_______,    _______,    _______,    _______,    _______,    KC_CPI_STD, KC_CPI_UP,      _______,    _______,    _______,    _______,    _______,    _______,    _______,
_______,    _______,    KC_SMO_SC,  _______,    _______,    _______,    KC_CPI_DOWN,    _______,    _______,    _______,    _______,    _______,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    GAMING,                                 _______,    _______,    _______,    _______,    _______,    _______,
                        _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
                                                _______,    _______,    _______,        _______,    _______
)
};



// [_LOWER] = LAYOUT_RHEUMATOID(
// _______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    _______,    _______,    _______,    _______,    _______,
// _______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    _______,    _______,    _______,    _______,    _______,
// _______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
//                         _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
//                                                 _______,    _______,    _______,        _______,    _______
// ),


/***************************
 * Trackball/Encoder handling
 **************************/
bool enc_used = false;
bool enc_skip = false;

#ifdef IS_RIGHT
void pointing_device_init(void){
	// if(!is_keyboard_master())
	// 	return;

	pmw_spi_init();
    pmw_set_cpi(8000);
}

int max(int num1, int num2) { return (num1 > num2 ) ? num1 : num2; }
int min(int num1, int num2) { return (num1 > num2 ) ? num2 : num1; }

int8_t sign(int x) { return (x > 0) - (x < 0); }
int8_t CLAMP_HID(int value) { return value < -127 ? -127 : value > 127 ? 127 : value; }

void tap_code_fast(uint8_t code) {
	register_code(code);
	// Dont do this:
	// if (code == KC_CAPS) {
	//	 wait_ms(TAP_HOLD_CAPS_DELAY);
	// } else {
	//	 wait_ms(TAP_CODE_DELAY);
	// }
	unregister_code(code);
}

void tap_tb(uint8_t keycode0, uint8_t keycode1, uint8_t keycode2, uint8_t keycode3) {
	if(abs(cum_x) + abs(cum_y) >= cur_factor){
		if(abs(cum_x) > abs(cum_y)) {
			if(cum_x > 0) {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode0);
					cum_x = max(cum_x - cur_factor, 0);
				}
				cum_y = 0;
			} else {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode1);
					cum_x = min(cum_x + cur_factor, 0);
				}
				cum_y = 0;
			}
		} else {
			if(cum_y > 0) {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode2);
					cum_y = max(cum_y - cur_factor, 0);
					}
				cum_x = 0;
			} else {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode3);
					cum_y = min(cum_y + cur_factor, 0);
				}
				cum_x = 0;
			}
		}
	}
}

void handle_pointing_device_modes(void){
	report_mouse_t mouse_report = pointing_device_get_report();

	if (track_mode == cursor_mode) {
		if (integration_mode)
			cur_factor = cursor_multiplier_inte;
		else
			cur_factor = cursor_multiplier;
		mouse_report.x = CLAMP_HID( sensor_x * cur_factor / 100);
		mouse_report.y = CLAMP_HID(-sensor_y * cur_factor / 100);
	} else {
		// accumulate movement until threshold reached
		cum_x += sensor_x;
		cum_y += sensor_y;
		if (track_mode == carret_mode) {
			if (integration_mode)
				cur_factor = carret_threshold_inte;
			else
				cur_factor = carret_threshold;
			tap_tb(KC_RIGHT, KC_LEFT, KC_UP, KC_DOWN);

		} else if(track_mode == scroll_mode) {
				if (integration_mode)
					cur_factor = scroll_threshold_inte;
				else
					cur_factor = scroll_threshold;
				if(abs(cum_x) + abs(cum_y) >= cur_factor) {
					if(abs(cum_x) > abs(cum_y)) {
						mouse_report.h = sign(cum_x) * (abs(cum_x) + abs(cum_y)) / cur_factor;
					} else {
						mouse_report.v = sign(cum_y) * (abs(cum_x) + abs(cum_y)) / cur_factor;
					}
					cum_x = 0;
					cum_y = 0;
				}
		}
	}
	pointing_device_set_report(mouse_report);
	pointing_device_send();
}

void get_sensor_data(void) {
	if(!is_keyboard_master())
		return;
	report_pmw_t pmw_report = pmw_read_burst();

	if (integration_mode) {
		sensor_x += pmw_report.dx;
		sensor_y += pmw_report.dy;
	} else {
		sensor_x = pmw_report.dx;
		sensor_y = pmw_report.dy;
	}
}

void pointing_device_task(void) {
#ifndef POLLING
	if ( is_keyboard_master() && integration_mode )
		handle_pointing_device_modes();
#else
	get_sensor_data();
	handle_pointing_device_modes();
#endif
}

#ifndef POLLING
	ISR(INT6_vect) {
	    // Don't allow us to be interrupted until we're done here
	    interruptsOff();
		get_sensor_data();
		handle_pointing_device_modes();
		interruptsOn();
	}
#endif

// ENCODER HANDLING
bool encoder_update_user(uint8_t index, bool clockwise) {
    if (index == 1) {
        enc_used = true;
        if(enc_skip) {
            if (clockwise) {
                tap_code16(KC_MNXT);
            } else {
                tap_code16(KC_MPRV);
            }
            return true;

        } else if(IS_LAYER_ON(_RAISE)) {
            if (clockwise) {
                tap_code16(KC_WH_D);
            } else {
                tap_code16(KC_WH_U);
            }
            return true;
        } else {
            if (clockwise) {
                tap_code(KC_VOLU);
            } else {
                tap_code(KC_VOLD);
            }
            return true;
        }
    }
    return false;
}
#endif


/***************************
 * process_record_kb
 **************************/

bool other_key_pressed = false;
uint16_t symbol_down_timer = 0;
bool symbol_is_down = false;

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if(!process_record_user(keycode, record)) {
        return false;
    }

    if (keycode != SYMBOL && symbol_is_down && record->event.pressed) {
        other_key_pressed = true;
     }

    switch (keycode) {
        // LAYERS
        case LOWER:
            if (record->event.pressed) {
                layer_on(_LOWER);
                track_mode = scroll_mode;
            } else {
                layer_off(_LOWER);
                track_mode = cursor_mode;
            }
            return false;
        break;

        case RAISE:
            if (record->event.pressed) {
                layer_on(_RAISE);
                track_mode = carret_mode;
            } else {
                layer_off(_RAISE);
                track_mode = cursor_mode;
            }
            return false;
        break;

        case GAMING:
            if (record->event.pressed) {
                if (IS_LAYER_ON(_GAMING)) {
                    layer_off(_GAMING);
                } else {
                    layer_on(_GAMING);
                }
            }
            return false;
        break;

        case SYMBOL:
            if (record->event.pressed) {
                layer_on(_SYMBOL);
                integration_mode = true;
                symbol_down_timer = timer_read();
                other_key_pressed = false;
                symbol_is_down = true;
            } else {
                layer_off(_SYMBOL);
                if (timer_elapsed(symbol_down_timer) < 300 && other_key_pressed == false) {
                    tap_code(KC_CAPSLOCK);
                }
                integration_mode = false;
                symbol_is_down = false;
            }
            return false;
        break;

      // handle mouse
      case KC_BTN1:
          on_mouse_button(MOUSE_BTN1, record->event.pressed);
          return false;

      case KC_BTN2:
          on_mouse_button(MOUSE_BTN2, record->event.pressed);
          return false;

      case KC_BTN3:
          on_mouse_button(MOUSE_BTN3, record->event.pressed);
          return false;

      case KC_BTN4:
          on_mouse_button(MOUSE_BTN4, record->event.pressed);
          return false;

      case KC_BTN5:
          on_mouse_button(MOUSE_BTN5, record->event.pressed);
          return false;

      case KC_CPI_DOWN:
          if (cursor_multiplier > CPI_STEP)
              cursor_multiplier = cursor_multiplier - CPI_STEP;
          return false;

      case KC_CPI_STD:
          cursor_multiplier = 250;
          return false;

      case KC_CPI_UP:
          cursor_multiplier = cursor_multiplier + CPI_STEP;
          return false;

      case KC_SMO_SC:
        if (record->event.pressed) {
            if (smooth_scroll) {
                scroll_threshold = scroll_threshold * regular_smoothscroll_factor;
                scroll_threshold_inte = scroll_threshold_inte * regular_smoothscroll_factor;
                smooth_scroll = false;
            } else {
                scroll_threshold = scroll_threshold / regular_smoothscroll_factor;
                scroll_threshold_inte = scroll_threshold_inte / regular_smoothscroll_factor;
                smooth_scroll = true;
            }
        }
        return false;
        break;

    case MACSLEEP:
        if (record->event.pressed) {
            register_code(KC_RSFT);
            register_code(KC_RCTL);
            register_code(KC_POWER);
            unregister_code(KC_POWER);
            unregister_code(KC_RCTL);
            unregister_code(KC_RSFT);
        }
        return false;
        break;

    case ENC_PLAY:
        if (record->event.pressed) {
            enc_skip = true;
            enc_used = false;
        } else {
            enc_skip = false;
            if (!enc_used) {
                tap_code(KC_MPLY);
            }
        }
        return false;
        break;

    default:
        return true;
    }


}



// disable tx/rx LEDs
// void matrix_init_kb(void) {
//    DDRD &= ~(1<<5);
//    PORTD &= ~(1<<5);

//    DDRB &= ~(1<<0);
//    PORTB &= ~(1<<0);
// }

// debug
//		char snum[5];
//		itoa(variable, snum, 10);
//		SEND_STRING(" ");
//		send_string(snum);
