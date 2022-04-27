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
#define _MOUSE 6
#define _NAV 7

// clang-format off
enum custom_keycodes {
    KC_BSPC_LCTL = SAFE_RANGE,
    KC_CPI_DOWN,
    KC_CPI_STD,
    KC_CPI_UP,
    RAISE,
    LOWER,
    SYMBOL,
    GAMING,
    MACSLEEP,
    ENC_PLAY,
    MLOC_X,
    MLOC_Y
};
// clang-format on

// Gaming
#define SCRSHT LSFT(KC_P5)
#define STATS MEH(KC_8)
#define RAISE MO(_RAISE)
#define LOWER MO(_LOWER)

#define LINE_DOWN LALT(KC_DOWN)
#define LINE_UP LALT(KC_UP)
#define WORD_LEFT LALT(KC_LEFT)
#define WORD_RIGHT LALT(KC_RIGHT)
#define BACK LGUI(KC_LEFT)
#define FORWARD LGUI(KC_RIGHT)
#define MAC_SCRNSHT LGUI(LSFT(KC_4))
#define MAC_SCRNRCD LGUI(LSFT(KC_5))

#define DEL_LINE MEH(KC_K)
#define SORT_LINES KC_F5

/***************************
 * Trackball related defines
 **************************/

uint8_t track_mode = 0;  // 0 Mousecursor; 1 arrowkeys/carret; 2 scrollwheel; 3 volume
#define cursor_mode 0
#define carret_mode 1
#define scroll_mode 2
#define misc_mode 3

uint8_t prev_track_mode = 0;
int16_t cum_x           = 0;
int16_t cum_y           = 0;
int16_t sensor_x        = 0;
int16_t sensor_y        = 0;
bool    m_lock_x        = false;
bool    m_lock_y        = false;

// Thresholds help to move only horizontal or vertical. When accumulated distance reaches threshold, only move one discrete value in direction with bigger delta.
uint8_t  carret_threshold      = 80;   // higher means slower
uint16_t carret_threshold_inte = 340;  // in integration mode higher threshold

uint8_t scroll_threshold = 6;  // divide if started smooth

#define cursor_multiplier_default 130;
uint16_t cursor_multiplier = cursor_multiplier_default;  // adjust cursor speed
#define CPI_STEP 20

int16_t cur_factor;

/***************************
 * Layers
 **************************/

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
[_MAIN] = LAYOUT_RHEUMATOID(
KC_TAB,     KC_Q,       KC_W,       KC_E,       KC_R,       KC_T,       KC_EQL,         ENC_PLAY,   KC_Y,       KC_U,       KC_I,       KC_O,       KC_P,      KC_GRV,
SYMBOL,     KC_A,       KC_S,       KC_D,       KC_F,       KC_G,       KC_MINS,        MO(_ADJUST),KC_H,       KC_J,       KC_K,       KC_L,       KC_SCLN,   KC_QUOT,
KC_LSPO,    KC_Z,       KC_X,       KC_C,       KC_V,       KC_B,                                   KC_N,       KC_M,       KC_COMM,    KC_DOT,     KC_SLSH,   KC_RSPC,
						KC_ESC,     KC_LALT,    KC_LGUI,    KC_SPC,     KC_ENT,                                 MO(_NAV),   LALT(KC_3), KC_AT,
											    KC_LCTL,    LOWER,      RAISE,          KC_BSPC,    KC_DEL
),

[_RAISE] = LAYOUT_RHEUMATOID(
KC_TILD,    KC_EXLM,    KC_AT,      KC_HASH,    KC_DLR,     KC_PERC,    _______,        _______,    KC_6,       KC_7,       KC_8,       KC_9,       KC_0,       KC_BSLS,
_______,    KC_1,       KC_2,       KC_3,       KC_4,       KC_5,       KC_6,           _______,    _______,    KC_4,       KC_5,       KC_6,       _______,    LALT(KC_3),
_______,    _______,    _______,    _______,    _______,    _______,                                _______,    KC_1,       KC_2,       KC_3,       _______,    _______,
                        _______,    _______,    _______,    _______,    _______,                                _______,    KC_0,       KC_DOT,
                                                _______,    _______,    _______,        _______,    _______
),

// Old number layout
// [_RAISE] = LAYOUT_RHEUMATOID(
// KC_TILD,    KC_EXLM,    KC_AT,      KC_HASH,    KC_DLR,     KC_PERC,    _______,        _______,    KC_CIRC,    KC_AMPR,    KC_ASTR,    KC_LPRN,    KC_RPRN,    KC_BSLS,
// _______,    KC_1,       KC_2,       KC_3,       KC_4,       KC_5,       _______,        _______,    KC_6,       KC_7,       KC_8,       KC_9,       KC_0,       LALT(KC_3),
// _______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
//                         _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
//                                                 _______,    _______,    _______,        _______,    _______
// ),

[_LOWER] = LAYOUT_RHEUMATOID(
_______,    KC_F1,      KC_F2,      KC_F3,      KC_F4,      KC_F5,       KC_F6,            _______,    KC_F6,      KC_F7,      KC_F8,      KC_F9,      KC_F10,     _______,
_______,    LGUI(KC_1), LGUI(KC_2), LGUI(KC_3), LGUI(KC_4), LGUI(KC_5), LGUI(KC_6),        _______,    _______,    KC_BTN1,    KC_BTN2,    KC_BTN3,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    _______,                                   _______,    _______,    _______,    _______,    _______,    _______,
                        _______,    _______,    _______,    _______,    _______,                                   _______,    _______,    _______,
                                                _______,    _______,    _______,           _______,    _______
),

[_SYMBOL] = LAYOUT_RHEUMATOID(
_______,    _______,    _______,    _______,    _______,    _______,    MLOC_X,         _______,    KC_EXLM,    KC_LBRC,    KC_RBRC,    KC_DLR,     KC_PERC,    KC_F12,
_______,    _______,    _______,    _______,    _______,    _______,    MLOC_Y,         KC_UNDS,    KC_PIPE,    KC_LCBR,    KC_RCBR,    KC_EQL,     KC_COLN,    KC_DQUO,
_______,    _______,    _______,    _______,    _______,    _______,                                KC_AMPR,    KC_LT,      KC_GT,      KC_MINS,    KC_BSLS,    _______,
						_______,    _______,    _______,    _______,    _______,                                _______,    KC_ASTR,    _______,
												_______,    _______,    _______,        SORT_LINES, DEL_LINE
),

[_GAMING] = LAYOUT_RHEUMATOID(
KC_TAB,     _______,    _______,    _______,    _______,    _______,    KC_Y,           _______,    _______,    _______,    _______,    _______,    _______,    _______,
KC_CAPS,    _______,    _______,    _______,    _______,    _______,    KC_H,           _______,    _______,    _______,    _______,    _______,    _______,    _______,
KC_LSFT,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    KC_RSFT,
						_______,    _______,    KC_LCTL,    _______,    _______,                                _______,    _______,    _______,
												KC_LGUI,    _______,    _______,        _______,    _______
),

[_ADJUST] = LAYOUT_RHEUMATOID(
_______,    _______,    _______,    _______,    _______,    KC_CPI_STD, KC_CPI_UP,      _______,    _______,    _______,    _______,    _______,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    GAMING,     KC_CPI_DOWN,    _______,    MACSLEEP,   _______,    _______,    _______,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
						_______,    _______,    LCG_SWP,    _______,    _______,                                _______,    _______,    _______,
												LCG_NRM,    _______,    _______,        SCRSHT,     STATS
),

[_MOUSE] = LAYOUT_RHEUMATOID(
_______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    _______,    _______,    _______,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    KC_BTN1,    KC_BTN2,    KC_BTN3,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
						_______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
											    _______,    _______,    _______,        _______,    _______
),

// [_MOUSE] = LAYOUT_RHEUMATOID(
// _______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    _______,    _______,    _______,    _______,    _______,
// _______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    KC_BTN1,    KC_BTN2,    KC_BTN3,    _______,    _______,
// _______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
// 						_______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
// 											    _______,    _______,    _______,        _______,    _______
// ),

[_NAV] = LAYOUT_RHEUMATOID(
_______,    _______,    _______,    _______,    MAC_SCRNSHT,MAC_SCRNRCD,_______,        _______,    _______,    LINE_DOWN,  LINE_UP,    _______,    _______,    _______,
_______,    _______,    _______,    BACK,       FORWARD,    _______,    _______,        _______,    KC_LEFT,    KC_DOWN,    KC_UP,      KC_RGHT,    _______,    _______,
_______,    _______,    _______,    _______,    _______,    _______,                                WORD_LEFT,  _______,    _______,    WORD_RIGHT, _______,    _______,
						_______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
												_______,    _______,    _______,        _______,    _______
),

};

// [_LOWER] = LAYOUT_RHEUMATOID(
// _______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    _______,    _______,    _______,    _______,    _______,
// _______,    _______,    _______,    _______,    _______,    _______,    _______,        _______,    _______,    _______,    _______,    _______,    _______,    _______,
// _______,    _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,    _______,    _______,    _______,
//                         _______,    _______,    _______,    _______,    _______,                                _______,    _______,    _______,
//                                                 _______,    _______,    _______,        _______,    _______
// ),
// clang-format on

void suspend_wakeup_init_user(void) { cursor_multiplier = cursor_multiplier_default; }

void led_set_user(uint8_t usb_led) { writePin(D2, usb_led & (1 << USB_LED_CAPS_LOCK)); }

/***************************
 * Mouse pressed
 **************************/

void on_mouse_button(uint8_t mouse_button, bool pressed) {
    report_mouse_t report = pointing_device_get_report();
    if (pressed)
        report.buttons |= mouse_button;
    else
        report.buttons &= ~mouse_button;
    pointing_device_set_report(report);
    pointing_device_send();
}

// utils
int max(int num1, int num2) { return (num1 > num2) ? num1 : num2; }
int min(int num1, int num2) { return (num1 > num2) ? num2 : num1; }

int8_t sign(int x) { return (x > 0) - (x < 0); }
float  clamp_hid(float value) { return constrain(value, -127, 127); }

/***************************
 * Trackball/Encoder handling
 **************************/
bool enc_used = false;
bool enc_skip = false;
bool inited   = false;

uint16_t last_motion_timer;
uint16_t start_motion_timer = 0;
uint16_t last_key_press     = 0;
uint16_t last_rthumb_press  = 0;
uint16_t last_mouse_press   = 0;
bool     mouse_is_down      = false;
bool     report_motion      = false;
bool     in_motion          = false;
int8_t   last_sensor_x      = 0;
int8_t   last_sensor_y      = 0;
bool     mods_active        = false;
bool     gui_active         = false;

#ifdef IS_RIGHT
void pointing_device_init(void) {
    if (!is_keyboard_master()) return;

    if (!inited) {
        pmw_spi_init();
        inited = true;
    }
}

void tap_code_fast(uint8_t code) {
    register_code(code);
    unregister_code(code);
}

void tap_tb(uint8_t keycode0, uint8_t keycode1, uint8_t keycode2, uint8_t keycode3) {
    if (abs(cum_x) + abs(cum_y) >= cur_factor) {
        if (abs(cum_x) > abs(cum_y)) {
            if (cum_x > 0) {
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
            if (cum_y > 0) {
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

void handle_pointing_device_modes(void) {
    report_mouse_t mouse_report = pointing_device_get_report();

    if (track_mode == cursor_mode) {
        cur_factor = cursor_multiplier;
        // if (!mouse_is_down && (get_mods() & (MOD_BIT(KC_LSFT)))) {
        //     cur_factor *= 2;
        // }
        mouse_report.x = !m_lock_y ? clamp_hid(sensor_x * cur_factor / 100) : 0;
        mouse_report.y = !m_lock_x ? clamp_hid(-sensor_y * cur_factor / 100) : 0;
        // xprintf("x: %d y: %d\n", mouse_report.x, mouse_report.y);
    } else {
        // accumulate movement until threshold reached
        cum_x += sensor_x;
        cum_y += sensor_y;
        if (track_mode == carret_mode) {
            cur_factor = carret_threshold;
            tap_tb(KC_RIGHT, KC_LEFT, KC_UP, KC_DOWN);

        } else if (track_mode == misc_mode) {
            cur_factor = carret_threshold;
            tap_tb(_______, _______, KC_VOLU, KC_VOLD);

        } else if (track_mode == scroll_mode) {
            cur_factor                = scroll_threshold;
            uint8_t factor_multiplier = 9;
            if (abs(cum_x) + abs(cum_y) >= cur_factor * factor_multiplier) {
                if (abs(cum_x) * 0.4 > abs(cum_y)) {
                    mouse_report.h = ceil(sign(cum_x) * (abs(cum_x) + abs(cum_y)) / (cur_factor * factor_multiplier));
                } else {
                    mouse_report.v = ceil(sign(cum_y) * (abs(cum_x) + abs(cum_y)) / (cur_factor * factor_multiplier));
                }
                cum_x = 0;
                cum_y = 0;
            }
        }
    }
    pointing_device_set_report(mouse_report);
    pointing_device_send();
}

int16_t olddx = 0;
int16_t olddy = 0;

void get_sensor_data(void) {
    if (!is_keyboard_master()) return;
    report_pmw_t data = pmw_read_burst();

    if (track_mode == cursor_mode) {
#    ifdef AUTO_MOUSEBUTTONS
        if (data.isMotion) {
            if (!in_motion) {
                start_motion_timer = timer_read();
            }
            last_motion_timer = timer_read();
            report_motion     = true;
            in_motion         = true;
            // if there's been a non-mouse key press, don't force the mouse layer too soon
            // clang-format off
            if (
                (
                    timer_elapsed(last_key_press) > 500 &&
                    timer_elapsed(start_motion_timer) > 30 &&
                    timer_elapsed(last_rthumb_press) > 200
                ) || mods_active
            ) {
                // clang-format on
                layer_on(_MOUSE);
                last_key_press = 0;
            } else if (abs(data.dx - olddx) + abs(data.dy - olddy) > 1) {
                layer_on(_MOUSE);
                last_key_press = 0;
            } else {
                // layer_off(_MOUSE);
                int8_t sensor_diff = abs(last_sensor_x - sensor_x) + abs(last_sensor_y - sensor_y);
                if (timer_elapsed(last_rthumb_press) < 800 || timer_elapsed(last_key_press) < 800) {
                    // if pointer has moved significantly, then enable the mouse buttons
                    if (sensor_diff > 1) {
                        layer_on(_MOUSE);
                    } //else {
                        // layer_off(_MOUSE);
                    // }
                }
            }

            if (timer_elapsed(last_mouse_press) < 110) {
                report_motion = false;
            }

        } else if (!data.isMotion) {
            if (timer_elapsed(last_motion_timer) > 1000) {  // in_motion &&
                in_motion          = false;
                report_motion      = false;
                start_motion_timer = 0;
                layer_off(_MOUSE);
            }
            if (timer_elapsed(last_key_press) < 800) {
                layer_off(_MOUSE);
            } else if (timer_elapsed(last_mouse_press) < 800) {
                layer_on(_MOUSE);
            }
        }

#    endif
#    ifndef AUTO_MOUSEBUTTONS
        if (data.isMotion) {
            if (!in_motion) {
                start_motion_timer = timer_read();
            }
            in_motion = true;
        } else {
            in_motion          = false;
            start_motion_timer = 0;
        }
        report_motion = true;
        if (gui_active) {
            layer_on(_MOUSE);
        } else {
            layer_off(_MOUSE);
        }
#    endif

    } else {
        report_motion = true;
        layer_off(_MOUSE);
    }

    if (report_motion) {
        float scale = constrain(timer_elapsed(start_motion_timer) / 1300.0, 0.8, 1.2);

        sensor_x = -data.dx * scale;
        sensor_y = -data.dy * scale;
    } else {
        sensor_x = 0;
        sensor_y = 0;
    }

    olddx = data.dx;
    olddy = data.dy;
}

void pointing_device_task(void) {
    get_sensor_data();
    handle_pointing_device_modes();
}

// ENCODER HANDLING
bool encoder_update_user(uint8_t index, bool clockwise) {
    if (index == 1) {
        enc_used = true;
        if (enc_skip) {
            if (clockwise) {
                tap_code16(KC_MNXT);
            } else {
                tap_code16(KC_MPRV);
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

layer_state_t layer_state_set_user(layer_state_t state) {
    if (IS_LAYER_ON_STATE(state, _GAMING) || IS_LAYER_ON_STATE(state, _MOUSE)) {
        writePin(F4, true);
    } else {
        writePin(F4, false);
    }
    return state;
}

bool     other_key_pressed = false;
uint16_t symbol_down_timer = 0;
uint16_t symbol_up_timer   = 0;
bool     symbol_is_down    = false;

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (!process_record_user(keycode, record)) {
        return false;
    }

    mods_active = !(get_mods() & (MOD_MASK_GUI | MOD_MASK_SHIFT | MOD_MASK_ALT | MOD_MASK_CTRL));
    gui_active  = !(get_mods() & (MOD_MASK_GUI));

    if (record->event.pressed) {
        if (keycode != SYMBOL && symbol_is_down) {
            other_key_pressed = true;
        }
        if (keycode != KC_BTN1 && keycode != KC_BTN2 && keycode != KC_BTN3) {  // && mods_active) {
            last_key_press = timer_read();
            last_sensor_x  = sensor_x;
            last_sensor_y  = sensor_y;
        }
    }

    if (keycode == KC_BTN1 || keycode == KC_BTN2 || keycode == KC_BTN3) {
        if (record->event.pressed) {
            mouse_is_down    = true;
            last_mouse_press = timer_read();
        } else {
            mouse_is_down = false;
        }
    }

    switch (keycode) {
        // LAYERS
        case LOWER:
            if (record->event.pressed) {
                layer_on(_LOWER);
                track_mode = misc_mode;  // carret_mode;

            } else {
                layer_off(_LOWER);
                track_mode = cursor_mode;
            }
            return false;

        case RAISE:
            if (record->event.pressed) {
                layer_on(_RAISE);
                track_mode = carret_mode;  // misc_mode;
            } else {
                layer_off(_RAISE);
                track_mode = cursor_mode;
            }
            return false;

        case GAMING:
            if (record->event.pressed) {
                if (IS_LAYER_ON(_GAMING)) {
                    layer_off(_GAMING);
                } else {
                    layer_on(_GAMING);
                }
            }
            return false;

        case SYMBOL:
            if (record->event.pressed) {
                layer_on(_SYMBOL);
                symbol_down_timer = timer_read();
                other_key_pressed = false;
                symbol_is_down    = true;
                track_mode        = scroll_mode;
            } else {
                layer_off(_SYMBOL);
                if (timer_elapsed(symbol_up_timer) > 50 && timer_elapsed(symbol_up_timer) < 250 && other_key_pressed == false) {
                    tap_code(KC_CAPSLOCK);
                }
                symbol_is_down  = false;
                track_mode      = cursor_mode;
                symbol_up_timer = timer_read();
            }
            return false;

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

        case KC_CPI_DOWN:
            if (cursor_multiplier > CPI_STEP) cursor_multiplier = cursor_multiplier - CPI_STEP;
            return false;

        case KC_CPI_STD:
            cursor_multiplier = cursor_multiplier_default;
            return false;

        case KC_CPI_UP:
            cursor_multiplier = cursor_multiplier + CPI_STEP;
            return false;

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

        case MLOC_X:
            if (record->event.pressed) {
                m_lock_x = true;
            } else {
                m_lock_x = false;
            }
            return false;

        case MLOC_Y:
            if (record->event.pressed) {
                m_lock_y = true;
            } else {
                m_lock_y = false;
            }
            return false;

        // store last time one of the thumb cluster keys was pressed so we can stop
        // the mouse layer being switched on on accidental trackball nudge
        case KC_DEL:
        case KC_BSPC:
            if (record->event.pressed) {
                last_rthumb_press = timer_read();
            }
            return true;

        default:
            return true;
    }
}
