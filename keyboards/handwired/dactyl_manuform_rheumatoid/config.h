/*
Copyright 2020 Qurn

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "config_common.h"

/* USB Device descriptor parameter */
#define VENDOR_ID 0xFEED
#define PRODUCT_ID 0x3060
#define DEVICE_VER 0x0001
#define MANUFACTURER   pixelbreaker
#define PRODUCT Rheumatoid
#define DESCRIPTION Dactyl Manuform with trackball

/* key matrix size */
#define MATRIX_ROWS 10
#define MATRIX_COLS 7

// encoders
#define ENCODERS_PAD_A { F5 }
#define ENCODERS_PAD_B { F6 }
#define ENCODER_RESOLUTION 4
#define ENCODER_DIRECTION_FLIP

// Is this the right (master side)?

#define IS_RIGHT
#define MASTER_RIGHT

#ifndef IS_RIGHT // left
    #define MATRIX_ROW_PINS { F7, B1, B3, B2, B6 }
    #define MATRIX_COL_PINS { D1, D4, C6, D7, E6, B4, B5 }
#else // right
    #define MATRIX_ROW_PINS { B7, D5, C7, F1 }
    #define MATRIX_COL_PINS { B5, B4, E6, D7, C6, D4, D1 }

    #define LED_CAPS_LOCK_PIN D2
    #define LED_PIN_ON_STATE 1
#endif

#define DIODE_DIRECTION COL2ROW

#define USB_POLLING_INTERVAL_MS 1
// #define USB_MAX_POWER_CONSUMPTION 100

#define SS_PIN F7
#define PMW_CPI 600
#define ROTATIONAL_TRANSFORM_ANGLE -0x30

/*
 * Split Keyboard specific options, make sure you have 'SPLIT_KEYBOARD = yes' in your rules.mk, and define SOFT_SERIAL_PIN.
 */
#define SOFT_SERIAL_PIN D0  // or D1, D2, D3, E6
// #define USE_I2C
#define SPLIT_USB_DETECT
// #define EE_HANDS

/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed */
#define DEBOUNCE 5

#define TAPPING_TERM 200
#define POLLING true

