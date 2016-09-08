/****************************************************************************************
* 4
* Duemilanove w/ ATMega328P
****************************************************************************************/

#define KNOWN_BOARD 1
#define BOARD_NAME "2009"

#ifndef __AVR_ATmega328P__
#error Oops!  Make sure you have 'Arduino Duemilanove w/ ATMega328' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         19
#define ORIG_X_DIR_PIN          18
#define ORIG_X_ENABLE_PIN       -1
#define X_STOP_PIN              17

#define ORIG_Y_STEP_PIN         10
#define ORIG_Y_DIR_PIN           7
#define ORIG_Y_ENABLE_PIN       -1
#define Y_STOP_PIN               8

#define ORIG_Z_STEP_PIN         13
#define ORIG_Z_DIR_PIN           3
#define ORIG_Z_ENABLE_PIN        2
#define Z_STOP_PIN               4

#define ORIG_E0_STEP_PIN         11
#define ORIG_E0_DIR_PIN          12
#define ORIG_E0_ENABLE_PIN       -1

#define SDPOWER                  -1
#define SDSS                     -1
#define LED_PIN                  -1
#define ORIG_FAN_PIN              5
#define ORIG_PS_ON_PIN           -1
#define KILL_PIN                 -1

#define ORIG_HEATER_0_PIN         6
#define ORIG_HEATER_1_PIN        -1
#define ORIG_HEATER_2_PIN        -1
#define ORIG_TEMP_0_PIN           0    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_TEMP_1_PIN          -1
#define ORIG_TEMP_2_PIN          -1
#define ORIG_HEATER_BED_PIN      -1
#define ORIG_TEMP_BED_PIN        -1

