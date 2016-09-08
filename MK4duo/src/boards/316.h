/****************************************************************************************
* 316
* PiBot Controller Rev2.0
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "PiBot R2"

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

// X axis pins
#define ORIG_X_STEP_PIN         24
#define ORIG_X_DIR_PIN          23
#define ORIG_X_ENABLE_PIN       22
#define ORIG_X_MIN_PIN          62
#define ORIG_X_MAX_PIN          63

// Y axis pins
#define ORIG_Y_STEP_PIN         27
#define ORIG_Y_DIR_PIN          26
#define ORIG_Y_ENABLE_PIN       25
#define ORIG_Y_MIN_PIN          64
#define ORIG_Y_MAX_PIN          65

// Z axis pins
#define ORIG_Z_STEP_PIN         15
#define ORIG_Z_DIR_PIN          14
#define ORIG_Z_ENABLE_PIN       39
#define ORIG_Z_MIN_PIN          66
#define ORIG_Z_MAX_PIN          67

// E axis pins
#define ORIG_E0_STEP_PIN        32
#define ORIG_E0_DIR_PIN         31
#define ORIG_E0_ENABLE_PIN      30

#define ORIG_E1_STEP_PIN        35
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      33

#define SDPOWER                 -1
#define SDSS                    53
#define LED_PIN                 -1
#define SD_DETECT_PIN           40

#define ORIG_FAN_PIN             6
#define ORIG_FAN2_PIN            7
#define ORIG_PS_ON_PIN          17

#define ORIG_HEATER_0_PIN        5
#define ORIG_HEATER_1_PIN        2
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_3_PIN       -1

#define ORIG_TEMP_0_PIN          2  // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN          4  // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1  // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN         -1  // ANALOG NUMBERING

#define ORIG_HEATER_BED_PIN      4  // BED
#define ORIG_TEMP_BED_PIN        1  // ANALOG NUMBERING
