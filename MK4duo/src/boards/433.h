/****************************************************************************************
* 433
* Arduino Due pin assignment
* for RAMPS4DUE (http://forums.reprap.org/read.php?219,479626,page=1)
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "RAMPS4DUE"

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN           2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define ORIG_HEATER_0_PIN       10
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN      8    // BED

#define ORIG_TEMP_0_PIN          9   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         -1   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1   // ANALOG NUMBERING
#define ORIG_TEMP_BED_PIN       10   // ANALOG NUMBERING

#define ORIG_FAN_PIN             9
#define ORIG_PS_ON_PIN          12
#define SDPOWER                 -1
#define SDSS                    53
#define LED_PIN                 13

