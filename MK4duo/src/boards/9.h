/****************************************************************************************
* 9
* Gen3+
****************************************************************************************/

#define MOTHERBOARD BOARD_SANGUINOLOLU_11   /*TODO: Figure out, Why is this done?*/
#define KNOWN_BOARD 1
#ifndef __AVR_ATmega644P__
#ifndef __AVR_ATmega1284P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif
#endif

#define ORIG_X_STEP_PIN         15
#define ORIG_X_DIR_PIN          18
#define X_STOP_PIN         20

#define ORIG_Y_STEP_PIN         23
#define ORIG_Y_DIR_PIN          22
#define Y_STOP_PIN         25

#define ORIG_Z_STEP_PIN         27
#define ORIG_Z_DIR_PIN          28
#define Z_STOP_PIN         30

#define ORIG_E0_STEP_PIN        17
#define ORIG_E0_DIR_PIN         21

#define LED_PIN            -1

#define ORIG_FAN_PIN            -1

#define ORIG_PS_ON_PIN         14
#define KILL_PIN           -1

#define ORIG_HEATER_0_PIN       12 // (extruder)

#define ORIG_HEATER_BED_PIN     16 // (bed)
#define ORIG_X_ENABLE_PIN       19
#define ORIG_Y_ENABLE_PIN       24
#define ORIG_Z_ENABLE_PIN       29
#define ORIG_E0_ENABLE_PIN      13

#define ORIG_TEMP_0_PIN          0   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1
#define ORIG_TEMP_BED_PIN        5   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
#define SDPOWER            -1
#define SDSS               4
#define ORIG_HEATER_2_PIN       -1

