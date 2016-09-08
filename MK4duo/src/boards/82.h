/****************************************************************************************
* 82
* Brainwave 1.0 pin assignments (AT90USB646)
* Requires hardware bundle for Arduino:
*  https://github.com/unrepentantgeek/brainwave-arduino
****************************************************************************************/

#define KNOWN_BOARD 1

#define AT90USB 646  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB646__
#error Oops!  Make sure you have 'Brainwave' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         27
#define ORIG_X_DIR_PIN          29
#define ORIG_X_ENABLE_PIN       28
#define X_STOP_PIN              7
#define X_ATT_PIN               26

#define ORIG_Y_STEP_PIN         31
#define ORIG_Y_DIR_PIN          33
#define ORIG_Y_ENABLE_PIN       32
#define Y_STOP_PIN              6
#define Y_ATT_PIN               30

#define ORIG_Z_STEP_PIN         17
#define ORIG_Z_DIR_PIN          19
#define ORIG_Z_ENABLE_PIN       18
#define Z_STOP_PIN              5
#define Z_ATT_PIN               16

#define ORIG_E0_STEP_PIN        21
#define ORIG_E0_DIR_PIN         23
#define ORIG_E0_ENABLE_PIN      22
#define E0_ATT_PIN              20

#define ORIG_HEATER_0_PIN        4  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     38  // Bed
#define ORIG_FAN_PIN             3  // Fan

#define ORIG_TEMP_0_PIN          7  // Extruder / Analog pin numbering
#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1
#define ORIG_TEMP_BED_PIN        6  // Bed / Analog pin numbering

#define SDPOWER            -1
#define SDSS               -1
#define LED_PIN            39
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN           -1
#define ALARM_PIN          -1


