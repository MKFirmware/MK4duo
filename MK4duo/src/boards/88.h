/****************************************************************************************
* 88
* 5DPrint D8 Driver board
* https://bitbucket.org/makible/5dprint-d8-controller-board
****************************************************************************************/

#define KNOWN_BOARD
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB1286__
  #error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH   true

#define ORIG_X_STEP_PIN          0
#define ORIG_X_DIR_PIN           1
#define ORIG_X_ENABLE_PIN       23
#define X_STOP_PIN              37

#define ORIG_Y_STEP_PIN          2
#define ORIG_Y_DIR_PIN           3
#define ORIG_Y_ENABLE_PIN       19
#define Y_STOP_PIN              36

#define ORIG_Z_STEP_PIN          4
#define ORIG_Z_DIR_PIN           5
#define ORIG_Z_ENABLE_PIN       18
#define Z_STOP_PIN              39

#define ORIG_E0_STEP_PIN         6
#define ORIG_E0_DIR_PIN          7
#define ORIG_E0_ENABLE_PIN      17

#define ORIG_HEATER_0_PIN       21  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     20  // Bed
// You may need to change ORIG_FAN_PIN to 16 because Marlin isn't using fastio.h
// for the fan and Teensyduino uses a different pin mapping.
#define ORIG_FAN_PIN            16  // Fan

#define ORIG_TEMP_0_PIN          1  // Extruder / Analog pin numbering
#define ORIG_TEMP_BED_PIN        0  // Bed / Analog pin numbering

#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1

#define SDPOWER                 -1
#define LED_PIN                 -1
#define ORIG_PS_ON_PIN          -1
#define KILL_PIN                -1
#define ALARM_PIN               -1

// The SDSS pin uses a different pin mapping from file Sd2PinMap.h
#define SDSS                    20

// Microstepping pins
// Note that the pin mapping is not from fastio.h
// See Sd2PinMap.h for the pin configurations
#define X_MS1_PIN 25
#define X_MS2_PIN 26
#define Y_MS1_PIN 9
#define Y_MS2_PIN 8
#define Z_MS1_PIN 7
#define Z_MS2_PIN 6
#define E0_MS1_PIN 5
#define E0_MS2_PIN 4
