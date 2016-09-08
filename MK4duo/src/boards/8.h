/****************************************************************************************
* 8 - 81
* Teensylu 0.7 / Printrboard pin assignments (AT90USB1286)
* Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
  http://www.pjrc.com/teensy/teensyduino.html
* See http://reprap.org/wiki/Printrboard for more info
****************************************************************************************/

#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#ifdef AT90USBxx_TEENSYPP_ASSIGNMENTS  // use Teensyduino Teensy++2.0 pin assignments instead of Marlin traditional.
#error These Teensylu/Printrboard assignments depend on traditional Marlin assignments, not AT90USBxx_TEENSYPP_ASSIGNMENTS in fastio.h
#endif

#define LARGE_FLASH        true

#define ORIG_X_STEP_PIN          0
#define ORIG_X_DIR_PIN           1
#define ORIG_X_ENABLE_PIN       39

#define ORIG_Y_STEP_PIN          2
#define ORIG_Y_DIR_PIN           3
#define ORIG_Y_ENABLE_PIN       38

#define ORIG_Z_STEP_PIN          4
#define ORIG_Z_DIR_PIN           5
#define ORIG_Z_ENABLE_PIN       23

#define ORIG_E0_STEP_PIN         6
#define ORIG_E0_DIR_PIN          7
#define ORIG_E0_ENABLE_PIN      19

#define ORIG_HEATER_0_PIN       21  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     20  // Bed
#define ORIG_FAN_PIN            22  // Fan
// You may need to change ORIG_FAN_PIN to 16 because Marlin isn't using fastio.h
// for the fan and Teensyduino uses a different pin mapping.

#if MB(TEENSYLU)  // Teensylu
  #define X_STOP_PIN         13
  #define Y_STOP_PIN         14
  #define Z_STOP_PIN         15
  #define ORIG_TEMP_0_PIN          7  // Extruder / Analog pin numbering
  #define ORIG_TEMP_BED_PIN        6  // Bed / Analog pin numbering
#else  // Printrboard
  #define X_STOP_PIN         35
  #define Y_STOP_PIN          8
  #define Z_STOP_PIN         36
  #define ORIG_TEMP_0_PIN          1  // Extruder / Analog pin numbering
  #define ORIG_TEMP_BED_PIN        0  // Bed / Analog pin numbering
#endif

#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1

#define SDPOWER            -1
#define SDSS                8
#define LED_PIN            -1
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN           -1
#define ALARM_PIN          -1


