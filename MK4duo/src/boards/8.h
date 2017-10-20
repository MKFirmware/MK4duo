/****************************************************************************************
* 8 - 81
* Teensylu 0.7 / Printrboard pin assignments (AT90USB1286)
* Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
  http://www.pjrc.com/teensy/teensyduino.html
* See http://reprap.org/wiki/Printrboard for more info
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_AT90USB1286__)
  #error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif
#if ENABLED(AT90USBxx_TEENSYPP_ASSIGNMENTS)  // use Teensyduino Teensy++2.0 pin assignments instead of Marlin traditional.
  #error These Teensylu/Printrboard assignments depend on traditional Marlin assignments, not AT90USBxx_TEENSYPP_ASSIGNMENTS in fastio.h
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Teensylu"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 0
#define ORIG_X_DIR_PIN 1
#define ORIG_X_ENABLE_PIN 39
#define ORIG_X_CS_PIN NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN 2
#define ORIG_Y_DIR_PIN 3
#define ORIG_Y_ENABLE_PIN 38
#define ORIG_Y_CS_PIN NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN 4
#define ORIG_Z_DIR_PIN 5
#define ORIG_Z_ENABLE_PIN 23
#define ORIG_Z_CS_PIN NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 6
#define ORIG_E0_DIR_PIN 7
#define ORIG_E0_ENABLE_PIN 19
#define ORIG_E0_CS_PIN NoPin
#define ORIG_SOL0_PIN NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN NoPin
#define ORIG_E1_DIR_PIN NoPin
#define ORIG_E1_ENABLE_PIN NoPin
#define ORIG_E1_CS_PIN NoPin
#define ORIG_SOL1_PIN NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN NoPin
#define ORIG_E2_DIR_PIN NoPin
#define ORIG_E2_ENABLE_PIN NoPin
#define ORIG_E2_CS_PIN NoPin
#define ORIG_SOL2_PIN NoPin

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN NoPin
#define ORIG_E3_DIR_PIN NoPin
#define ORIG_E3_ENABLE_PIN NoPin
#define ORIG_E3_CS_PIN NoPin
#define ORIG_SOL3_PIN NoPin

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN NoPin
#define ORIG_E4_DIR_PIN NoPin
#define ORIG_E4_ENABLE_PIN NoPin
#define ORIG_E4_CS_PIN NoPin
#define ORIG_SOL4_PIN NoPin

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN NoPin
#define ORIG_E5_DIR_PIN NoPin
#define ORIG_E5_ENABLE_PIN NoPin
#define ORIG_E5_CS_PIN NoPin
#define ORIG_SOL5_PIN NoPin

//###EXTRUDER_6
#define ORIG_E6_STEP_PIN NoPin
#define ORIG_E6_DIR_PIN NoPin
#define ORIG_E6_ENABLE_PIN NoPin
#define ORIG_E6_CS_PIN NoPin
#define ORIG_SOL6_PIN NoPin

//###EXTRUDER_7
#define ORIG_E7_STEP_PIN NoPin
#define ORIG_E7_DIR_PIN NoPin
#define ORIG_E7_ENABLE_PIN NoPin
#define ORIG_E7_CS_PIN NoPin
#define ORIG_SOL7_PIN NoPin

//###ENDSTOP
#define ORIG_X_MIN_PIN NoPin
#define ORIG_X_MAX_PIN NoPin
#define ORIG_Y_MIN_PIN NoPin
#define ORIG_Y_MAX_PIN NoPin
#define ORIG_Z_MIN_PIN NoPin
#define ORIG_Z_MAX_PIN NoPin
#define ORIG_Z2_MIN_PIN NoPin
#define ORIG_Z2_MAX_PIN NoPin
#define ORIG_Z3_MIN_PIN NoPin
#define ORIG_Z3_MAX_PIN NoPin
#define ORIG_Z4_MIN_PIN NoPin
#define ORIG_Z4_MAX_PIN NoPin
#define ORIG_E_MIN_PIN NoPin
#define ORIG_Z_PROBE_PIN NoPin

//###SINGLE_ENDSTOP
#define X_STOP_PIN NoPin
#define Y_STOP_PIN NoPin
#define Z_STOP_PIN NoPin

//###HEATER
#define ORIG_HEATER_0_PIN 21
#define ORIG_HEATER_1_PIN NoPin
#define ORIG_HEATER_2_PIN NoPin
#define ORIG_HEATER_3_PIN NoPin
#define ORIG_HEATER_BED_PIN 20
#define ORIG_HEATER_CHAMBER_PIN NoPin
#define ORIG_COOLER_PIN NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN NoPin
#define ORIG_TEMP_1_PIN NoPin
#define ORIG_TEMP_2_PIN NoPin
#define ORIG_TEMP_3_PIN NoPin
#define ORIG_TEMP_BED_PIN NoPin
#define ORIG_TEMP_CHAMBER_PIN NoPin
#define ORIG_TEMP_COOLER_PIN NoPin

//###FAN
#define ORIG_FAN0_PIN 22
#define ORIG_FAN1_PIN NoPin
#define ORIG_FAN2_PIN NoPin
#define ORIG_FAN3_PIN NoPin

//###MISC
#define ORIG_PS_ON_PIN NoPin
#define ORIG_BEEPER_PIN NoPin
#define LED_PIN NoPin
#define SDPOWER_PIN NoPin
#define SD_DETECT_PIN NoPin
#define SDSS 8
#define KILL_PIN NoPin
#define DEBUG_PIN NoPin
#define SUICIDE_PIN NoPin

//###LASER
#define ORIG_LASER_PWR_PIN NoPin
#define ORIG_LASER_PWM_PIN NoPin

//###SERVOS
#if NUM_SERVOS > 0
  #define SERVO0_PIN NoPin
  #if NUM_SERVOS > 1
    #define SERVO1_PIN NoPin
    #if NUM_SERVOS > 2
      #define SERVO2_PIN NoPin
      #if NUM_SERVOS > 3
        #define SERVO3_PIN NoPin
      #endif
    #endif
  #endif
#endif
//@@@

//###UNKNOWN_PINS
#define AT90USB 1286  // Disable MarlinSerial etc.
//@@@

//###IF_BLOCKS
// You may need to change ORIG_FAN0_PIN to 16 because Marlin isn't using fastio.h
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
//@@@
