/****************************************************************************************
* 11
* Gen7 v1.1, v1.2
****************************************************************************************/

  #define KNOWN_BOARD

  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops! Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
  #endif

  #if DISABLED(GEN7_VERSION)
    #define GEN7_VERSION   12  // v1.x
  #endif

  //X axis pins
  #define ORIG_X_STEP_PIN       19
  #define ORIG_X_DIR_PIN        18
  #define ORIG_X_ENABLE_PIN     24
  #define X_STOP_PIN            7

  //Y axis pins
  #define ORIG_Y_STEP_PIN       23
  #define ORIG_Y_DIR_PIN        22
  #define ORIG_Y_ENABLE_PIN     24
  #define Y_STOP_PIN            5

  //Z axis pins
  #define ORIG_Z_STEP_PIN       26
  #define ORIG_Z_DIR_PIN        25
  #define ORIG_Z_ENABLE_PIN     24
  #define ORIG_Z_MIN_PIN        1
  #define ORIG_Z_MAX_PIN        0

  //extruder pins
  #define ORIG_E0_STEP_PIN      28
  #define ORIG_E0_DIR_PIN       27
  #define ORIG_E0_ENABLE_PIN    24

  #define ORIG_TEMP_0_PIN       1
  #define ORIG_TEMP_1_PIN      -1
  #define ORIG_TEMP_2_PIN      -1
  #define ORIG_TEMP_BED_PIN     2

  #define ORIG_HEATER_0_PIN     4
  #define ORIG_HEATER_1_PIN    -1
  #define ORIG_HEATER_2_PIN    -1
  #define ORIG_HEATER_BED_PIN   3

  #define KILL_PIN             -1

  #define SDPOWER              -1
  #define SDSS                 -1  // SCL pin of I2C header
  #define LED_PIN              -1

  #define ORIG_FAN_PIN          31
  #define ORIG_PS_ON_PIN        15

  //All these generations of Gen7 supply thermistor power
  //via PS_ON, so ignore bad thermistor readings
  #define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

  //our pin for debugging.
  #define DEBUG_PIN             0

  //our RS485 pins
  #define TORIG_X_ENABLE_PIN    12
  #define RORIG_X_ENABLE_PIN    13
