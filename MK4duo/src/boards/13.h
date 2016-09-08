/****************************************************************************************
* 13
* Gen7 v1.4 pin assignment
****************************************************************************************/

  #define GEN7_VERSION     14 // v1.4
  #define KNOWN_BOARD

  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops! Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
  #endif

  //X axis pins
  #define ORIG_X_STEP_PIN       29
  #define ORIG_X_DIR_PIN        28
  #define ORIG_X_ENABLE_PIN     25
  #define X_STOP_PIN        0

  //Y axis pins
  #define ORIG_Y_STEP_PIN       27
  #define ORIG_Y_DIR_PIN        26
  #define ORIG_Y_ENABLE_PIN     25
  #define Y_STOP_PIN        1

  //Z axis pins
  #define ORIG_Z_STEP_PIN       23
  #define ORIG_Z_DIR_PIN        22
  #define ORIG_Z_ENABLE_PIN     25
  #define Z_STOP_PIN        2

  //extruder pins
  #define ORIG_E0_STEP_PIN      19
  #define ORIG_E0_DIR_PIN       18
  #define ORIG_E0_ENABLE_PIN    25

  #define ORIG_TEMP_0_PIN        1
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_TEMP_BED_PIN      0

  #define ORIG_HEATER_0_PIN      4
  #define ORIG_HEATER_1_PIN     -1
  #define ORIG_HEATER_2_PIN     -1
  #define ORIG_HEATER_BED_PIN    3

  #define KILL_PIN         -1

  #define SDPOWER          -1
  #define SDSS             -1  // SCL pin of I2C header
  #define LED_PIN          -1

  #define ORIG_FAN_PIN          -1

  #define ORIG_PS_ON_PIN        15

  //our pin for debugging.
  #define DEBUG_PIN         0

  //our RS485 pins
  #define TORIG_X_ENABLE_PIN    12
  #define RORIG_X_ENABLE_PIN    13
