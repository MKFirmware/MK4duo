/****************************************************************************************
* 22
* Gen3  Monolithic Electronics
****************************************************************************************/

  #define KNOWN_BOARD 1

  #ifndef __AVR_ATmega644P__
    #error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
  #endif

  #define DEBUG_PIN 0

  // x axis
  #define ORIG_X_STEP_PIN       15
  #define ORIG_X_DIR_PIN        18
  #define ORIG_X_MIN_PIN        20
  //Alex Checar #define X_STOP_PIN         20
  #define ORIG_X_ENABLE_PIN     24  // actually uses ORIG_Y_DIR_PIN
  #define ORIG_X_MAX_PIN        -1

  // y axes
  #define ORIG_Y_STEP_PIN       23
  #define ORIG_Y_DIR_PIN        22
  #define ORIG_Y_MIN_PIN        25
  //Alex Checar #define Y_STOP_PIN         25
  #define ORIG_Y_ENABLE_PIN     24  // shared with ORIG_X_ENABLE_PIN
  #define ORIG_Y_MAX_PIN        -1

  // z axes
  #define ORIG_Z_STEP_PIN       27
  #define ORIG_Z_DIR_PIN        28
  #define ORIG_Z_MIN_PIN        30
  //Alex Checar #define Z_STOP_PIN         30
  #define ORIG_Z_ENABLE_PIN     29
  #define ORIG_Z_MAX_PIN        -1

  //extruder pins
  #define ORIG_E0_STEP_PIN      12
  #define ORIG_E0_DIR_PIN       17
  #define ORIG_E0_ENABLE_PIN    3

  #define ORIG_HEATER_0_PIN     16
  #define ORIG_TEMP_0_PIN       0

  #define ORIG_FAN_PIN -1

  //bed pins
  #define ORIG_HEATER_BED_PIN   -1
  #define ORIG_TEMP_BED_PIN     -1

  #define SDSS                  -1
  #define SDPOWER               -1
  #define LED_PIN               -1

  //pin for controlling the PSU.
  #define ORIG_PS_ON_PIN        14

  //Alex extras from Gen3+
  #define KILL_PIN              -1
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_HEATER_2_PIN     -1
