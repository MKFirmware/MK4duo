/****************************************************************************************
* 403 - 404
* Arduino pin assignment
* Ramps - FD v1 & v2
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "RAMPS FD v1"

#ifndef __SAM3X8E__
  #error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#if MB(RAMPS_FD_V1)
  #define RAMPS_FD_V1
  #define INVERTED_HEATER_PINS
  #define INVERTED_BED_PIN
  #define INVERTED_CHAMBER_PIN
  #define INVERTED_COOLER_PIN
  // No EEPROM
  // Use 4k7 thermistor tables
#else
  #define RAMPS_FD_V2
  // EEPROM supported
  // Use 1k thermistor tables
#endif

#define ORIG_X_STEP_PIN         63
#define ORIG_X_DIR_PIN          62
#define ORIG_X_ENABLE_PIN       48
#define ORIG_X_MIN_PIN          22
#define ORIG_X_MAX_PIN          30

#define ORIG_Y_STEP_PIN         65
#define ORIG_Y_DIR_PIN          64
#define ORIG_Y_ENABLE_PIN       46
#define ORIG_Y_MIN_PIN          24
#define ORIG_Y_MAX_PIN          38

#define ORIG_Z_STEP_PIN         67
#define ORIG_Z_DIR_PIN          66
#define ORIG_Z_ENABLE_PIN       44
#define ORIG_Z_MIN_PIN          26
#define ORIG_Z_MAX_PIN          34

#define ORIG_E0_STEP_PIN        36
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      42

#define ORIG_E1_STEP_PIN        43
#define ORIG_E1_DIR_PIN         41
#define ORIG_E1_ENABLE_PIN      39

#define ORIG_E2_STEP_PIN        32
#define ORIG_E2_DIR_PIN         47
#define ORIG_E2_ENABLE_PIN      45

#define SDPOWER                 -1
#define SDSS                     4
#define LED_PIN                 13

#define ORIG_BEEPER_PIN         -1

#define ORIG_FAN_PIN            -1

#define CONTROLLER_FAN_PIN      -1

#define ORIG_PS_ON_PIN          -1

#define KILL_PIN                -1


#define ORIG_HEATER_BED_PIN      8  // BED

#define ORIG_HEATER_0_PIN        9
#define ORIG_HEATER_1_PIN       10
#define ORIG_HEATER_2_PIN       11

#define ORIG_TEMP_BED_PIN        0  // ANALOG NUMBERING

#define ORIG_TEMP_0_PIN          1  // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN          2  // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN          3  // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN          4  // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN             7
  #if NUM_SERVOS > 1
    #define SERVO1_PIN           6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN       3
      #endif
    #endif
  #endif
#endif

#if ENABLED(ULTRA_LCD)
  #if ENABLED(NEWPANEL)
    // ramps-fd lcd adaptor
    #define LCD_PINS_RS         16
    #define LCD_PINS_ENABLE     17
    #define LCD_PINS_D4         23
    #define LCD_PINS_D5         25
    #define LCD_PINS_D6         27
    #define LCD_PINS_D7         29

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define ORIG_BEEPER_PIN   37

      #define BTN_EN1           33
      #define BTN_EN2           31
      #define BTN_ENC           35

      #define SD_DETECT_PIN     49
    #endif
  #endif
#endif //ULTRA_LCD

// SPI for Max6675 Thermocouple
#define MAX6675_SS              53




