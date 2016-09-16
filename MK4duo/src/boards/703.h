/****************************************************************************************
* 703
* MegaTronics v3.0
****************************************************************************************/

#define KNOWN_BOARD 1
#define BOARD_NAME "MegaTronics V3.0"

#ifndef __AVR_ATmega2560__
  #error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

#define ORIG_X_STEP_PIN         58
#define ORIG_X_DIR_PIN          57
#define ORIG_X_ENABLE_PIN       59
#define ORIG_X_MIN_PIN          37
#define ORIG_X_MAX_PIN          40   //2 //Max endstops default to disabled "-1", set to commented value to enable.

#define ORIG_Y_STEP_PIN          5 // A6
#define ORIG_Y_DIR_PIN          17 // A0
#define ORIG_Y_ENABLE_PIN        4
#define ORIG_Y_MIN_PIN          41
#define ORIG_Y_MAX_PIN          38   //15

#define ORIG_Z_STEP_PIN         16 // A2
#define ORIG_Z_DIR_PIN          11 // A6
#define ORIG_Z_ENABLE_PIN       3 // A1
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN        28
#define ORIG_E0_DIR_PIN         27
#define ORIG_E0_ENABLE_PIN      29

#define ORIG_E1_STEP_PIN        25
#define ORIG_E1_DIR_PIN         24
#define ORIG_E1_ENABLE_PIN      26

#define ORIG_E2_STEP_PIN        22
#define ORIG_E2_DIR_PIN         60
#define ORIG_E2_ENABLE_PIN      23

#define ORIG_E3_STEP_PIN        54
#define ORIG_E3_DIR_PIN         55
#define ORIG_E3_ENABLE_PI       55

#define SDPOWER -1
#define SDSS 53
#define LED_PIN 13

#if NUM_SERVOS > 0
  #define SERVO0_PIN            46 //AUX3-6
  #if NUM_SERVOS > 1
    #define SERVO1_PIN          47 //AUX3-5
    #if NUM_SERVOS > 2
      #define SERVO2_PIN        48 //AUX3-4
      #if NUM_SERVOS > 3
        #define SERVO2_PIN      49 //AUX3-3
      #endif
    #endif
  #endif
#endif

#define ORIG_PS_ON_PIN 12
#define KILL_PIN -1

#define ORIG_HEATER_0_PIN 2
#define ORIG_HEATER_1_PIN 8
#define ORIG_HEATER_2_PIN 9
#define ORIG_HEATER_BED_PIN 10
#define ORIG_FAN_PIN 6
#define ORIG_FAN2_PIN 7

#if TEMP_SENSOR_0 == -1
  #define ORIG_TEMP_0_PIN 11 // ANALOG NUMBERING
#else
  #define ORIG_TEMP_0_PIN 15 // ANALOG NUMBERING
#endif

#if TEMP_SENSOR_1 == -1
  #define ORIG_TEMP_1_PIN 10 // ANALOG NUMBERING
#else
  #define ORIG_TEMP_1_PIN 13 // ANALOG NUMBERING
#endif

#if TEMP_SENSOR_2 == -1
  #define ORIG_TEMP_2_PIN 9 // ANALOG NUMBERING
#else
  #define ORIG_TEMP_2_PIN 12 // ANALOG NUMBERING
#endif

#if TEMP_SENSOR_BED == -1
  #define ORIG_TEMP_BED_PIN 8 // ANALOG NUMBERING
#else
  #define ORIG_TEMP_BED_PIN 14 // ANALOG NUMBERING
#endif

#define ORIG_BEEPER_PIN 61

#define LCD_PINS_RS 32
#define LCD_PINS_ENABLE 31
#define LCD_PINS_D4 14
#define LCD_PINS_D5 30
#define LCD_PINS_D6 39
#define LCD_PINS_D7 15

#define SHIFT_CLK 43
#define SHIFT_LD 35
#define SHIFT_OUT 34
#define SHIFT_EN 44

//buttons are directly attached using keypad
#define BTN_EN1 44
#define BTN_EN2 45
#define BTN_ENC 33 //the click

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#define SD_DETECT_PIN -1  // Megatronics does not use this port
