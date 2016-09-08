/****************************************************************************************
* 702
* Minitronics v1.0
****************************************************************************************/

#define KNOWN_BOARD 1
#define BOARD_NAME "Minitronics v1.0"

#ifndef __AVR_ATmega1281__
  #error Oops! Make sure you have 'Minitronics ' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

#define ORIG_X_STEP_PIN 48
#define ORIG_X_DIR_PIN 47
#define ORIG_X_ENABLE_PIN 49
#define ORIG_X_MIN_PIN 5
#define ORIG_X_MAX_PIN -1 //2 //Max endstops default to disabled "-1", set to commented value to enable.

#define ORIG_Y_STEP_PIN 39 // A6
#define ORIG_Y_DIR_PIN 40 // A0
#define ORIG_Y_ENABLE_PIN 38
#define ORIG_Y_MIN_PIN 2
#define ORIG_Y_MAX_PIN -1 //15

#define ORIG_Z_STEP_PIN 42 // A2
#define ORIG_Z_DIR_PIN 43 // A6
#define ORIG_Z_ENABLE_PIN 41 // A1
#define ORIG_Z_MIN_PIN 6
#define ORIG_Z_MAX_PIN -1

#define ORIG_E0_STEP_PIN 45
#define ORIG_E0_DIR_PIN 44
#define ORIG_E0_ENABLE_PIN 27

#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 35
#define ORIG_E1_ENABLE_PIN 37

#define ORIG_E2_STEP_PIN -1
#define ORIG_E2_DIR_PIN -1
#define ORIG_E2_ENABLE_PIN -1

#define SDPOWER -1
#define SDSS               53

#define LED_PIN 46

#define ORIG_FAN_PIN 9
#define ORIG_FAN2_PIN -1
#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1

#define ORIG_HEATER_0_PIN 7 // EXTRUDER 1
#define ORIG_HEATER_1_PIN 8 // EXTRUDER 2
#define ORIG_HEATER_2_PIN 9 // thermo couple

#if TEMP_SENSOR_0 == -1
  #define ORIG_TEMP_0_PIN 5 // ANALOG NUMBERING
#else
  #define ORIG_TEMP_0_PIN 7 // ANALOG NUMBERING
#endif
#define ORIG_TEMP_1_PIN 6   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN -1  // ANALOG NUMBERING

#define ORIG_HEATER_BED_PIN 3 // BED
#define ORIG_TEMP_BED_PIN 6   // ANALOG NUMBERING

#define ORIG_BEEPER_PIN -1

#define LCD_PINS_RS -1
#define LCD_PINS_ENABLE -1
#define LCD_PINS_D4 -1
#define LCD_PINS_D5 -1
#define LCD_PINS_D6 -1
#define LCD_PINS_D7 -1

//buttons are directly attached using keypad
#define BTN_EN1 -1
#define BTN_EN2 -1
#define BTN_ENC -1 //the click

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#define SD_DETECT_PIN -1  // Megatronics does not use this port
