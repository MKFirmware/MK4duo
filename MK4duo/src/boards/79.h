/****************************************************************************************
* 79
* 3DVERTEX
****************************************************************************************/

#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME 			"3DVERTEX"
#endif

#define LARGE_FLASH true

// X AXIS
#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          -1
#define ORIG_X_MAX_PIN           3

// Y AXIS
#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          -1
#define ORIG_Y_MAX_PIN          14

// Z AXIS
#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       63
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1

// EXTRUDER 0
#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

// EXTRUDER 1
#define ORIG_E1_STEP_PIN        32
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

// FANS
#define ORIG_FAN_PIN             8  // IO pin. Buffer needed
#define ORIG_FAN1_PIN            2  // IO pin. Buffer needed

// HEATERS
#define ORIG_HEATER_0_PIN       10
#define ORIG_HEATER_1_PIN       11
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_3_PIN       -1
#define ORIG_HEATER_BED_PIN      9  // BED

// TEMPERATURE SENSORS
#define ORIG_TEMP_0_PIN         13  // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         15  // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1  // ANALOG NUMBERING
#define ORIG_TEMP_BED_PIN       14  // ANALOG NUMBERING

// MISC
#define SDPOWER                 -1
#define SD_DETECT_PIN           -1
#define SDSS                    25
#define ORIG_PS_ON_PIN          -1
#define LED_PIN                 13
#define ORIG_BEEPER_PIN         -1

// SERVOS
///TODO Add servo pins if present!

// DISPLAY
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  // LCD
  #define LCD_PINS_RS           27
  #define LCD_PINS_ENABLE       29
  #define LCD_PINS_D4           37
  #define LCD_PINS_D5           35
  #define LCD_PINS_D6           33
  #define LCD_PINS_D7           31

  // BUTTONS
  #define BTN_EN1               17
  #define BTN_EN2               16
  #define BTN_ENC               23  // the click
#endif

// SPI for Max6675 Thermocouple
// Do not use pin 53 if there is even the remote possibility of using Display/SD card
#define MAX6675_SS              66
