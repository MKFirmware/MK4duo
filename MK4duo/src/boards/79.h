/****************************************************************************************
* 79
* 3DVERTEX
****************************************************************************************/

#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          -1 //changed over K8200 (was 3)
#define ORIG_X_MAX_PIN           3 //changed over K8200 (was 2)

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          -1 //changed over K8200 (was 14)
#define ORIG_Y_MAX_PIN          14 //changed over K8200 (was 15)

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       63 //changed over K8200 (was 62)
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        32 //changed over K8200 (was 36)
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define SDPOWER                 -1
#define SDSS                    25
#define LED_PIN                 13


#define ORIG_FAN_PIN             8 // IO pin. Buffer needed
#define ORIG_FAN1_PIN            2 // IO pin. Buffer needed

#define ORIG_PS_ON_PIN          -1 //changed over K8200 (was 12)

#define ORIG_HEATER_0_PIN       10
#define ORIG_HEATER_1_PIN       11 //changed over K8200 (was 12)
#define ORIG_HEATER_2_PIN       -1 //changed over K8200 (was 6)
#define ORIG_HEATER_3_PIN       -1
#define ORIG_HEATER_BED_PIN      9 // BED

#define ORIG_TEMP_0_PIN         13 // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         15 // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1 // ANALOG NUMBERING
#define ORIG_TEMP_BED_PIN       14 // ANALOG NUMBERING

#define ORIG_BEEPER_PIN         -1 //changed over K8200 (was 33)


#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)

  #define LCD_PINS_RS 27
  #define LCD_PINS_ENABLE 29
  #define LCD_PINS_D4 37
  #define LCD_PINS_D5 35
  #define LCD_PINS_D6 33
  #define LCD_PINS_D7 31

  ///Buttons are directly attached using AUX-2
  #define BTN_EN1 17 //changed over K8200 (was 16)
  #define BTN_EN2 16 //changed over K8200 (was 17)
  #define BTN_ENC 23 //the click
#endif // ULTRA_LCD && NEWPANEL

// SPI for Max6675 Thermocouple
// Do not use pin 53 if there is even the remote possibility of using Display/SD card
#define MAX6675_SS          66

