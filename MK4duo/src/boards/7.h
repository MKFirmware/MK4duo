/****************************************************************************************
* 7
* Ultimaker pin assignment
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "ULTIMAKER"

#ifndef __AVR_ATmega1280__
  #ifndef __AVR_ATmega2560__
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN 25
#define ORIG_X_DIR_PIN 23
#define ORIG_X_MIN_PIN 22
#define ORIG_X_MAX_PIN 24
#define ORIG_X_ENABLE_PIN 27

#define ORIG_Y_STEP_PIN 31
#define ORIG_Y_DIR_PIN 33
#define ORIG_Y_MIN_PIN 26
#define ORIG_Y_MAX_PIN 28
#define ORIG_Y_ENABLE_PIN 29

#define ORIG_Z_STEP_PIN 37
#define ORIG_Z_DIR_PIN 39
#define ORIG_Z_MIN_PIN 30
#define ORIG_Z_MAX_PIN 32
#define ORIG_Z_ENABLE_PIN 35

#define ORIG_HEATER_BED_PIN 4
#define ORIG_TEMP_BED_PIN 10

#define ORIG_HEATER_0_PIN  2
#define ORIG_TEMP_0_PIN 8

#define ORIG_HEATER_1_PIN 3
#define ORIG_TEMP_1_PIN 9

#define ORIG_HEATER_2_PIN -1
#define ORIG_TEMP_2_PIN -1

#define ORIG_E0_STEP_PIN         43
#define ORIG_E0_DIR_PIN          45
#define ORIG_E0_ENABLE_PIN       41

#define ORIG_E1_STEP_PIN         49
#define ORIG_E1_DIR_PIN          47
#define ORIG_E1_ENABLE_PIN       48

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
#define ORIG_FAN_PIN        7
#define ORIG_PS_ON_PIN     12
#define KILL_PIN           -1
#define SUICIDE_PIN        54  //PIN that has to be turned on right after start, to keep power flowing.

#if ENABLED(ULTRA_LCD)

  #if ENABLED(NEWPANEL)
  //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 18

    #define LCD_PINS_RS 20
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 16
    #define LCD_PINS_D5 21
    #define LCD_PINS_D6 5
    #define LCD_PINS_D7 6

    //buttons are directly attached
    #define BTN_EN1 40
    #define BTN_EN2 42
    #define BTN_ENC 19  //the click

    #define SD_DETECT_PIN 38

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 18

    //buttons are attached to a shift register
    #define SHIFT_CLK 38
    #define SHIFT_LD 42
    #define SHIFT_OUT 40
    #define SHIFT_EN 17

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 5
    #define LCD_PINS_D4 6
    #define LCD_PINS_D5 21
    #define LCD_PINS_D6 20
    #define LCD_PINS_D7 19

    #define SD_DETECT_PIN -1
  #endif
#endif //ULTRA_LCD

