/****************************************************************************************
* 301
* Rambo
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "Rambo"

#ifndef __AVR_ATmega2560__
     #error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN 37
#define ORIG_X_DIR_PIN 48
#define ORIG_X_MIN_PIN 12
#define ORIG_X_MAX_PIN 24
#define ORIG_X_ENABLE_PIN 29
#define X_MS1_PIN 40
#define X_MS2_PIN 41

#define ORIG_Y_STEP_PIN 36
#define ORIG_Y_DIR_PIN 49
#define ORIG_Y_MIN_PIN 11
#define ORIG_Y_MAX_PIN 23
#define ORIG_Y_ENABLE_PIN 28
#define Y_MS1_PIN 69
#define Y_MS2_PIN 39

#define ORIG_Z_STEP_PIN 35
#define ORIG_Z_DIR_PIN 47
#define ORIG_Z_MIN_PIN 10
#define ORIG_Z_MAX_PIN 30
#define ORIG_Z_ENABLE_PIN 27
#define Z_MS1_PIN 68
#define Z_MS2_PIN 67

#define ORIG_HEATER_BED_PIN 3
#define ORIG_TEMP_BED_PIN 2

#define ORIG_HEATER_0_PIN  9
#define ORIG_TEMP_0_PIN 0

#define ORIG_HEATER_1_PIN 7
#define ORIG_TEMP_1_PIN 1

#if ENABLED(BARICUDA)
#define ORIG_HEATER_2_PIN 6
#else
#define ORIG_HEATER_2_PIN -1
#endif
#define ORIG_TEMP_2_PIN -1

#define ORIG_E0_STEP_PIN         34
#define ORIG_E0_DIR_PIN          43
#define ORIG_E0_ENABLE_PIN       26
#define E0_MS1_PIN 65
#define E0_MS2_PIN 66

#define ORIG_E1_STEP_PIN         33
#define ORIG_E1_DIR_PIN          42
#define ORIG_E1_ENABLE_PIN       25
#define E1_MS1_PIN 63
#define E1_MS2_PIN 64

#define DIGIPOTSS_PIN 38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
#define ORIG_FAN_PIN        8
#define ORIG_PS_ON_PIN      4
#define KILL_PIN           -1 // 80 with Smart Controller LCD
#define SUICIDE_PIN        -1 // PIN that has to be turned on right after start, to keep power flowing.

#if ENABLED(ULTRA_LCD)
  #define KILL_PIN 80
  #if ENABLED(NEWPANEL)
   // arduino pin which triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 79      // Beeper on AUX-4
    #define LCD_PINS_RS 70
    #define LCD_PINS_ENABLE 71
    #define LCD_PINS_D4 72
    #define LCD_PINS_D5 73
    #define LCD_PINS_D6 74
    #define LCD_PINS_D7 75

    //buttons are directly attached using AUX-2
    #define BTN_EN1 76
    #define BTN_EN2 77
    #define BTN_ENC 78  //the click

    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0

    #define SD_DETECT_PIN 81    // Ramps does not use this port

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 33    No Beeper added
    //buttons are attached to a shift register
    // Not wired this yet
    // #define SHIFT_CLK 38
    // #define SHIFT_LD 42
    // #define SHIFT_OUT 40
    // #define SHIFT_EN 17

    #define LCD_PINS_RS 75
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    //bits in the shift register that carry the buttons for:
    // left up center down right red
    #define BL_LE 7
    #define BL_UP 6
    #define BL_MI 5
    #define BL_DW 4
    #define BL_RI 3
    #define BL_ST 2
    #define BLEN_B 1
    #define BLEN_A 0
  #endif
#endif //ULTRA_LCD

