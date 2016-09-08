/****************************************************************************************
* 71
* Ultimaker pin assignment (Old electronics)
****************************************************************************************/

  #define KNOWN_BOARD

  #ifndef __AVR_ATmega1280__
    #ifndef __AVR_ATmega2560__
      #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
    #endif
  #endif

  #define LARGE_FLASH true

  #define ORIG_X_STEP_PIN 25
  #define ORIG_X_DIR_PIN 23
  #define ORIG_X_MIN_PIN 15
  #define ORIG_X_MAX_PIN 14
  #define ORIG_X_ENABLE_PIN 27

  #define ORIG_Y_STEP_PIN 31
  #define ORIG_Y_DIR_PIN 33
  #define ORIG_Y_MIN_PIN 17
  #define ORIG_Y_MAX_PIN 16
  #define ORIG_Y_ENABLE_PIN 29

  #define ORIG_Z_STEP_PIN 37
  #define ORIG_Z_DIR_PIN 39
  #define ORIG_Z_MIN_PIN 19
  #define ORIG_Z_MAX_PIN 18
  #define ORIG_Z_ENABLE_PIN 35

  #define ORIG_HEATER_BED_PIN -1
  #define ORIG_TEMP_BED_PIN -1

  #define ORIG_HEATER_0_PIN  2
  #define ORIG_TEMP_0_PIN 8

  #define ORIG_HEATER_1_PIN 1
  #define ORIG_TEMP_1_PIN 1

  #define ORIG_HEATER_2_PIN -1
  #define ORIG_TEMP_2_PIN -1

  #define ORIG_E0_STEP_PIN         43
  #define ORIG_E0_DIR_PIN          45
  #define ORIG_E0_ENABLE_PIN       41

  #define ORIG_E1_STEP_PIN         -1
  #define ORIG_E1_DIR_PIN          -1
  #define ORIG_E1_ENABLE_PIN       -1

  #define SDPOWER                  -1
  #define SDSS                     -1
  #define LED_PIN                  -1
  #define ORIG_FAN_PIN             -1
  #define ORIG_PS_ON_PIN           -1
  #define KILL_PIN                 -1
  #define SUICIDE_PIN              -1  //PIN that has to be turned on right after start, to keep power flowing.

  #define LCD_PINS_RS 24
  #define LCD_PINS_ENABLE 22
  #define LCD_PINS_D4 36
  #define LCD_PINS_D5 34
  #define LCD_PINS_D6 32
  #define LCD_PINS_D7 30


