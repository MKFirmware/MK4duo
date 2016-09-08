/****************************************************************************************
* 68
* AZTEEG X3 PRO
****************************************************************************************/

#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          3
#define ORIG_X_MAX_PIN          2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define ORIG_E2_STEP_PIN        23
#define ORIG_E2_DIR_PIN         25
#define ORIG_E2_ENABLE_PIN      40

#define ORIG_E3_STEP_PIN        27
#define ORIG_E3_DIR_PIN         29
#define ORIG_E3_ENABLE_PIN      41

#define SDPOWER                 -1
#define SDSS                    53
#define LED_PIN                 13

#define ORIG_FAN_PIN             6
#define ORIG_BEEPER_PIN         33
#define ORIG_PS_ON_PIN          12

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN              41
#else
  #define KILL_PIN              -1
#endif

#define ORIG_HEATER_0_PIN       10   // HOTEND 1
#define ORIG_HEATER_1_PIN        9   // HOTEND 2
#define ORIG_HEATER_2_PIN       16   // HOTEND 3
#define ORIG_HEATER_3_PIN       17   // HOTEND 4

#define ORIG_TEMP_0_PIN         13   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         15   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         12   // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN         11   // ANALOG NUMBERING

#define TC1                      4    // ANALOG NUMBERING Thermo couple on Azteeg X3Pro
#define TC2                      5    // ANALOG NUMBERING Thermo couple on Azteeg X3Pro

#define ORIG_HEATER_BED_PIN      8   // BED

#define ORIG_TEMP_BED_PIN       14   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN            47
  #if NUM_SERVOS > 1
    #define SERVO1_PIN          -1
    #if NUM_SERVOS > 2
      #define SERVO2_PIN        -1
      #if NUM_SERVOS > 3
        #define SERVO3_PIN      -1
      #endif
    #endif
  #endif
#endif

#if ENABLED(TEMP_STAT_LEDS)
  #define STAT_LED_RED      32
  #define STAT_LED_BLUE     35
#endif

#if ENABLED(ULTRA_LCD)

  #if ENABLED(NEWPANEL)
    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define ORIG_BEEPER_PIN 37

      #define BTN_EN1 31
      #define BTN_EN2 33
      #define BTN_ENC 35

      #define SD_DETECT_PIN 49
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1 47  //reverse if the encoder turns the wrong way.
      #define BTN_EN2 43
      #define BTN_ENC 32
      #define LCD_SDSS 53
      #define SD_DETECT_PIN -1
      #define KILL_PIN 41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1 22  //reverse if the encoder turns the wrong way.
      #define BTN_EN2 7
      #define BTN_ENC -1
      #define LCD_SDSS 53
      #define SD_DETECT_PIN 49
    #else
      //arduino pin which triggers an piezzo beeper
      #define ORIG_BEEPER_PIN 33  // Beeper on AUX-4

      //buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1 64 // encoder
        #define BTN_EN2 59 // encoder
        #define BTN_ENC 63 // enter button
        #define SHIFT_OUT 40 // shift register
        #define SHIFT_CLK 44 // shift register
        #define SHIFT_LD 42 // shift register
      #else
        #define BTN_EN1 37
        #define BTN_EN2 35
        #define BTN_ENC 31  //the click
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN 49
      #else
        #define SD_DETECT_PIN -1  // Ramps does not use this port
      #endif

    #endif

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 33   //No Beeper added

    //buttons are attached to a shift register
    // Not wired this yet
    //#define SHIFT_CLK 38
    //#define SHIFT_LD 42
    //#define SHIFT_OUT 40
    //#define SHIFT_EN 17

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29
  #endif

#endif //ULTRA_LCD

#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #define ORIG_BEEPER_PIN  33
 // Pins for DOGM SPI LCD Support
  #define DOGLCD_A0        44
  #define DOGLCD_CS        45
  #define LCD_SCREEN_ROT_180

 //The encoder and click button
  #define BTN_EN1           22
  #define BTN_EN2            7
  #define BTN_ENC           39  // the click switch

  #define SDSS              53
  #define SD_DETECT_PIN     49

  #define KILL_PIN          31
#endif

#define MAX6675_SS          66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present


