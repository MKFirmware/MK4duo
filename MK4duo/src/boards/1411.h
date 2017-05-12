/****************************************************************************************
* 1411
* Arduino pin assignment
* for BOARD_RAMPS_SMART_HFB
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "SMART RAMPS"

#ifndef ARDUINO_ARCH_SAM
  #error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN       54
#define ORIG_X_DIR_PIN        55
#define ORIG_X_ENABLE_PIN     38
#define ORIG_X_MIN_PIN         3
#define ORIG_X_MAX_PIN         2

#define ORIG_Y_STEP_PIN       60
#define ORIG_Y_DIR_PIN        61
#define ORIG_Y_ENABLE_PIN     56
#define ORIG_Y_MIN_PIN        14
#define ORIG_Y_MAX_PIN        15

#define ORIG_Z_STEP_PIN       46
#define ORIG_Z_DIR_PIN        48
#define ORIG_Z_ENABLE_PIN     62
#define ORIG_Z_MIN_PIN        18
#define ORIG_Z_MAX_PIN        19

#define ORIG_HEATER_0_PIN     10
#define ORIG_HEATER_1_PIN     -1
#define ORIG_HEATER_BED_PIN    8

#define ORIG_FAN_PIN           9
#define ORIG_FAN1_PIN         -1

#define ORIG_TEMP_0_PIN        9  // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN       10  // ANALOG NUMBERING
#define ORIG_TEMP_BED_PIN     11  // ANALOG NUMBERING

#define ORIG_E0_STEP_PIN      26
#define ORIG_E0_DIR_PIN       28
#define ORIG_E0_ENABLE_PIN    24

#define ORIG_E1_STEP_PIN      36
#define ORIG_E1_DIR_PIN       34
#define ORIG_E1_ENABLE_PIN    30

#define SDPOWER               -1
#define SDSS                  53  // 10 if using HW SPI. 53 if using SW SPI
#define LED_PIN               13
#define ORIG_PS_ON_PIN        12
#define KILL_PIN              -1
#define SUICIDE_PIN           -1  // PIN that has to be turned on right after start, to keep power flowing

#define SERVO0_PIN            11
#define SERVO1_PIN             6
#define SERVO2_PIN             5
#define SERVO3_PIN             4

// SPI for Max6675 or Max31855 Thermocouple
#define MAX6675_SS            66 // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

//
// LCD / Controller
//
#if ENABLED(ULTRA_LCD)

  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    #define LCD_PINS_RS     49 // CS chip select /SS chip slave select
    #define LCD_PINS_ENABLE 51 // SID (MOSI)
    #define LCD_PINS_D4     52 // SCK (CLK) clock
  #elif ENABLED(NEWPANEL) && ENABLED(PANEL_ONE)
    #define LCD_PINS_RS 40
    #define LCD_PINS_ENABLE 42
    #define LCD_PINS_D4 65
    #define LCD_PINS_D5 66
    #define LCD_PINS_D6 44
    #define LCD_PINS_D7 64
  #else
    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29
    #if DISABLED(NEWPANEL)
      #define BEEPER_PIN 33
      // Buttons are attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK 38
      //#define SHIFT_LD 42
      //#define SHIFT_OUT 40
      //#define SHIFT_EN 17
    #endif
  #endif

  #if ENABLED(NEWPANEL)

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define BEEPER_PIN 37

      #define BTN_EN1 31
      #define BTN_EN2 33
      #define BTN_ENC 35

      #define SD_DETECT_PIN 49
      #define KILL_PIN 41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN 39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
      #define BTN_EN1 64
      #define BTN_EN2 59
      #define BTN_ENC 63
      #define SD_DETECT_PIN 42
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1 47  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 43
      #define BTN_ENC 32
      #define LCD_SDSS 53
      #define SD_DETECT_PIN -1
      #define KILL_PIN 41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1 22  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 7   // http://files.panucatt.com/datasheets/viki_wiring_diagram.pdf
                          // tells about 40/42.
                          // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.
      #define BTN_ENC -1
      #define LCD_SDSS 53
      #define SD_DETECT_PIN 49
    #elif ENABLED(VIKI2) || ENABLED(miniVIKI)
      #define BEEPER_PIN       33

      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0        44
      #define DOGLCD_CS        45
      #define LCD_SCREEN_ROT_180

      #define BTN_EN1          22
      #define BTN_EN2           7
      #define BTN_ENC          39

      #define SDSS             53
      #define SD_DETECT_PIN    -1  // Pin 49 for display sd interface, 72 for easy adapter board

      #define KILL_PIN         31

      #define STAT_LED_RED_PIN 32
      #define STAT_LED_BLUE_PIN 35

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1 35  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 37
      #define BTN_ENC 31
      #define SD_DETECT_PIN 49
      #define LCD_SDSS 53
      #define KILL_PIN 41
      #define BEEPER_PIN 23
      #define DOGLCD_CS 29
      #define DOGLCD_A0 27
      #define LCD_BACKLIGHT_PIN 33
    #elif ENABLED(MINIPANEL)
      #define BEEPER_PIN 42
      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0  44
      #define DOGLCD_CS  66
      #define LCD_BACKLIGHT_PIN 65 // backlight LED on A11/D65
      #define SDSS   53

      #define KILL_PIN 64
      // GLCD features
      //#define LCD_CONTRAST 190
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270
      // The encoder and click button
      #define BTN_EN1 40
      #define BTN_EN2 63
      #define BTN_ENC 59
      // not connected to a pin
      #define SD_DETECT_PIN 49

    #else

      // Beeper on AUX-4
      #define BEEPER_PIN 33

      // buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1 64 // encoder
        #define BTN_EN2 59 // encoder
        #define BTN_ENC 63 // enter button
        #define SHIFT_OUT 40 // shift register
        #define SHIFT_CLK 44 // shift register
        #define SHIFT_LD 42 // shift register
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1 59 // AUX2 PIN 3
        #define BTN_EN2 63 // AUX2 PIN 4
        #define BTN_ENC 49 // AUX3 PIN 7
      #else
        #define BTN_EN1 37
        #define BTN_EN2 35
        #define BTN_ENC 31 // the click
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN 49
        #define KILL_PIN 41
      #else
        //#define SD_DETECT_PIN -1 // Ramps doesn't use this
      #endif

    #endif
  #endif // NEWPANEL

#endif // ULTRA_LCD