/****************************************************************************************
* 36
* RAMPS 1.3 / 1.4
* RAMPS_13_HHF (Hotend0, Hotend1, Fan)
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega1280__) && DISABLED(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Ramps HHF"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            54
#define ORIG_X_DIR_PIN             55
#define ORIG_X_ENABLE_PIN          38
#define ORIG_X_CS_PIN              53

//###Y_AXIS
#define ORIG_Y_STEP_PIN            60
#define ORIG_Y_DIR_PIN             61
#define ORIG_Y_ENABLE_PIN          56
#define ORIG_Y_CS_PIN              49

//###Z_AXIS
#define ORIG_Z_STEP_PIN            46
#define ORIG_Z_DIR_PIN             48
#define ORIG_Z_ENABLE_PIN          62
#define ORIG_Z_CS_PIN              40

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           26
#define ORIG_E0_DIR_PIN            28
#define ORIG_E0_ENABLE_PIN         24
#define ORIG_E0_CS_PIN             42
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           36
#define ORIG_E1_DIR_PIN            34
#define ORIG_E1_ENABLE_PIN         30
#define ORIG_E1_CS_PIN             44
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           NoPin
#define ORIG_E2_DIR_PIN            NoPin
#define ORIG_E2_ENABLE_PIN         NoPin
#define ORIG_E2_CS_PIN             NoPin
#define ORIG_SOL2_PIN              NoPin

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN           NoPin
#define ORIG_E3_DIR_PIN            NoPin
#define ORIG_E3_ENABLE_PIN         NoPin
#define ORIG_E3_CS_PIN             NoPin
#define ORIG_SOL3_PIN              NoPin

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN           NoPin
#define ORIG_E4_DIR_PIN            NoPin
#define ORIG_E4_ENABLE_PIN         NoPin
#define ORIG_E4_CS_PIN             NoPin
#define ORIG_SOL4_PIN              NoPin

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN           NoPin
#define ORIG_E5_DIR_PIN            NoPin
#define ORIG_E5_ENABLE_PIN         NoPin
#define ORIG_E5_CS_PIN             NoPin
#define ORIG_SOL5_PIN              NoPin

//###EXTRUDER_6
#define ORIG_E6_STEP_PIN           NoPin
#define ORIG_E6_DIR_PIN            NoPin
#define ORIG_E6_ENABLE_PIN         NoPin
#define ORIG_E6_CS_PIN             NoPin
#define ORIG_SOL6_PIN              NoPin

//###EXTRUDER_7
#define ORIG_E7_STEP_PIN           NoPin
#define ORIG_E7_DIR_PIN            NoPin
#define ORIG_E7_ENABLE_PIN         NoPin
#define ORIG_E7_CS_PIN             NoPin
#define ORIG_SOL7_PIN              NoPin

//###ENDSTOP
#define ORIG_X_MIN_PIN              3
#define ORIG_X_MAX_PIN              2
#define ORIG_Y_MIN_PIN             14
#define ORIG_Y_MAX_PIN             15
#define ORIG_Z_MIN_PIN             18
#define ORIG_Z_MAX_PIN             19
#define ORIG_Z2_MIN_PIN            NoPin
#define ORIG_Z2_MAX_PIN            NoPin
#define ORIG_Z3_MIN_PIN            NoPin
#define ORIG_Z3_MAX_PIN            NoPin
#define ORIG_Z4_MIN_PIN            NoPin
#define ORIG_Z4_MAX_PIN            NoPin
#define ORIG_Z_PROBE_PIN           NoPin

//###SINGLE_ENDSTOP
#define X_STOP_PIN                 NoPin
#define Y_STOP_PIN                 NoPin
#define Z_STOP_PIN                 NoPin

//###HEATER
#define ORIG_HEATER_0_PIN          10
#define ORIG_HEATER_1_PIN           9
#define ORIG_HEATER_2_PIN          NoPin
#define ORIG_HEATER_3_PIN          NoPin
#define ORIG_HEATER_BED_PIN         8
#define ORIG_HEATER_CHAMBER_PIN    NoPin
#define ORIG_COOLER_PIN            NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN            13
#define ORIG_TEMP_1_PIN            15
#define ORIG_TEMP_2_PIN            NoPin
#define ORIG_TEMP_3_PIN            NoPin
#define ORIG_TEMP_BED_PIN          14
#define ORIG_TEMP_CHAMBER_PIN      NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN               8
#define ORIG_FAN1_PIN              NoPin
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin

//###SERVO
#define SERVO0_PIN                 11
#define SERVO1_PIN                  6
#define SERVO2_PIN                  5
#define SERVO3_PIN                  4

//###MISC
#define ORIG_PS_ON_PIN             12
#define ORIG_BEEPER_PIN            NoPin
#define LED_PIN                    13
#define SDPOWER_PIN                NoPin
#define SD_DETECT_PIN              NoPin
#define SDSS                       53
#define KILL_PIN                   NoPin
#define DEBUG_PIN                  NoPin
#define SUICIDE_PIN                NoPin

//###LASER
#define ORIG_LASER_PWR_PIN         NoPin
#define ORIG_LASER_PWM_PIN         NoPin


//###UNKNOWN_PINS
#define MAX6675_SS_PIN             66
//@@@

//###IF_BLOCKS
#if ENABLED(ULTRA_LCD)

  //
  // LCD Display output pins
  //
  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    #define LCD_PINS_RS         49
    #define LCD_PINS_ENABLE     51
    #define LCD_PINS_D4         52

  #elif ENABLED(NEWPANEL) && ENABLED(PANEL_ONE)

    #define LCD_PINS_RS         40
    #define LCD_PINS_ENABLE     42
    #define LCD_PINS_D4         65
    #define LCD_PINS_D5         66
    #define LCD_PINS_D6         44
    #define LCD_PINS_D7         64

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      #define LCD_PINS_RS       27
      #define LCD_PINS_ENABLE   29
      #define LCD_PINS_D4       25

      #if DISABLED(NEWPANEL)
        #define ORIG_BEEPER_PIN 37
      #endif

    #elif ENABLED(ZONESTAR_LCD)

      #define LCD_PINS_RS       64
      #define LCD_PINS_ENABLE   44
      #define LCD_PINS_D4       63
      #define LCD_PINS_D5       40
      #define LCD_PINS_D6       42
      #define LCD_PINS_D7       65

    #else

      #if ENABLED(MKS_12864OLED) || ENABLED(MKS_12864OLED_SSD1306)
        #define LCD_PINS_DC     25
        #define LCD_PINS_RS     27
        // DOGM SPI LCD Support
        #define DOGLCD_CS       16
        #define DOGLCD_MOSI     17
        #define DOGLCD_SCK      23
        #define DOGLCD_A0       LCD_PINS_DC
      #else
        #define LCD_PINS_RS     16
        #define LCD_PINS_ENABLE 17
        #define LCD_PINS_D4     23
        #define LCD_PINS_D5     25
        #define LCD_PINS_D6     27
      #endif

      #define LCD_PINS_D7       29

      #if DISABLED(NEWPANEL)
        #define ORIG_BEEPER_PIN 33
      #endif

    #endif

    #if DISABLED(NEWPANEL)
      // Buttons are attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK       38
      //#define SHIFT_LD        42
      //#define SHIFT_OUT       40
      //#define SHIFT_EN        17
    #endif

  #endif

  //
  // LCD Display input pins
  //
  #if ENABLED(NEWPANEL)

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)

      #define ORIG_BEEPER_PIN   37

      #if ENABLED(CR10_STOCKDISPLAY)
        #define BTN_EN1         17
        #define BTN_EN2         23
      #else
        #define BTN_EN1         31
        #define BTN_EN2         33
      #endif

      #define BTN_ENC           35
      #define SD_DETECT_PIN     49
      #define KILL_PIN          41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN 39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1           64
      #define BTN_EN2           59
      #define BTN_ENC           63
      #define SD_DETECT_PIN     42

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1           47
      #define BTN_EN2           43
      #define BTN_ENC           32
      #define LCD_SDSS          53
      #define KILL_PIN          41

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1           22
      #define BTN_EN2            7
      #define BTN_ENC           NoPin

      #define LCD_SDSS          53
      #define SD_DETECT_PIN     49

    #elif ENABLED(VIKI2) || ENABLED(miniVIKI)

      #define DOGLCD_CS         45
      #define DOGLCD_A0         44
      #define LCD_SCREEN_ROT_180

      #define ORIG_BEEPER_PIN   33
      #define STAT_LED_RED_PIN  32
      #define STAT_LED_BLUE_PIN 35

      #define BTN_EN1           22
      #define BTN_EN2            7
      #define BTN_ENC           39

      #define SDSS              53
      #define SD_DETECT_PIN     NoPin
      #define KILL_PIN          31

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      #define DOGLCD_CS         29
      #define DOGLCD_A0         27

      #define ORIG_BEEPER_PIN   23
      #define LCD_BACKLIGHT_PIN 33

      #define BTN_EN1           35
      #define BTN_EN2           37
      #define BTN_ENC           31

      #define LCD_SDSS          53
      #define SD_DETECT_PIN     49
      #define KILL_PIN          41

    #elif ENABLED(MKS_MINI_12864)
      #define DOGLCD_A0         27
      #define DOGLCD_CS         25

      // GLCD features
      //#define LCD_CONTRAST   190
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270

      #define ORIG_BEEPER_PIN   37
      // not connected to a pin
      #define LCD_BACKLIGHT_PIN 65

      #define BTN_EN1           31
      #define BTN_EN2           33
      #define BTN_ENC           35

      #define SDSS              53
      #define SD_DETECT_PIN     49
      #define KILL_PIN          64

    #elif ENABLED(MINIPANEL)

      #define ORIG_BEEPER_PIN   42
      // not connected to a pin
      #define LCD_BACKLIGHT_PIN 65

      #define DOGLCD_A0         44
      #define DOGLCD_CS         66

      // GLCD features
      //#define LCD_CONTRAST   190
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270

      #define BTN_EN1           40
      #define BTN_EN2           63
      #define BTN_ENC           59

      #define SDSS              53
      #define SD_DETECT_PIN     49
      #define KILL_PIN          64

    #else

      // Beeper on AUX-4
      #define ORIG_BEEPER_PIN   33

      // Buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define SHIFT_OUT       40
        #define SHIFT_CLK       44
        #define SHIFT_LD        42
        #define BTN_EN1         64
        #define BTN_EN2         59
        #define BTN_ENC         63
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1         59
        #define BTN_EN2         63
        #define BTN_ENC         49
      #else
        #define BTN_EN1         37
        #define BTN_EN2         35
        #define BTN_ENC         31
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN   49
        #define KILL_PIN        41
      #endif

    #endif
  #endif // NEWPANEL

#endif // ULTRA_LCD
//@@@

