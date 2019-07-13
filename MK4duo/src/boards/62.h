/****************************************************************************************
* 62 - Sanguinololu 1.2 and above
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega644__) && DISABLED(__AVR_ATmega644P__) && DISABLED(__AVR_ATmega1284P__)
  #error "Oops! Select 'Sanguino' in 'Tools > Board.'"
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Sanguinololu v1.2"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            15
#define ORIG_X_DIR_PIN             21
#define ORIG_X_ENABLE_PIN          14
#define ORIG_X_CS_PIN              NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN            22
#define ORIG_Y_DIR_PIN             23
#define ORIG_Y_ENABLE_PIN          14
#define ORIG_Y_CS_PIN              NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN             3
#define ORIG_Z_DIR_PIN              2
#define ORIG_Z_ENABLE_PIN          26
#define ORIG_Z_CS_PIN              NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN            1
#define ORIG_E0_DIR_PIN             0
#define ORIG_E0_ENABLE_PIN         14
#define ORIG_E0_CS_PIN             NoPin
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           NoPin
#define ORIG_E1_DIR_PIN            NoPin
#define ORIG_E1_ENABLE_PIN         NoPin
#define ORIG_E1_CS_PIN             NoPin
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
#define ORIG_X_MIN_PIN             NoPin
#define ORIG_X_MAX_PIN             NoPin
#define ORIG_Y_MIN_PIN             NoPin
#define ORIG_Y_MAX_PIN             NoPin
#define ORIG_Z_MIN_PIN             NoPin
#define ORIG_Z_MAX_PIN             NoPin
#define ORIG_Z2_MIN_PIN            NoPin
#define ORIG_Z2_MAX_PIN            NoPin
#define ORIG_Z3_MIN_PIN            NoPin
#define ORIG_Z3_MAX_PIN            NoPin
#define ORIG_Z4_MIN_PIN            NoPin
#define ORIG_Z4_MAX_PIN            NoPin
#define ORIG_Z_PROBE_PIN           NoPin

//###SINGLE_ENDSTOP
#define X_STOP_PIN                 18
#define Y_STOP_PIN                 19
#define Z_STOP_PIN                 20

//###HEATER
#define ORIG_HEATER_HE0_PIN        13
#define ORIG_HEATER_HE1_PIN        NoPin
#define ORIG_HEATER_HE2_PIN        NoPin
#define ORIG_HEATER_HE3_PIN        NoPin
#define ORIG_HEATER_HE4_PIN        NoPin
#define ORIG_HEATER_HE5_PIN        NoPin
#define ORIG_HEATER_BED0_PIN       12
#define ORIG_HEATER_BED1_PIN       NoPin
#define ORIG_HEATER_BED2_PIN       NoPin
#define ORIG_HEATER_BED3_PIN       NoPin
#define ORIG_HEATER_CHAMBER0_PIN   NoPin
#define ORIG_HEATER_CHAMBER1_PIN   NoPin
#define ORIG_HEATER_CHAMBER2_PIN   NoPin
#define ORIG_HEATER_CHAMBER3_PIN   NoPin
#define ORIG_HEATER_COOLER_PIN     NoPin

//###TEMPERATURE
#define ORIG_TEMP_HE0_PIN           7
#define ORIG_TEMP_HE1_PIN          NoPin
#define ORIG_TEMP_HE2_PIN          NoPin
#define ORIG_TEMP_HE3_PIN          NoPin
#define ORIG_TEMP_HE4_PIN          NoPin
#define ORIG_TEMP_HE5_PIN          NoPin
#define ORIG_TEMP_BED0_PIN          6
#define ORIG_TEMP_BED1_PIN         NoPin
#define ORIG_TEMP_BED2_PIN         NoPin
#define ORIG_TEMP_BED3_PIN         NoPin
#define ORIG_TEMP_CHAMBER0_PIN     NoPin
#define ORIG_TEMP_CHAMBER1_PIN     NoPin
#define ORIG_TEMP_CHAMBER2_PIN     NoPin
#define ORIG_TEMP_CHAMBER3_PIN     NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN              NoPin
#define ORIG_FAN1_PIN              NoPin
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin
#define ORIG_FAN4_PIN              NoPin
#define ORIG_FAN5_PIN              NoPin

//###SERVO
#define SERVO0_PIN                 NoPin
#define SERVO1_PIN                 NoPin
#define SERVO2_PIN                 NoPin
#define SERVO3_PIN                 NoPin

//###SAM_SDSS
#define SDSS                       NoPin

//###MAX6675
#define MAX6675_SS_PIN             NoPin

//###MAX31855
#define MAX31855_SS0_PIN           NoPin
#define MAX31855_SS1_PIN           NoPin
#define MAX31855_SS2_PIN           NoPin
#define MAX31855_SS3_PIN           NoPin

//###LASER
#define ORIG_LASER_PWR_PIN         NoPin
#define ORIG_LASER_PWM_PIN         NoPin

//###MISC
#define ORIG_PS_ON_PIN             NoPin
#define ORIG_BEEPER_PIN            NoPin
#define LED_PIN                    NoPin



//###IF_BLOCKS
#if ENABLED(LCD_I2C_PANELOLU2)
  #define ORIG_FAN0_PIN         4 // Uses Transistor1 (PWM) on Panelolu2's Sanguino Adapter Board to drive the fan
#endif

#if HAS_SPI_LCD && ENABLED(NEWPANEL)

  // No buzzer installed
  #undef ORIG_BEEPER_PIN
  #define ORIG_BEEPER_PIN NoPin

  //LCD Pins
  #if HAS_GRAPHICAL_LCD

    #if ENABLED(U8GLIB_ST7920)
      #define LCD_PINS_RS      4
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4     30
      #define LCD_PINS_D5     29
      #define LCD_PINS_D6     28
      #define LCD_PINS_D7     27
    #else

      #define DOGLCD_A0         30
      #define DOGLCD_CS         29
      #define LCD_CONTRAST       1
    #endif

    // Uncomment screen orientation
    #define LCD_SCREEN_ROT_0
    //#define LCD_SCREEN_ROT_90
    //#define LCD_SCREEN_ROT_180
    //#define LCD_SCREEN_ROT_270

  #else // !DOGLCD - Standard Hitachi LCD controller
    #define LCD_PINS_RS          4
    #define LCD_PINS_ENABLE     17
    #define LCD_PINS_D4         30
    #define LCD_PINS_D5         29
    #define LCD_PINS_D6         28
    #define LCD_PINS_D7         27
  #endif // !DOGLCD

  //The encoder and click button
  #define BTN_EN1               11
  #define BTN_EN2               10
  #if ENABLED(LCD_I2C_PANELOLU2)
    #define BTN_ENC           30
  #else
    #define BTN_ENC             16
    #define LCD_SDSS            28
  #endif

  #define SD_DETECT_PIN         NoPin

#elif ENABLED(MAKRPANEL)
  #define ORIG_BEEPER_PIN       29

  // Pins for DOGM SPI LCD Support
  #define DOGLCD_A0             30
  #define DOGLCD_CS             17
  #define LCD_BACKLIGHT_PIN     28
  // GLCD features
  #define LCD_CONTRAST           1
  // Uncomment screen orientation
  #define LCD_SCREEN_ROT_0
  //#define LCD_SCREEN_ROT_90
  //#define LCD_SCREEN_ROT_180
  //#define LCD_SCREEN_ROT_270
  //The encoder and click button
  #define BTN_EN1               11
  #define BTN_EN2               10
  #define BTN_ENC               16

  #define SD_DETECT_PIN         NoPin

#endif // MAKRPANEL
//@@@

