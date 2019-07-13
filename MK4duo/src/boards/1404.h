/****************************************************************************************
* 1404
*
* Ramps - FD v2
****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_SAM)
  #error "Oops! Select 'Arduino Due' in 'Tools > Board.'"
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Ramps FD v2"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            63
#define ORIG_X_DIR_PIN             62
#define ORIG_X_ENABLE_PIN          48
#define ORIG_X_CS_PIN              NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN            65
#define ORIG_Y_DIR_PIN             64
#define ORIG_Y_ENABLE_PIN          46
#define ORIG_Y_CS_PIN              NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN            67
#define ORIG_Z_DIR_PIN             66
#define ORIG_Z_ENABLE_PIN          44
#define ORIG_Z_CS_PIN              NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           36
#define ORIG_E0_DIR_PIN            28
#define ORIG_E0_ENABLE_PIN         42
#define ORIG_E0_CS_PIN             NoPin
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           43
#define ORIG_E1_DIR_PIN            41
#define ORIG_E1_ENABLE_PIN         39
#define ORIG_E1_CS_PIN             NoPin
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           32
#define ORIG_E2_DIR_PIN            47
#define ORIG_E2_ENABLE_PIN         45
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
#define ORIG_X_MIN_PIN             22
#define ORIG_X_MAX_PIN             30
#define ORIG_Y_MIN_PIN             24
#define ORIG_Y_MAX_PIN             38
#define ORIG_Z_MIN_PIN             26
#define ORIG_Z_MAX_PIN             34
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
#define ORIG_HEATER_HE0_PIN         9
#define ORIG_HEATER_HE1_PIN        10
#define ORIG_HEATER_HE2_PIN        11
#define ORIG_HEATER_HE3_PIN        NoPin
#define ORIG_HEATER_HE4_PIN        NoPin
#define ORIG_HEATER_HE5_PIN        NoPin
#define ORIG_HEATER_BED0_PIN        8
#define ORIG_HEATER_BED1_PIN       NoPin
#define ORIG_HEATER_BED2_PIN       NoPin
#define ORIG_HEATER_BED3_PIN       NoPin
#define ORIG_HEATER_CHAMBER0_PIN   NoPin
#define ORIG_HEATER_CHAMBER1_PIN   NoPin
#define ORIG_HEATER_CHAMBER2_PIN   NoPin
#define ORIG_HEATER_CHAMBER3_PIN   NoPin
#define ORIG_HEATER_COOLER_PIN     NoPin

//###TEMPERATURE
#define ORIG_TEMP_HE0_PIN           1
#define ORIG_TEMP_HE1_PIN           2
#define ORIG_TEMP_HE2_PIN           3
#define ORIG_TEMP_HE3_PIN          NoPin
#define ORIG_TEMP_HE4_PIN          NoPin
#define ORIG_TEMP_HE5_PIN          NoPin
#define ORIG_TEMP_BED0_PIN          0
#define ORIG_TEMP_BED1_PIN         NoPin
#define ORIG_TEMP_BED2_PIN         NoPin
#define ORIG_TEMP_BED3_PIN         NoPin
#define ORIG_TEMP_CHAMBER0_PIN     NoPin
#define ORIG_TEMP_CHAMBER1_PIN     NoPin
#define ORIG_TEMP_CHAMBER2_PIN     NoPin
#define ORIG_TEMP_CHAMBER3_PIN     NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN              12
#define ORIG_FAN1_PIN               2
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin
#define ORIG_FAN4_PIN              NoPin
#define ORIG_FAN5_PIN              NoPin

//###SERVO
#define SERVO0_PIN                  7
#define SERVO1_PIN                  6
#define SERVO2_PIN                  5
#define SERVO3_PIN                  3

//###SAM_SDSS
#define SDSS                        4

//###MAX6675
#define MAX6675_SS_PIN             53

//###MAX31855
#define MAX31855_SS0_PIN           NoPin
#define MAX31855_SS1_PIN           NoPin
#define MAX31855_SS2_PIN           NoPin
#define MAX31855_SS3_PIN           NoPin

//###LASER
#define ORIG_LASER_PWR_PIN         NoPin
#define ORIG_LASER_PWM_PIN         NoPin

//###MISC
#define ORIG_PS_ON_PIN             53
#define ORIG_BEEPER_PIN            NoPin
#define LED_PIN                    13


//###UNKNOWN_PINS
#define EEPROM_I2C
#define E2END                      0xFFFF
//@@@

//###IF_BLOCKS
#if HAS_SPI_LCD

  #undef ORIG_BEEPER_PIN

  #if HAS_GRAPHICAL_LCD
      #define ORIG_BEEPER_PIN   37
      #define BTN_EN1           33
      #define BTN_EN2           31
      #define BTN_ENC           35
      #define SD_DETECT_PIN     49
  #endif

  #if ENABLED(MKS_12864OLED) || ENABLED(MKS_12864OLED_SSD1306)
    #define LCD_PINS_DC     25
    #define LCD_PINS_RS     27
    // DOGM SPI LCD Support
    #define DOGLCD_CS       16
    #define DOGLCD_MOSI     17
    #define DOGLCD_SCK      23
    #define DOGLCD_A0       LCD_PINS_DC
  #endif

  #if ENABLED(NEWPANEL)
    // ramps-fd lcd adaptor
    #define LCD_PINS_RS         16
    #define LCD_PINS_ENABLE     17
    #define LCD_PINS_D4         23
    #define LCD_PINS_D5         25
    #define LCD_PINS_D6         27
    #define LCD_PINS_D7         29
  #endif

#endif //HAS_SPI_LCD
//@@@

