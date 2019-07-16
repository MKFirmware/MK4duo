/****************************************************************************************
* 1401
* 
* RADDS
****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_SAM)
  #error "Oops! Select 'Arduino Due' in 'Tools > Board.'"
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Radds"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            24
#define ORIG_X_DIR_PIN             23
#define ORIG_X_ENABLE_PIN          26
#define ORIG_X_CS_PIN              25

//###Y_AXIS
#define ORIG_Y_STEP_PIN            17
#define ORIG_Y_DIR_PIN             16
#define ORIG_Y_ENABLE_PIN          22
#define ORIG_Y_CS_PIN              27

//###Z_AXIS
#define ORIG_Z_STEP_PIN             2
#define ORIG_Z_DIR_PIN              3
#define ORIG_Z_ENABLE_PIN          15
#define ORIG_Z_CS_PIN              29

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           61
#define ORIG_E0_DIR_PIN            60
#define ORIG_E0_ENABLE_PIN         62
#define ORIG_E0_CS_PIN             31
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           64
#define ORIG_E1_DIR_PIN            63
#define ORIG_E1_ENABLE_PIN         65
#define ORIG_E1_CS_PIN             33
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           51
#define ORIG_E2_DIR_PIN            53
#define ORIG_E2_ENABLE_PIN         49
#define ORIG_E2_CS_PIN             35
#define ORIG_SOL2_PIN              NoPin

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN           35
#define ORIG_E3_DIR_PIN            33
#define ORIG_E3_ENABLE_PIN         37
#define ORIG_E3_CS_PIN              6
#define ORIG_SOL3_PIN              NoPin

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN           29
#define ORIG_E4_DIR_PIN            27
#define ORIG_E4_ENABLE_PIN         31
#define ORIG_E4_CS_PIN             39
#define ORIG_SOL4_PIN              NoPin

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN           67
#define ORIG_E5_DIR_PIN            66
#define ORIG_E5_ENABLE_PIN         68
#define ORIG_E5_CS_PIN              6
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
#define ORIG_X_MIN_PIN             28
#define ORIG_X_MAX_PIN             34
#define ORIG_Y_MIN_PIN             30
#define ORIG_Y_MAX_PIN             36
#define ORIG_Z_MIN_PIN             32
#define ORIG_Z_MAX_PIN             38
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
#define ORIG_HEATER_HE0_PIN        13
#define ORIG_HEATER_HE1_PIN        12
#define ORIG_HEATER_HE2_PIN        11
#define ORIG_HEATER_HE3_PIN        NoPin
#define ORIG_HEATER_HE4_PIN        NoPin
#define ORIG_HEATER_HE5_PIN        NoPin
#define ORIG_HEATER_BED0_PIN        7
#define ORIG_HEATER_BED1_PIN       NoPin
#define ORIG_HEATER_BED2_PIN       NoPin
#define ORIG_HEATER_BED3_PIN       NoPin
#define ORIG_HEATER_CHAMBER0_PIN   NoPin
#define ORIG_HEATER_CHAMBER1_PIN   NoPin
#define ORIG_HEATER_CHAMBER2_PIN   NoPin
#define ORIG_HEATER_CHAMBER3_PIN   NoPin
#define ORIG_HEATER_COOLER_PIN     NoPin

//###TEMPERATURE
#define ORIG_TEMP_HE0_PIN           0
#define ORIG_TEMP_HE1_PIN           1
#define ORIG_TEMP_HE2_PIN           2
#define ORIG_TEMP_HE3_PIN           3
#define ORIG_TEMP_HE4_PIN          NoPin
#define ORIG_TEMP_HE5_PIN          NoPin
#define ORIG_TEMP_BED0_PIN          4
#define ORIG_TEMP_BED1_PIN         NoPin
#define ORIG_TEMP_BED2_PIN         NoPin
#define ORIG_TEMP_BED3_PIN         NoPin
#define ORIG_TEMP_CHAMBER0_PIN     NoPin
#define ORIG_TEMP_CHAMBER1_PIN     NoPin
#define ORIG_TEMP_CHAMBER2_PIN     NoPin
#define ORIG_TEMP_CHAMBER3_PIN     NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN               9
#define ORIG_FAN1_PIN               8
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin
#define ORIG_FAN4_PIN              NoPin
#define ORIG_FAN5_PIN              NoPin

//###SERVO
#define SERVO0_PIN                  5
#define SERVO1_PIN                  6
#define SERVO2_PIN                 39
#define SERVO3_PIN                 40

//###SAM_SDSS
#define SDSS                        4

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
#define ORIG_PS_ON_PIN             40
#define ORIG_BEEPER_PIN            NoPin
#define LED_PIN                    NoPin


//###UNKNOWN_PINS
// I2C EEPROM with 8K of space
#define EEPROM_I2C
#define E2END 0x1FFF
#define RADDS_EXT_VDD_PIN          25
#define RADDS_EXT_VDD2_PIN         66
//@@@

//###IF_BLOCKS
#if HAS_SPI_LCD

  #undef ORIG_BEEPER_PIN

  #if ENABLED(RADDS_DISPLAY)

    #define LCD_PINS_RS 		42
    #define LCD_PINS_ENABLE 43
    #define LCD_PINS_D4 		44
    #define LCD_PINS_D5 		45
    #define LCD_PINS_D6 		46
    #define LCD_PINS_D7 		47

    #define ORIG_BEEPER_PIN 41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48

    #define BTN_BACK        71

    #undef SDSS
    #define SDSS            10
    #define SD_DETECT_PIN   14

  #elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)

    // The REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER requires
    // an adapter such as https://www.thingiverse.com/thing:1740725
    #define LCD_PINS_RS     42
    #define LCD_PINS_ENABLE 43
    #define LCD_PINS_D4     44

    #define ORIG_BEEPER_PIN 41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48

  #elif ENABLED(SSD1306_OLED_I2C_CONTROLLER)

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48
    #define ORIG_BEEPER_PIN 41
    #define LCD_SDSS        10
    #define SD_DETECT_PIN   14
    #define KILL_PIN        NoPin

  #elif ENABLED(SPARK_FULL_GRAPHICS)

    #define LCD_PINS_D4     29
    #define LCD_PINS_ENABLE 27
    #define LCD_PINS_RS     25

    #define BTN_EN1         35
    #define BTN_EN2         33
    #define BTN_ENC         37

    #define KILL_PIN        NoPin
    #undef ORIG_BEEPER_PIN
    #define ORIG_BEEPER_PIN NoPin

	#endif // SPARK_FULL_GRAPHICS

#endif // HAS_SPI_LCD
//@@@

//###MB_SETUP
#define MB_SETUP                      \
  OUT_WRITE(RADDS_EXT_VDD_PIN, HIGH); \
  OUT_WRITE(RADDS_EXT_VDD2_PIN, HIGH);
//@@@
