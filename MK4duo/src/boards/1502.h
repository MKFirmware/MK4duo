/****************************************************************************************
* 1502
* Alligator R2
*
* The Alligator board requires Arduino Alligator addons installed.
* Add the following URL to Arduino IDE's Additional Board Manager URLs
* http://www.chew-z.it/download/alligator/dist/package_Alligator_index.json
* In the Arduino IDE Board Manager search for Alligator and install the package.
* Change your target board to "Alligator R2 native USB port".
*
* http://www.3dartists.org/
*
****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_SAM)
  #error "Oops! Select 'Alligator 3D Printer Board' in 'Tools > Board.'"
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Alligator R2"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            96
#define ORIG_X_DIR_PIN              2
#define ORIG_X_ENABLE_PIN          24
#define ORIG_X_CS_PIN              NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN            94
#define ORIG_Y_DIR_PIN             95
#define ORIG_Y_ENABLE_PIN          24
#define ORIG_Y_CS_PIN              NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN            98
#define ORIG_Z_DIR_PIN              3
#define ORIG_Z_ENABLE_PIN          24
#define ORIG_Z_CS_PIN              NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN            5
#define ORIG_E0_DIR_PIN             4
#define ORIG_E0_ENABLE_PIN         24
#define ORIG_E0_CS_PIN             NoPin
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           28
#define ORIG_E1_DIR_PIN            27
#define ORIG_E1_ENABLE_PIN         24
#define ORIG_E1_CS_PIN             NoPin
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           11
#define ORIG_E2_DIR_PIN            29
#define ORIG_E2_ENABLE_PIN         24
#define ORIG_E2_CS_PIN             NoPin
#define ORIG_SOL2_PIN              NoPin

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN           30
#define ORIG_E3_DIR_PIN            12
#define ORIG_E3_ENABLE_PIN         24
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
#define ORIG_X_MIN_PIN             33
#define ORIG_X_MAX_PIN             34
#define ORIG_Y_MIN_PIN             35
#define ORIG_Y_MAX_PIN             37
#define ORIG_Z_MIN_PIN             38
#define ORIG_Z_MAX_PIN             39
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
#define ORIG_HEATER_HE0_PIN        69
#define ORIG_HEATER_HE1_PIN         8
#define ORIG_HEATER_HE2_PIN         9
#define ORIG_HEATER_HE3_PIN        97
#define ORIG_HEATER_HE4_PIN        NoPin
#define ORIG_HEATER_HE5_PIN        NoPin
#define ORIG_HEATER_BED0_PIN       68
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
#define ORIG_TEMP_HE3_PIN           4
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
#define ORIG_FAN0_PIN              92
#define ORIG_FAN1_PIN              31
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin
#define ORIG_FAN4_PIN              NoPin
#define ORIG_FAN5_PIN              NoPin

//###SERVO
#define SERVO0_PIN                 36
#define SERVO1_PIN                 40
#define SERVO2_PIN                 41
#define SERVO3_PIN                 NoPin

//###SAM_SDSS
#define SDSS                       77

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


//###UNKNOWN_PINS
#undef NUM_DIGITAL_PINS
#define NUM_DIGITAL_PINS     111
#define SPI_CHAN               0
#define SPI_CHAN_DAC           1
#define X_MS1_PIN             99
#define Y_MS1_PIN             10
#define Z_MS1_PIN             44
#define E0_MS1_PIN            45
#define MOTOR_FAULT_PIN       22
#define EXP_VOLTAGE_LEVEL_PIN 65
#define DAC0_SYNC_PIN         53
#define DAC1_SYNC_PIN          6
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25
#define SPI_EEPROM2_CS        26
#define SPI_FLASH_CS          23
#define EEPROM_SPI
#define E2END                 0x2000
#define HAVE_MCU_TEMPERATURE
//@@@

//###IF_BLOCKS
#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #undef ORIG_BEEPER_PIN

  #define LCD_PINS_RS         18
  #define LCD_PINS_ENABLE     15
  #define LCD_PINS_D4         19
  #define LCD_PINS_D5         50
  #define LCD_PINS_D6         52
  #define LCD_PINS_D7         53
  #define ORIG_BEEPER_PIN     64

  #define BTN_EN1             14
  #define BTN_EN2             16
  #define BTN_ENC             17

  #if LCD_ALLIGATOR_VOLTAGE_LEVEL != 1
    #undef LCD_ALLIGATOR_VOLTAGE_LEVEL
    #define LCD_ALLIGATOR_VOLTAGE_LEVEL  1
  #endif
#endif
//@@@

//###MB_SETUP
#define MB_SETUP                    \
  OUT_WRITE(DAC0_SYNC_PIN, HIGH);   \
  OUT_WRITE(DAC1_SYNC_PIN, HIGH);   \
  OUT_WRITE(SPI_EEPROM1_CS, HIGH);  \
  OUT_WRITE(SPI_EEPROM2_CS, HIGH);  \
  OUT_WRITE(SPI_FLASH_CS, HIGH);    \
  SET_INPUT(MOTOR_FAULT_PIN);       \
  OUT_WRITE(EXP_VOLTAGE_LEVEL_PIN, LCD_ALLIGATOR_VOLTAGE_LEVEL)
//@@@
