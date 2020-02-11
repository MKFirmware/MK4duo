/****************************************************************************************
* 4203
*
* RUMBA32 Makerbase
*****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_STM32)
  #error "Oops! Select 'RUMBA32' in 'Tools > Board.'"
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "RUMBA32 MKS"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            PA0
#define ORIG_X_DIR_PIN             PC15
#define ORIG_X_ENABLE_PIN          PC11
#define ORIG_X_CS_PIN              PC14

//###Y_AXIS
#define ORIG_Y_STEP_PIN            PE5
#define ORIG_Y_DIR_PIN             PE6
#define ORIG_Y_ENABLE_PIN          PE3
#define ORIG_Y_CS_PIN              PE4

//###Z_AXIS
#define ORIG_Z_STEP_PIN            PE1
#define ORIG_Z_DIR_PIN             PE2
#define ORIG_Z_ENABLE_PIN          PB7
#define ORIG_Z_CS_PIN              PE0

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           PB5
#define ORIG_E0_DIR_PIN            PB6
#define ORIG_E0_ENABLE_PIN         PC12
#define ORIG_E0_CS_PIN             PC13
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           PD6
#define ORIG_E1_DIR_PIN            PD7
#define ORIG_E1_ENABLE_PIN         PD4
#define ORIG_E1_CS_PIN             PD5
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           PD2
#define ORIG_E2_DIR_PIN            PD3
#define ORIG_E2_ENABLE_PIN         PD0
#define ORIG_E2_CS_PIN             PD1
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
#define ORIG_X_MIN_PIN             PB12
#define ORIG_X_MAX_PIN             PB13
#define ORIG_Y_MIN_PIN             PB15
#define ORIG_Y_MAX_PIN             PD8
#define ORIG_Z_MIN_PIN             PD9
#define ORIG_Z_MAX_PIN             PD10
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
#define ORIG_HEATER_HE0_PIN        PC6
#define ORIG_HEATER_HE1_PIN        PC7
#define ORIG_HEATER_HE2_PIN        PC8
#define ORIG_HEATER_HE3_PIN        NoPin
#define ORIG_HEATER_HE4_PIN        NoPin
#define ORIG_HEATER_HE5_PIN        NoPin
#define ORIG_HEATER_BED0_PIN       PA1
#define ORIG_HEATER_BED1_PIN       NoPin
#define ORIG_HEATER_BED2_PIN       NoPin
#define ORIG_HEATER_BED3_PIN       NoPin
#define ORIG_HEATER_CHAMBER0_PIN   NoPin
#define ORIG_HEATER_CHAMBER1_PIN   NoPin
#define ORIG_HEATER_CHAMBER2_PIN   NoPin
#define ORIG_HEATER_CHAMBER3_PIN   NoPin
#define ORIG_HEATER_COOLER_PIN     NoPin

//###TEMPERATURE
#define ORIG_TEMP_HE0_PIN          PC4
#define ORIG_TEMP_HE1_PIN          PC3
#define ORIG_TEMP_HE2_PIN          PC2
#define ORIG_TEMP_HE3_PIN          PC1
#define ORIG_TEMP_HE4_PIN          NoPin
#define ORIG_TEMP_HE5_PIN          NoPin
#define ORIG_TEMP_BED0_PIN         PC0
#define ORIG_TEMP_BED1_PIN         NoPin
#define ORIG_TEMP_BED2_PIN         NoPin
#define ORIG_TEMP_BED3_PIN         NoPin
#define ORIG_TEMP_CHAMBER0_PIN     NoPin
#define ORIG_TEMP_CHAMBER1_PIN     NoPin
#define ORIG_TEMP_CHAMBER2_PIN     NoPin
#define ORIG_TEMP_CHAMBER3_PIN     NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN              PC9
#define ORIG_FAN1_PIN              PA8
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin
#define ORIG_FAN4_PIN              NoPin
#define ORIG_FAN5_PIN              NoPin

//###SERVO
#define SERVO0_PIN                 PD15
#define SERVO1_PIN                 PD14
#define SERVO2_PIN                 NoPin
#define SERVO3_PIN                 NoPin

//###SDSS
#define SDSS                       PA2

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
#define STEP_TIMER                 TIM2
#define SERVO_TIMER                TIM12
#define LED_PIN                    PB14
#define SD_DETECT_PIN              PB0
#define ORIG_PS_ON_PIN             PE11
#define ORIG_BEEPER_PIN            PE8

//###UNKNOWN_PINS
#define EEPROM_FLASH
//#define HAVE_MCU_TEMPERATURE
//#define HAVE_VREF_MONITOR

//###IF_BLOCKS
#define ST7920_DELAY_1             DELAY_NS(350)
#define ST7920_DELAY_2             DELAY_NS(100)
#define ST7920_DELAY_3             DELAY_NS(350)
#define LCD_PINS_RS                PE10
#define LCD_PINS_ENABLE            PE9
#define LCD_PINS_D4                PE12
#define LCD_PINS_D5                PE13
#define LCD_PINS_D6                PE14
#define LCD_PINS_D7                PE15
#define BTN_EN1                    PB1
#define BTN_EN2                    PB2
#define BTN_ENC                    PE7
//@@@
