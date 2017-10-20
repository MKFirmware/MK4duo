/****************************************************************************************
* 1503
* Alligator R3
* http://www.3dartists.org/
****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_SAM)
  #error Oops!  Make sure you have 'Alligator 3D Printer Board' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Alligator R3"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 96
#define ORIG_X_DIR_PIN 2
#define ORIG_X_ENABLE_PIN 24
#define ORIG_X_CS_PIN NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN 94
#define ORIG_Y_DIR_PIN 95
#define ORIG_Y_ENABLE_PIN 3
#define ORIG_Y_CS_PIN NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN 65
#define ORIG_Z_DIR_PIN 52
#define ORIG_Z_ENABLE_PIN 4
#define ORIG_Z_CS_PIN NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 63
#define ORIG_E0_DIR_PIN 64
#define ORIG_E0_ENABLE_PIN 98
#define ORIG_E0_CS_PIN NoPin
#define ORIG_SOL0_PIN NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN 62
#define ORIG_E1_DIR_PIN 53
#define ORIG_E1_ENABLE_PIN 29
#define ORIG_E1_CS_PIN NoPin
#define ORIG_SOL1_PIN NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN 20
#define ORIG_E2_DIR_PIN 21
#define ORIG_E2_ENABLE_PIN 12
#define ORIG_E2_CS_PIN NoPin
#define ORIG_SOL2_PIN NoPin

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN 66
#define ORIG_E3_DIR_PIN 67
#define ORIG_E3_ENABLE_PIN 30
#define ORIG_E3_CS_PIN NoPin
#define ORIG_SOL3_PIN NoPin

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN NoPin
#define ORIG_E4_DIR_PIN NoPin
#define ORIG_E4_ENABLE_PIN NoPin
#define ORIG_E4_CS_PIN NoPin
#define ORIG_SOL4_PIN NoPin

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN NoPin
#define ORIG_E5_DIR_PIN NoPin
#define ORIG_E5_ENABLE_PIN NoPin
#define ORIG_E5_CS_PIN NoPin
#define ORIG_SOL5_PIN NoPin

//###EXTRUDER_6
#define ORIG_E6_STEP_PIN NoPin
#define ORIG_E6_DIR_PIN NoPin
#define ORIG_E6_ENABLE_PIN NoPin
#define ORIG_E6_CS_PIN NoPin
#define ORIG_SOL6_PIN NoPin

//###EXTRUDER_7
#define ORIG_E7_STEP_PIN NoPin
#define ORIG_E7_DIR_PIN NoPin
#define ORIG_E7_ENABLE_PIN NoPin
#define ORIG_E7_CS_PIN NoPin
#define ORIG_SOL7_PIN NoPin

//###ENDSTOP
#define ORIG_X_MIN_PIN 33
#define ORIG_X_MAX_PIN 34
#define ORIG_Y_MIN_PIN 35
#define ORIG_Y_MAX_PIN 37
#define ORIG_Z_MIN_PIN 38
#define ORIG_Z_MAX_PIN 39
#define ORIG_Z2_MIN_PIN NoPin
#define ORIG_Z2_MAX_PIN NoPin
#define ORIG_Z3_MIN_PIN NoPin
#define ORIG_Z3_MAX_PIN NoPin
#define ORIG_Z4_MIN_PIN NoPin
#define ORIG_Z4_MAX_PIN NoPin
#define ORIG_E_MIN_PIN NoPin
#define ORIG_Z_PROBE_PIN NoPin

//###SINGLE_ENDSTOP
#define X_STOP_PIN NoPin
#define Y_STOP_PIN NoPin
#define Z_STOP_PIN NoPin

//###HEATER
#define ORIG_HEATER_0_PIN 69
#define ORIG_HEATER_1_PIN 8
#define ORIG_HEATER_2_PIN 9
#define ORIG_HEATER_3_PIN 97
#define ORIG_HEATER_BED_PIN 42
#define ORIG_HEATER_CHAMBER_PIN NoPin
#define ORIG_COOLER_PIN NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN 5
#define ORIG_TEMP_1_PIN 4
#define ORIG_TEMP_2_PIN 3
#define ORIG_TEMP_3_PIN 2
#define ORIG_TEMP_BED_PIN 6
#define ORIG_TEMP_CHAMBER_PIN NoPin
#define ORIG_TEMP_COOLER_PIN NoPin

//###FAN
#define ORIG_FAN0_PIN 92
#define ORIG_FAN1_PIN 31
#define ORIG_FAN2_PIN NoPin
#define ORIG_FAN3_PIN NoPin

//###MISC
#define ORIG_PS_ON_PIN NoPin
#define ORIG_BEEPER_PIN NoPin
#define LED_PIN NoPin
#define SDPOWER_PIN NoPin
#define SD_DETECT_PIN 87
#define SDSS 77
#define KILL_PIN NoPin
#define DEBUG_PIN NoPin
#define SUICIDE_PIN NoPin

//###LASER
#define ORIG_LASER_PWR_PIN NoPin
#define ORIG_LASER_PWM_PIN NoPin

//###SERVOS
#if NUM_SERVOS > 0
  #define SERVO0_PIN NoPin
  #if NUM_SERVOS > 1
    #define SERVO1_PIN NoPin
    #if NUM_SERVOS > 2
      #define SERVO2_PIN NoPin
      #if NUM_SERVOS > 3
        #define SERVO3_PIN NoPin
      #endif
    #endif
  #endif
#endif
//@@@

//###UNKNOWN_PINS
#define NUM_DIGITAL_PINS 110
#define SPI_CHAN_DAC 1
#define X_MS1_PIN             99
#define Y_MS1_PIN             10
#define Z_MS1_PIN             44
#define E0_MS1_PIN            45
#define MOTOR_FAULT_PIN       22
#define MOTOR_FAULT_PIGGY_PIN  7
#define DAC0_SYNC             27
#define DAC1_SYNC              6
#define SPI_EEPROM
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25
#define SPI_EEPROM2_CS        26
#define SPI_FLASH_CS          23
#define ESP_WIFI_MODULE_COM    3
#define ESP_WIFI_MODULE_RESET_PIN 43
#define PIGGY_GPIO_PIN        11
#define FTDI_COM_RESET_PIN    32
#define SDA_PIN 	            70
#define SCL_PIN 	            71
#define EXP3_J19_5            73
#define EXP3_J19_9           101
#define EXP3_J19_7           102
#define EXP3_J19_3           103
#define EXP3_J19_14          104
#define EXP3_J19_12          105
#define EXP3_J19_10          106
#define EXP3_J19_8           107
#define EXP3_J19_6           110
#define EXP3_J19_4            13
#define EXP3_J19_2            72
#define EXP3_J19_1            28
//@@@

//###IF_BLOCKS
#define EXP1_OUT_ENABLE_PIN   49
#define EXP1_VOLTAGE_SELECT    5
#define EXP1_J14_4            48
#define EXP1_J14_6            47
#define EXP1_J14_8            46
// USART-0 RX
#define EXP1_J14_5            19
// USART-0 TX
#define EXP1_J14_3            18
#define EXP1_J14_9           108
#define EXP1_J14_10          109

//** Expansion#2 port J5
// USARTNoPin RX
#define EXP2_J5_5             17
// USARTNoPin TX
#define EXP2_J5_1             16
//A0
#define EXP2_J5_3             54
#define EXP1_J5_4             36
#define EXP1_J5_8             40
#define EXP1_J5_6             41
//@@@
