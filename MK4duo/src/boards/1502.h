/****************************************************************************************
* 1502
* Alligator R2
* http://www.3dartists.org/
****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_SAM) //AlligatorBoard
  #error Oops!  Make sure you have 'Alligator 3D Printer Board' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Alligator R2"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 96
#define ORIG_X_DIR_PIN 2
#define ORIG_X_ENABLE_PIN 24
#define ORIG_X_CS_PIN -1

//###Y_AXIS
#define ORIG_Y_STEP_PIN 94
#define ORIG_Y_DIR_PIN 95
#define ORIG_Y_ENABLE_PIN 24
#define ORIG_Y_CS_PIN -1

//###Z_AXIS
#define ORIG_Z_STEP_PIN 98
#define ORIG_Z_DIR_PIN 3
#define ORIG_Z_ENABLE_PIN 24
#define ORIG_Z_CS_PIN -1

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 5
#define ORIG_E0_DIR_PIN 4
#define ORIG_E0_ENABLE_PIN 24
#define ORIG_E0_CS_PIN -1
#define ORIG_SOL0_PIN -1

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN 28
#define ORIG_E1_DIR_PIN 27
#define ORIG_E1_ENABLE_PIN 24
#define ORIG_E1_CS_PIN -1
#define ORIG_SOL1_PIN -1

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN 11
#define ORIG_E2_DIR_PIN 29
#define ORIG_E2_ENABLE_PIN 24
#define ORIG_E2_CS_PIN -1
#define ORIG_SOL2_PIN -1

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN 30
#define ORIG_E3_DIR_PIN 12
#define ORIG_E3_ENABLE_PIN 24
#define ORIG_E3_CS_PIN -1
#define ORIG_SOL3_PIN -1

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN -1
#define ORIG_E4_DIR_PIN -1
#define ORIG_E4_ENABLE_PIN -1
#define ORIG_E4_CS_PIN -1
#define ORIG_SOL4_PIN -1

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN -1
#define ORIG_E5_DIR_PIN -1
#define ORIG_E5_ENABLE_PIN -1
#define ORIG_E5_CS_PIN -1
#define ORIG_SOL5_PIN -1

//###EXTRUDER_6
#define ORIG_E6_STEP_PIN -1
#define ORIG_E6_DIR_PIN -1
#define ORIG_E6_ENABLE_PIN -1
#define ORIG_E6_CS_PIN -1
#define ORIG_SOL6_PIN -1

//###EXTRUDER_7
#define ORIG_E7_STEP_PIN -1
#define ORIG_E7_DIR_PIN -1
#define ORIG_E7_ENABLE_PIN -1
#define ORIG_E7_CS_PIN -1
#define ORIG_SOL7_PIN -1

//###ENDSTOP
#define ORIG_X_MIN_PIN 33
#define ORIG_X_MAX_PIN 34
#define ORIG_Y_MIN_PIN 35
#define ORIG_Y_MAX_PIN 37
#define ORIG_Z_MIN_PIN 38
#define ORIG_Z_MAX_PIN 39
#define ORIG_Z2_MIN_PIN -1
#define ORIG_Z2_MAX_PIN -1
#define ORIG_Z3_MIN_PIN -1
#define ORIG_Z3_MAX_PIN -1
#define ORIG_Z4_MIN_PIN -1
#define ORIG_Z4_MAX_PIN -1
#define ORIG_E_MIN_PIN -1
#define ORIG_Z_PROBE_PIN -1

//###SINGLE_ENDSTOP
#define X_STOP_PIN -1
#define Y_STOP_PIN -1
#define Z_STOP_PIN -1

//###HEATER
#define ORIG_HEATER_0_PIN 69
#define ORIG_HEATER_1_PIN 8
#define ORIG_HEATER_2_PIN 9
#define ORIG_HEATER_3_PIN 97
#define ORIG_HEATER_BED_PIN 68
#define ORIG_HEATER_CHAMBER_PIN -1
#define ORIG_COOLER_PIN -1

//###TEMPERATURE
#define ORIG_TEMP_0_PIN 1
#define ORIG_TEMP_1_PIN 2
#define ORIG_TEMP_2_PIN 3
#define ORIG_TEMP_3_PIN 4
#define ORIG_TEMP_BED_PIN 0
#define ORIG_TEMP_CHAMBER_PIN -1
#define ORIG_TEMP_COOLER_PIN -1

//###FAN
#define ORIG_FAN_PIN 92
#define ORIG_FAN1_PIN 31
#define ORIG_FAN2_PIN -1
#define ORIG_FAN3_PIN -1

//###MISC
#define ORIG_PS_ON_PIN -1
#define ORIG_BEEPER_PIN -1
#define LED_PIN -1
#define SDPOWER -1
#define SD_DETECT_PIN 87
#define SDSS 77
#define KILL_PIN -1
#define DEBUG_PIN -1
#define SUICIDE_PIN -1

//###LASER
#define ORIG_LASER_PWR_PIN -1
#define ORIG_LASER_PWM_PIN -1

//###SERVOS
#if NUM_SERVOS > 0
  #define SERVO0_PIN 36
  #if NUM_SERVOS > 1
    #define SERVO1_PIN 40
    #if NUM_SERVOS > 2
      #define SERVO2_PIN 41
      #if NUM_SERVOS > 3
        #define SERVO3_PIN -1
      #endif
    #endif
  #endif
#endif
//@@@

//###UNKNOWN_PINS
#undef NUM_DIGITAL_PINS
#define NUM_DIGITAL_PINS 100

#define SPI_CHAN_DAC 1
#define X_MS1_PIN             99  // PC10
#define Y_MS1_PIN             10  // PC29
#define Z_MS1_PIN             44  // PC19
#define E0_MS1_PIN            45  // PC18
#define MOTOR_FAULT_PIN       22 // PB26 , motor X-Y-Z-E0 motor FAULT
#define EXP_VOLTAGE_LEVEL_PIN 65
#define DAC0_SYNC             53 // PB14
#define DAC1_SYNC              6 // PC24
#define SPI_EEPROM
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25 // PD0
#define SPI_EEPROM2_CS        26 // PD1
#define SPI_FLASH_CS          23 //PA14
//@@@

