/****************************************************************************************
* 1400
*
* PICCOLO_3D
****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_SAM)
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Piccolo 3D"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 35
#define ORIG_X_DIR_PIN 36
#define ORIG_X_ENABLE_PIN 37
#define ORIG_X_CS_PIN -1

//###Y_AXIS
#define ORIG_Y_STEP_PIN 15
#define ORIG_Y_DIR_PIN 22
#define ORIG_Y_ENABLE_PIN 37
#define ORIG_Y_CS_PIN -1

//###Z_AXIS
#define ORIG_Z_STEP_PIN 26
#define ORIG_Z_DIR_PIN 28
#define ORIG_Z_ENABLE_PIN 53
#define ORIG_Z_CS_PIN -1

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 39
#define ORIG_E0_DIR_PIN 42
#define ORIG_E0_ENABLE_PIN 63
#define ORIG_E0_CS_PIN -1
#define ORIG_SOL0_PIN -1

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN 39
#define ORIG_E1_DIR_PIN 42
#define ORIG_E1_ENABLE_PIN 64
#define ORIG_E1_CS_PIN -1
#define ORIG_SOL1_PIN -1

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN 39
#define ORIG_E2_DIR_PIN 42
#define ORIG_E2_ENABLE_PIN 65
#define ORIG_E2_CS_PIN -1
#define ORIG_SOL2_PIN -1

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN 39
#define ORIG_E3_DIR_PIN 42
#define ORIG_E3_ENABLE_PIN 38
#define ORIG_E3_CS_PIN -1
#define ORIG_SOL3_PIN -1

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN 39
#define ORIG_E4_DIR_PIN 42
#define ORIG_E4_ENABLE_PIN 62
#define ORIG_E4_CS_PIN -1
#define ORIG_SOL4_PIN -1

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN 39
#define ORIG_E5_DIR_PIN 42
#define ORIG_E5_ENABLE_PIN 61
#define ORIG_E5_CS_PIN -1
#define ORIG_SOL5_PIN -1

//###EXTRUDER_6
#define ORIG_E6_STEP_PIN 39
#define ORIG_E6_DIR_PIN 42
#define ORIG_E6_ENABLE_PIN 32
#define ORIG_E6_CS_PIN -1
#define ORIG_SOL6_PIN -1

//###EXTRUDER_7
#define ORIG_E7_STEP_PIN 39
#define ORIG_E7_DIR_PIN 42
#define ORIG_E7_ENABLE_PIN 34
#define ORIG_E7_CS_PIN -1
#define ORIG_SOL7_PIN -1

//###ENDSTOP
#define ORIG_X_MIN_PIN 23
#define ORIG_X_MAX_PIN 25
#define ORIG_Y_MIN_PIN 27
#define ORIG_Y_MAX_PIN 29
#define ORIG_Z_MIN_PIN 31
#define ORIG_Z_MAX_PIN 33
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
#define ORIG_HEATER_0_PIN 12
#define ORIG_HEATER_1_PIN 11
#define ORIG_HEATER_2_PIN 9
#define ORIG_HEATER_3_PIN 8
#define ORIG_HEATER_BED_PIN 13
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
#define ORIG_FAN0_PIN 8
#define ORIG_FAN1_PIN 7
#define ORIG_FAN2_PIN -1
#define ORIG_FAN3_PIN -1

//###MISC
#define ORIG_PS_ON_PIN 24
#define ORIG_BEEPER_PIN 44
#define LED_PIN -1
#define SDPOWER -1
#define SD_DETECT_PIN -1
#define SDSS 4
#define KILL_PIN -1
#define DEBUG_PIN -1
#define SUICIDE_PIN -1

//###LASER
#define ORIG_LASER_PWR_PIN -1
#define ORIG_LASER_PWM_PIN -1

//###SERVOS
#if NUM_SERVOS > 0
  #define SERVO0_PIN 2
  #if NUM_SERVOS > 1
    #define SERVO1_PIN 3
    #if NUM_SERVOS > 2
      #define SERVO2_PIN 5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN 6
      #endif
    #endif
  #endif
#endif
//@@@

//###UNKNOWN_PINS
#define S0_MUX            41
#define S1_MUX            40
#define S2_MUX            50
#define MOTOR_FAULT_PIN   10
#define I2C_EEPROM
//@@@

//###IF_BLOCKS
//OE S2 S1 S0
//L  L  L  L 	A port = B1 port EXTRUDER8
//L  L  L  H 	A port = B2 port EXTRUDER7
//L  L  H  L 	A port = B3 port EXTRUDER6
//L  L  H  H 	A port = B4 port EXTRUDER5
//L  H  L  L 	A port = B5 port EXTRUDER4
//L  H  L  H 	A port = B6 port EXTRUDER3
//L  H  H  L 	A port = B7 port EXTRUDER2
//L  H  H  H 	A port = B8 port EXTRUDER1
//H  X  X  X 	Disconnect
//#define EXP_VOLTAGE_LEVEL_PIN    2 //(ADC1 pin A5)
//#define DAC0_SYNC               53 // PB14
//#define DAC1_SYNC                6 // PC24

#if ENABLED(ULTRA_LCD)
  // LCD panel
  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define LCD_PINS_RS         46
    #define LCD_PINS_ENABLE     45
    #define LCD_PINS_D4         47
    #define LCD_PINS_D5         48
    #define LCD_PINS_D6         49
    #define LCD_PINS_D7         43

    #define BEEPER              44

    #define BTN_EN1             52
    #define BTN_EN2             51
    #define BTN_ENC             30

    #define SD_DETECT_PIN       14
   #endif 
#endif // ULTRA_LCD
//@@@
