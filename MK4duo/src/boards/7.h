/****************************************************************************************
* 7
* Ultimaker pin assignment
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega1280__) && DISABLED(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Ultimaker"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 25
#define ORIG_X_DIR_PIN 23
#define ORIG_X_ENABLE_PIN 27
#define ORIG_X_CS_PIN NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN 31
#define ORIG_Y_DIR_PIN 33
#define ORIG_Y_ENABLE_PIN 29
#define ORIG_Y_CS_PIN NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN 37
#define ORIG_Z_DIR_PIN 39
#define ORIG_Z_ENABLE_PIN 35
#define ORIG_Z_CS_PIN NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 43
#define ORIG_E0_DIR_PIN 45
#define ORIG_E0_ENABLE_PIN 41
#define ORIG_E0_CS_PIN NoPin
#define ORIG_SOL0_PIN NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN 49
#define ORIG_E1_DIR_PIN 47
#define ORIG_E1_ENABLE_PIN 48
#define ORIG_E1_CS_PIN NoPin
#define ORIG_SOL1_PIN NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN NoPin
#define ORIG_E2_DIR_PIN NoPin
#define ORIG_E2_ENABLE_PIN NoPin
#define ORIG_E2_CS_PIN NoPin
#define ORIG_SOL2_PIN NoPin

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN NoPin
#define ORIG_E3_DIR_PIN NoPin
#define ORIG_E3_ENABLE_PIN NoPin
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
#define ORIG_X_MIN_PIN 22
#define ORIG_X_MAX_PIN 24
#define ORIG_Y_MIN_PIN 26
#define ORIG_Y_MAX_PIN 28
#define ORIG_Z_MIN_PIN 30
#define ORIG_Z_MAX_PIN 32
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
#define ORIG_HEATER_0_PIN 2
#define ORIG_HEATER_1_PIN 3
#define ORIG_HEATER_2_PIN NoPin
#define ORIG_HEATER_3_PIN NoPin
#define ORIG_HEATER_BED_PIN 4
#define ORIG_HEATER_CHAMBER_PIN NoPin
#define ORIG_COOLER_PIN NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN 8
#define ORIG_TEMP_1_PIN 9
#define ORIG_TEMP_2_PIN NoPin
#define ORIG_TEMP_3_PIN NoPin
#define ORIG_TEMP_BED_PIN 10
#define ORIG_TEMP_CHAMBER_PIN NoPin
#define ORIG_TEMP_COOLER_PIN NoPin

//###FAN
#define ORIG_FAN0_PIN 7
#define ORIG_FAN1_PIN NoPin
#define ORIG_FAN2_PIN NoPin
#define ORIG_FAN3_PIN NoPin

//###MISC
#define ORIG_PS_ON_PIN 12
#define ORIG_BEEPER_PIN NoPin
#define LED_PIN 13
#define SDPOWER_PIN NoPin
#define SD_DETECT_PIN NoPin
#define SDSS 53
#define KILL_PIN NoPin
#define DEBUG_PIN NoPin
#define SUICIDE_PIN 54

//###LASER
#define ORIG_LASER_PWR_PIN NoPin
#define ORIG_LASER_PWM_PIN NoPin

//###SERVOS
#if NUM_SERVOS > 0
  #define SERVO0_PIN 32
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


//###IF_BLOCKS
#if ENABLED(ULTRA_LCD)

  #define ORIG_BEEPER_PIN     18

  #if ENABLED(NEWPANEL)

    #define LCD_PINS_RS       20
    #define LCD_PINS_ENABLE   17
    #define LCD_PINS_D4       16
    #define LCD_PINS_D5       21
    #define LCD_PINS_D6        5
    #define LCD_PINS_D7        6

    // buttons are directly attached
    #define BTN_EN1           40
    #define BTN_EN2           42
    #define BTN_ENC           19

    #define SD_DETECT_PIN     38

  #else // !NEWPANEL - Old style panel with shift register

    // buttons are attached to a shift register
    #define SHIFT_CLK         38
    #define SHIFT_LD          42
    #define SHIFT_OUT         40
    #define SHIFT_EN          17

    #define LCD_PINS_RS       16
    #define LCD_PINS_ENABLE    5
    #define LCD_PINS_D4        6
    #define LCD_PINS_D5       21
    #define LCD_PINS_D6       20
    #define LCD_PINS_D7       19

    #define SD_DETECT_PIN     NoPin

  #endif // !NEWPANEL

#endif // ULTRA_LCD
//@@@
