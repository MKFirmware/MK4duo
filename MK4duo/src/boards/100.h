/****************************************************************************************
* 100
* ANET board A2, A6 and A8
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega644P__) && DISABLED(__AVR_ATmega1284P__)
  #error Oops!  Make sure you have 'Sanguino' or 'Anet' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Anet"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 15
#define ORIG_X_DIR_PIN 21
#define ORIG_X_ENABLE_PIN 14
#define ORIG_X_CS_PIN NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN 22
#define ORIG_Y_DIR_PIN 23
#define ORIG_Y_ENABLE_PIN 14
#define ORIG_Y_CS_PIN NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN 3
#define ORIG_Z_DIR_PIN 2
#define ORIG_Z_ENABLE_PIN 26
#define ORIG_Z_CS_PIN NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 1
#define ORIG_E0_DIR_PIN 0
#define ORIG_E0_ENABLE_PIN 14
#define ORIG_E0_CS_PIN NoPin
#define ORIG_SOL0_PIN NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN NoPin
#define ORIG_E1_DIR_PIN NoPin
#define ORIG_E1_ENABLE_PIN NoPin
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
#define ORIG_X_MIN_PIN NoPin
#define ORIG_X_MAX_PIN NoPin
#define ORIG_Y_MIN_PIN NoPin
#define ORIG_Y_MAX_PIN NoPin
#define ORIG_Z_MIN_PIN NoPin
#define ORIG_Z_MAX_PIN NoPin
#define ORIG_Z2_MIN_PIN NoPin
#define ORIG_Z2_MAX_PIN NoPin
#define ORIG_Z3_MIN_PIN NoPin
#define ORIG_Z3_MAX_PIN NoPin
#define ORIG_Z4_MIN_PIN NoPin
#define ORIG_Z4_MAX_PIN NoPin
#define ORIG_E_MIN_PIN NoPin
#define ORIG_Z_PROBE_PIN NoPin

//###SINGLE_ENDSTOP
#define X_STOP_PIN 18
#define Y_STOP_PIN 19
#define Z_STOP_PIN 20

//###HEATER
#define ORIG_HEATER_0_PIN 13
#define ORIG_HEATER_1_PIN NoPin
#define ORIG_HEATER_2_PIN NoPin
#define ORIG_HEATER_3_PIN NoPin
#define ORIG_HEATER_BED_PIN 12
#define ORIG_HEATER_CHAMBER_PIN NoPin
#define ORIG_COOLER_PIN NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN 7
#define ORIG_TEMP_1_PIN NoPin
#define ORIG_TEMP_2_PIN NoPin
#define ORIG_TEMP_3_PIN NoPin
#define ORIG_TEMP_BED_PIN 6
#define ORIG_TEMP_CHAMBER_PIN NoPin
#define ORIG_TEMP_COOLER_PIN NoPin

//###FAN
#define ORIG_FAN0_PIN 4
#define ORIG_FAN1_PIN NoPin
#define ORIG_FAN2_PIN NoPin
#define ORIG_FAN3_PIN NoPin

//###MISC
#define ORIG_PS_ON_PIN NoPin
#define ORIG_BEEPER_PIN NoPin
#define LED_PIN NoPin
#define SDPOWER_PIN NoPin
#define SD_DETECT_PIN NoPin
#define SDSS 31
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


//###IF_BLOCKS
/**
 * LCD / Controller
 *
 * Only the following displays are supported:
 *  ANET_KEYPAD_LCD
 *  ANET_FULL_GRAPHICS_LCD
 *  REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
*/
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define LCD_SDSS           28
  #if ENABLED(ADC_KEYPAD)
    #define SERVO0_PIN       27 // free for BLTouch/3D-Touch
    #define LCD_PINS_RS      28
    #define LCD_PINS_ENABLE  29
    #define LCD_PINS_D4      10
    #define LCD_PINS_D5      11
    #define LCD_PINS_D6      16
    #define LCD_PINS_D7      17
    #define BTN_EN1          -1
    #define BTN_EN2          -1
    #define BTN_ENC          -1
    #define ADC_KEYPAD_PIN    1
    #define ENCODER_FEEDRATE_DEADZONE 2
  #elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER) || ENABLED(ANET_FULL_GRAPHICS_LCD)
    // Pin definitions for the Anet A6 Full Graphics display and the RepRapDiscount Full Graphics
    // display using an adapter board  // https://go.aisler.net/benlye/anet-lcd-adapter/pcb
    // See below for alternative pin definitions for use with https://www.thingiverse.com/thing:2103748
    #define SERVO0_PIN       29 // free for BLTouch/3D-Touch
    #define BEEPER_PIN       17
    #define LCD_PINS_RS      27
    #define LCD_PINS_ENABLE  28
    #define LCD_PINS_D4      30
    #define BTN_EN1          11
    #define BTN_EN2          10
    #define BTN_ENC          16
    #define ST7920_DELAY_1 DELAY_0_NOP
    #define ST7920_DELAY_2 DELAY_1_NOP
    #define ST7920_DELAY_3 DELAY_2_NOP
    #define STD_ENCODER_PULSES_PER_STEP 4
    #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
  #endif
#else
  #define SERVO0_PIN         27
#endif
//@@@
