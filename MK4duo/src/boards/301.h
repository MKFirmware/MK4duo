/****************************************************************************************
* 301
* Rambo
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Rambo"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 37
#define ORIG_X_DIR_PIN 48
#define ORIG_X_ENABLE_PIN 29
#define ORIG_X_CS_PIN NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN 36
#define ORIG_Y_DIR_PIN 49
#define ORIG_Y_ENABLE_PIN 28
#define ORIG_Y_CS_PIN NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN 35
#define ORIG_Z_DIR_PIN 47
#define ORIG_Z_ENABLE_PIN 27
#define ORIG_Z_CS_PIN NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 34
#define ORIG_E0_DIR_PIN 43
#define ORIG_E0_ENABLE_PIN 26
#define ORIG_E0_CS_PIN NoPin
#define ORIG_SOL0_PIN NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN 33
#define ORIG_E1_DIR_PIN 42
#define ORIG_E1_ENABLE_PIN 25
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
#define ORIG_X_MIN_PIN 12
#define ORIG_X_MAX_PIN 24
#define ORIG_Y_MIN_PIN 11
#define ORIG_Y_MAX_PIN 23
#define ORIG_Z_MIN_PIN 10
#define ORIG_Z_MAX_PIN 30
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
#define ORIG_HEATER_0_PIN 9
#define ORIG_HEATER_1_PIN 7
#define ORIG_HEATER_2_PIN NoPin
#define ORIG_HEATER_3_PIN NoPin
#define ORIG_HEATER_BED_PIN 3
#define ORIG_HEATER_CHAMBER_PIN NoPin
#define ORIG_COOLER_PIN NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN 0
#define ORIG_TEMP_1_PIN 1
#define ORIG_TEMP_2_PIN NoPin
#define ORIG_TEMP_3_PIN NoPin
#define ORIG_TEMP_BED_PIN 2
#define ORIG_TEMP_CHAMBER_PIN NoPin
#define ORIG_TEMP_COOLER_PIN NoPin

//###FAN
#define ORIG_FAN0_PIN 8
#define ORIG_FAN1_PIN NoPin
#define ORIG_FAN2_PIN NoPin
#define ORIG_FAN3_PIN NoPin

//###MISC
#define ORIG_PS_ON_PIN 4
#define ORIG_BEEPER_PIN NoPin
#define LED_PIN 13
#define SDPOWER_PIN NoPin
#define SD_DETECT_PIN NoPin
#define SDSS 53
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
#define X_MS1_PIN 40
#define X_MS2_PIN 41
#define Y_MS1_PIN 69
#define Y_MS2_PIN 39
#define Z_MS1_PIN 68
#define Z_MS2_PIN 67
#define E0_MS1_PIN 65
#define E0_MS2_PIN 66
#define E1_MS1_PIN 63
#define E1_MS2_PIN 64
#define DIGIPOTSS_PIN 38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping
//@@@

//###IF_BLOCKS
#if ENABLED(BARICUDA)
#define ORIG_HEATER_2_PIN 6
#else
#define ORIG_HEATER_2_PIN NoPin
#endif

#if ENABLED(ULTRA_LCD)
  #define KILL_PIN 80
  #if ENABLED(NEWPANEL)
   // arduino pin which triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 79      // Beeper on AUX-4
    #define LCD_PINS_RS 70
    #define LCD_PINS_ENABLE 71
    #define LCD_PINS_D4 72
    #define LCD_PINS_D5 73
    #define LCD_PINS_D6 74
    #define LCD_PINS_D7 75

    //buttons are directly attached using AUX-2
    #define BTN_EN1 76
    #define BTN_EN2 77
    #define BTN_ENC 78  //the click

    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0

    #define SD_DETECT_PIN 81    // Ramps does not use this port

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 33    No Beeper added
    //buttons are attached to a shift register
    // Not wired this yet
    // #define SHIFT_CLK 38
    // #define SHIFT_LD 42
    // #define SHIFT_OUT 40
    // #define SHIFT_EN 17

    #define LCD_PINS_RS 75
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    //bits in the shift register that carry the buttons for:
    // left up center down right red
    #define BL_LE 7
    #define BL_UP 6
    #define BL_MI 5
    #define BL_DW 4
    #define BL_RI 3
    #define BL_ST 2
    #define BLEN_B 1
    #define BLEN_A 0
  #endif
#endif //ULTRA_LCD
//@@@
