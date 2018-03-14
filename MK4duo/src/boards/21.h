/****************************************************************************************
* 21
* Elefu RA Board
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Elefu"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            49
#define ORIG_X_DIR_PIN             13
#define ORIG_X_ENABLE_PIN          48
#define ORIG_X_CS_PIN              NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN            11
#define ORIG_Y_DIR_PIN              9
#define ORIG_Y_ENABLE_PIN          12
#define ORIG_Y_CS_PIN              NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN             7
#define ORIG_Z_DIR_PIN              6
#define ORIG_Z_ENABLE_PIN           8
#define ORIG_Z_CS_PIN              NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           40
#define ORIG_E0_DIR_PIN            41
#define ORIG_E0_ENABLE_PIN         37
#define ORIG_E0_CS_PIN             NoPin
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           18
#define ORIG_E1_DIR_PIN            19
#define ORIG_E1_ENABLE_PIN         38
#define ORIG_E1_CS_PIN             NoPin
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           43
#define ORIG_E2_DIR_PIN            47
#define ORIG_E2_ENABLE_PIN         42
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
#define ORIG_X_MIN_PIN             35
#define ORIG_X_MAX_PIN             NoPin
#define ORIG_Y_MIN_PIN             33
#define ORIG_Y_MAX_PIN             NoPin
#define ORIG_Z_MIN_PIN             31
#define ORIG_Z_MAX_PIN             NoPin
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
#define ORIG_HEATER_0_PIN          45
#define ORIG_HEATER_1_PIN          46
#define ORIG_HEATER_2_PIN          17
#define ORIG_HEATER_3_PIN          NoPin
#define ORIG_HEATER_BED_PIN        44
#define ORIG_HEATER_CHAMBER_PIN    NoPin
#define ORIG_COOLER_PIN            NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN             3
#define ORIG_TEMP_1_PIN             2
#define ORIG_TEMP_2_PIN             1
#define ORIG_TEMP_3_PIN            NoPin
#define ORIG_TEMP_BED_PIN           0
#define ORIG_TEMP_CHAMBER_PIN      NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN              16
#define ORIG_FAN1_PIN              NoPin
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin

//###SERVO
#define SERVO0_PIN                 NoPin
#define SERVO1_PIN                 NoPin
#define SERVO2_PIN                 NoPin
#define SERVO3_PIN                 NoPin

//###MISC
#define ORIG_PS_ON_PIN             10
#define ORIG_BEEPER_PIN            36
#define LED_PIN                    NoPin
#define SDPOWER_PIN                NoPin
#define SD_DETECT_PIN              NoPin
#define SDSS                       NoPin
#define KILL_PIN                   NoPin
#define DEBUG_PIN                  NoPin
#define SUICIDE_PIN                NoPin

//###LASER
#define ORIG_LASER_PWR_PIN         NoPin
#define ORIG_LASER_PWM_PIN         NoPin

//###FLOWMETER
#define ORIG_FLOMETER_PIN          NoPin

//###UNKNOWN_PINS
#define SLEEP_WAKE_PIN        26  // This feature still needs work
#define PHOTOGRAPH_PIN        29
//@@@

//###IF_BLOCKS
#if ENABLED(RA_CONTROL_PANEL)
  #define SDSS                53
  #define SD_DETECT_PIN       28

  #define BTN_EN1             14
  #define BTN_EN2             39
  #define BTN_ENC             15  // the click

  #define BLEN_C              2
  #define BLEN_B              1
  #define BLEN_A              0
#endif // RA_CONTROL_PANEL

#if ENABLED(RA_DISCO)
  //variables for which pins the TLC5947 is using
  #define TLC_CLOCK_PIN       25
  #define TLC_BLANK_PIN       23
  #define TLC_XLAT_PIN        22
  #define TLC_DATA_PIN        24

  //We also need to define pin to port number mapping for the 2560 to match the pins listed above. If you change the TLC pins, update this as well per the 2560 datasheet!
  //This currently only works with the RA Board.
  #define TLC_CLOCK_BIT 3 //bit 3 on port A
  #define TLC_CLOCK_PORT &PORTA //bit 3 on port A

  #define TLC_BLANK_BIT 1 //bit 1 on port A
  #define TLC_BLANK_PORT &PORTA //bit 1 on port A

  #define TLC_DATA_BIT 2 //bit 2 on port A
  #define TLC_DATA_PORT &PORTA //bit 2 on port A

  #define TLC_XLAT_BIT 0 //bit 0 on port A
  #define TLC_XLAT_PORT &PORTA //bit 0 on port A

  //change this to match your situation. Lots of TLCs takes up the arduino SRAM very quickly, so be careful
  //Leave it at at least 1 if you have enabled RA_LIGHTING
  //The number of TLC5947 boards chained together for use with the animation, additional ones will repeat the animation on them, but are not individually addressable and mimic those before them. You can leave the default at 2 even if you only have 1 TLC5947 module.
  #define NUM_TLCS 2

  //These TRANS_ARRAY values let you change the order the LEDs on the lighting modules will animate for chase functions.
  //Modify them according to your specific situation.
  //NOTE: the array should be 8 long for every TLC you have. These defaults assume (2) TLCs.
  #define TRANS_ARRAY {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8} //forwards
  //#define TRANS_ARRAY {7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 13, 14, 15} //backwards
#endif //RA_LIGHTING
//@@@
