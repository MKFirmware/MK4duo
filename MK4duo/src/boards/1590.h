/****************************************************************************************
* 1590
* UltiMachine Archim2
*
* The Archim2 board requires Arduino Archim addons installed.
* Add the following URL to Arduino IDE's Additional Board Manager URLs
* https://raw.githubusercontent.com/ultimachine/ArduinoAddons/master/package_ultimachine_index.json
* In the Arduino IDE Board Manager search for Archim and install the package.
* Change your target board to "Archim".
*
* Further information is provided by UltiMachine
* https://github.com/ultimachine/Archim/wiki/Archim-v2.0
* https://github.com/ultimachine/Archim/wiki
*
****************************************************************************************/

//###CHIP
#if DISABLED(ARDUINO_ARCH_SAM)
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Archim2"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            38
#define ORIG_X_DIR_PIN             37
#define ORIG_X_ENABLE_PIN          41
#define ORIG_X_CS_PIN              39

//###Y_AXIS
#define ORIG_Y_STEP_PIN            51
#define ORIG_Y_DIR_PIN             92
#define ORIG_Y_ENABLE_PIN          49
#define ORIG_Y_CS_PIN              50

//###Z_AXIS
#define ORIG_Z_STEP_PIN            46
#define ORIG_Z_DIR_PIN             47
#define ORIG_Z_ENABLE_PIN          44
#define ORIG_Z_CS_PIN              45

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           107
#define ORIG_E0_DIR_PIN            96
#define ORIG_E0_ENABLE_PIN         105
#define ORIG_E0_CS_PIN             104
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           22
#define ORIG_E1_DIR_PIN            97
#define ORIG_E1_ENABLE_PIN         18
#define ORIG_E1_CS_PIN             19
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           NoPin
#define ORIG_E2_DIR_PIN            NoPin
#define ORIG_E2_ENABLE_PIN         NoPin
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
#define ORIG_X_MIN_PIN             14
#define ORIG_X_MAX_PIN             32
#define ORIG_Y_MIN_PIN             29
#define ORIG_Y_MAX_PIN             15
#define ORIG_Z_MIN_PIN             31
#define ORIG_Z_MAX_PIN             30
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
#define ORIG_HEATER_0_PIN           6
#define ORIG_HEATER_1_PIN           7
#define ORIG_HEATER_2_PIN           8
#define ORIG_HEATER_3_PIN          NoPin
#define ORIG_HEATER_BED_PIN         9
#define ORIG_HEATER_CHAMBER_PIN    NoPin
#define ORIG_COOLER_PIN            NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN             1
#define ORIG_TEMP_1_PIN             2
#define ORIG_TEMP_2_PIN             4
#define ORIG_TEMP_3_PIN            NoPin
#define ORIG_TEMP_BED_PIN           3
#define ORIG_TEMP_CHAMBER_PIN      NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN               4
#define ORIG_FAN1_PIN               5
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin

//###SERVO
#define SERVO0_PIN                 NoPin
#define SERVO1_PIN                 NoPin
#define SERVO2_PIN                 NoPin
#define SERVO3_PIN                 NoPin

//###MISC
#define ORIG_PS_ON_PIN             NoPin
#define ORIG_BEEPER_PIN            NoPin
#define LED_PIN                    NoPin
#define SDPOWER_PIN                NoPin
#define SD_DETECT_PIN              NoPin
#define SDSS                       87
#define KILL_PIN                   NoPin
#define DEBUG_PIN                  NoPin
#define SUICIDE_PIN                NoPin

//###LASER
#define ORIG_LASER_PWR_PIN         NoPin
#define ORIG_LASER_PWM_PIN         NoPin


//###UNKNOWN_PINS
#undef NUM_DIGITAL_PINS
#define NUM_DIGITAL_PINS 108
#define E2END 0x2000
//@@@

//###IF_BLOCKS
#if ENABLED(ULTRA_LCD)

  #if ENABLED(NEWPANEL)
   //arduino pin which triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 23
    #define LCD_PINS_RS     17
    #define LCD_PINS_ENABLE 24
    #define LCD_PINS_D4     69
    #define LCD_PINS_D5     54
    #define LCD_PINS_D6     68
    #define LCD_PINS_D7     34

    #define SD_DETECT_PIN    2
    #define SDSS            87

    //buttons are directly attached using AUX-2
    #define BTN_EN1         60
    #define BTN_EN2         13
    #define BTN_ENC         16

    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0

    //encoder rotation values
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #endif  // NEWPANEL

#endif // ULTRA_LCD
//@@@

