/****************************************************************************************
* 303
* Mini Rambo
****************************************************************************************/


#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Mini Rambo"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            37
#define ORIG_X_DIR_PIN             48
#define ORIG_X_ENABLE_PIN          29
#define ORIG_X_CS_PIN              NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN            36
#define ORIG_Y_DIR_PIN             49
#define ORIG_Y_ENABLE_PIN          28
#define ORIG_Y_CS_PIN              NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN            35
#define ORIG_Z_DIR_PIN             47
#define ORIG_Z_ENABLE_PIN          27
#define ORIG_Z_CS_PIN              NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           34
#define ORIG_E0_DIR_PIN            43
#define ORIG_E0_ENABLE_PIN         26
#define ORIG_E0_CS_PIN             NoPin
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           NoPin
#define ORIG_E1_DIR_PIN            NoPin
#define ORIG_E1_ENABLE_PIN         NoPin
#define ORIG_E1_CS_PIN             NoPin
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
#define ORIG_X_MIN_PIN             12
#define ORIG_X_MAX_PIN             30
#define ORIG_Y_MIN_PIN             11
#define ORIG_Y_MAX_PIN             24
#define ORIG_Z_MIN_PIN             10
#define ORIG_Z_MAX_PIN             23
#define ORIG_Z2_MIN_PIN            NoPin
#define ORIG_Z2_MAX_PIN            NoPin
#define ORIG_Z3_MIN_PIN            NoPin
#define ORIG_Z3_MAX_PIN            NoPin
#define ORIG_Z4_MIN_PIN            NoPin
#define ORIG_Z4_MAX_PIN            NoPin
#define ORIG_Z_PROBE_PIN           23

//###SINGLE_ENDSTOP
#define X_STOP_PIN                 NoPin
#define Y_STOP_PIN                 NoPin
#define Z_STOP_PIN                 NoPin

//###HEATER
#define ORIG_HEATER_0_PIN           3
#define ORIG_HEATER_1_PIN           7
#define ORIG_HEATER_2_PIN           6
#define ORIG_HEATER_3_PIN          NoPin
#define ORIG_HEATER_BED_PIN         4
#define ORIG_HEATER_CHAMBER_PIN    NoPin
#define ORIG_COOLER_PIN            NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN             0
#define ORIG_TEMP_1_PIN             1
#define ORIG_TEMP_2_PIN            NoPin
#define ORIG_TEMP_3_PIN            NoPin
#define ORIG_TEMP_BED_PIN           2
#define ORIG_TEMP_CHAMBER_PIN      NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN               8
#define ORIG_FAN1_PIN               6
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
#define LED_PIN                    13
#define SDPOWER_PIN                NoPin
#define SD_DETECT_PIN              NoPin
#define SDSS                       53
#define KILL_PIN                   NoPin
#define DEBUG_PIN                  NoPin
#define SUICIDE_PIN                NoPin

//###LASER
#define ORIG_LASER_PWR_PIN         NoPin
#define ORIG_LASER_PWM_PIN          9


//###UNKNOWN_PINS
// Microstepping pins - Mapping not from fastio.h (?)
#define X_MS1_PIN           40
#define X_MS2_PIN           41
#define Y_MS1_PIN           69
#define Y_MS2_PIN           39
#define Z_MS1_PIN           68
#define Z_MS2_PIN           67
#define E0_MS1_PIN          65
#define E0_MS2_PIN          66

#define MOTOR_CURRENT_PWM_XY_PIN  46
#define MOTOR_CURRENT_PWM_Z_PIN   45
#define MOTOR_CURRENT_PWM_E_PIN   44

#define CASE_LIGHT_PIN 9

// use P1 connector for spindle pins
#define SPINDLE_LASER_ENABLE_PIN 18  // Pin should have a pullup!
#define SPINDLE_DIR_PIN          19
//@@@

//###IF_BLOCKS
// Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#if DISABLED(MOTOR_CURRENT_PWM_RANGE)
  #define MOTOR_CURRENT_PWM_RANGE 2000
#endif
#define DEFAULT_PWM_MOTOR_CURRENT  {1300, 1300, 1250}

#if ENABLED(ULTRA_LCD)

    #define KILL_PIN                32

  #if ENABLED(NEWPANEL)

      // AUX-4
      #define ORIG_BEEPER_PIN       84

      // AUX-2
      #define BTN_EN1               14
      #define BTN_EN2               72
      #define BTN_ENC                9

      #define LCD_PINS_RS           82
      #define LCD_PINS_ENABLE       18
      #define LCD_PINS_D4           19
      #define LCD_PINS_D5           70
      #define LCD_PINS_D6           85
      #define LCD_PINS_D7           71

      #define SD_DETECT_PIN         15

  #endif // NEWPANEL

#endif // ULTRA_LCD
//@@@

