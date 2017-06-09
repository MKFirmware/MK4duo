/****************************************************************************************
* 1400
*
* PICCOLO_3D
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "PICCOLO 3D"

#ifndef ARDUINO_ARCH_SAM
  #error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define PICCOLO_3D

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

#define S0_MUX                  41
#define S1_MUX                  40
#define S2_MUX                  50

#define MOTOR_FAULT_PIN         10

//#define EXP_VOLTAGE_LEVEL_PIN    2 //(ADC1 pin A5)

//#define DAC0_SYNC               53 // PB14
//#define DAC1_SYNC                6 // PC24

#define ORIG_X_STEP_PIN         35
#define ORIG_X_DIR_PIN          36
#define ORIG_X_ENABLE_PIN       37

#define ORIG_Y_STEP_PIN         15
#define ORIG_Y_DIR_PIN          22
#define ORIG_Y_ENABLE_PIN       37

#define ORIG_Z_STEP_PIN         26
#define ORIG_Z_DIR_PIN          28
#define ORIG_Z_ENABLE_PIN       53

#define ORIG_X_MIN_PIN          23
#define ORIG_X_MAX_PIN          25
#define ORIG_Y_MIN_PIN          27
#define ORIG_Y_MAX_PIN          29
#define ORIG_Z_MIN_PIN          31
#define ORIG_Z_MAX_PIN          33

#define ORIG_E0_STEP_PIN        39
#define ORIG_E0_DIR_PIN         42
#define ORIG_E0_ENABLE_PIN      63

#define ORIG_E1_STEP_PIN        39
#define ORIG_E1_DIR_PIN         42
#define ORIG_E1_ENABLE_PIN      64

#define ORIG_E2_STEP_PIN        39
#define ORIG_E2_DIR_PIN         42
#define ORIG_E2_ENABLE_PIN      65

#define ORIG_E3_STEP_PIN        39
#define ORIG_E3_DIR_PIN         42
#define ORIG_E3_ENABLE_PIN      38

#define ORIG_E4_STEP_PIN        39
#define ORIG_E4_DIR_PIN         42
#define ORIG_E4_ENABLE_PIN      62

#define ORIG_E5_STEP_PIN        39
#define ORIG_E5_DIR_PIN         42
#define ORIG_E5_ENABLE_PIN      61

#define ORIG_E6_STEP_PIN        39
#define ORIG_E6_DIR_PIN         42
#define ORIG_E6_ENABLE_PIN      32

#define ORIG_E7_STEP_PIN        39
#define ORIG_E7_DIR_PIN         42
#define ORIG_E7_ENABLE_PIN      34

#define SDPOWER                 -1
#define SDSS                     4
#define LED_PIN                 -1

#define ORIG_FAN_PIN 	           8  // FET5
#define ORIG_FAN2_PIN            7  // FET6

#define ORIG_PS_ON_PIN          24

#define ORIG_BEEPER_PIN         44

#define KILL_PIN                -1

#define ORIG_HEATER_BED_PIN     13  // BED FET1 VICINO FUSIBILE
#define ORIG_HEATER_0_PIN       12  // FET2 SUCCESSIVO
#define ORIG_HEATER_1_PIN       11
#define ORIG_HEATER_2_PIN        9
#define ORIG_HEATER_3_PIN        8  // OPZIONALE FAN
#define ORIG_HEATER_4_PIN        7  // OPZIONALE FAN

#define ORIG_TEMP_BED_PIN        0  // ANALOG NUMBERING

#define ORIG_TEMP_0_PIN          1  // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN          2  // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN          3  // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN          4  // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN             2
  #if NUM_SERVOS > 1
    #define SERVO1_PIN           3
    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN       6
      #endif
    #endif
  #endif
#endif

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
