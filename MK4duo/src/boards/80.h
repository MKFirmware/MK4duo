/****************************************************************************************
* 80
* RUMBA
****************************************************************************************/

#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         17
#define ORIG_X_DIR_PIN          16
#define ORIG_X_ENABLE_PIN       48
#define ORIG_X_MIN_PIN          37
#define ORIG_X_MAX_PIN          36

#define ORIG_Y_STEP_PIN         54
#define ORIG_Y_DIR_PIN          47
#define ORIG_Y_ENABLE_PIN       55
#define ORIG_Y_MIN_PIN          35
#define ORIG_Y_MAX_PIN          34

#define Y2_STEP_PIN             26
#define Y2_DIR_PIN              25
#define Y2_ENABLE_PIN           27

#define ORIG_Z_STEP_PIN         57
#define ORIG_Z_DIR_PIN          56
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          33
#define ORIG_Z_MAX_PIN          32

#define Z2_STEP_PIN             26
#define Z2_DIR_PIN              25
#define Z2_ENABLE_PIN           27

#define ORIG_E0_STEP_PIN        23
#define ORIG_E0_DIR_PIN         22
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        26
#define ORIG_E1_DIR_PIN         25
#define ORIG_E1_ENABLE_PIN      27

#define ORIG_E2_STEP_PIN        29
#define ORIG_E2_DIR_PIN         28
#define ORIG_E2_ENABLE_PIN      39

#define LED_PIN                 13

#define ORIG_FAN_PIN            7
//additional FAN1 PIN (e.g. useful for electronics fan or light on/off) on PIN 8

#define ORIG_PS_ON_PIN          45
#define KILL_PIN                46

#if (TEMP_SENSOR_0==0)
 #define ORIG_TEMP_0_PIN             -1
 #define ORIG_HEATER_0_PIN           -1
#else
 #define ORIG_HEATER_0_PIN           2    // EXTRUDER 1
 #if (TEMP_SENSOR_0==-1)
  #define ORIG_TEMP_0_PIN            6    // ANALOG NUMBERING - connector *K1* on RUMBA thermocouple ADD ON is used
 #else
  #define ORIG_TEMP_0_PIN            15   // ANALOG NUMBERING - default connector for thermistor *T0* on rumba board is used
 #endif
#endif

#if (TEMP_SENSOR_1==0)
 #define ORIG_TEMP_1_PIN             -1
 #define ORIG_HEATER_1_PIN           -1
#else
 #define ORIG_HEATER_1_PIN           3    // EXTRUDER 2
 #if (TEMP_SENSOR_1==-1)
  #define ORIG_TEMP_1_PIN            5    // ANALOG NUMBERING - connector *K2* on RUMBA thermocouple ADD ON is used
 #else
  #define ORIG_TEMP_1_PIN            14   // ANALOG NUMBERING - default connector for thermistor *T1* on rumba board is used
 #endif
#endif

#if (TEMP_SENSOR_2==0)
 #define ORIG_TEMP_2_PIN         -1
 #define ORIG_HEATER_2_PIN       -1
#else
 #define ORIG_HEATER_2_PIN        6    // EXTRUDER 3
 #if (TEMP_SENSOR_2==-1)
  #define ORIG_TEMP_2_PIN         7    // ANALOG NUMBERING - connector *K3* on RUMBA thermocouple ADD ON is used <-- this can not be used when TEMP_SENSOR_BED is defined as thermocouple
 #else
  #define ORIG_TEMP_2_PIN         13   // ANALOG NUMBERING - default connector for thermistor *T2* on rumba board is used
 #endif
#endif

//optional for extruder 4 or chamber: #define TEMP_X_PIN         12   // ANALOG NUMBERING - default connector for thermistor *T3* on rumba board is used
//optional FAN1 can be used as 4th heater output: #define ORIG_HEATER_3_PIN       8    // EXTRUDER 4

#if (TEMP_SENSOR_BED==0)
 #define ORIG_TEMP_BED_PIN       -1
 #define ORIG_HEATER_BED_PIN     -1
#else
 #define ORIG_HEATER_BED_PIN      9    // BED
 #if (TEMP_SENSOR_BED==-1)
  #define ORIG_TEMP_BED_PIN       7    // ANALOG NUMBERING - connector *K3* on RUMBA thermocouple ADD ON is used <-- this can not be used when TEMP_SENSOR_2 is defined as thermocouple
 #else
  #define ORIG_TEMP_BED_PIN       11   // ANALOG NUMBERING - default connector for thermistor *THB* on rumba board is used
 #endif
#endif

#define SDPOWER            -1
#define SDSS               53
#define SD_DETECT_PIN      49
#define ORIG_BEEPER_PIN    44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43


