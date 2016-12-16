/****************************************************************************************
* 503
* Alligator R3
* http://www.3dartists.org/
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "Alligator R3"
#define NUM_DIGITAL_PINS 110

#ifndef ARDUINO_ARCH_SAM
  #error Oops!  Make sure you have 'Alligator 3D Printer Board' selected from the 'Tools -> Boards' menu.
#endif

#define ALLIGATOR
#define SPI_CHAN_DAC 1

// X AXIS
#define ORIG_X_STEP_PIN       96
#define ORIG_X_DIR_PIN         2
#define ORIG_X_ENABLE_PIN     24
#define ORIG_X_MIN_PIN        33
#define ORIG_X_MAX_PIN        34
#define X_MS1_PIN             99

// Y AXIS
#define ORIG_Y_STEP_PIN       94
#define ORIG_Y_DIR_PIN        95
#define ORIG_Y_ENABLE_PIN      3
#define ORIG_Y_MIN_PIN        35
#define ORIG_Y_MAX_PIN        37
#define Y_MS1_PIN             10

// Z AXIS
#define ORIG_Z_STEP_PIN       65
#define ORIG_Z_DIR_PIN        52
#define ORIG_Z_ENABLE_PIN      4
#define ORIG_Z_MIN_PIN        38
#define ORIG_Z_MAX_PIN        39
#define Z_MS1_PIN             44

// E0 AXIS
#define ORIG_E0_STEP_PIN      63
#define ORIG_E0_DIR_PIN       64
#define ORIG_E0_ENABLE_PIN    98
#define E0_MS1_PIN            45

// E1 AXIS
#define ORIG_E1_STEP_PIN      62
#define ORIG_E1_DIR_PIN       53
#define ORIG_E1_ENABLE_PIN    29

// E2 AXIS
#define ORIG_E2_STEP_PIN      20
#define ORIG_E2_DIR_PIN       21
#define ORIG_E2_ENABLE_PIN    12

// E3 AXIS
#define ORIG_E3_STEP_PIN      66
#define ORIG_E3_DIR_PIN       67
#define ORIG_E3_ENABLE_PIN    30

// Motor Fault onboard stepper driver (X/Y/Z/E0)
#define MOTOR_FAULT_PIN       22
// Motor Fault on piggy module (E1/E2/E3)
#define MOTOR_FAULT_PIGGY_PIN  7

#define SDPOWER               -1
#define SDSS                  77
#define SD_DETECT_PIN         87
#define LED_PIN               -1

#define ORIG_FAN_PIN          92
#define ORIG_FAN2_PIN         31

#define ORIG_PS_ON_PIN        -1
#define KILL_PIN              -1
#define SUICIDE_PIN           -1 // PIN that has to be turned on right after start, to keep power flowing.

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define ORIG_HEATER_BED_PIN   69
#define ORIG_HEATER_0_PIN     42
#define ORIG_HEATER_1_PIN      8
#define ORIG_HEATER_2_PIN      9
#define ORIG_HEATER_3_PIN     97

#define ORIG_TEMP_BED_PIN      6
#define ORIG_TEMP_0_PIN        5
#define ORIG_TEMP_1_PIN        4
#define ORIG_TEMP_2_PIN        3
#define ORIG_TEMP_3_PIN        2

#define DAC0_SYNC             27
#define DAC1_SYNC              6

//** Onboard addictional memory
// 64K EEPROM
#define SPI_EEPROM
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25
// 2K EEPROM
#define SPI_EEPROM2_CS        26
// 32Mb FLASH
#define SPI_FLASH_CS          23

// ESP Use internal USART-3
#define ESP_WIFI_MODULE_COM    3
#define ESP_WIFI_MODULE_RESET_PIN 43
#define PIGGY_GPIO_PIN        11

#define FTDI_COM_RESET_PIN    32

/** Expansion#1 port J14  3V3/5V voltage level firmware selectable**/
#define EXP1_OUT_ENABLE_PIN   49
#define EXP1_VOLTAGE_SELECT    5
#define EXP1_J14_4            48
#define EXP1_J14_6            47
#define EXP1_J14_8            46
// USART-0 RX
#define EXP1_J14_5            19
// USART-0 TX
#define EXP1_J14_3            18
#define EXP1_J14_9           108
#define EXP1_J14_10          109

//** Expansion#2 port J5
// USART-1 RX
#define EXP2_J5_5             17
// USART-1 TX
#define EXP2_J5_1             16
//A0
#define EXP2_J5_3             54
#define EXP1_J5_4             36
#define EXP1_J5_8             40
#define EXP1_J5_6             41
/* i2c on EXP2 , SDA J5-7 , SCL J5-9*/
#define SDA_PIN 	            70
#define SCL_PIN 	            71

//** Expansion#3 port J19
#define EXP3_J19_5            73
#define EXP3_J19_9           101
#define EXP3_J19_7           102
#define EXP3_J19_3           103
#define EXP3_J19_14          104
#define EXP3_J19_12          105
#define EXP3_J19_10          106
#define EXP3_J19_8           107
#define EXP3_J19_6           110
#define EXP3_J19_4            13
#define EXP3_J19_2            72
#define EXP3_J19_1            28

/** Display **/

// GLCD on expansion port
#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)

  #define LCD_PINS_RS         18
  #define LCD_PINS_ENABLE     15
  #define LCD_PINS_D4         19
  #define ORIG_BEEPER_PIN     64

  #define BTN_EN1             14
  #define BTN_EN2             16
  #define BTN_ENC             17

  #if UI_VOLTAGE_LEVEL != 1
    #undef UI_VOLTAGE_LEVEL
    #define UI_VOLTAGE_LEVEL  1
  #endif

#endif // REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

#if NUM_SERVOS > 0
  #define SERVO0_PIN          36
  #if NUM_SERVOS > 1
    #define SERVO1_PIN        40
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      41
      #if NUM_SERVOS > 3
        #define SERVO3_PIN    -1
      #endif
    #endif
  #endif
#endif
