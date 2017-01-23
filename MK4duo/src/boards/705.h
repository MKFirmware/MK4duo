/****************************************************************************************
* 705
* ULTRATRONICS
*****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "Ultratronics v1.0"

#ifndef ARDUINO_ARCH_SAM
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

// X AXIS
#define ORIG_X_STEP_PIN       35
#define ORIG_X_DIR_PIN        34
#define ORIG_X_ENABLE_PIN     37
#define ORIG_X_MIN_PIN        31
#define ORIG_X_MAX_PIN        30

// Y AXIS
#define ORIG_Y_STEP_PIN       22
#define ORIG_Y_DIR_PIN        23
#define ORIG_Y_ENABLE_PIN     33
#define ORIG_Y_MIN_PIN        12
#define ORIG_Y_MAX_PIN        11

// Z AXIS
#define ORIG_Z_STEP_PIN       25
#define ORIG_Z_DIR_PIN        26
#define ORIG_Z_ENABLE_PIN     24
#define ORIG_Z_MIN_PIN        29
#define ORIG_Z_MAX_PIN        28

// E0 AXIS
#define ORIG_E0_STEP_PIN      47
#define ORIG_E0_DIR_PIN       46
#define ORIG_E0_ENABLE_PIN    48

// E1 AXIS
#define ORIG_E1_STEP_PIN      44
#define ORIG_E1_DIR_PIN       36
#define ORIG_E1_ENABLE_PIN    45

// E2 AXIS
#define ORIG_E2_STEP_PIN      42
#define ORIG_E2_DIR_PIN       41
#define ORIG_E2_ENABLE_PIN    43

// E3 AXIS
#define ORIG_E3_STEP_PIN      39
#define ORIG_E3_DIR_PIN       38
#define ORIG_E3_ENABLE_PIN    40

#define SDPOWER               -1
#define SDSS                  59
#define SD_DETECT_PIN         60
#define LED_PIN               13

#define ORIG_FAN_PIN           6
#define ORIG_FAN2_PIN          5

#define ORIG_PS_ON_PIN        32
#define KILL_PIN              -1
#define SUICIDE_PIN           -1

#define ORIG_HEATER_BED_PIN    2
#define ORIG_HEATER_0_PIN      3
#define ORIG_HEATER_1_PIN      8
#define ORIG_HEATER_2_PIN      7
#define ORIG_HEATER_3_PIN      9

#define ORIG_TEMP_BED_PIN      1  // ANALOG NUMBERING
#define ORIG_TEMP_0_PIN        0  // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN        2  // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN        3  // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN        4  // ANALOG NUMBERING

// SPI for Max6675 or Max31855 Thermocouple
#define MAX31855_SS0          65
#define MAX31855_SS1          52
#define MAX31855_SS2          50
#define MAX31855_SS3          51

#define ENC424_SS             61 

#define ORIG_BEEPER_PIN       27

#if NUM_SERVOS > 0
  #define SERVO0_PIN          11
  #if NUM_SERVOS > 1
    #define SERVO1_PIN        12
  #endif
#endif

#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #define LCD_PINS_RS         62  // CS chip select / SS chip slave select
  #define LCD_PINS_ENABLE     75  // SID (MOSI)
  #define LCD_PINS_D4         76  // SCK (CLK) clock
  #define ORIG_BEEPER_PIN     -1

  #define BTN_EN1             20
  #define BTN_EN2             21
  #define BTN_ENC             64

#endif // REPRAPWORLD_GRAPHICAL_LCD
