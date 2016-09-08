/****************************************************************************************
* 502
* Alligator R2
* http://www.3dartists.org/
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "Alligator R2"

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Alligator 3D Printer Board' selected from the 'Tools -> Boards' menu.
#endif

#define ALLIGATOR
#define SPI_CHAN_DAC 1

// X AXIS
#define ORIG_X_STEP_PIN       96  // PB24
#define ORIG_X_DIR_PIN         2  // PB25
#define ORIG_X_ENABLE_PIN     24  // PA15, motor RESET pin
#define ORIG_X_MIN_PIN        33  // PC1
#define ORIG_X_MAX_PIN        34  // PC2
#define X_MS1_PIN             99  // PC10

// Y AXIS
#define ORIG_Y_STEP_PIN       94  // PB22
#define ORIG_Y_DIR_PIN        95  // PB23
#define ORIG_Y_ENABLE_PIN     24  // PA15, motor RESET pin
#define ORIG_Y_MIN_PIN        35  // PC3
#define ORIG_Y_MAX_PIN        37  // PC5
#define Y_MS1_PIN             10  // PC29

// Z AXIS
#define ORIG_Z_STEP_PIN       98  // PC27
#define ORIG_Z_DIR_PIN         3  // PC28
#define ORIG_Z_ENABLE_PIN     24  // PA15, motor RESET pin
#define ORIG_Z_MIN_PIN        38  // PC6
#define ORIG_Z_MAX_PIN        39  // PC7
#define Z_MS1_PIN             44  // PC19

// E0 AXIS
#define ORIG_E0_STEP_PIN       5  // PC25
#define ORIG_E0_DIR_PIN        4  // PC26
#define ORIG_E0_ENABLE_PIN    24  // PA15, motor RESET pin
#define E0_MS1_PIN            45  // PC18

// E1 AXIS
#define ORIG_E1_STEP_PIN      28  // PD3 on piggy
#define ORIG_E1_DIR_PIN       27  // PD2 on piggy
#define ORIG_E1_ENABLE_PIN    24  // PA15, motor RESET pin

// E2 AXIS
#define ORIG_E2_STEP_PIN      11 // PD7 on piggy
#define ORIG_E2_DIR_PIN       29 // PD6 on piggy
#define ORIG_E2_ENABLE_PIN    24 // PA15, motor RESET pin

// E3 AXIS
#define ORIG_E3_STEP_PIN      30 // PD9 on piggy
#define ORIG_E3_DIR_PIN       12 // PD8 on piggy
#define ORIG_E3_ENABLE_PIN    24 // PA15, motor RESET pin

#define MOTOR_FAULT_PIN       22 // PB26 , motor X-Y-Z-E0 motor FAULT

#define SDPOWER               -1
#define SDSS                  77 // PA28
#define SD_DETECT_PIN         87 // PA29
#define LED_PIN               -1

#define ORIG_FAN_PIN          92 // PA5
#define ORIG_FAN2_PIN         31 // PA7

#define ORIG_PS_ON_PIN        -1
#define KILL_PIN              -1
#define SUICIDE_PIN           -1 //PIN that has to be turned on right after start, to keep power flowing.

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define ORIG_HEATER_BED_PIN   69 // PA0
#define ORIG_HEATER_0_PIN     68 // PA1
#define ORIG_HEATER_1_PIN      8 // PC22 on piggy
#define ORIG_HEATER_2_PIN      9 // PC21 on piggy
#define ORIG_HEATER_3_PIN     97 // PC20 on piggy

#define ORIG_TEMP_BED_PIN      0 // PA16
#define ORIG_TEMP_0_PIN        1 // PA24, analog pin
#define ORIG_TEMP_1_PIN        2 // PA23 analog pin on piggy
#define ORIG_TEMP_2_PIN        3 // PA22, analog pin on piggy
#define ORIG_TEMP_3_PIN        4 // PA6, analog on piggy

#define LED_PWM1_PIN          36 // PC4
#define LED_PWM2_PIN          40 // PC8
#define LED_PWM3_PIN          41 // PC9

#define EXP_VOLTAGE_LEVEL_PIN 65

#define DAC0_SYNC             53 // PB14
#define DAC1_SYNC              6 // PC24

//64K SPI EEPROM
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25 // PD0

//2K SPI EEPROM
#define SPI_EEPROM2_CS        26 // PD1

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS          23 //PA14

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

#endif //REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

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
