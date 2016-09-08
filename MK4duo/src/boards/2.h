/****************************************************************************************
* 2
* Cheaptronic v1.0
****************************************************************************************/

#define KNOWN_BOARD 1
#define BOARD_NAME "Cheaptronic v1.0"

#ifndef __AVR_ATmega2560__
  #error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

//X motor stepper
#define ORIG_X_STEP_PIN 14
#define ORIG_X_DIR_PIN 15
#define ORIG_X_ENABLE_PIN 24

//X endstop
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN -1

//Y motor stepper
#define ORIG_Y_STEP_PIN 35
#define ORIG_Y_DIR_PIN 36
#define ORIG_Y_ENABLE_PIN 31

//Y endstop
#define ORIG_Y_MIN_PIN 2
#define ORIG_Y_MAX_PIN -1

//Z motor stepper
#define ORIG_Z_STEP_PIN 40
#define ORIG_Z_DIR_PIN 41
#define ORIG_Z_ENABLE_PIN 37

//Z endstop
#define ORIG_Z_MIN_PIN 5
#define ORIG_Z_MAX_PIN -1

//Extruder 0 stepper
#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 25

//Extruder 1 stepper
#define ORIG_E1_STEP_PIN 33
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30

#define SDPOWER -1
#define SDSS -1
#define LED_PIN -1

//FAN
#define ORIG_FAN_PIN -1

#define ORIG_PS_ON_PIN -1
#define KILL_PIN -1

#define ORIG_HEATER_0_PIN 19 // EXTRUDER 1
#define ORIG_HEATER_1_PIN 23 // EXTRUDER 2
//HeatedBad
#define ORIG_HEATER_BED_PIN 22
//Cheaptronic v1.0 hasent EXTRUDER 3
#define ORIG_HEATER_2_PIN -1

//Temperature sensors
#define ORIG_TEMP_0_PIN 15
#define ORIG_TEMP_1_PIN 14
#define ORIG_TEMP_2_PIN -1
#define ORIG_TEMP_BED_PIN 13

//Cheaptronic v1.0 dont support LCD
#define LCD_PINS_RS -1
#define LCD_PINS_ENABLE -1
#define LCD_PINS_D4 -1
#define LCD_PINS_D5 -1
#define LCD_PINS_D6 -1
#define LCD_PINS_D7 -1

//Cheaptronic v1.0 dont support keypad
#define BTN_EN1 -1
#define BTN_EN2 -1
#define BTN_ENC -1
#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

//Cheaptronic v1.0 does not use this port
#define SD_DETECT_PIN -1
