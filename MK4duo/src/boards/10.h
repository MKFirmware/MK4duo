/******************************************************************************
* 10
* Gen7 Alfons
* These Pins are assigned for the modified GEN7
* Board from Alfons3 Please review the pins and adjust it for your needs
******************************************************************************/

  #define KNOWN_BOARD

  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops!  Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
  #endif

  //x axis pins
  #define ORIG_X_STEP_PIN       21  // different from standard GEN7
  #define ORIG_X_DIR_PIN        20  // different from standard GEN7
  #define ORIG_X_ENABLE_PIN     24
  #define X_STOP_PIN             0

  //y axis pins
  #define ORIG_Y_STEP_PIN       23
  #define ORIG_Y_DIR_PIN        22
  #define ORIG_Y_ENABLE_PIN     24
  #define Y_STOP_PIN             1

  //z axis pins
  #define ORIG_Z_STEP_PIN       26
  #define ORIG_Z_DIR_PIN        25
  #define ORIG_Z_ENABLE_PIN     24
  #define Z_STOP_PIN             2

  //extruder pins
  #define ORIG_E0_STEP_PIN      28
  #define ORIG_E0_DIR_PIN       27
  #define ORIG_E0_ENABLE_PIN    24

  #define ORIG_TEMP_0_PIN        2
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_TEMP_BED_PIN      1  // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)

  #define ORIG_HEATER_0_PIN      4
  #define ORIG_HEATER_1_PIN     -1
  #define ORIG_HEATER_2_PIN     -1
  #define ORIG_HEATER_BED_PIN    3  // (bed)

  #define SDPOWER               -1
  #define SDSS                  31  // SCL pin of I2C header || CS Pin for SD Card support
  #define LED_PIN               -1

  #define ORIG_FAN_PIN          -1
  #define ORIG_PS_ON_PIN        19

  //our pin for debugging.
  #define DEBUG_PIN             -1

  //our RS485 pins
  //#define TORIG_X_ENABLE_PIN  12
  //#define RORIG_X_ENABLE_PIN  13

  #define ORIG_BEEPER_PIN  -1
  #define SD_DETECT_PIN    -1
  #define SUICIDE_PIN      -1  //has to be defined; otherwise Power_off doesn't work

  #define KILL_PIN         -1
  //Pins for 4bit LCD Support
  #define LCD_PINS_RS      18
  #define LCD_PINS_ENABLE  17
  #define LCD_PINS_D4      16
  #define LCD_PINS_D5      15
  #define LCD_PINS_D6      13
  #define LCD_PINS_D7      14

  //buttons are directly attached
  #define BTN_EN1          11
  #define BTN_EN2          10
  #define BTN_ENC          12  //the click


