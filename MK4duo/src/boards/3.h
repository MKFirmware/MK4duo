/****************************************************************************************
* 3
* RAMPS OLD
****************************************************************************************/

#define KNOWN_BOARD 1
#define BOARD_NAME "RAMPS OLD"

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

// Uncomment the following line for RAMPS v1.0
//#define RAMPS_V_1_0

#define ORIG_X_STEP_PIN         26
#define ORIG_X_DIR_PIN          28
#define ORIG_X_ENABLE_PIN       24
#define ORIG_X_MIN_PIN          3
#define ORIG_X_MAX_PIN          2

#define ORIG_Y_STEP_PIN         38
#define ORIG_Y_DIR_PIN          40
#define ORIG_Y_ENABLE_PIN       36
#define ORIG_Y_MIN_PIN          16
#define ORIG_Y_MAX_PIN          17

#define ORIG_Z_STEP_PIN         44
#define ORIG_Z_DIR_PIN          46
#define ORIG_Z_ENABLE_PIN       42
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN        32
#define ORIG_E0_DIR_PIN         34
#define ORIG_E0_ENABLE_PIN      30

#define SDPOWER                 48
#define SDSS                    53
#define LED_PIN                 13
#define ORIG_PS_ON_PIN          -1
#define KILL_PIN                -1

#if ENABLED(RAMPS_V_1_0) // RAMPS_V_1_0
  #define ORIG_HEATER_0_PIN     12    // RAMPS 1.0
  #define ORIG_HEATER_BED_PIN   -1    // RAMPS 1.0
  #define ORIG_FAN_PIN          11    // RAMPS 1.0
#else // RAMPS_V_1_1 or RAMPS_V_1_2
  #define ORIG_HEATER_0_PIN     10    // RAMPS 1.1
  #define ORIG_HEATER_BED_PIN    8    // RAMPS 1.1
  #define ORIG_FAN_PIN           9    // RAMPS 1.1
#endif

#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_TEMP_0_PIN          2    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1
#define ORIG_TEMP_BED_PIN        1    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!

// SPI for Max6675 Thermocouple
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS            66  // Do not use pin 53 if there is even the remote possibility of using Display/SD card
#else
  #define MAX6675_SS            66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present
#endif
