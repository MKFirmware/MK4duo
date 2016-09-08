/****************************************************************************************
* 83
* SAV MkI pin assignments (AT90USB1286)
* Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
* http://www.pjrc.com/teensy/teensyduino.html
* RepRap Clone Wars project board.
****************************************************************************************/

#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true


#define ORIG_X_STEP_PIN         0
#define ORIG_X_DIR_PIN          1
#define ORIG_X_ENABLE_PIN       39

#define ORIG_Y_STEP_PIN         2
#define ORIG_Y_DIR_PIN          3
#define ORIG_Y_ENABLE_PIN       38

#define ORIG_Z_STEP_PIN         4
#define ORIG_Z_DIR_PIN          5
#define ORIG_Z_ENABLE_PIN       23

#define ORIG_E0_STEP_PIN         6
#define ORIG_E0_DIR_PIN          7
#define ORIG_E0_ENABLE_PIN       19

#define ORIG_HEATER_0_PIN       21  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     20  // Bed
#define ORIG_FAN_PIN            16  // Fan   -- from Teensyduino environment.
                                    // For the fan and Teensyduino uses a different pin mapping.
#define X_STOP_PIN              13
#define Y_STOP_PIN              14
#define Z_STOP_PIN              15
//#define Z_STOP_PIN            36  // For inductive sensor.

#define ORIG_TEMP_0_PIN          7  // Extruder / Analog pin numbering
#define ORIG_TEMP_BED_PIN        6  // Bed / Analog pin numbering

#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1

#define SDPOWER                 -1
#define SDSS                    20  // PB0 - 8 in marlin env.
#define LED_PIN                 -1
#define ORIG_PS_ON_PIN          -1
#define ALARM_PIN               -1
#define SD_DETECT_PIN           -1

#define ORIG_BEEPER_PIN         -1
#define LCD_PINS_RS             -1
#define LCD_PINS_ENABLE         -1
#define LCD_PINS_D4             -1
#define LCD_PINS_D5             -1
#define LCD_PINS_D6             -1
#define LCD_PINS_D7             -1

#if ENABLED(SAV_3DLCD)
// For LCD SHIFT register LCD
#define SR_DATA_PIN         1
#define SR_CLK_PIN          0

#define BTN_EN1            41
#define BTN_EN2            40
#define BTN_ENC            12

#define KILL_PIN           42 // A2 = 42 - teensy = 40
#define HOME_PIN          -1 // A4 = marlin 44 - teensy = 42

#if NUM_SERVOS > 0
  #define SERVO0_PIN       41 // In teensy's pin definition for pinMode (in Servo.cpp)
#endif

#endif

