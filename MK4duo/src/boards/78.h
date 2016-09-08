/****************************************************************************************
* 78
* K8200
****************************************************************************************/

#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN           2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define SDPOWER                 -1
#define SDSS                    25
#define LED_PIN                 13


#define ORIG_FAN_PIN             8 // IO pin. Buffer needed

#define ORIG_PS_ON_PIN          12

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN               41
#else
  #define KILL_PIN               -1
#endif

#define ORIG_HEATER_0_PIN        10  // HOTEND 1
#define ORIG_HEATER_1_PIN        12  // HOTEND 2
#define ORIG_HEATER_2_PIN         6  // HOTEND 3
#define ORIG_HEATER_3_PIN        -1

#define ORIG_TEMP_0_PIN          13   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN          15   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN          -1   // ANALOG NUMBERING

#define ORIG_HEATER_BED_PIN       9   // NO BED

#define ORIG_TEMP_BED_PIN        14   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN             11
  #if NUM_SERVOS > 1
    #define SERVO1_PIN            6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN          5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN        4
      #endif
    #endif
  #endif
#endif

#define ORIG_BEEPER_PIN           33

#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define ORIG_BEEPER_PIN -1

  #define LCD_PINS_RS 27
  #define LCD_PINS_ENABLE 29
  #define LCD_PINS_D4 37
  #define LCD_PINS_D5 35
  #define LCD_PINS_D6 33
  #define LCD_PINS_D7 31

  // Buttons
  #define BTN_EN1 16
  #define BTN_EN2 17
  #define BTN_ENC 23 //the click
#endif // ULTRA_LCD && NEWPANEL

// SPI for Max6675 Thermocouple
#define MAX6675_SS          66  // Do not use pin 53 if there is even the remote possibility of using Display/SD card


