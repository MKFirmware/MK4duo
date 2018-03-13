/****************************************************************************************
* 39
* SAINSMART 2 IN 1
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega1280__) && DISABLED(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Sainsmart"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN            54
#define ORIG_X_DIR_PIN             55
#define ORIG_X_ENABLE_PIN          38
#define ORIG_X_CS_PIN              NoPin

//###Y_AXIS
#define ORIG_Y_STEP_PIN            60
#define ORIG_Y_DIR_PIN             61
#define ORIG_Y_ENABLE_PIN          56
#define ORIG_Y_CS_PIN              NoPin

//###Z_AXIS
#define ORIG_Z_STEP_PIN            46
#define ORIG_Z_DIR_PIN             48
#define ORIG_Z_ENABLE_PIN          62
#define ORIG_Z_CS_PIN              NoPin

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN           26
#define ORIG_E0_DIR_PIN            28
#define ORIG_E0_ENABLE_PIN         24
#define ORIG_E0_CS_PIN             NoPin
#define ORIG_SOL0_PIN              NoPin

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN           36
#define ORIG_E1_DIR_PIN            34
#define ORIG_E1_ENABLE_PIN         30
#define ORIG_E1_CS_PIN             NoPin
#define ORIG_SOL1_PIN              NoPin

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN           64
#define ORIG_E2_DIR_PIN            59
#define ORIG_E2_ENABLE_PIN         44
#define ORIG_E2_CS_PIN             NoPin
#define ORIG_SOL2_PIN              NoPin

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN           40
#define ORIG_E3_DIR_PIN            63
#define ORIG_E3_ENABLE_PIN         42
#define ORIG_E3_CS_PIN             NoPin
#define ORIG_SOL3_PIN              NoPin

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN           NoPin
#define ORIG_E4_DIR_PIN            NoPin
#define ORIG_E4_ENABLE_PIN         NoPin
#define ORIG_E4_CS_PIN             NoPin
#define ORIG_SOL4_PIN              NoPin

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN           NoPin
#define ORIG_E5_DIR_PIN            NoPin
#define ORIG_E5_ENABLE_PIN         NoPin
#define ORIG_E5_CS_PIN             NoPin
#define ORIG_SOL5_PIN              NoPin

//###EXTRUDER_6
#define ORIG_E6_STEP_PIN           NoPin
#define ORIG_E6_DIR_PIN            NoPin
#define ORIG_E6_ENABLE_PIN         NoPin
#define ORIG_E6_CS_PIN             NoPin
#define ORIG_SOL6_PIN              NoPin

//###EXTRUDER_7
#define ORIG_E7_STEP_PIN           NoPin
#define ORIG_E7_DIR_PIN            NoPin
#define ORIG_E7_ENABLE_PIN         NoPin
#define ORIG_E7_CS_PIN             NoPin
#define ORIG_SOL7_PIN              NoPin

//###ENDSTOP
#define ORIG_X_MIN_PIN              3
#define ORIG_X_MAX_PIN              2
#define ORIG_Y_MIN_PIN             14
#define ORIG_Y_MAX_PIN             15
#define ORIG_Z_MIN_PIN             18
#define ORIG_Z_MAX_PIN             19
#define ORIG_Z2_MIN_PIN            NoPin
#define ORIG_Z2_MAX_PIN            NoPin
#define ORIG_Z3_MIN_PIN            NoPin
#define ORIG_Z3_MAX_PIN            NoPin
#define ORIG_Z4_MIN_PIN            NoPin
#define ORIG_Z4_MAX_PIN            NoPin
#define ORIG_E_MIN_PIN             NoPin
#define ORIG_Z_PROBE_PIN           NoPin

//###SINGLE_ENDSTOP
#define X_STOP_PIN                 NoPin
#define Y_STOP_PIN                 NoPin
#define Z_STOP_PIN                 NoPin

//###HEATER
#define ORIG_HEATER_0_PIN           9
#define ORIG_HEATER_1_PIN          NoPin
#define ORIG_HEATER_2_PIN          NoPin
#define ORIG_HEATER_3_PIN          NoPin
#define ORIG_HEATER_BED_PIN         8
#define ORIG_HEATER_CHAMBER_PIN    NoPin
#define ORIG_COOLER_PIN            NoPin

//###TEMPERATURE
#define ORIG_TEMP_0_PIN            13
#define ORIG_TEMP_1_PIN            15
#define ORIG_TEMP_2_PIN            NoPin
#define ORIG_TEMP_3_PIN            NoPin
#define ORIG_TEMP_BED_PIN          14
#define ORIG_TEMP_CHAMBER_PIN      NoPin
#define ORIG_TEMP_COOLER_PIN       NoPin

//###FAN
#define ORIG_FAN0_PIN               7
#define ORIG_FAN1_PIN              NoPin
#define ORIG_FAN2_PIN              NoPin
#define ORIG_FAN3_PIN              NoPin

//###SERVO
#define SERVO0_PIN                 11
#define SERVO1_PIN                  6
#define SERVO2_PIN                  5
#define SERVO3_PIN                  4

//###MISC
#define ORIG_PS_ON_PIN             12
#define ORIG_BEEPER_PIN            NoPin
#define LED_PIN                    13
#define SDPOWER_PIN                NoPin
#define SD_DETECT_PIN              NoPin
#define SDSS                       53
#define KILL_PIN                   NoPin
#define DEBUG_PIN                  NoPin
#define SUICIDE_PIN                NoPin

//###LASER
#define ORIG_LASER_PWR_PIN         NoPin
#define ORIG_LASER_PWM_PIN         NoPin

//###FLOWMETER
#define ORIG_FLOMETER_PIN          NoPin


//###UNKNOWN_PINS
#define MAX6675_SS_PIN              66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present
//@@@

//###IF_BLOCKS
#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN              41
#else
  #define KILL_PIN              NoPin
#endif

#if ENABLED(ULTRA_LCD)
  #if ENABLED(NEWPANEL)
    #if ENABLED(PANEL_ONE)
      #define LCD_PINS_RS       40
      #define LCD_PINS_ENABLE   42
      #define LCD_PINS_D4       65
      #define LCD_PINS_D5       66
      #define LCD_PINS_D6       44
      #define LCD_PINS_D7       64
    #else
      #define LCD_PINS_RS       16
      #define LCD_PINS_ENABLE   17
      #define LCD_PINS_D4       23
      #define LCD_PINS_D5       25
      #define LCD_PINS_D6       27
      #define LCD_PINS_D7       29
    #endif // PANEL_ONE

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define ORIG_BEEPER_PIN   37

      #define BTN_EN1           31
      #define BTN_EN2           33
      #define BTN_ENC           35

      #define SD_DETECT_PIN     49
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1           47  // reverse if the encoder turns the wrong way.
      #define BTN_EN2           43
      #define BTN_ENC           32
      #define LCD_SDSS          53
      #define SD_DETECT_PIN     NoPin
      #define KILL_PIN          41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1           22  // reverse if the encoder turns the wrong way.
      #define BTN_EN2           7
      #define BTN_ENC           NoPin
      #define LCD_SDSS          53
      #define SD_DETECT_PIN     49
    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1           35  // reverse if the encoder turns the wrong way.
      #define BTN_EN2           37
      #define BTN_ENC           31
      #define SD_DETECT_PIN     49
      #define LCD_SDSS          53
      #define KILL_PIN          41
      #define ORIG_BEEPER_PIN   23
      #define DOGLCD_CS         29
      #define DOGLCD_A0         27
      #define LCD_BACKLIGHT_PIN 33
    #else
      // arduino pin which triggers an piezzo beeper
      #define ORIG_BEEPER_PIN   33  // Beeper on AUX-4

      // buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1         64  // encoder
        #define BTN_EN2         59  // encoder
        #define BTN_ENC         63  // enter button
        #define SHIFT_OUT       40  // shift register
        #define SHIFT_CLK       44  // shift register
        #define SHIFT_LD        42  // shift register
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1         59  // AUX2 PIN 3
        #define BTN_EN2         63  // AUX2 PIN 4
        #define BTN_ENC         49  // AUX3 PIN 7
      #else
        #define BTN_EN1         37
        #define BTN_EN2         35
        #define BTN_ENC         31  // the click
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN   49
      #else
        #define SD_DETECT_PIN   NoPin  // Ramps does not use this port
      #endif

    #endif

  #else // old style panel with shift register
    // arduino pin witch triggers an piezo beeper
    #define ORIG_BEEPER_PIN     33  // No Beeper added

    //buttons are attached to a shift register
    // Not wired this yet
    //#define SHIFT_CLK         38
    //#define SHIFT_LD          42
    //#define SHIFT_OUT         40
    //#define SHIFT_EN          17

    #define LCD_PINS_RS         16
    #define LCD_PINS_ENABLE     17
    #define LCD_PINS_D4         23
    #define LCD_PINS_D5         25
    #define LCD_PINS_D6         27
    #define LCD_PINS_D7         29
  #endif // NEWPANEL
#endif // ULTRA_LCD
//@@@
