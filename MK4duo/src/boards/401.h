/****************************************************************************************
* 401
*
* RADDS
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "RADDS"

#ifndef __SAM3X8E__
  #error Oops! Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define RADDS

#define ORIG_X_STEP_PIN         24
#define ORIG_X_DIR_PIN          23
#define ORIG_X_ENABLE_PIN       26

#define ORIG_Y_STEP_PIN         17
#define ORIG_Y_DIR_PIN          16
#define ORIG_Y_ENABLE_PIN       22

#define ORIG_Z_STEP_PIN          2
#define ORIG_Z_DIR_PIN           3
#define ORIG_Z_ENABLE_PIN       15

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define ORIG_X_MIN_PIN          28
#define ORIG_X_MAX_PIN          34
#define ORIG_Y_MIN_PIN          30
#define ORIG_Y_MAX_PIN          36
#define ORIG_Z_MIN_PIN          32
#define ORIG_Z_MAX_PIN          38

#define ORIG_E0_STEP_PIN        61
#define ORIG_E0_DIR_PIN         60
#define ORIG_E0_ENABLE_PIN      62

#define ORIG_E1_STEP_PIN        64
#define ORIG_E1_DIR_PIN         63
#define ORIG_E1_ENABLE_PIN      65

#define ORIG_E2_STEP_PIN        51
#define ORIG_E2_DIR_PIN         53
#define ORIG_E2_ENABLE_PIN      49

#define ORIG_E3_STEP_PIN        35
#define ORIG_E3_DIR_PIN         33
#define ORIG_E3_ENABLE_PIN      37

#define ORIG_E4_STEP_PIN        29
#define ORIG_E4_DIR_PIN         27
#define ORIG_E4_ENABLE_PIN      31

#define SDPOWER                 -1
#define SDSS                     4
#define LED_PIN                 -1

#define ORIG_BEEPER_PIN         41

#define ORIG_FAN_PIN 	           9
#define ORIG_FAN2_PIN            8

#define ORIG_PS_ON_PIN          40

#define KILL_PIN                -1

#define ORIG_HEATER_BED_PIN      7  // BED
#define ORIG_HEATER_0_PIN       13
#define ORIG_HEATER_1_PIN       12
#define ORIG_HEATER_2_PIN       11

#define ORIG_TEMP_BED_PIN        4  // ANALOG NUMBERING #57
#define ORIG_TEMP_0_PIN          0  // ANALOG NUMBERING #54
#define ORIG_TEMP_1_PIN          1  // ANALOG NUMBERING #58
#define ORIG_TEMP_2_PIN          2  // ANALOG NUMBERING #55
#define ORIG_TEMP_3_PIN          3  // ANALOG NUMBERING #56

// #define THERMOCOUPLE_0_PIN  2   // Dua analog pin #59 = A5 -> AD 2

// http://doku.radds.org/wp-content/uploads/2015/02/RADDS_Wiring.png

#if NUM_SERVOS > 0
  #define SERVO0_PIN           5
  #if NUM_SERVOS > 1
    #define SERVO1_PIN         6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      39
      #if NUM_SERVOS > 3
        #define SERVO3_PIN    40
      #endif
    #endif
  #endif
#endif

#if ENABLED(ULTRA_LCD)
  // RADDS LCD panel
  #if ENABLED(RADDS_DISPLAY)
    #define LCD_PINS_RS 		42
    #define LCD_PINS_ENABLE 43
    #define LCD_PINS_D4 		44
    #define LCD_PINS_D5 		45
    #define LCD_PINS_D6 		46
    #define LCD_PINS_D7 		47

    #define BEEPER          41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48

    #define BTN_BACK        71

    #undef SDSS
    #define SDSS            10 // 4,10,52 if using HW SPI.
    #define SD_DETECT_PIN   14

  #elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define LCD_PINS_RS     46
    #define LCD_PINS_ENABLE 47
    #define LCD_PINS_D4     44

    #define ORIG_BEEPER_PIN 41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48

  #elif ENABLED(SSD1306_OLED_I2C_CONTROLLER)
    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48
    #define BEEPER          41
    #define LCD_SDSS        10
    #define SD_DETECT_PIN   14
    #define KILL_PIN        -1

  #elif ENABLED(SPARK_FULL_GRAPHICS)
    #define LCD_PINS_D4     29
    #define LCD_PINS_ENABLE 27
    #define LCD_PINS_RS     25

    #define BTN_EN1         35
    #define BTN_EN2         33
    #define BTN_ENC         37

    #define KILL_PIN        -1
    #undef BEEPER
    #define BEEPER          -1
	#endif // SPARK_FULL_GRAPHICS
#endif // ULTRA_LCD



