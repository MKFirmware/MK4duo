/****************************************************************************************
* 100
* ANET board A2, A6 and A8
****************************************************************************************/

//###CHIP
#if DISABLED(__AVR_ATmega644P__) && DISABLED(__AVR_ATmega1284P__)
  #error Oops!  Make sure you have 'Sanguino' or 'Anet' selected from the 'Tools -> Boards' menu.
#endif
//@@@

#define KNOWN_BOARD 1

//###BOARD_NAME
#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Anet"
#endif
//@@@


//###X_AXIS
#define ORIG_X_STEP_PIN 15
#define ORIG_X_DIR_PIN 21
#define ORIG_X_ENABLE_PIN 14
#define ORIG_X_CS_PIN -1

//###Y_AXIS
#define ORIG_Y_STEP_PIN 22
#define ORIG_Y_DIR_PIN 23
#define ORIG_Y_ENABLE_PIN 14
#define ORIG_Y_CS_PIN -1

//###Z_AXIS
#define ORIG_Z_STEP_PIN 3
#define ORIG_Z_DIR_PIN 2
#define ORIG_Z_ENABLE_PIN 26
#define ORIG_Z_CS_PIN -1

//###EXTRUDER_0
#define ORIG_E0_STEP_PIN 1
#define ORIG_E0_DIR_PIN 0
#define ORIG_E0_ENABLE_PIN 14
#define ORIG_E0_CS_PIN -1
#define ORIG_SOL0_PIN -1

//###EXTRUDER_1
#define ORIG_E1_STEP_PIN -1
#define ORIG_E1_DIR_PIN -1
#define ORIG_E1_ENABLE_PIN -1
#define ORIG_E1_CS_PIN -1
#define ORIG_SOL1_PIN -1

//###EXTRUDER_2
#define ORIG_E2_STEP_PIN -1
#define ORIG_E2_DIR_PIN -1
#define ORIG_E2_ENABLE_PIN -1
#define ORIG_E2_CS_PIN -1
#define ORIG_SOL2_PIN -1

//###EXTRUDER_3
#define ORIG_E3_STEP_PIN -1
#define ORIG_E3_DIR_PIN -1
#define ORIG_E3_ENABLE_PIN -1
#define ORIG_E3_CS_PIN -1
#define ORIG_SOL3_PIN -1

//###EXTRUDER_4
#define ORIG_E4_STEP_PIN -1
#define ORIG_E4_DIR_PIN -1
#define ORIG_E4_ENABLE_PIN -1
#define ORIG_E4_CS_PIN -1
#define ORIG_SOL4_PIN -1

//###EXTRUDER_5
#define ORIG_E5_STEP_PIN -1
#define ORIG_E5_DIR_PIN -1
#define ORIG_E5_ENABLE_PIN -1
#define ORIG_E5_CS_PIN -1
#define ORIG_SOL5_PIN -1

//###EXTRUDER_6
#define ORIG_E6_STEP_PIN -1
#define ORIG_E6_DIR_PIN -1
#define ORIG_E6_ENABLE_PIN -1
#define ORIG_E6_CS_PIN -1
#define ORIG_SOL6_PIN -1

//###EXTRUDER_7
#define ORIG_E7_STEP_PIN -1
#define ORIG_E7_DIR_PIN -1
#define ORIG_E7_ENABLE_PIN -1
#define ORIG_E7_CS_PIN -1
#define ORIG_SOL7_PIN -1

//###ENDSTOP
#define ORIG_X_MIN_PIN -1
#define ORIG_X_MAX_PIN -1
#define ORIG_Y_MIN_PIN -1
#define ORIG_Y_MAX_PIN -1
#define ORIG_Z_MIN_PIN -1
#define ORIG_Z_MAX_PIN -1
#define ORIG_Z2_MIN_PIN -1
#define ORIG_Z2_MAX_PIN -1
#define ORIG_Z3_MIN_PIN -1
#define ORIG_Z3_MAX_PIN -1
#define ORIG_Z4_MIN_PIN -1
#define ORIG_Z4_MAX_PIN -1
#define ORIG_E_MIN_PIN -1
#define ORIG_Z_PROBE_PIN -1

//###SINGLE_ENDSTOP
#define X_STOP_PIN 18
#define Y_STOP_PIN 19
#define Z_STOP_PIN 20

//###HEATER
#define ORIG_HEATER_0_PIN 13
#define ORIG_HEATER_1_PIN -1
#define ORIG_HEATER_2_PIN -1
#define ORIG_HEATER_3_PIN -1
#define ORIG_HEATER_BED_PIN 12
#define ORIG_HEATER_CHAMBER_PIN -1
#define ORIG_COOLER_PIN -1

//###TEMPERATURE
#define ORIG_TEMP_0_PIN 7
#define ORIG_TEMP_1_PIN -1
#define ORIG_TEMP_2_PIN -1
#define ORIG_TEMP_3_PIN -1
#define ORIG_TEMP_BED_PIN 6
#define ORIG_TEMP_CHAMBER_PIN -1
#define ORIG_TEMP_COOLER_PIN -1

//###FAN
#define ORIG_FAN0_PIN 4
#define ORIG_FAN1_PIN -1
#define ORIG_FAN2_PIN -1
#define ORIG_FAN3_PIN -1

//###MISC
#define ORIG_PS_ON_PIN -1
#define ORIG_BEEPER_PIN -1
#define LED_PIN -1
#define SDPOWER -1
#define SD_DETECT_PIN -1
#define SDSS 31
#define KILL_PIN -1
#define DEBUG_PIN -1
#define SUICIDE_PIN -1

//###LASER
#define ORIG_LASER_PWR_PIN -1
#define ORIG_LASER_PWM_PIN -1

//###SERVOS
#if NUM_SERVOS > 0
  #define SERVO0_PIN -1
  #if NUM_SERVOS > 1
    #define SERVO1_PIN -1
    #if NUM_SERVOS > 2
      #define SERVO2_PIN -1
      #if NUM_SERVOS > 3
        #define SERVO3_PIN -1
      #endif
    #endif
  #endif
#endif
//@@@


//###IF_BLOCKS
#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)

	#if ENABLED(ADC_KEYPAD)
		#undef BTN_EN1
		#undef BTN_EN2
		#undef BTN_ENC

		#define LCD_PINS_RS         28
		#define LCD_PINS_ENABLE     29
		#define LCD_PINS_D4         10
		#define LCD_PINS_D5         11
		#define LCD_PINS_D6         16
		#define LCD_PINS_D7         17

		#define BTN_EN1 			      -1
		#define BTN_EN2 			      -1
		#define BTN_ENC 			      -1

    #define ADC_KEYPAD_PIN       1

		#define ENCODER_FEEDRATE_DEADZONE 2
   
      #ifndef ENCODER_STEPS_PER_MENU_ITEM
      #define ENCODER_STEPS_PER_MENU_ITEM 1
    #endif
    #ifndef ENCODER_PULSES_PER_STEP
      #define ENCODER_PULSES_PER_STEP 1
    #endif
    
	#elif ENABLED(U8GLIB_ST7920)

    #define DOGLCD_A0           30
    #define DOGLCD_CS           29
    #define LCD_CONTRAST         1

    #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
      // For RepRap Discount (with Anet Adapter wiring)
      #define SERVO0_PIN        27 // free for BLTouch/3D-Touch
      #define BEEPER_PIN        28
      #define LCD_PINS_RS       30
      #define	LCD_PINS_ENABLE   29
      #define LCD_PINS_D4       17
      #define BTN_EN1           11
      #define BTN_EN2           10
      #define BTN_ENC           16
    #elif ENABLED(ANET_FULL_GRAPHICS_LCD)
      #define BEEPER_PIN 17

      #define LCD_PINS_RS       27
      #define LCD_PINS_ENABLE   28
      #define LCD_PINS_D4       30

      #define BTN_EN1           11
      #define BTN_EN2           10
      #define BTN_ENC           16
    #else
      #error "You need to select ANET or RepRap Version"
    #endif

		#define ST7920_DELAY_1 DELAY_0_NOP
		#define ST7920_DELAY_2 DELAY_1_NOP
		#define ST7920_DELAY_3 DELAY_2_NOP

		#ifndef ENCODER_STEPS_PER_MENU_ITEM
			#define ENCODER_STEPS_PER_MENU_ITEM 1
		#endif
		#ifndef ENCODER_PULSES_PER_STEP
			#define ENCODER_PULSES_PER_STEP 4
		#endif
	#endif

#endif
//@@@
