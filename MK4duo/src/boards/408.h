/****************************************************************************************
* 408
* Arduino pin assignment
* for SMART_RAMPS
****************************************************************************************/

#define KNOWN_BOARD
#define BOARD_NAME "SMART RAMPS"

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__) && !defined(ARDUINO_ARCH_SAM)
  #error Oops! Make sure you have 'Arduino Due' or 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN       54
#define ORIG_X_DIR_PIN        55
#define ORIG_X_ENABLE_PIN     38
#define ORIG_X_MIN_PIN         3
#define ORIG_X_MAX_PIN         2

#define ORIG_Y_STEP_PIN       60
#define ORIG_Y_DIR_PIN        61
#define ORIG_Y_ENABLE_PIN     56
#define ORIG_Y_MIN_PIN        14
#define ORIG_Y_MAX_PIN        15

#define ORIG_Z_STEP_PIN       46
#define ORIG_Z_DIR_PIN        48
#define ORIG_Z_ENABLE_PIN     62
#define ORIG_Z_MIN_PIN        18
#define ORIG_Z_MAX_PIN        19

#define ORIG_HEATER_0_PIN     10
#define ORIG_HEATER_1_PIN      9
#define ORIG_HEATER_BED_PIN    8

#define ORIG_TEMP_0_PIN        9  // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN       10  // ANALOG NUMBERING
#define ORIG_TEMP_BED_PIN     11  // ANALOG NUMBERING

#define ORIG_E0_STEP_PIN      26
#define ORIG_E0_DIR_PIN       28
#define ORIG_E0_ENABLE_PIN    24

#define ORIG_E1_STEP_PIN      36
#define ORIG_E1_DIR_PIN       34
#define ORIG_E1_ENABLE_PIN    30

#define SDPOWER               -1
#define SDSS                  53  // 10 if using HW SPI. 53 if using SW SPI
#define LED_PIN               13
#define ORIG_FAN_PIN           9
#define ORIG_PS_ON_PIN        12
#define KILL_PIN              -1
#define SUICIDE_PIN           -1  // PIN that has to be turned on right after start, to keep power flowing

// SPI for Max6675 or Max31855 Thermocouple
#define MAX6675_SS            66 // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present


#if ENABLED(ULTRA_LCD)

  #if ENABLED(AZSMZ_12864_LCD)  
  
	#define DOGLCD_SCK 		  -1	 
	#define DOGLCD_MOSI		  -1

	#ifdef ARDUINO_ARCH_SAM
		#define DOGLCD_SCK 		  52	 
		#define DOGLCD_MOSI		  51  
	#endif
	
    #define DOGLCD_A0         59
    #define DOGLCD_CS         44
	#define LCD_SCREEN_ROT_180
	#define DEFAULT_LCD_CONTRAST 59
	
    #define ORIG_BEEPER_PIN 66

    #define BTN_EN1         40
    #define BTN_EN2         58
    #define BTN_ENC         67

    #define SDSS              53
    #define SD_DETECT_PIN     49 
	
	#endif
#endif // ULTRA_LCD
