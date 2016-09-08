/****************************************************************************************
* 5 - 51
* Gen6 - Gen6 Deluxe
****************************************************************************************/

#define KNOWN_BOARD 1
#define BOARD_NAME "Gen6"

#ifndef __AVR_ATmega644P__
  #ifndef __AVR_ATmega1284P__
    #error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
  #endif
#endif

//x axis pins
#define ORIG_X_STEP_PIN      15
#define ORIG_X_DIR_PIN       18
#define ORIG_X_ENABLE_PIN    19
#define X_STOP_PIN           20

//y axis pins
#define ORIG_Y_STEP_PIN      23
#define ORIG_Y_DIR_PIN       22
#define ORIG_Y_ENABLE_PIN    24
#define Y_STOP_PIN           25

//z axis pins
#define ORIG_Z_STEP_PIN      27
#define ORIG_Z_DIR_PIN       28
#define ORIG_Z_ENABLE_PIN    29
#define Z_STOP_PIN           30

//extruder pins
#define ORIG_E0_STEP_PIN      4    //Edited @ EJE Electronics 20100715
#define ORIG_E0_DIR_PIN       2    //Edited @ EJE Electronics 20100715
#define ORIG_E0_ENABLE_PIN    3    //Added @ EJE Electronics 20100715
#define ORIG_TEMP_0_PIN       5    //changed @ rkoeppl 20110410
#define ORIG_TEMP_1_PIN      -1    //changed @ rkoeppl 20110410


#define ORIG_TEMP_2_PIN      -1    //changed @ rkoeppl 20110410
#define ORIG_HEATER_0_PIN    14    //changed @ rkoeppl 20110410
#define ORIG_HEATER_1_PIN    -1
#define ORIG_HEATER_2_PIN    -1
#if MOTHERBOARD == 5
#define ORIG_HEATER_BED_PIN  -1    //changed @ rkoeppl 20110410
#define ORIG_TEMP_BED_PIN    -1    //changed @ rkoeppl 20110410
#else
#define ORIG_HEATER_BED_PIN   1    //changed @ rkoeppl 20110410
#define ORIG_TEMP_BED_PIN     0    //changed @ rkoeppl 20110410
#endif
#define SDPOWER              -1
#define SDSS                 17
#define LED_PIN              -1    //changed @ rkoeppl 20110410
#define ORIG_FAN_PIN         -1    //changed @ rkoeppl 20110410
#define ORIG_PS_ON_PIN       -1    //changed @ rkoeppl 20110410
#define KILL_PIN             -1    //changed @ drakelive 20120830
//our pin for debugging.

#define DEBUG_PIN             0

//our RS485 pins
#define TORIG_X_ENABLE_PIN   12
#define RORIG_X_ENABLE_PIN   13
