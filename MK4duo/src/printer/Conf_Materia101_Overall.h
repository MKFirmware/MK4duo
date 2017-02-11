/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */





// Arduino MATERIA 101

//-----------------------------------
//--- Configuration_Basic.h
//-----------------------------------
#define BAUDRATE 115200
#define MOTHERBOARD BOARD_RAMPS_13_HHB //BOARD_RAMPS_13_HFB
#define MECHANISM MECH_CARTESIAN

//#define ORIG_FAN_PIN            4

//-----------------------------------
//--- Configuration_Cartesian
//-----------------------------------

#define CUSTOM_MACHINE_NAME "Materia 101"


#define X_MIN_ENDSTOP_LOGIC   true   // set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_LOGIC   true   // set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_LOGIC   true   // set to true to invert the logic of the endstop.
#define Z2_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z3_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z4_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z2_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z3_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z4_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.




// Invert the stepper direction.                                                         *
#define INVERT_X_DIR  true
#define INVERT_Y_DIR  true
#define INVERT_Z_DIR  false
#define INVERT_E0_DIR true
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false


#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR -1


#define X_MAX_POS 140
#define X_MIN_POS 0
#define Y_MAX_POS 110
#define Y_MIN_POS 0
#define Z_MAX_POS 100
#define Z_MIN_POS 0
#define E_MIN_POS 0



#define MANUAL_Z_HOME_POS 200

#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,400,96.2753, 625, 625, 625}
#define DEFAULT_MAX_FEEDRATE          {300, 300, 19, 35, 100, 100, 100}
#define MANUAL_FEEDRATE               {2000, 2000, 100, 60}
#define DEFAULT_MINIMUMFEEDRATE       0.0
#define DEFAULT_MINTRAVELFEEDRATE     0.0

#define DEFAULT_MAX_ACCELERATION      {2000, 2000, 1000, 1000, 1000, 1000, 1000}
#define DEFAULT_RETRACT_ACCELERATION  {10000, 10000, 10000, 10000}
#define DEFAULT_ACCELERATION          2000
#define DEFAULT_TRAVEL_ACCELERATION   2000

#define DEFAULT_XJERK 35.0
#define DEFAULT_YJERK 35.0
#define DEFAULT_ZJERK 20.0
#define DEFAULT_EJERK {5.0, 5.0, 5.0, 5.0}

#define HOMING_FEEDRATE_XYZ (2000)

//-----------------------------------
//--- Configuration_Feature.h
//-----------------------------------



#define EEPROM_SETTINGS         //
#define EEPROM_CHITCHAT         //

#define SDSUPPORT
#define LCD_LANGUAGE en
//#define NEXTION
#define REPRAP_DISCOUNT_SMART_CONTROLLER

//#define PINS_DEBUGGING

//-----------------------------------
//--- Configuration_Temperature.h
//-----------------------------------

#define TEMP_SENSOR_0    1
#define TEMP_SENSOR_BED  0






