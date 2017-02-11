/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */



//-----------------------------------
//--- Configuration_Basic.h
//-----------------------------------
#define BAUDRATE 115200
#define MOTHERBOARD BOARD_MKS_MINI
#define MECHANISM MECH_DELTA

//-----------------------------------
//--- Configuration_Delta.h
//-----------------------------------

#define CUSTOM_MACHINE_NAME "Drake Kossel"

#define DELTA_SEGMENTS_PER_SECOND 100
#define DELTA_DIAGONAL_ROD      217.0   // mm
#define DELTA_SMOOTH_ROD_OFFSET 151.0   // mm
#define DELTA_EFFECTOR_OFFSET   25.0    // mm
#define DELTA_CARRIAGE_OFFSET   20.0    // mm
#define DELTA_PRINTABLE_RADIUS  75.0    // mm


//Endstop Offset Adjustment - All values are in mm and must be negative (to move down away from endstop switches)
#define TOWER_A_ENDSTOP_ADJ 0   // Front Left Tower
#define TOWER_B_ENDSTOP_ADJ 0   // Front Right Tower
#define TOWER_C_ENDSTOP_ADJ 0   // Rear Tower

//Tower Position Adjustment - Adj x Degrees around delta radius (- move clockwise / + move anticlockwise)
#define TOWER_A_RADIUS_ADJ 0    // Front Left Tower
#define TOWER_B_RADIUS_ADJ 0    // Front Right Tower
#define TOWER_C_RADIUS_ADJ 0    // Rear Tower

//Tower Radius Adjustment - Adj x mm in/out from centre of printer (- move in / + move out)
#define TOWER_A_POSITION_ADJ 0  // Front Left Tower
#define TOWER_B_POSITION_ADJ 0  // Front Right Tower
#define TOWER_C_POSITION_ADJ 0  // Rear Tower

//Diagonal Rod Adjustment - Adj diag rod for Tower by x mm from DELTA_DIAGONAL_ROD value
#define TOWER_A_DIAGROD_ADJ 0   // Front Left Tower
#define TOWER_B_DIAGROD_ADJ 0   // Front Right Tower
#define TOWER_C_DIAGROD_ADJ 0   // Rear Tower

// Invert the stepper direction.                                                         *
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false

#define MANUAL_Z_HOME_POS 200

#define DEFAULT_AXIS_STEPS_PER_UNIT   {200, 200, 200, 299, 625, 625, 625}
#define DEFAULT_MAX_FEEDRATE          {500, 500, 500, 100, 100, 100, 100}
#define MANUAL_FEEDRATE               {3000, 3000, 3000, 60}
#define DEFAULT_MINIMUMFEEDRATE       0.0
#define DEFAULT_MINTRAVELFEEDRATE     0.0

#define DEFAULT_MAX_ACCELERATION      {3000, 3000, 3000, 1000, 1000, 1000, 1000}
#define DEFAULT_RETRACT_ACCELERATION  {10000, 10000, 10000, 10000}
#define DEFAULT_ACCELERATION          3000
#define DEFAULT_TRAVEL_ACCELERATION   3000

#define DEFAULT_XJERK 35.0
#define DEFAULT_YJERK 35.0
#define DEFAULT_ZJERK 20.0
#define DEFAULT_EJERK {5.0, 5.0, 5.0, 5.0}

#define HOMING_FEEDRATE_XYZ (2000)
#define XYZ_HOME_BUMP_MM 5
#define XYZ_BUMP_DIVISOR 10

#define DELTA_HOME_TO_SAFE_ZONE

#define X_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z2_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z3_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z4_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z2_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z3_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z4_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.

#define Z_PROBE_ENDSTOP_LOGIC false   // set to true to invert the logic of the probe.

#define E_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.


#define BLTOUCH
#define AUTO_BED_LEVELING_FEATURE
//#define DEBUG_LEVELING_FEATURE
#define Z_ENDSTOP_SERVO_NR 0

#define ENABLE_SERVOS
#define NUM_SERVOS         1
#define SERVO0_PIN         7

#define X_PROBE_OFFSET_FROM_NOZZLE  0     // X offset: -left  [of the nozzle] +right
#define Y_PROBE_OFFSET_FROM_NOZZLE  27    // Y offset: -front [of the nozzle] +behind
#define Z_PROBE_OFFSET_FROM_NOZZLE -1     // Z offset: -below [of the nozzle] (always negative!)

#define XY_PROBE_SPEED 2000               // X and Y axis travel speed between probes, in mm/min
#define Z_PROBE_SPEED  2000               // Z probe speed, in mm/min

// Probe Raise options provide clearance for the probe to deploy, stow, and travel.
#define Z_PROBE_DEPLOY_HEIGHT  10  // Z position for the probe to deploy/stow
#define Z_PROBE_BETWEEN_HEIGHT 10  // Z position for travel between points

// For M666 give a range for adjusting the Z probe offset
#define Z_PROBE_OFFSET_RANGE_MIN -5
#define Z_PROBE_OFFSET_RANGE_MAX  5

//-----------------------------------
//--- Configuration_Feature.h
//-----------------------------------

#define EASY_LOAD
#define BOWDEN_LENGTH       650    // mm
#define LCD_PURGE_LENGTH      3    // mm
#define LCD_RETRACT_LENGTH    3    // mm
#define LCD_PURGE_FEEDRATE    3    // mm/s
#define LCD_RETRACT_FEEDRATE 10    // mm/s
#define LCD_LOAD_FEEDRATE    20    // mm/s
#define LCD_UNLOAD_FEEDRATE  20    // mm/s

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

#define TEMP_SENSOR_0   -2
#define TEMP_SENSOR_BED  0

