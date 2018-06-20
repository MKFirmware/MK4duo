/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */

/********************************
 * Firmware Version V4.3.29 dev *
 ********************************/

/********************************
 * Overall settings for HMS434  *
 ********************************/

// board.h
#define BOARD_CNCONTROLS_V14        1405    // Cartesio CNControls V14


//Configuration_Basic.h
#define STRING_CONFIG_H_AUTHOR "(MaukCC, Scheepers)"
#define MOTHERBOARD 1405
#define EXTRUDERS 2
#define DRIVER_EXTRUDERS 2


//Configuration_Cartesian.h
#define CUSTOM_MACHINE_NAME "Cartesio434"
#define X_MIN_ENDSTOP_LOGIC   true
#define Y_MIN_ENDSTOP_LOGIC   true
#define Z_MIN_ENDSTOP_LOGIC   true
#define MIN_Z_HEIGHT_FOR_HOMING   5
#define X_ENABLE_ON 1
#define Y_ENABLE_ON 1
#define Z_ENABLE_ON 1
#define INVERT_X_STEP_PIN true
#define INVERT_Y_STEP_PIN true
#define INVERT_Z_STEP_PIN true
#define INVERT_Y_DIR true
#define X_MAX_POS 200
#define Y_MAX_POS 200
#define Z_MAX_POS 200
#define DEFAULT_AXIS_STEPS_PER_UNIT   {71.128, 71.128, 640, 152, 152, 152, 152}
#define DEFAULT_MAX_FEEDRATE          {200, 200, 20, 100, 100, 100, 100}
#define MANUAL_FEEDRATE               {3000, 3000, 600, 20}
#define DEFAULT_MAX_ACCELERATION      {1000, 1000, 100, 10000, 10000, 10000, 10000}
#define DEFAULT_RETRACT_ACCELERATION  {10000, 10000, 10000, 10000}
#define DEFAULT_ACCELERATION          300
#define DEFAULT_TRAVEL_ACCELERATION   1000
#define HOMING_FEEDRATE_Z (600)
#define MESH_BED_LEVELING
#define MESH_MIN_X 1
#define MESH_MAX_X 180
#define MESH_MIN_Y 50
#define MESH_MAX_Y 256
#define GRID_MAX_POINTS_Y 2


//Configuration_Feature.h
#define AUTO_FAN { 0, 0, -1, -1, -1, -1 }
#define HOTEND_AUTO_FAN_TEMPERATURE  35
#define HOME_Y_BEFORE_X
#define FORCE_HOME_XY_BEFORE_Z
//#define BABYSTEPPING
#define BABYSTEP_MULTIPLICATOR 6.4
#define DOUBLECLICK_FOR_Z_BABYSTEPPING
//#define FWRETRACT
//#define EXTRUDER_ENCODER_CONTROL
//#define FILAMENT_RUNOUT_SENSOR
#define FIL_RUNOUT_PIN_INVERTING false
//#define DOOR_OPEN
#define DOOR_OPEN_LOGIC true
#define SDSUPPORT
//#define DOGLCD
//#define ULTIPANEL
//#define DEFAULT_LCD_CONTRAST 90
//#define LCD_CONTRAST_MIN 0
//#define LCD_CONTRAST_MAX 140
#define LCD_SCREEN_ROT_0
#define ENCODER_PULSES_PER_STEP 2
#define ENCODER_STEPS_PER_MENU_ITEM 1
#define SPEAKER
#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
#define LCD_FEEDBACK_FREQUENCY_HZ 800
#define STATUS_MESSAGE_SCROLLING
//#define NEXTION
//#define NEXTION_SERIAL 1
//#define LASER
//#define CNCROUTER
//#define CASE_LIGHT
#define CASE_LIGHT_DEFAULT_BRIGHTNESS 9
#define DEFAULT_STEPPER_DEACTIVE_TIME 60
#define MINIMUM_STEPPER_PULSE 0
#define BLOCK_BUFFER_SIZE 16
//#define NOZZLE_CLEAN_FEATURE
//#define NOZZLE_PARK_FEATURE // use G27 P2
#define NOZZLE_PARK_POINT { ( 20, 300, 5 }
//#define ADVANCED_PAUSE_FEATURE
#define EXTENDED_CAPABILITIES_REPORT


//Configuration_Pins.h
//#define HEATER_CHAMBER_PIN  ORIG_HEATER_CHAMBER_PIN
//#define HEATER_COOLER_PIN   ORIG_HEATER_COOLER_PIN
//#define TEMP_CHAMBER_PIN    ORIG_TEMP_CHAMBER_PIN
//#define TEMP_COOLER_PIN     ORIG_TEMP_COOLER_PIN
//#define FIL_RUNOUT_PIN      ORIG_FIL_RUNOUT_PIN
//#define FIL_RUNOUT_DAV_PIN  ORIG_FIL_RUNOUT_DAV_PIN
//#define CASE_LIGHT_PIN      ORIG_CASE_LIGHT_PIN 
//#define DOOR_PIN            ORIG_DOOR_PIN  
//#define DHT_DATA_PIN        ORIG_DHT_DATA_PIN 


//Configuration_Temperature.h
#define TEMP_SENSOR_0            -1
#define TEMP_SENSOR_1            -1
#define TEMP_SENSOR_2             0
#define TEMP_SENSOR_3             0
#define TEMP_SENSOR_BED           1
//#define TEMP_SENSOR_CHAMBER       1
#define TEMP_SENSOR_COOLER        0
#define THERMISTOR_SERIES_RS      40000.0 // compensation for Due 3V3
#define DHT_TYPE                  22
#define TEMP_RESIDENCY_TIME       4  
#define TEMP_SENSOR_AD595_OFFSET  0.0
#define TEMP_SENSOR_AD595_GAIN    2.0
#define HEATER_0_MAXTEMP          400 // (degC)
#define HEATER_1_MAXTEMP          400 // (degC)
#define HEATER_2_MAXTEMP          400 // (degC)
#define HEATER_3_MAXTEMP          400 // (degC)
#define BED_MAXTEMP               150 // (degC)
#define CHAMBER_MAXTEMP           70 // (degC)
#define COOLER_MAXTEMP            35  // (degC) 
#define HEATER_0_MINTEMP          5 // (degC)
#define HEATER_1_MINTEMP          5 // (degC)
#define HEATER_2_MINTEMP          5 // (degC)
#define HEATER_3_MINTEMP          5 // (degC)
#define BED_MINTEMP               5 // (degC)
#define CHAMBER_MINTEMP           5 // (degC)
#define COOLER_MINTEMP            10 // (degC) 
//#define TEMP_STAT_LEDS
#define DEFAULT_Kp                {10, 10, 10, 10}     // Kp for H0, H1, H2, H3
#define DEFAULT_Ki                {0.8, 0.8, 0.8, 0.8}     // Ki for H0, H1, H2, H3
#define DEFAULT_Kd                {100, 100, 100, 100}     // Kd for H0, H1, H2, H3
#define DEFAULT_Kc                {100, 100, 100, 100} // heating power = Kc * (e_speed)
#define PIDTEMPBED                true
#define DEFAULT_bedKp             390.00
#define DEFAULT_bedKi             70.0
#define DEFAULT_bedKd             546.0
#define PIDTEMPCHAMBER            false // FIND YOUR OWN: "M303 E-2 C8 S90" to run autotune on the chamber at 90 degreesC for 8 cycles.
#define DEFAULT_chamberKp         10.00
#define DEFAULT_chamberKi         0.1
#define DEFAULT_chamberKd         300.0
