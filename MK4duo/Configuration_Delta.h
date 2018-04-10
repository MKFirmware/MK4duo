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
 *
 */

/**
 * Configuration_Delta.h
 *
 * This configuration file contains mechanism settings for delta printer.
 *
 * - Machine name
 * - Delta settings
 * - Delta Fast SQRT (only for 8 bit)
 * - Endstop pullup resistors
 * - Endstops logic
 * - Endstop Interrupts Feature
 * - Z probe Options
 * - Endstops min or max
 * - Stepper enable logic
 * - Stepper step logic
 * - Stepper direction
 * - Disables axis
 * - Axis relative mode
 * - Auto Bed Leveling (ABL)
 * - Auto Calibration
 * - Delta Home Safe Zone
 * - Axis steps per unit
 * - Axis feedrate
 * - Axis acceleration
 * - Axis jerk
 * - Homing feedrate
 * - Hotend offset
 *
 * Basic-settings can be found in Configuration_Basic.h
 * Temperature-settings can be found in Configuration_Temperature.h
 * Feature-settings can be found in Configuration_Feature.h
 * Pins-settings can be found in "Configuration_Pins.h"
 */

#ifndef _CONFIGURATION_DELTA_H_
#define _CONFIGURATION_DELTA_H_

#define KNOWN_MECH

/*****************************************************************************************
 *********************************** Machine name ****************************************
 *****************************************************************************************
 *                                                                                       *
 * This to set a custom name for your generic Mendel.                                    *
 * Displayed in the LCD "Ready" message.                                                 *
 *                                                                                       *
 *****************************************************************************************/
#define CUSTOM_MACHINE_NAME "Delta"
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Delta configuration ************************************
 ****************************************************************************************/
// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
// The new function do not use segments per second but segments per mm
// if you want use new function comment this (using // at the start of the line)
#define DELTA_SEGMENTS_PER_SECOND 200

// NOTE: All following values for DELTA_* MUST be floating point,
// so always have a decimal point in them.
//
// Towers and rod nomenclature for the following defines:
//
//                     C, Y-axis
//                     |
// DELTA_ALPHA_CA=120° |  DELTA_ALPHA_CB=120°
//                     |
//                     |______ X-axis
//                    / \
//                   /   \
//                  /     \
//                 /       \
//                A         B
//
//    |___| DELTA CARRIAGE OFFSET
//    |   \
//    |    \
//    |     \  DELTA DIAGONAL ROD
//    |      \
//    |       \   | Effector is at printer center!
//    |        \__|__/
//    |        |--| DELTA EFFECTOR OFFSET
//        |----|    DELTA RADIUS Calculated in fw (DELTA SMOOTH ROD OFFSET - DELTA EFFECTOR OFFSET - DELTA CARRIAGE OFFSET)
//      |---------| DELTA PRINTABLE RADIUS
//    |-----------| DELTA SMOOTH ROD OFFSET
  
// Center-to-center distance of the holes in the diagonal push rods.
#define DELTA_DIAGONAL_ROD 220.0            // mm

// Horizontal offset from middle of printer to smooth rod center.
#define DELTA_SMOOTH_ROD_OFFSET 150.0       // mm

// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 20.0          // mm

// Horizontal offset of the universal joints on the carriages.
#define DELTA_CARRIAGE_OFFSET 20.0          // mm

// Delta Height: Distance between nozzle (when in maximal/home position) and print surface (z = 0.00 mm).
#define DELTA_HEIGHT 200                    // mm

// Delta Printable radius
#define DELTA_PRINTABLE_RADIUS 75.0         // mm

// Endstop Offset Adjustment - All values are in mm and must be negative (to move down away from endstop switches) 
#define TOWER_A_ENDSTOP_ADJ  0.00  // Front Left Tower
#define TOWER_B_ENDSTOP_ADJ  0.00  // Front Right Tower
#define TOWER_C_ENDSTOP_ADJ  0.00  // Rear Tower

// Tower Angular Adjustment - Adj x Degrees around delta radius (- move clockwise / + move anticlockwise)
#define TOWER_A_ANGLE_ADJ    0.00  // Front Left Tower
#define TOWER_B_ANGLE_ADJ    0.00  // Front Right Tower
#define TOWER_C_ANGLE_ADJ    0.00  // Rear Tower

// Tower Radius Adjustment - Adj x mm in/out from centre of printer (- move in / + move out)
#define TOWER_A_RADIUS_ADJ   0.00  // Front Left Tower
#define TOWER_B_RADIUS_ADJ   0.00  // Front Right Tower
#define TOWER_C_RADIUS_ADJ   0.00  // Rear Tower

// Diagonal Rod Adjustment - Adj diag rod for Tower by x mm from DELTA_DIAGONAL_ROD value
#define TOWER_A_DIAGROD_ADJ  0.00  // Front Left Tower
#define TOWER_B_DIAGROD_ADJ  0.00  // Front Right Tower
#define TOWER_C_DIAGROD_ADJ  0.00  // Rear Tower
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Delta Fast SQRT ****************************************
 *****************************************************************************************
 *                                                                                       *
 * Fast inverse sqrt from Quake III Arena                                                *
 * See: https://en.wikipedia.org/wiki/Fast_inverse_square_root                           *
 *                                                                                       *
 *****************************************************************************************/
//#define DELTA_FAST_SQRT
/*****************************************************************************************/


/*****************************************************************************************
 ************************* Endstop pullup resistors **************************************
 *****************************************************************************************
 *                                                                                       *
 * Put true for enable or put false for disable the endstop pullup resistors             *
 *                                                                                       *
 *****************************************************************************************/
#define ENDSTOPPULLUP_XMAX    false
#define ENDSTOPPULLUP_YMAX    false
#define ENDSTOPPULLUP_ZMAX    false
#define ENDSTOPPULLUP_ZPROBE  false
/*****************************************************************************************/


/*****************************************************************************************
 ************************************ Endstops logic *************************************
 *****************************************************************************************
 *                                                                                       *
 * Mechanical endstop with COM to ground and NC to Signal                                *
 * uses "false" here (most common setup).                                                *
 *                                                                                       *
 *****************************************************************************************/
#define X_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_PROBE_ENDSTOP_LOGIC false   // set to true to invert the logic of the probe.
/*****************************************************************************************/


/*****************************************************************************************
 ***************************** Endstop Interrupts Feature ********************************
 *****************************************************************************************
 *                                                                                       *
 * Enable this feature if all enabled endstop pins are interrupt-capable.                *
 * This will remove the need to poll the interrupt pins, saving many CPU cycles.         *
 *                                                                                       *
 *****************************************************************************************/
//#define ENDSTOP_INTERRUPTS_FEATURE
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Z probe Options *****************************************
 *****************************************************************************************
 *                                                                                       *
 * Probes are sensors/switches that need to be activated before they can be used         *
 * and deactivated after their use.                                                      *
 * Servo Probes, Z Allen Key, Fix mounted Probe, etc.                                    *
 * You must activate one of these to use AUTO BED LEVELING FEATURE below.                *
 *                                                                                       *
 * If you want to still use the Z min endstop for homing,                                *
 * disable Z SAFE HOMING.                                                                *
 * Eg: to park the head outside the bed area when homing with G28.                       *
 *                                                                                       *
 * WARNING: The Z MIN endstop will need to set properly as it would                      *
 * without a Z PROBE to prevent head crashes and premature stopping                      *
 * during a print.                                                                       *
 * To use a separte Z PROBE endstop, you must have a Z PROBE PIN                         *
 * defined in the Configuration_Pins.h file for your control board.                      *
 *                                                                                       *
 * Use M851 X Y Z to set the probe offset from the nozzle. Store with M500.              *
 * WARNING: Setting the wrong pin may have unexpected and potentially                    *
 * disastrous outcomes. Use with caution and do your homework.                           *
 *                                                                                       *
 *****************************************************************************************/
// Z Servo Endstop
// Remember active servos in Configuration_Feature.h
// Define nr servo for endstop -1 not define. Servo index start 0
#define Z_ENDSTOP_SERVO_NR -1
#define Z_ENDSTOP_SERVO_ANGLES {90,0} // Z Servo Deploy and Stow angles

// The "Manual Probe" provides a means to do "Auto" Bed Leveling and calibration without a probe.
// Use G29 or G30 A repeatedly, adjusting the Z height at each point with movement commands
// or (with LCD BED LEVELING) the LCD controller.
//#define PROBE_MANUALLY

// A Fix-Mounted Probe either doesn't deploy or needs manual deployment.
// For example an inductive probe, or a setup that uses the nozzle to probe.
// An inductive probe must be deactivated to go below
// its trigger-point if hardware endstops are active.
//#define Z_PROBE_FIX_MOUNTED

// The BLTouch probe uses a Hall effect sensor and emulates a servo.
// The default connector is SERVO 0.
//#define BLTOUCH
//#define BLTOUCH_DELAY 375 // (ms) Enable and increase if needed

// Allen key retractable z-probe as seen on many Kossel delta printers - http://reprap.org/wiki/Kossel#Automatic_bed_leveling_probe
// Deploys by touching z-axis belt. Retracts by pushing the probe down.
//#define Z_PROBE_ALLEN_KEY

// Start and end location values are used to deploy/retract the probe (will move from start to end and back again)
#define Z_PROBE_DEPLOY_START_LOCATION   {0, 0, 30}  // X, Y, Z, start location for z-probe deployment sequence
#define Z_PROBE_DEPLOY_END_LOCATION     {0, 0, 30}  // X, Y, Z, end location for z-probe deployment sequence
#define Z_PROBE_RETRACT_START_LOCATION  {0, 0, 30}  // X, Y, Z, start location for z-probe retract sequence
#define Z_PROBE_RETRACT_END_LOCATION    {0, 0, 30}  // X, Y, Z, end location for z-probe retract sequence

// Offsets to the probe relative to the nozzle tip (Nozzle - Probe)
// X and Y offsets MUST be INTEGERS
//
//    +-- BACK ---+
//    |           |
//  L |    (+) P  | R <-- probe (10,10)
//  E |           | I
//  F | (-) N (+) | G <-- nozzle (0,0)
//  T |           | H
//    |  P (-)    | T <-- probe (-10,-10)
//    |           |
//    O-- FRONT --+
//  (0,0)
#define X_PROBE_OFFSET_FROM_NOZZLE  0     // X offset: -left  [of the nozzle] +right
#define Y_PROBE_OFFSET_FROM_NOZZLE  0     // Y offset: -front [of the nozzle] +behind
#define Z_PROBE_OFFSET_FROM_NOZZLE -1     // Z offset: -below [of the nozzle] (always negative!)

// X and Y axis travel speed between probes, in mm/min
#define XY_PROBE_SPEED 10000
// Z probe speed, in mm/min
#define Z_PROBE_SPEED 3000

// Z Probe repetitions, median for best result
#define Z_PROBE_REPETITIONS 1

// Enable Z Probe Repeatability test to see how accurate your probe is
//#define Z_MIN_PROBE_REPEATABILITY_TEST

// Probe Raise options provide clearance for the probe to deploy, stow, and travel.
#define Z_PROBE_DEPLOY_HEIGHT 15  // Z position for the probe to deploy/stow
#define Z_PROBE_BETWEEN_HEIGHT 5  // Z position for travel between points
#define Z_PROBE_AFTER_PROBING  0  // Z position after probing is done

// For M851 give a range for adjusting the Probe Z Offset
#define Z_PROBE_OFFSET_RANGE_MIN -50
#define Z_PROBE_OFFSET_RANGE_MAX  50

// Enable if probing seems unreliable. Heaters and/or fans - consistent with the
// options selected below - will be disabled during probing so as to minimize
// potential EM interference by quieting/silencing the source of the 'noise' (the change
// in current flowing through the wires).  This is likely most useful to users of the
// BLTouch probe, but may also help those with inductive or other probe types.
//#define PROBING_HEATERS_OFF       // Turn heaters off when probing
//#define PROBING_FANS_OFF          // Turn fans off when probing

// Use the LCD controller for bed leveling
// Requires MESH BED LEVELING or PROBE MANUALLY
//#define LCD_BED_LEVELING
#define LCD_Z_STEP 0.025    // Step size while manually probing Z axis.
#define LCD_PROBE_Z_RANGE 4 // Z Range centered on Z MIN POS for LCD Z adjustment
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Endstops min or max **********************************
 *****************************************************************************************
 *                                                                                       *
 * Sets direction of endstop when homing; 1=MAX, -1=MIN                                  *
 *                                                                                       *
 *****************************************************************************************/
#define X_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
#define Y_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
#define Z_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Stepper enable logic **********************************
 *****************************************************************************************
 *                                                                                       *
 * For Inverting Stepper Enable Pins                                                     *
 * (Active Low) use 0                                                                    *
 * Non Inverting (Active High) use 1                                                     *
 *                                                                                       *
 *****************************************************************************************/
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0      // For all extruder
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Stepper step logic **********************************
 *****************************************************************************************
 *                                                                                       *
 * By default pololu step drivers require an active high signal.                         *
 * However, some high power drivers require an active low signal as step.                *
 *                                                                                       *
 *****************************************************************************************/
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Stepper direction ************************************
 *****************************************************************************************
 *                                                                                       *
 * Invert the stepper direction.                                                         *
 * Change (or reverse the motor connector) if an axis goes the wrong way.                *
 *                                                                                       *
 *****************************************************************************************/
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
/*****************************************************************************************/


/*****************************************************************************************
 ************************************* Disables axis *************************************
 *****************************************************************************************
 *                                                                                       *
 * Disables axis when it's not being used.                                               *
 *                                                                                       *
 *****************************************************************************************/
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false      // For all extruder
// Disable only inactive extruder and keep active extruder enabled
//#define DISABLE_INACTIVE_EXTRUDER
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis relative mode ***********************************
 *****************************************************************************************/
#define AXIS_RELATIVE_MODES {false, false, false, false}
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Auto Bed Leveling (ABL) *********************************
 *****************************************************************************************
 *                                                                                       *
 * - UBL (Unified Bed Leveling)                                                          *
 *   A comprehensive bed leveling system combining the features and benefits             *
 *   of other systems. UBL also includes integrated Mesh Generation, Mesh                *
 *   Validation and Mesh Editing systems.                                                *
 *                                                                                       * 
 * - BILINEAR                                                                            *
 *   Probe several points in a grid.                                                     *
 *   You specify the rectangle and the density of sample points.                         *
 *   The result is a mesh, best for large or uneven beds.                                *
 *                                                                                       *
 *****************************************************************************************/
//#define AUTO_BED_LEVELING_UBL
//#define AUTO_BED_LEVELING_BILINEAR

// Enable detailed logging of G28, G29, G30, M48, etc.
// Turn on with the command 'M111 S32'.
// NOTE: Requires a lot of PROGMEM!
//#define DEBUG_LEVELING_FEATURE

// enable a graphics overly while editing the mesh from auto-level
//#define MESH_EDIT_GFX_OVERLAY

// Mesh inset margin on print area
#define MESH_INSET 10

// Enable the G26 Mesh Validation Pattern tool.
//#define G26_MESH_VALIDATION
#define MESH_TEST_NOZZLE_SIZE    0.4  // (mm) Diameter of primary nozzle.
#define MESH_TEST_LAYER_HEIGHT   0.2  // (mm) Default layer height for the G26 Mesh Validation Tool.
#define MESH_TEST_HOTEND_TEMP  200.0  // (c)  Default nozzle temperature for the G26 Mesh Validation Tool.
#define MESH_TEST_BED_TEMP      60.0  // (c)  Default bed temperature for the G26 Mesh Validation Tool.

/** START Unified Bed Leveling */
// Sophisticated users prefer no movement of nozzle
#define UBL_MESH_EDIT_MOVES_Z

// Save the currently active mesh in the current slot on M500
#define UBL_SAVE_ACTIVE_ON_M500

// When the nozzle is off the mesh, this value is used as the Z-Height correction value.
//#define UBL_Z_RAISE_WHEN_OFF_MESH 2.5
/** END Unified Bed Leveling */

// Set the number of grid points per dimension
// Works best with 5 or more points in each dimension.
#define GRID_MAX_POINTS_X 7
#define GRID_MAX_POINTS_Y 7

// The Z probe minimum outer margin (to validate G29 parameters).
#define MIN_PROBE_EDGE 10

// Probe along the Y axis, advancing X after each column
//#define PROBE_Y_FIRST

// Experimental Subdivision of the grid by Catmull-Rom method.
// Synthesizes intermediate points to produce a more detailed mesh.
//#define ABL_BILINEAR_SUBDIVISION
// Number of subdivisions between probe points
#define BILINEAR_SUBDIVISIONS 3

// Commands to execute at the end of G29 probing.
// Useful to retract or move the Z probe out of the way.
//#define Z_PROBE_END_SCRIPT "G1 Z10 F8000\nG1 X10 Y10\nG1 Z0.5"
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Auto Calibration **************************************
 *****************************************************************************************
 *                                                                                       *
 * Auto Calibration Delta system  G33 command                                            *
 * Three type of the calibration DELTA                                                   *
 *  1) Algorithm of Minor Squares based on DC42 RepRapFirmware 7 points           ~3.2Kb *
 *  2) Algorithm based on LVD-AC(Luc Van Daele) 1 - 7 points + iteration          ~4.5Kb *
 *                                                                                       *
 * To use one of this you must have a PROBE, please define you type probe.               *
 *                                                                                       *
 *****************************************************************************************/
//#define DELTA_AUTO_CALIBRATION_1
//#define DELTA_AUTO_CALIBRATION_2

#define DELTA_AUTO_CALIBRATION_2_DEFAULT_POINTS 4
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Delta Home Safe Zone **********************************
 *****************************************************************************************
 *                                                                                       *
 * After homing move down to a height where XY movement is unconstrained                 *
 *                                                                                       *
 *****************************************************************************************/
//#define DELTA_HOME_TO_SAFE_ZONE
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Delta Home On Power ***********************************
 *****************************************************************************************
 *                                                                                       *
 * Home printer on power on.                                                             *
 *                                                                                       *
 *****************************************************************************************/
//#define DELTA_HOME_ON_POWER
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Movement Settings *************************************
 *****************************************************************************************
 *                                                                                       *
 * Default Settings                                                                      *
 *                                                                                       *
 * These settings can be reset by M502                                                   *
 *                                                                                       *
 * Note that if EEPROM is enabled, saved values will override these.                     *
 *                                                                                       *
 *****************************************************************************************/


/*****************************************************************************************
 ******************************* Axis steps per unit *************************************
 *****************************************************************************************
 *                                                                                       *
 * Default Axis Steps Per Unit (steps/mm)                                                *
 * Override with M92                                                                     *
 *                                                                                       *
 *****************************************************************************************/
// Default steps per unit               X,  Y,  Z,  E0...(per extruder)
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 80, 625, 625, 625, 625}
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis feedrate ****************************************
 *****************************************************************************************/
//                                       X,   Y,   Z,  E0...(per extruder). (mm/sec)
#define DEFAULT_MAX_FEEDRATE          {500, 500, 500, 100, 100, 100, 100}
// Feedrates for manual moves along        X,     Y,     Z,  E from panel
#define MANUAL_FEEDRATE               {50*60, 50*60, 50*60, 10*60}
// Minimum feedrate
#define DEFAULT_MINIMUMFEEDRATE       0.0
#define DEFAULT_MINTRAVELFEEDRATE     0.0
// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED         0.05                      // (mm/sec)
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Axis acceleration **************************************
 *****************************************************************************************/
//  Maximum start speed for accelerated moves.    X,    Y,    Z,   E0...(per extruder)
#define DEFAULT_MAX_ACCELERATION              {5000, 5000, 5000, 1000, 1000, 1000, 1000}
//  Maximum acceleration in mm/s^2 for retracts   E0... (per extruder)
#define DEFAULT_RETRACT_ACCELERATION          {10000, 10000, 10000, 10000}
//  X, Y, Z and E* maximum acceleration in mm/s^2 for printing moves
#define DEFAULT_ACCELERATION          3000
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
#define DEFAULT_TRAVEL_ACCELERATION   3000
/*****************************************************************************************/


/*****************************************************************************************
 ************************************* Axis jerk *****************************************
 *****************************************************************************************
 *                                                                                       *
 * Default Jerk (mm/s)                                                                   *
 * Override with M205 X Y Z E                                                            *
 *                                                                                       *
 * "Jerk" specifies the minimum speed change that requires acceleration.                 *
 * When changing speed and direction, if the difference is less than the                 *
 * value set here, it may happen instantaneously.                                        *
 *                                                                                       *
 *****************************************************************************************/
#define DEFAULT_XJERK 20.0
#define DEFAULT_YJERK 20.0
#define DEFAULT_ZJERK 20.0
// E0... (mm/sec) per extruder
#define DEFAULT_EJERK {5.0, 5.0, 5.0, 5.0}
/*****************************************************************************************/


/*****************************************************************************************
 ************************************ Homing feedrate ************************************
 *****************************************************************************************/
// delta homing speeds must be the same on xyz. Homing speeds (mm/m)
#define HOMING_FEEDRATE_XYZ (100*60)
// homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define XYZ_HOME_BUMP_MM 5
// Re-Bump Speed Divisor (Divides the Homing Feedrate)
#define XYZ_BUMP_DIVISOR 10
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** Hotend offset ***************************************
 *****************************************************************************************
 *                                                                                       *
 * Offset of the hotends (uncomment if using more than one and relying on firmware       *
 * to position when changing).                                                           *
 * The offset has to be X=0, Y=0, Z=0 for the hotend 0 (default hotend).                 *
 * For the other hotends it is their distance from the hotend 0.                         *
 *                                                                                       *
 *****************************************************************************************/
#define HOTEND_OFFSET_X {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the X axis
#define HOTEND_OFFSET_Y {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Y axis
#define HOTEND_OFFSET_Z {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Z axis
/*****************************************************************************************/

#endif /* _CONFIGURATION_DELTA_H_ */
