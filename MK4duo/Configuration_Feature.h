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
 * Configuration_Feature.h
 *
 * This configuration file contains all features that can be enabled.
 *
 * EXTRUDER FEATURES:
 * - Fan configuration
 * - Volumetric extrusion
 * - Default nominal filament diameter
 * - Single nozzle
 * - BariCUDA paste extruder
 * - Solenoid extruder
 * - Color Mixing Extruder
 * - Multiextruder old MKR4
 * - Multiextruder MKR6
 * - Multiextruder MKR12
 * - Multiextruder MKSE6 (multiextruder with Servo)
 * - Multiextruder NPr2
 * - Multiextruder DONDOLO
 * - Extruder idle oozing prevention
 * - Extruder run-out prevention
 * - Bowden Filament management
 * - Extruder advance constant
 * - Extruder Advance Linear Pressure Control
 * MOTION FEATURES:
 * - Software endstops
 * - Endstops only for homing
 * - Abort on endstop hit feature
 * - G38.2 and G38.3 Probe Target
 * - Scad Mesh Output
 * - R/C Servo
 * - Late Z axis
 * - Ahead slowdown
 * - Quick home
 * - Home Y before X
 * - Force Home XY before Home Z
 * - Babystepping
 * - Firmware retract
 * - Dual X-carriage
 * - X-axis two driver
 * - Y-axis two driver
 * - Z-axis two - three - four driver
 * - XY Frequency limit
 * - Skeinforge arc fix
 * SENSORS FEATURES:
 * - Extruder Encoder Control
 * - Filament diameter sensor
 * - Filament Runout sensor
 * - Power consumption sensor
 * - Flow sensor
 * - Door open sensor
 * - Power check sensor
 * ADDON FEATURES:
 * - EEPROM
 * - SDCARD
 * - LCD Language
 * - LCD
 * - Canon RC-1 Remote
 * - Camera trigger
 * - RFID card reader
 * - BLINKM
 * - RGB LED
 * - PCA 9632 PWM LED
 * - Adafruit Neopixel LED driver
 * - Printer Event LEDs
 * - Laser beam
 * - CNC Router
 * - Case Light
 * ADVANCED MOTION FEATURES:
 * - Stepper auto deactivation
 * - Double / Quad Stepping
 * - Low speed stepper
 * - Microstepping
 * - Motor's current
 * - I2C DIGIPOT
 * - Toshiba steppers
 * - TMC26X motor drivers
 * - Trinamic TMC2130 motor drivers
 * - L6470 motor drivers
 * ADVANCED FEATURES:
 * - PWM Hardware
 * - Buffer stuff
 * - Nozzle Clean Feature
 * - Nozzle Park
 * - Advanced Pause Park
 * - G20/G21 Inch mode support
 * - Report JSON-style response
 * - M43 command for pins info and testing
 * - M115 Auto report temperatures
 * - Extend capabilities report
 * - Watchdog
 * - Start / Stop Gcode
 * - Proportional Font ratio
 * - User menu items
 *
 * Basic-settings can be found in Configuration_Basic.h
 * Mechanisms-settings can be found in Configuration_Xxxxxx.h (where Xxxxxx can be: Cartesian - Delta - Core - Scara)
 * Pins-settings can be found in "Configuration_Pins.h"
 * 
 */

#ifndef _CONFIGURATION_FEATURE_H_
#define _CONFIGURATION_FEATURE_H_

/**************************************************************************
 **************************** Fan configuration ***************************
 **************************************************************************/
// FAN PWM speed
// 0 -  15Hz 256 values
// 1 -  30Hz 128 values
// 2 -  61Hz  64 values
// 3 - 122Hz  32 values
// 4 - 244Hz  16 values
#define FAN_PWM_SPEED 0

// When first starting the main fan, run it at full speed for the
// given number of milliseconds.  This gets the fan spinning reliably
// before setting a PWM value.
//#define FAN_KICKSTART_TIME 200

// This defines the minimal speed for the main fan, run in PWM mode
// to enable uncomment and set minimal PWM speed for reliable running (1-255)
//#define FAN_MIN_PWM 50

// To reverse the logic of fan pins
//#define INVERTED_FAN_PINS

// This is for controlling a fan to cool down the stepper drivers
// it will turn on when any driver is enabled
// and turn off after the set amount of seconds from last driver being disabled again
// You need to set CONTROLLERFAN_PIN in Configuration_pins.h
//#define CONTROLLERFAN
#define CONTROLLERFAN_SECS       60   // How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED     255   // 255 = full speed
#define CONTROLLERFAN_MIN_SPEED   0

// Hotend cooling fans
// Configure fan pin outputs to automatically turn on/off when the associated
// hotend temperature is above/below HOTEND AUTO FAN TEMPERATURE.
// Multiple hotends can be assigned to the same pin in which case
// the fan will turn on when any selected hotend is above the threshold.
// You need to set HOTEND AUTO FAN PIN in Configuration_pins.h
//#define HOTEND_AUTO_FAN
//#define INVERTED_AUTO_FAN_PINS
#define HOTEND_AUTO_FAN_TEMPERATURE  50
#define HOTEND_AUTO_FAN_SPEED       255  // 255 = full speed
#define HOTEND_AUTO_FAN_MIN_SPEED     0
/**************************************************************************/


/***********************************************************************
 ************************ Volumetric extrusion *************************
 ***********************************************************************
 *                                                                     *
 * Volumetric extrusion default state                                  *
 * Activate to make volumetric extrusion the default method,           *
 * with DEFAULT NOMINAL FILAMENT DIA as the default diameter.          *
 *                                                                     *
 * M200 D0 to disable, M200 Dn to set a new diameter.                  *
 *                                                                     *
 ***********************************************************************/
//#define VOLUMETRIC_DEFAULT_ON
/***********************************************************************/


/***********************************************************************
 ******************** DEFAULT NOMINAL FILAMENT DIA *********************
 ***********************************************************************
 *                                                                     *
 * Enter the diameter (in mm) of the filament generally used           *
 * (3.0 mm or 1.75 mm)                                                 *
 * This is then used in the slicer software.                           *
 * Used for volumetric.                                                *
 * Used for sensor reading validation.                                 *
 *                                                                     *
 ***********************************************************************/
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75
/***********************************************************************/


/***********************************************************************
 **************************** Single nozzle ****************************
 ***********************************************************************
 *                                                                     *
 * This is used for single nozzle and multiple extrusion configuration *
 *                                                                     *
 * Uncomment SINGLENOZZLE to enable this feature                       *
 *                                                                     *
 ***********************************************************************/
//#define SINGLENOZZLE
/***********************************************************************/


/***********************************************************************
 *********************** BariCUDA paste extruder ***********************
 ***********************************************************************
 *                                                                     *
 * Support for the BariCUDA paste extruder.                            *
 *                                                                     *
 ***********************************************************************/
//#define BARICUDA
/***********************************************************************/


/***********************************************************************
 ************************** Solenoid extruder **************************
 ***********************************************************************
 *                                                                     *
 * Activate a solenoid on the active extruder with M380.               *
 * Disable all with M381.                                              *
 * Define SOL0_PIN, SOL1_PIN, etc., for each extruder that             *
 * has a solenoid.                                                     *
 *                                                                     *
 ***********************************************************************/
//#define EXT_SOLENOID
/***********************************************************************/


/***********************************************************************
 ********************** COLOR MIXING EXTRUDER **************************
 ***********************************************************************
 *                                                                     *
 * Extends G0/G1 with mixing factors ABCDHI for up to 6 steppers.      *
 * Adds a new code, M165, to set the current mix factors.              *
 * Optional support for Repetier M163, M164, and virtual tools.        *
 * Extends the stepping routines to move multiple steppers in          *
 * proportion to the mix.                                              *
 *                                                                     *
 ***********************************************************************/
//#define COLOR_MIXING_EXTRUDER

// Number of steppers in your mixing extruder
#define MIXING_STEPPERS 2
// Use the Virtual Tool method with M163 and M164
#define MIXING_VIRTUAL_TOOLS 16
/***********************************************************************/


/***********************************************************************
 ************************* Multiextruder MKR4 **************************
 ***********************************************************************
 *                                                                     *
 * Setting for more extruder width relay system                        *
 * This is old system for 4 extruder and 8 relay.                      *
 * See Configuration_pins.h for pin command relay                      *
 *                                                                     *
 * Uncomment MKR4 to enable this feature                               *
 *                                                                     *
 * Uncomment INVERTED_RELE_PINS if your relay switches with GND        *
 ***********************************************************************/
//#define MKR4
//#define INVERTED_RELE_PINS
/***********************************************************************/


/***********************************************************************
 ************************* Multiextruder MKR6 **************************
 ***********************************************************************
 *                                                                     *
 * Setting for more extruder width relay system                        *
 * This is new system for 6 extruder width 2 driver and 8 relay.       *
 * See Configuration_pins.h for pin command relay                      *
 *                                                                     *
 * Uncomment MKR6 to enable this feature                               *
 *                                                                     *
 * Uncomment INVERTED_RELE_PINS if your relay switches with GND        *
 ***********************************************************************/
//#define MKR6
//#define INVERTED_RELE_PINS
/***********************************************************************/


/***********************************************************************
 ************************* Multiextruder MKR12 *************************
 ***********************************************************************
 *                                                                     *
 * Setting for more extruder width relay system                        *
 * This is new system for 12 extruder width 4 driver and 16 relay.     *
 * See Configuration_pins.h for pin command relay                      *
 *                                                                     *
 * Uncomment MKR12 to enable this feature                              *
 *                                                                     *
 * Uncomment INVERTED_RELE_PINS if your relay switches with GND        *
 ***********************************************************************/
//#define MKR12
//#define INVERTED_RELE_PINS
/***********************************************************************/


/***********************************************************************
 ************************* Multiextruder MKSE6 *************************
 ***********************************************************************
 *                                                                     *
 * Setting for more extruder width servo system                        *
 * This is new system for 6 extruder width 1 driver and 1 servo.       *
 *                                                                     *
 * Uncomment MKSE6 to enable this feature                              *
 *                                                                     *
 ***********************************************************************/
//#define MKSE6

#define MKSE6_SERVO_INDEX    0
#define MKSE6_SERVOPOS_E0  -60
#define MKSE6_SERVOPOS_E1  -30
#define MKSE6_SERVOPOS_E2    0
#define MKSE6_SERVOPOS_E3   30
#define MKSE6_SERVOPOS_E4   60
#define MKSE6_SERVOPOS_E5   90
#define MKSE6_SERVO_DELAY 1000
/***********************************************************************/


/***********************************************************************
 *********************** Multiextruder NPr2 ****************************
 ***********************************************************************
 *                                                                     *
 * Setting for color meccanism NPr2 by NicolaP (www.3dmakerlab.it)     *
 * Find angle setting by g-Code "M997 Cxxx"                            *
 *                                                                     *
 * Uncomment NPR2 to enable this feature                               *
 * You also need to set E_MIN_PIN in Configuration_pins.h              *
 *                                                                     *
 ***********************************************************************/
//#define NPR2
#define COLOR_STEP {0, 10, 20, 30}   // CARTER ANGLE
#define COLOR_SLOWRATE 170           // MICROSECOND delay for carter motor routine (Carter Motor Feedrate: upper value-slow feedrate)  
#define COLOR_HOMERATE 4             // FEEDRATE for carter home
#define MOTOR_ANGLE 1.8              // Nema angle for single step 
#define DRIVER_MICROSTEP 4           // Microstep moltiplicator driver (set jumper MS1-2-3) off-on-off 1/4 microstepping.
#define CARTER_MOLTIPLICATOR 14.22   // CARTER MOLTIPLICATOR (gear ratio 13/31-10/31)
/***********************************************************************/


/***********************************************************************
 ********************* Dual Extruder DONDOLO ***************************
 ***********************************************************************
 *                                                                     *
 * Setting for multiextruder DONDOLO 1.0b by Gianni Franci             *
 * Enable DONDOLO SINGLE MOTOR for original DONDOLO by Gianni Franci   *
 * Enable DONDOLO DUAL MOTOR for bowden and dual EXTRUDER              *
 * http://www.thingiverse.com/thing:673816                             *
 * For function set NUM_SERVOS +1 if you use for endstop or probe      *
 * Set DONDOLO SERVO INDEX for servo you use for DONDOLO               *
 * Set DONDOLO SERVOPOS E0 angle for E0 extruder                       *
 * Set DONDOLO SERVOPOS E1 angle for E1 extruder                       *
 * Remember set HOTEND OFFSET X Y Z                                    *
 *                                                                     *
 ***********************************************************************/
//#define DONDOLO_SINGLE_MOTOR
//#define DONDOLO_DUAL_MOTOR

#define DONDOLO_SERVO_INDEX 0
#define DONDOLO_SERVOPOS_E0 120
#define DONDOLO_SERVOPOS_E1 10
#define DONDOLO_SERVO_DELAY 1000
/***********************************************************************/


/***********************************************************************
 **************** Extruder idle oozing prevention **********************
 ***********************************************************************
 *                                                                     *
 * This prevents undesired ejection of  filament while the printer     *
 * is in idle with the hotend turned on.                               *
 * Eg. during the heating up process.                                  *
 *                                                                     *
 * If the extruder motor is idle for more than SECONDS and the         *
 * temperature is over IDLE_OOZING_MINTEMP some filament is retracted. *
 * The filament retracted is re-added before the next extrusion or     *
 * when the target temperature is less than IDLE_OOZING_MINTEMP and    *
 * the actual temperature is greater than IDLE_OOZING_MINTEMP.         *
 *                                                                     *
 * PS: Always remember to set your extruder target temperature to 0°C  *
 * before shutdown the printer if you enable this feature.             *
 *                                                                     *
 * Uncomment IDLE OOZING PREVENT to enable this feature                *
 *                                                                     *
 ***********************************************************************/
//#define IDLE_OOZING_PREVENT
#define IDLE_OOZING_MINTEMP           190
#define IDLE_OOZING_FEEDRATE          50    //default feedrate for retracting (mm/s)
#define IDLE_OOZING_SECONDS           5
#define IDLE_OOZING_LENGTH            15    //default retract length (positive mm)
#define IDLE_OOZING_RECOVER_LENGTH    0     //default additional recover length (mm, added to retract length when recovering)
#define IDLE_OOZING_RECOVER_FEEDRATE  50    //default feedrate for recovering from retraction (mm/s)
/***********************************************************************/


/*****************************************************************************************
 ***************************** Extruder run-out prevention *******************************
 *****************************************************************************************
 *                                                                                       *
 * If the machine is idle, and the temperature over MINTEMP, every couple of SECONDS     *
 * some filament is extruded                                                             *
 *                                                                                       *
 * Uncomment EXTRUDER RUNOUT PREVENT to enable this feature                              *
 *                                                                                       *
 *****************************************************************************************/
//#define EXTRUDER_RUNOUT_PREVENT
#define EXTRUDER_RUNOUT_MINTEMP 190
#define EXTRUDER_RUNOUT_SECONDS  30
#define EXTRUDER_RUNOUT_SPEED  1500 // mm/m
#define EXTRUDER_RUNOUT_EXTRUDE   5 // mm
/*****************************************************************************************/


/***********************************************************************
 ******************** Bowden Filament management ***********************
 ***********************************************************************
 *                                                                     *
 * Uncomment EASY_LOAD to enable this feature                          *
 *                                                                     *
 ***********************************************************************/
//#define EASY_LOAD
#define BOWDEN_LENGTH 250       // mm
#define LCD_PURGE_LENGTH 3      // mm
#define LCD_RETRACT_LENGTH 3    // mm
#define LCD_PURGE_FEEDRATE 3    // mm/s
#define LCD_RETRACT_FEEDRATE 10 // mm/s
#define LCD_LOAD_FEEDRATE 8     // mm/s
#define LCD_UNLOAD_FEEDRATE 8   // mm/s
/***********************************************************************/


/*****************************************************************************************
 ****************************** Extruder advance constant ********************************
 *****************************************************************************************
 *                                                                                       *
 * extruder advance constant (s2/mm3)                                                    *
 * advance (steps) = STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K * cubic mm per second ^ 2 *
 *                                                                                       *
 * Hooke's law says:    force = k * distance                                             *
 * Bernoulli's principle says:  v ^ 2 / 2 + g . h + pressure / density = constant        *
 * so: v ^ 2 is proportional to number of steps we advance the extruder                  *
 *                                                                                       *
 * This feature is obsolete needs update                                                 *
 * Uncomment ADVANCE to enable this feature                                              *
 *                                                                                       *
 *****************************************************************************************/
//#define ADVANCE

#define EXTRUDER_ADVANCE_K 0.0
#define D_FILAMENT 1.75
/*****************************************************************************************/


/*****************************************************************************************
 ****************** Extruder Advance Linear Pressure Control *****************************
 *****************************************************************************************
 *                                                                                       *
 * Assumption: advance = k * (delta velocity)                                            *
 * K=0 means advance disabled.                                                           *
 * To get a rough start value for calibration, measure your "free filament length"       *
 * between the hobbed bolt and the nozzle (in cm). Use the formula below that fits       *
 * your setup, where L is the "free filament length":                                    *
 *                                                                                       *
 * Filament diameter           |   1.75mm  |    3.0mm   |                                *
 * ----------------------------|-----------|------------|                                *
 * Stiff filament (PLA)        | K=47*L/10 | K=139*L/10 |                                *
 * Softer filament (ABS, nGen) | K=88*L/10 | K=260*L/10 |                                *
 *                                                                                       *
 * Some Slicers produce Gcode with randomly jumping extrusion widths occasionally.       *
 * For example within a 0.4mm perimeter it may produce a single segment of 0.05mm width. *
 * While this is harmless for normal printing (the fluid nature of the filament will     *
 * close this very, very tiny gap), it throws off the LIN ADVANCE pressure adaption.     *
 *                                                                                       *
 * For this case LIN ADVANCE E D RATIO can be used to set the extrusion:distance ratio   *
 * to a fixed value. Note that using a fixed ratio will lead to wrong nozzle pressures   *
 * if the slicer is using variable widths or layer heights within one print!             *
 *                                                                                       *
 * This option sets the default E:D ratio at startup. Use `M905` to override this value. *
 *                                                                                       *
 * Example: `M905 W0.4 H0.2 D1.75`, where:                                               *
 *   - W is the extrusion width in mm                                                    *
 *   - H is the layer height in mm                                                       *
 *   - D is the filament diameter in mm                                                  *
 *                                                                                       *
 * Set to 0 to auto-detect the ratio based on given Gcode G1 print moves.                *
 *                                                                                       *
 * Slic3r (including Prusa Slic3r) produces Gcode compatible with the automatic mode.    *
 * Cura (as of this writing) may produce Gcode incompatible with the automatic mode.     *
 *                                                                                       *
 *****************************************************************************************/
//#define LIN_ADVANCE

#define LIN_ADVANCE_K 75

// The calculated ratio (or 0) according to the formula W * H / ((D / 2) ^ 2 * PI)
// Example: 0.4 * 0.2 / ((1.75 / 2) ^ 2 * PI) = 0.033260135
#define LIN_ADVANCE_E_D_RATIO 0
/*****************************************************************************************/


//===========================================================================
//============================= MOTION FEATURES =============================
//===========================================================================

/**************************************************************************
 *************************** Workspace offsets ****************************
 **************************************************************************
 *                                                                        *
 * Enable this option for a leaner build of MK4duo that enable all        *
 * workspace offsets, simplifying coordinate transformations,             *
 * leveling, etc.                                                         *
 *                                                                        *
 *  - G92                                                                 *
 *  - M206 and M428 are enabled.                                          *
 **************************************************************************/
//#define WORKSPACE_OFFSETS
/**************************************************************************/


/**************************************************************************
 *************************** Software endstops ****************************
 **************************************************************************/
// If enabled, axis won't move to coordinates less than MIN POS.
#define MIN_SOFTWARE_ENDSTOPS
// If enabled, axis won't move to coordinates greater than MAX POS.
#define MAX_SOFTWARE_ENDSTOPS
/**************************************************************************/


/**************************************************************************
 *********************** Endstops only for homing *************************
 **************************************************************************
 *                                                                        *
 * If defined the endstops will only be used for homing                   *
 *                                                                        *
 * If you use all six endstop enable ENABLE ALL SIX ENDSTOP               *
 *                                                                        *
 **************************************************************************/
#define ENDSTOPS_ONLY_FOR_HOMING
//#define ENABLED_ALL_SIX_ENDSTOP
/**************************************************************************/


/**************************************************************************
 ******************** Abort on endstop hit feature ************************
 **************************************************************************
 *                                                                        *
 * This option allows you to abort printing when any endstop is triggered.*
 * This feature must be enabled with "M540 S1" or from the LCD menu or    *
 * by define ABORT ON ENDSTOP HIT INIT true.                              *
 * To have any effect, endstops must be enabled during SD printing.       *
 * With ENDSTOPS ONLY FOR HOMING you must send "M120" to enable endstops. *
 *                                                                        *
 **************************************************************************/
//#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#define ABORT_ON_ENDSTOP_HIT_INIT true
/**************************************************************************/


/**************************************************************************
 ********************* G38.2 and G38.3 Probe Target ***********************
 **************************************************************************
 *                                                                        *
 * Probe target - similar to G28 except it uses the Z_MIN endstop         *
 * for all three axes                                                     *
 *                                                                        *
 **************************************************************************/
//#define G38_PROBE_TARGET

// minimum distance in mm that will produce a move
// (determined using the print statement in check_move)
#define G38_MINIMUM_MOVE 0.0275
/**************************************************************************/


/**************************************************************************
 ************************* Scad Mesh Output *******************************
 **************************************************************************
 *                                                                        *
 * Enable if you prefer your output in JSON format                        *
 * suitable for SCAD or JavaScript mesh visualizers.                      *
 *                                                                        *
 * Visualize meshes in OpenSCAD using the included script.                *
 *                                                                        *
 * scad/MK4duoMesh.scad                                                   *
 *                                                                        *
 * By Scott Latherine @Thinkyhead                                         *
 *                                                                        *
 **************************************************************************/
//#define SCAD_MESH_OUTPUT
/**************************************************************************/


/**************************************************************************
 ****************************** R/C Servo *********************************
 **************************************************************************/
//#define ENABLE_SERVOS
// Number of servos
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it defining as 0 will disable the servo subsystem
#define NUM_SERVOS 0
// Servo index starts with 0 for M280 command
//
// Servo deactivation
// With this option servos are powered only during movement, then turned off to prevent jitter.
//#define DEACTIVATE_SERVOS_AFTER_MOVE

// Delay (in milliseconds) before turning the servo off. This depends on the servo speed.
// 300ms is a good value but you can try less delay.
// If the servo can't reach the requested position, increase it.
#define SERVO_DEACTIVATION_DELAY 300
/**************************************************************************/


/***********************************************************************
 *************************** Late Z axis *******************************
 ***********************************************************************
 *                                                                     *
 * Enable Z the last moment.                                           *
 * Needed if your Z driver overheats.                                  *
 *                                                                     *
 * This feature is not compatible with delta printer.                  *
 *                                                                     *
 * Uncomment Z_LATE_ENABLE to enable this feature                      *
 *                                                                     *
 ***********************************************************************/
//#define Z_LATE_ENABLE
/***********************************************************************/


/***********************************************************************
 ************************* Ahead slowdown ******************************
 ***********************************************************************
 *                                                                     *
 * The movements slow down when the look ahead buffer                  *
 * is only half full.                                                  *
 *                                                                     *
 ***********************************************************************/
#define SLOWDOWN
/***********************************************************************/


/***********************************************************************
 *************************** Quick home ********************************
 ***********************************************************************
 *                                                                     *
 * If both x and y are to be homed, a diagonal move will               *
 * be performed initially.                                             *
 *                                                                     *
 * This feature is not compatible with delta printer.                  *
 * This feature is enabled by default for scara printer.               *
 *                                                                     *
 ***********************************************************************/
//#define QUICK_HOME
/***********************************************************************/


/***********************************************************************
 ************************* Home Y before X *****************************
 ***********************************************************************
 *                                                                     *
 * When G28 is called, this option will make Y home before X           *
 *                                                                     *
 * This feature is not compatible with delta and scara printer.        *
 *                                                                     *
 ***********************************************************************/
//#define HOME_Y_BEFORE_X
/***********************************************************************/


/***********************************************************************
 *********************** Force Home XY before Z ************************
 ***********************************************************************
 *                                                                     *
 * When G28 is called, this option force XY home before Z              *
 *                                                                     *
 * This feature is not compatible with delta and scara printer.        *
 *                                                                     *
 ***********************************************************************/
//#define FORCE_HOME_XY_BEFORE_Z
/***********************************************************************/


/**************************************************************************
 ***************************** Babystepping *******************************
 **************************************************************************
 *                                                                        *
 * Babystepping enables movement of the axes by tiny increments without   *
 * changing the current position values. This feature is used primarily   *
 * to adjust the Z axis in the first layer of a print in real-time.       *
 *                                                                        *
 * Warning: Does not respect endstops!                                    *
 *                                                                        *
 **************************************************************************/
//#define BABYSTEPPING

// Also enable X/Y Babystepping. Not supported on DELTA!
//#define BABYSTEP_XY  

// Change if Z babysteps should go the other way
#define BABYSTEP_INVERT_Z false
// Babysteps are very small. Increase for faster motion.
#define BABYSTEP_MULTIPLICATOR 100
// Enable to combine M851 and Babystepping
//#define BABYSTEP_ZPROBE_OFFSET
// Double-click on the Status Screen for Z Babystepping.
//#define DOUBLECLICK_FOR_Z_BABYSTEPPING
// Maximum interval between clicks, in milliseconds.
// Note: Extra time may be added to mitigate controller latency.
#define DOUBLECLICK_MAX_INTERVAL 1250

// Enable graphical overlay on Z-offset editor
//#define BABYSTEP_ZPROBE_GFX_OVERLAY
// Reverses the direction of the CW/CCW indicators 
//#define BABYSTEP_ZPROBE_GFX_REVERSE
/**************************************************************************/


/**************************************************************************
 *************************** Firmware retract *****************************
 **************************************************************************
 *                                                                        *
 * Firmware based and LCD controlled retract                              *
 *                                                                        *
 * Add G10 / G11 commands for automatic firmware-based retract / recover. *
 * Use M207 and M208 to define parameters for retract / recover.          *
 *                                                                        *
 * Use M209 to enable or disable auto-retract.                            *
 * With auto-retract enabled, all G1 E moves over the MIN_RETRACT length  *
 * will be converted to firmware-based retract/recover moves.             *
 *                                                                        *
 * Be sure to turn off auto-retract during filament change.               *
 *                                                                        *
 * Note that M207 / M208 / M209 settings are saved to EEPROM.             *
 *                                                                        *
 **************************************************************************/
//#define FWRETRACT                       // ONLY PARTIALLY TESTED

#define MIN_AUTORETRACT               0.1 // When auto-retract is on, convert E moves of this length and over
#define MAX_AUTORETRACT              10.0 // Upper limit for auto-retract conversion
#define RETRACT_LENGTH                3   // Default retract length (positive mm)
#define RETRACT_LENGTH_SWAP          13   // Default swap retract length (positive mm), for extruder change
#define RETRACT_FEEDRATE             45   // Default feedrate for retracting (mm/s)
#define RETRACT_ZLIFT                 0   // Default retract Z-lift
#define RETRACT_RECOVER_LENGTH        0   // Default additional recover length (mm, added to retract length when recovering)
#define RETRACT_RECOVER_LENGTH_SWAP   0   // Default additional swap recover length (mm, added to retract length when recovering from extruder change)
#define RETRACT_RECOVER_FEEDRATE      8   // Default feedrate for recovering from retraction (mm/s)
#define RETRACT_RECOVER_FEEDRATE_SWAP 8   // Default feedrate for recovering from swap retraction (mm/s)
/**************************************************************************/


/*****************************************************************************************
 ************************************ Dual X-carriage ************************************
 *****************************************************************************************
 *                                                                                       *
 * A dual x-carriage design has the advantage that the inactive extruder can be parked   *
 * which prevents hot-end ooze contaminating the print. It also reduces the weight of    *
 * each x-carriage allowing faster printing speeds.                                      *
 *                                                                                       *
 *****************************************************************************************/
//#define DUAL_X_CARRIAGE

// Configuration for second X-carriage
// Note: the first x-carriage is defined as the x-carriage which homes to the minimum endstop;
// the second x-carriage always homes to the maximum endstop.
#define X2_MIN_POS 80     // set minimum to ensure second x-carriage doesn't hit the parked first X-carriage
#define X2_MAX_POS 353    // set maximum to the distance between toolheads when both heads are homed
#define X2_HOME_DIR 1     // the second X-carriage always homes to the maximum endstop position
#define X2_HOME_POS X2_MAX_POS // default home position is the maximum carriage position
// However: In this mode the HOTEND_OFFSET_X value for the second extruder provides a software
// override for X2_HOME_POS. This also allow recalibration of the distance between the two endstops
// without modifying the firmware (through the "M218 T1 X???" command).
// Remember: you should set the second extruder x-offset to 0 in your slicer.

// There are a few selectable movement modes for dual x-carriages using M605 S<mode>
//    Mode 0 (DXC_FULL_CONTROL_MODE): Full control. The slicer has full control over both x-carriages and can achieve optimal travel results
//                                    as long as it supports dual x-carriages. (M605 S0)
//    Mode 1 (DXC_AUTO_PARK_MODE)   : Auto-park mode. The firmware will automatically park and unpark the x-carriages on tool changes so
//                                    that additional slicer support is not required. (M605 S1)
//    Mode 2 (DXC_DUPLICATION_MODE) : Duplication mode. The firmware will transparently make the second x-carriage and extruder copy all
//                                    actions of the first x-carriage. This allows the printer to print 2 arbitrary items at
//                                    once. (2nd extruder x offset and temp offset are set using: M605 S2 [Xnnn] [Rmmm])

// This is the default power-up mode which can be later using M605.
#define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_FULL_CONTROL_MODE

// Default settings in "Auto-park Mode"
#define TOOLCHANGE_PARK_ZLIFT   0.2      // the distance to raise Z axis when parking an extruder
#define TOOLCHANGE_UNPARK_ZLIFT 1        // the distance to raise Z axis when unparking an extruder

// Default x offset in duplication mode (typically set to half print bed width)
#define DEFAULT_DUPLICATION_X_OFFSET 100
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** X-axis two driver ************************************
 *****************************************************************************************
 *                                                                                       *
 * A single X stepper driver is usually used to drive 2 stepper motors.                  *
 * Uncomment this define to utilize a separate stepper driver for each X axis motor.     *
 *                                                                                       *
 *****************************************************************************************/
//#define X_TWO_STEPPER

// Define if the two X drives need to rotate in opposite directions
#define INVERT_X2_VS_X_DIR false
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Y-axis two driver ************************************
 *****************************************************************************************
 *                                                                                       *
 * A single Y stepper driver is usually used to drive 2 stepper motors.                  *
 * Uncomment this define to utilize a separate stepper driver for each Y axis motor.     *
 *                                                                                       *
 *****************************************************************************************/
//#define Y_TWO_STEPPER

// Define if the two Y drives need to rotate in opposite directions
#define INVERT_Y2_VS_Y_DIR false
/*****************************************************************************************/


/*****************************************************************************************
 ************************** Z-axis two - three - four  driver ****************************
 *****************************************************************************************
 *                                                                                       *
 * A single Z stepper driver is usually used to drive 2 stepper motors.                  *
 * Uncomment this define to utilize a separate stepper driver for each Z axis motor.     *
 *                                                                                       *
 *****************************************************************************************/
//#define Z_TWO_STEPPER
//#define Z_THREE_STEPPER
//#define Z_FOUR_STEPPER

// Define directions of the Z2, Z3 and Z4 drives (if they're enabled) relative to the Z
// drive direction
#define INVERT_Z2_VS_Z_DIR false
#define INVERT_Z3_VS_Z_DIR false
#define INVERT_Z4_VS_Z_DIR false

// Z TWO ENDSTOPS is a feature to enable the use of 2 endstops for both Z steppers
//#define Z_TWO_ENDSTOPS
// Z THREE ENDSTOPS is a feature to enable the use of 3 endstops for three Z steppers
//#define Z_THREE_ENDSTOPS
// Z FOUR ENDSTOPS is a feature to enable the use of 4 endstops for four Z steppers
//#define Z_FOUR_ENDSTOPS
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** XY Frequency limit ***********************************
 *****************************************************************************************
 *                                                                                       *
 * See nophead's blog for more info.                                                     *
 * Not working O                                                                         *
 *                                                                                       *
 *****************************************************************************************/
//#define XY_FREQUENCY_LIMIT  15
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Skeinforge arc fix ***********************************
 *****************************************************************************************
 *                                                                                       *
 * SF send wrong arc g-codes when using Arc Point as fillet procedure.                   *
 *                                                                                       *
 *****************************************************************************************/
//#define SF_ARC_FIX
/*****************************************************************************************/


//===========================================================================
//============================= SENSORS FEATURES ============================
//===========================================================================


/**********************************************************************************
 *************************** Extruder Encoder Control *****************************
 **********************************************************************************
 *                                                                                *
 * Support for Encoder on extruder for control filament movement                  *
 * EXPERIMENTAL Function                                                          *
 *                                                                                *
 * You can compare filament moves with extruder moves to detect if the extruder   *
 * is jamming, the spool is knotted or if you are running out of filament.        *
 * You need a movement tracker, that changes a digital signal every x extrusion   *
 * steps.                                                                         *
 *                                                                                *
 * Please define/ Encoder pin for any extruder in configuration pins.              *
 *                                                                                *
 **********************************************************************************/
//#define EXTRUDER_ENCODER_CONTROL

// Enc error step is step for error detect 
#define ENC_ERROR_STEPS     500
// Enc min step It must be the minimum number of steps that the extruder does
// to get a signal from the encoder
#define ENC_MIN_STEPS        10
// For invert read signal
//#define INVERTED_ENCODER_PINS
/**********************************************************************************/


/**********************************************************************************
 *************************** Filament diameter sensor *****************************
 **********************************************************************************
 *                                                                                *
 * Support for a filament diameter sensor                                         *
 * Also allows adjustment of diameter at print time (vs  at slicing)              *
 * Single extruder only at this point (extruder 0)                                *
 *                                                                                *
 * You also need to set FILWIDTH_PIN in Configuration_pins.h                      *
 *                                                                                *
 **********************************************************************************/
//#define FILAMENT_SENSOR

#define FILAMENT_SENSOR_EXTRUDER_NUM  0     //The number of the extruder that has the filament sensor (0,1,2,3)
#define MEASUREMENT_DELAY_CM         14     //measurement delay in cm.  This is the distance from filament sensor to middle of barrel
#define MEASURED_UPPER_LIMIT          2.00  //upper limit factor used for sensor reading validation in mm
#define MEASURED_LOWER_LIMIT          1.35  //lower limit factor for sensor reading validation in mm
#define MAX_MEASUREMENT_DELAY        20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)

//defines used in the code
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially

//When using an LCD, uncomment the line below to display the Filament sensor data on the last line instead of status.  Status will appear for 5 sec.
//#define FILAMENT_LCD_DISPLAY
/**********************************************************************************/


/**********************************************************************************
 **************************** Filament Runout sensor ******************************
 **********************************************************************************
 *                                                                                *
 * Filament runout sensor such as a mechanical or opto endstop to check the       *
 * existence of filament                                                          *
 * By default the firmware assumes                                                *
 * logic high = filament available                                                *
 * low = filament run out                                                         *
 * Single extruder only at this point (extruder 0)                                *
 *                                                                                *
 * If you mount DAV system encoder filament runout (By D'angella Vincenzo)        *
 * define FILAMENT RUNOUT DAV SYSTEM                                              *
 * Put DAV_PIN for encoder input in Configuration_Pins.h                          *
 *                                                                                *
 * You also need to set FIL RUNOUT PIN in Configuration_pins.h                    *
 *                                                                                *
 **********************************************************************************/
//#define FILAMENT_RUNOUT_SENSOR

// DAV system ancoder filament runout
//#define FILAMENT_RUNOUT_DAV_SYSTEM

// Set true or false should assigned
#define FIL_RUNOUT_PIN_INVERTING true
// Uncomment to use internal pullup for pin if the sensor is defined
//#define ENDSTOPPULLUP_FIL_RUNOUT
// Time for double check switch in millisecond. Set 0 for disabled
#define FILAMENT_RUNOUT_DOUBLE_CHECK 0
// Script execute when filament run out
#define FILAMENT_RUNOUT_SCRIPT "M600"
/**********************************************************************************/


/**************************************************************************
 *********************** Power consumption sensor *************************
 **************************************************************************
 *                                                                        *
 * Support for a current sensor (Hall effect sensor like ACS712) for      *
 * measure the power consumption. Since it's more simple to deal with,    *
 * we measure the DC current and we assume that POWER_VOLTAGE that comes  *
 * from your power supply it's almost stable.                             *
 * You have to change the POWER_SENSITIVITY with the one that you can     *
 * find in the datasheet. (in case of ACS712: set to .100 for 20A version *
 * or set .066 for 30A version).                                          *
 *                                                                        *
 * After setted POWER_VOLTAGE and POWER_SENSITIVITY you have to found     *
 * correct value for POWER_ZERO.                                          *
 * You can do it by using "M70 Z" gcode and read the calculated value     *
 * from serial messages.                                                  *
 * Before calling "M70 Z" you have to disconnect the cable for measure    *
 * the current from the sensor leaving only +5, OUT and GND connections.  *
 * Insert new values into FW and recompile.                               *
 * Now you can reconnect the current cable to the sensor.                 *
 *                                                                        *
 * Now you have to set right value for POWER_ERROR.                       *
 * Get a good multimeter and meacure DC current coming out from the       *
 * power supply.                                                          *
 * In order to get an accurate value power-on something                   *
 * (Eg. Heater, Motor, Fan) DO NOT POWER-ON THE BED OR YOU MAY KILL IT!   *
 * Call "M70 Ax" where 'x' is the value measured by the multimeter.       *
 * Insert new values into FW and recompile.                               *
 *                                                                        *
 * With this module we measure the Printer power consumption ignoring     *
 * the Power Supply power consumption,                                    *
 * so we consider the POWER_EFFICIENCY of our supply to be 100%.          *
 * WARNING: from this moment the procedure can be REALLY HARMFUL to       *
 * health unless you have a little experience so:                         *
 * DO NOT DO IT IF YOU DO NOT KNOW WHAT YOU ARE DOING!!!                  *
 * If you want to approximately add the supply consumption you have       *
 * measure the AC current with a good multimeter and moltiple it with the *
 * mains voltage (110V AC - 220V AC).                                     *
 * MULTIMETER_WATT = MULTIMETER_CURRENT * MAINS_VOLTAGE                   *
 * Call "M70 Wx" where 'x' is MULTIMETER_WATT;                            *
 * Insert new values into FW and recompile.                               *
 *                                                                        *
 * Now you AC712 it should be calibrated.                                 *
 *                                                                        *
 * You also need to set POWER_CONSUMPTION_PIN in pins.h                   *
 *                                                                        *
 **************************************************************************/
//#define POWER_CONSUMPTION

#define POWER_VOLTAGE      12.00    //(V) The power supply OUT voltage
#define POWER_SENSITIVITY   0.066   //(V/A) How much increase V for 1A of increase
#define POWER_OFFSET        0.005   //(A) Help to get 0A when no load is connected.
#define POWER_ZERO          2.500   //(V) The /\V coming out from the sensor when no current flow.
#define POWER_ERROR         0.0     //(%) Ammortize measure error.
#define POWER_EFFICIENCY  100.0     //(%) The power efficency of the power supply

//When using an LCD, uncomment the line below to display the Power consumption sensor data on the last line instead of status. Status will appear for 5 sec.
//#define POWER_CONSUMPTION_LCD_DISPLAY
/**************************************************************************/


/**************************************************************************
 ****************************** Flow sensor *******************************
 **************************************************************************
 *                                                                        *
 * Flow sensors for water circulators, usefull in case of coolers using   *
 * water or other liquid as heat vector                                   *
 *                                                                        *
 * You also need to set FLOWMETER PIN in Configurations_pins.h            *
 *                                                                        *
 **************************************************************************/
//#define FLOWMETER_SENSOR

#define FLOWMETER_MAXFLOW  6.0      // Liters per minute max
#define FLOWMETER_MAXFREQ  55       // frequency of pulses at max flow

// uncomment this to kill print job under the min flow rate, in liters/minute
//#define MINFLOW_PROTECTION 4      
/**************************************************************************/


/**************************************************************************
 ************************** Door open sensor ******************************
 **************************************************************************
 *                                                                        *
 * A triggered door will prevent new commands from serial or sd card.     *
 * Setting DOOR PIN in Configuration_Pins.h                               *
 *                                                                        *
 **************************************************************************/
//#define DOOR_OPEN

// Set true or false should assigned
#define DOOR_OPEN_LOGIC false
// Uncomment to use internal pullup for pin if the sensor is defined.
//#define DOOR_OPEN_PULLUP
/**************************************************************************/


/**************************************************************************
 ***************************** Power Check ********************************
 **************************************************************************
 *                                                                        *
 * A triggered when the pin detects lack of voltage                       *
 * Setting POWER CHECK PIN in Configuration_Pins.h                        *
 *                                                                        *
 **************************************************************************/
//#define POWER_CHECK

// Set true or false should assigned
#define POWER_CHECK_LOGIC false
// Uncomment to use internal pullup for pin if the sensor is defined.
//#define POWER_CHECK_PULLUP
/**************************************************************************/


//===========================================================================
//============================= ADDON FEATURES ==============================
//===========================================================================

/************************************************************************************************************************
 ***************************************************** EEPROM ***********************************************************
 ************************************************************************************************************************
 *                                                                                                                      *
 * The microcontroller can store settings in the EEPROM, e.g. max velocity...                                           *
 * M500 - Stores parameters in EEPROM                                                                                   *
 * M501 - Reads parameters from EEPROM (if you need reset them after you changed them temporarily).                     *
 * M502 - Reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.  *
 * M503 - Print parameters on host                                                                                      *
 *                                                                                                                      *
 * Uncomment EEPROM SETTINGS to enable this feature.                                                                    *
 * Uncomment EEPROM CHITCHAT to enable EEPROM Serial responses.                                                         *
 * Uncomment EEPROM SD for use writing EEPROM on SD                                                                     *
 *                                                                                                                      *
 ************************************************************************************************************************/
//#define EEPROM_SETTINGS

//#define EEPROM_CHITCHAT // Uncomment this to enable EEPROM Serial responses.
//#define EEPROM_SD
//#define DISABLE_M503
/************************************************************************************************************************/


/*****************************************************************************************
 *************************************** SDCARD *******************************************
 ****************************************************************************************/
//#define SDSUPPORT

//#define SDSLOW              // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SDEXTRASLOW         // Use even slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY  // Use CRC checks and retries on the SD communication
//#define SD_EXTENDED_DIR     // Show extended directory including file length. Don't use this with Pronterface

// Decomment this if you have external SD without DETECT_PIN
//#define SD_DISABLED_DETECT
// Some RAMPS and other boards don't detect when an SD card is inserted. You can work
// around this by connecting a push button or single throw switch to the pin defined
// as SD_DETECT_PIN in your board's pins definitions.
// This setting should be disabled unless you are using a push button, pulling the pin to ground.
// Note: This is always disabled for ULTIPANEL (except ELB_FULL_GRAPHIC_CONTROLLER).
//#define SD_DETECT_INVERTED

#define SD_FINISHED_STEPPERRELEASE true  //if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E" // You might want to keep the z enabled so your bed stays in place.

#define SDCARD_RATHERRECENTFIRST  //reverse file order of sd card menu display. Its sorted practically after the file system block order.
// if a file is deleted, it frees a block. hence, the order is not purely chronological. To still have auto0.g accessible, there is again the option to do that.
// using:
//#define MENU_ADDAUTOSTART

// This enable the firmware to write some configuration that require frequent update, on the SD card
//#define SD_SETTINGS                     // Uncomment to enable
#define SD_CFG_SECONDS        300         // seconds between update
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** LCD Language ****************************************
 *****************************************************************************************
 *                                                                                       *
 * Here you may choose the language used by MK4duo on the LCD menus,                     *
 * the following list of languages are available:                                        *
 *    en, an, bg, ca, cn, cz, de, el, el-gr, es, eu, fi, fr, gl, hr, it,                 *
 *    kana, kana_utf8, nl, pl, pt, pt_utf8, pt-br, pt-br_utf8, ru, tr                    *
 *                                                                                       *
 * 'en':'English',          'an':'Aragonese',   'bg':'Bulgarian',       'ca':'Catalan',  *
 * 'cn':'Chinese',          'cz':'Czech',       'de':'German',          'el':'Greek',    *
 * 'el-gr':'Greek (Greece)' 'es':'Spanish',     'eu':'Basque-Euskera',  'fi':'Finnish',  *
 * 'fr':'French',           'gl':'Galician',    'hr':'Croatian',        'it':'Italian',  *
 * 'kana':'Japanese',       'kana_utf8':'Japanese (UTF8)'               'nl':'Dutch',    *
 * 'pl':'Polish',           'pt':'Portuguese',  'ru':'Russian',         'tr':'Turkish',  *
 * 'uk':'Ukrainian',        'pt_utf8':'Portuguese (UTF8)',              'hu':'Hungarian',*
 * 'pt-br':'Portuguese (Brazilian)',                                                     *
 * 'pt-br_utf8':'Portuguese (Brazilian UTF8)',                                           *
 *                                                                                       *
 *****************************************************************************************/
#define LCD_LANGUAGE en
/*****************************************************************************************/


/***********************************************************************
 ******************************* LCD ***********************************
 ***********************************************************************/

// LCD Character Set
//
// Note: This option is NOT applicable to Graphical Displays.
//
// All character-based LCD's provide ASCII plus one of these
// language extensions:
//
//  - JAPANESE ... the most common
//  - WESTERN  ... with more accented characters
//  - CYRILLIC ... for the Russian language
//
// To determine the language extension installed on your controller:
//
//  - Compile and upload with LCD_LANGUAGE set to 'test'
//  - Click the controller to view the LCD menu
//  - The LCD will display Japanese, Western, or Cyrillic text
//
// :['JAPANESE', 'WESTERN', 'CYRILLIC']
//
#define DISPLAY_CHARSET_HD44780 JAPANESE
 
#define SHOW_BOOTSCREEN
//#define SHOW_CUSTOM_BOOTSCREEN
#define STRING_SPLASH_LINE1 "v" SHORT_BUILD_VERSION       // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE      // will be shown during bootup in line 2
#define BOOTSCREEN_TIMEOUT 2000

// LCD TYPE
//
// You may choose ULTRA_LCD if you have character based LCD with 16x2, 16x4, 20x2,
// 20x4 char/lines or DOGLCD for the full graphics display with 128x64 pixels
// (ST7565R family). (This option will be set automatically for certain displays.)
//
// IMPORTANT NOTE: The U8glib library is required for Full Graphic Display!
//                 https://github.com/olikraus/U8glib_Arduino
//
//#define ULTRA_LCD   // Character based
//#define DOGLCD      // Full graphics display


// Additional options for Graphical Displays
// 
// Use the optimizations here to improve printing performance,
// which can be adversely affected by graphical display drawing,
// especially when doing several short moves, and when printing
// on DELTA and SCARA machines.
// 
// Some of these options may result in the display lagging behind
// controller events, as there is a trade-off between reliable
// printing performance versus fast display updates.

// Enable to save many cycles by drawing a hollow frame on the Info Screen
#define XYZ_HOLLOW_FRAME

// Enable to save many cycles by drawing a hollow frame on Menu Screens
#define MENU_HOLLOW_FRAME

// A bigger font is available for edit items. Costs 3120 bytes of PROGMEM.
// Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.
//#define USE_BIG_EDIT_FONT

// A smaller font may be used on the Info Screen. Costs 2300 bytes of PROGMEM.
// Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.
//#define USE_SMALL_INFOFONT

// Enable this option and reduce the value to optimize screen updates.
// The normal delay is 10µs. Use the lowest value that still gives a reliable display.
//#define DOGM_SPI_DELAY_US 5

// ENCODER SETTINGS

// This option overrides the default number of encoder pulses needed to
// produce one step. Should be increased for high-resolution encoders.
//#define ENCODER_PULSES_PER_STEP 1

// Use this option to override the number of step signals required to
// move between next/prev menu items.
//#define ENCODER_STEPS_PER_MENU_ITEM 5

//#define LCD_SCREEN_ROT_90    // Rotate screen orientation for graphics display by 90 degree clockwise
//#define LCD_SCREEN_ROT_180   // Rotate screen orientation for graphics display by 180 degree clockwise
//#define LCD_SCREEN_ROT_270   // Rotate screen orientation for graphics display by 270 degree clockwise

//#define INVERT_CLICK_BUTTON           // Option for invert encoder button logic
//#define INVERT_BACK_BUTTON            // Option for invert back button logic if avaible

// Encoder Direction Options
// Test your encoder's behavior first with both options disabled.
//
//  Reversed Value Edit and Menu Nav? Enable REVERSE_ENCODER_DIRECTION.
//  Reversed Menu Navigation only?    Enable REVERSE_MENU_DIRECTION.
//  Reversed Value Editing only?      Enable BOTH options.

// This option reverses the encoder direction everywhere
//  Set this option if CLOCKWISE causes values to DECREASE
//#define REVERSE_ENCODER_DIRECTION

// This option reverses the encoder direction for navigating LCD menus.
//  If CLOCKWISE normally moves DOWN this makes it go UP.
//  If CLOCKWISE normally moves UP this makes it go DOWN.
//#define REVERSE_MENU_DIRECTION

#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value

// Double-click the Encoder button on the Status Screen for Z Babystepping.
//#define DOUBLECLICK_FOR_Z_BABYSTEPPING
// Maximum interval between clicks, in milliseconds.
// Note: You may need to add extra time to mitigate controller latency.
#define DOUBLECLICK_MAX_INTERVAL 1250

// Comment to disable setting feedrate multiplier via encoder
#define ULTIPANEL_FEEDMULTIPLY

// SPEAKER/BUZZER
// If you have a speaker that can produce tones, enable it here.
// By default Marlin assumes you have a buzzer with a fixed frequency.
//#define SPEAKER

// The duration and frequency for the UI feedback sound.
// Set these to 0 to disable audio feedback in the LCD menus.

// Note: Test audio output with the G-Code:
//  M300 S<frequency Hz> P<duration ms>
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000

//Display Voltage Logic Selector on Alligator Board
//#define UI_VOLTAGE_LEVEL 0 // 3.3 V
#define UI_VOLTAGE_LEVEL 1   // 5 V

// Include a page of printer information in the LCD Main Menu
#define LCD_INFO_MENU

// Scroll a longer status message into view
//#define STATUS_MESSAGE_SCROLLING

// On the Info Screen, display XY with one decimal place when possible
//#define LCD_DECIMAL_SMALL_XY

// The timeout (in ms) to return to the status screen from sub-menus
//#define LCD_TIMEOUT_TO_STATUS 15000

// CONTROLLER TYPE: Standard

// MK4duo supports a wide variety of controllers.
// Enable one of the following options to specify your controller.

// ULTIMAKER Controller.
//#define ULTIMAKERCONTROLLER

// ULTIPANEL as seen on Thingiverse.
//#define ULTIPANEL

// Cartesio UI
// http://mauk.cc/webshop/cartesio-shop/electronics/user-interface
//
//#define CARTESIO_UI

// Original RADDS Display from Willy
// http://max3dshop.org/index.php/default/elektronik/radds-lcd-sd-display-with-reset-and-back-buttom.html
//#define RADDS_DISPLAY

// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)
// http://reprap.org/wiki/PanelOne
//
//#define PANEL_ONE

// MaKr3d Makr-Panel with graphic controller and SD support.
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//
//#define MAKRPANEL

// ReprapWorld Graphical LCD
// https://reprapworld.com/?products_details&products_id/1218
//
//#define REPRAPWORLD_GRAPHICAL_LCD

// Activate one of these if you have a Panucatt Devices
// Viki 2.0 or mini Viki with Graphic LCD
// http://panucatt.com
//
//#define VIKI2
//#define miniVIKI

// Adafruit ST7565 Full Graphic Controller.
// https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/
//
//#define ELB_FULL_GRAPHIC_CONTROLLER

// RepRapDiscount Smart Controller.
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//
// Note: Usually sold with a white PCB.
//
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

// GADGETS3D G3D LCD/SD Controller
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//
// Note: Usually sold with a blue PCB.
//
//#define G3D_PANEL

// RepRapDiscount FULL GRAPHIC Smart Controller
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

// MakerLab Mini Panel with graphic
// controller and SD support - http://reprap.org/wiki/Mini_panel
//
//#define MINIPANEL

// RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//
// REPRAPWORLD_KEYPAD_MOVE_STEP sets how much should the robot move when a key
// is pressed, a value of 10.0 means 10mm per click.
//
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0

// RigidBot Panel V1.0
// http://www.inventapart.com/
//
//#define RIGIDBOT_PANEL

// BQ LCD Smart Controller shipped by
// default with the BQ Hephestos 2 and Witbox 2.
//
//#define BQ_LCD_SMART_CONTROLLER

// CONTROLLER TYPE: I2C
//
// Note: These controllers require the installation of Arduino's LiquidCrystal_I2C
// library. For more info: https://github.com/kiyoshigawa/LiquidCrystal_I2C

// Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
//
//#define RA_CONTROL_PANEL

// Sainsmart YW Robot (LCM1602) LCD Display
//
//#define LCD_I2C_SAINSMART_YWROBOT

// Generic LCM1602 LCD adapter
//
//#define LCM1602

// PANELOLU2 LCD with status LEDs,
// separate encoder and click inputs.
//
// Note: This controller requires Arduino's LiquidTWI2 library v1.2.3 or later.
// For more info: https://github.com/lincomatic/LiquidTWI2
//
// Note: The PANELOLU2 encoder click input can either be directly connected to
// a pin (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
//
//#define LCD_I2C_PANELOLU2

// Panucatt VIKI LCD with status LEDs,
// integrated click & L/R/U/D buttons, separate encoder inputs.
//
//#define LCD_I2C_VIKI

// SSD1306 OLED full graphics generic display
//
//#define U8GLIB_SSD1306

// WANHAO D6 SSD1309 OLED full graphics
//
//#define WANHAO_D6_OLED

// SAV OLEd LCD module support using either SSD1306 or SH1106 based LCD modules
//
//#define SAV_3DGLCD

// ANET DISPLAY
//
//#define ANET_KEYPAD_LCD
//#define ANET_FULL_GRAPHICS_LCD

// CONTROLLER TYPE: Shift register panels
//
// 2 wire Non-latching LCD SR from https://goo.gl/aJJ4sH
// LCD configuration: http://reprap.org/wiki/SAV_3D_LCD
//
//#define SAV_3DLCD

// CONTROLLER TYPE: Serial display

// Nextion 4.3" HMI panel model NX4827T043_11
//#define NEXTION
// Define Serial it use
#define NEXTION_SERIAL 1
// Define ms for update display (for 8 the default value is best, for 32 bit 1500 is best)
#define NEXTION_UPDATE_INTERVAL 3000
// For GFX preview visualization enable NEXTION GFX
//#define NEXTION_GFX
// Define name firmware file for Nextion on SD
#define NEXTION_FIRMWARE_FILE "mk4duo.tft"

// Show a progress bar on HD44780 LCDs for SD printing
//#define LCD_PROGRESS_BAR
// Amount of time (ms) to show the bar
#define PROGRESS_BAR_BAR_TIME 5000
// Amount of time (ms) to show the status message
#define PROGRESS_BAR_MSG_TIME 1500
// Amount of time (ms) to retain the status message (0=forever)
#define PROGRESS_MSG_EXPIRE 0
// Uncomment this to show messages for MSG_TIME then hide them
//#define PROGRESS_MSG_ONCE
// Add a menu item to test the progress bar:
//#define LCD_PROGRESS_BAR_TEST
/************************************************************************************************/


/**************************************************************************
 *************************** Canon RC-1 Remote ****************************
 **************************************************************************
 *                                                                        *
 * M240 Triggers a camera by emulating a Canon RC-1 Remote                *
 * Data from: http://www.doc-diy.net/photo/rc-1_hacked/                   *
 *                                                                        *
 * You also need to set PHOTOGRAPH_PIN in Configuration_pins.h            *
 *                                                                        *
 **************************************************************************/
//#define PHOTOGRAPH
/**************************************************************************/


/**************************************************************************
 ***************************** Camera trigger *****************************
 **************************************************************************
 *                                                                        *
 * M240 Triggering CHDK to take a picture see how to use it here:         *
 * http://captain-slow.dk/2014/03/09/3d-printing-timelapses/              *
 *                                                                        *
 * You also need to set CHDK_PIN in Configuration_pins.h                  *
 *                                                                        *
 **************************************************************************/
//#define CHDK

#define CHDK_DELAY 50   //How long in ms the pin should stay HIGH before going LOW again
/**************************************************************************/


/**************************************************************************
 *********************** RIFD module card reader **************************
 **************************************************************************
 *                                                                        *
 * Support RFID module card reader width UART interface.                  *
 * This module mount chip MFRC522 designed to communicate with            *
 * ISO/IEC 14443 A/MIFARE cards and transponders without additional       *
 * active circuitry                                                       *
 *                                                                        *
 * New command for this system is:                                        *
 * M522 T<extruder> R<read> or W<write>                                   *
 *                                                                        *
 * Define if you used and Serial used.                                    *
 *                                                                        *
 **************************************************************************/
//#define RFID_MODULE

#define RFID_SERIAL 1
/**************************************************************************/


/**************************************************************************
 ********************************* BLINKM *********************************
 **************************************************************************
 *                                                                        *
 * Support for BlinkM/CyzRgb                                              *
 *                                                                        *
 **************************************************************************/
//#define BLINKM
/**************************************************************************/


/**************************************************************************
 ******************************** RGB LED *********************************
 **************************************************************************
 *                                                                        *
 * Support for an RGB LED using 3 separate pins with optional PWM         *
 *                                                                        *
 **************************************************************************/
//#define RGB_LED
//#define RGBW_LED
/**************************************************************************/


/**************************************************************************
 *************************** PCA 9632 PWM LED *****************************
 **************************************************************************
 *                                                                        *
 * Support PCA 9632 PWM LED driver                                        *
 *                                                                        *
 **************************************************************************/
//#define PCA9632
/**************************************************************************/


/**************************************************************************
 ********************* Adafruit Neopixel LED driver ***********************
 **************************************************************************
 *                                                                        *
 * Support for Adafruit Neopixel LED driver                               *
 *                                                                        *
 **************************************************************************/
//#define NEOPIXEL_RGB_LED
//#define NEOPIXEL_RGBW_LED

#define NEOPIXEL_PIXELS 16
// Cycle through colors at startup
//#define NEOPIXEL_STARTUP_TEST
/**************************************************************************/


/********************************************************************************
 ***************************** Printer Event LEDs *******************************
 ********************************************************************************
 *                                                                              *
 * During printing, the LEDs will reflect the printer status:                   *
 *                                                                              *
 *  - Gradually change from blue to violet as the heated bed gets to target temp*                                                                 *
 *  - Gradually change from violet to red as the hotend gets to temperature     *
 *  - Change to white to illuminate work surface                                *
 *  - Change to green once print has finished                                   *
 *  - Turn off after the print has finished and the user has pushed a button    *
 *                                                                              *
 ********************************************************************************/
//#define PRINTER_EVENT_LEDS
/********************************************************************************/


/**************************************************************************
 ********************************* Laser **********************************
 **************************************************************************
 *                                                                        *
 * Support for laser beam                                                 *
 * Check also Configuration_Laser.h                                       *
 *                                                                        *
 **************************************************************************/
//#define LASER
/**************************************************************************/


/**************************************************************************
 ******************************* CNC Router *******************************
 **************************************************************************
 *                                                                        *
 * Support for CNC Router                                                 *
 * Check also Configuration_CNCRouter.h                                   *
 *                                                                        *
 **************************************************************************/
//#define CNCROUTER
/**************************************************************************/


/**************************************************************************
 ******************************* Case Light *******************************
 **************************************************************************
 *                                                                        *
 * M355 Case Light on-off / brightness                                    *
 *                                                                        *
 **************************************************************************/
//#define CASE_LIGHT

// set to true if case light is ON when pin is at 0
#define INVERT_CASE_LIGHT false
// set default power up state to on or off
#define CASE_LIGHT_DEFAULT_ON false
// set power up brightness 0-255 ( only used if on PWM
// and if CASE_LIGHT_DEFAULT is set to on)
#define CASE_LIGHT_DEFAULT_BRIGHTNESS 255   
/**************************************************************************/


//===========================================================================
//========================= ADVANCED MOTION FEATURES ========================
//===========================================================================

/***********************************************************************
 ********************* Stepper auto deactivation ***********************
 ***********************************************************************
 *                                                                     *
 * Default stepper release if idle. Set to 0 to deactivate.            *
 * Steppers will shut down DEFAULT_STEPPER_DEACTIVE_TIME seconds after *
 * the last move when DISABLE_INACTIVE_? is defined.                   *
 * Time can be set by M18 and M84.                                     *
 *                                                                     *
 ***********************************************************************/
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X
#define DISABLE_INACTIVE_Y
#define DISABLE_INACTIVE_Z
#define DISABLE_INACTIVE_E
/***********************************************************************/


/***********************************************************************
 ********************** Double / Quad Stepping *************************
 ***********************************************************************
 *                                                                     *
 * Disable double / quad stepping                                      *
 *                                                                     *
 ***********************************************************************/
//#define DISABLE_DOUBLE_QUAD_STEPPING
/***********************************************************************/


/***********************************************************************
 ************************* Low speed stepper ***************************
 ***********************************************************************
 *                                                                     *
 * Set this if you find stepping unreliable,                           *
 * or if using a very fast CPU.                                        *
 *                                                                     *
 ***********************************************************************/
// (µs) The smallest stepper pulse allowed
#define MINIMUM_STEPPER_PULSE 0
/***********************************************************************/


/***********************************************************************
 *************************** Microstepping *****************************
 ***********************************************************************
 *                                                                     *
 * Microstep setting (Only functional when stepper driver              *
 * microstep pins are connected to MCU.                                *
 *                                                                     *
 * Alligator Board support 16 or 32 only value                         *
 *                                                                     *
 ***********************************************************************/
//#define USE_MICROSTEPS

// X Y Z E - [1,2,4,8,16,32]
#define MICROSTEP_MODES {16, 16, 16, 16}
/***********************************************************************/


/***********************************************************************
 ************************** Motor's current ****************************
 ***********************************************************************/
// Motor Current setting (Only functional on ALLIGATOR BOARD)
// X Y Z E0 E1 E2 E3 - Values 0 - 2.5 A
#define MOTOR_CURRENT {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}

// Motor Current setting (Only functional when motor driver current
// ref pins are connected to a digital trimpot on supported boards)
// Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)
#define DIGIPOT_MOTOR_CURRENT {135, 135, 135, 135, 135}

// Motor Current for XY, Z, E in mA
#define PWM_MOTOR_CURRENT {1200, 1000, 1000}
/***********************************************************************/


/***********************************************************************
 **************************** I2C DIGIPOT ******************************
 ***********************************************************************
 *                                                                     *
 * I2C based DIGIPOT like on the Azteeg X3 Pro                         *
 *                                                                     *
 ***********************************************************************/
//#define DIGIPOT_I2C
// Number of channels available for I2C digipot, For Azteeg X3 Pro we have 8
#define DIGIPOT_I2C_NUM_CHANNELS 8
// actual motor currents in Amps, need as many here as DIGIPOT_I2C_NUM_CHANNELS
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
/***********************************************************************/


/***********************************************************************
 *************************** Toshiba steppers **************************
 ***********************************************************************
 *                                                                     *
 * Support for Toshiba steppers                                        *
 *                                                                     *
 ***********************************************************************/
//#define CONFIG_STEPPERS_TOSHIBA
/***********************************************************************/


/**********************************************************************************
 **************************** TMC26X motor drivers ********************************
 **********************************************************************************
 *                                                                                *
 * Support for TMC26X motor drivers                                               *
 * See Configuration_Motor_Driver.h for configuration stepper driver              *
 *                                                                                *
 **********************************************************************************/
//#define HAVE_TMCDRIVER
/**********************************************************************************/


/**********************************************************************************
 *********************** Trinamic TMC2130 motor drivers ***************************
 **********************************************************************************
 *                                                                                *
 * Enable this for SilentStepStick Trinamic TMC2130 SPI-configurable stepper      *
 * drivers.                                                                       *
 *                                                                                *
 * You'll also need the TMC2130Stepper Arduino library                            *
 * (https://github.com/teemuatlut/TMC2130Stepper).                                *
 *                                                                                *
 * To use TMC2130 stepper drivers in SPI mode connect your SPI2130 pins to        *
 * the hardware SPI interface on your board and define the required CS pins       *
 * in your `MYBOARD.h` file. (e.g., RAMPS 1.4 uses AUX3 pins `X_CS_PIN 53`,       *
 * Y_CS_PIN 49`, etc.).                                                           *
 *                                                                                *
 * See Configuration_Motor_Driver.h for configuration stepper driver              *
 *                                                                                *
 **********************************************************************************/
//#define HAVE_TMC2130
/**********************************************************************************/


/**********************************************************************************
 ****************************** L6470 motor drivers *******************************
 **********************************************************************************
 *                                                                                *
 * Support for L6470 motor drivers                                                *
 * You need to import the L6470 library into the arduino IDE for this.            *
 *                                                                                *
 * See Configuration_Motor_Driver.h for configuration stepper driver              *
 *                                                                                *
 **********************************************************************************/
//#define HAVE_L6470DRIVER
/**********************************************************************************/


//===========================================================================
//============================= ADVANCED FEATURES ===========================
//===========================================================================


/**********************************************************************************
 ********************************* PWM Hardware ***********************************
 **********************************************************************************
 *                                                                                *
 * Support PWM hardware for SAM processor                                         *
 * This function ability PWM hardware on SAM proccesor, tested only on ALLIGATOR! *
 *                                                                                *
 **********************************************************************************/
#define PWM_HARDWARE false
/**********************************************************************************/


/****************************************************************************************
 ************************************** Buffer stuff ************************************
 ****************************************************************************************/
// The number of linear motions that can be in the plan at any give time.
// THE BLOCK BUFFER SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts
// and ors are used to do the ring-buffering.
// For Arduino DUE setting BLOCK BUFFER SIZE to 32
#define BLOCK_BUFFER_SIZE 16

// The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
// For Arduino DUE setting to 8
#define BUFSIZE 4

// Transmission to Host Buffer Size
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
// To buffer a simple "ok" you need 4 bytes.
// For ADVANCED_OK (M105) you need 32 bytes.
// For debug-echo: 128 bytes for the optimal speed.
// Other output doesn't need to be that speedy.
// :[0, 2, 4, 8, 16, 32, 64, 128, 256]
#define TX_BUFFER_SIZE 32

// Host Receive Buffer Size
// To use flow control, set this buffer size to at least 1024 bytes.
// :[0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048]
#define RX_BUFFER_SIZE 128

// Defines the number of memory slots for saving/restoring position (G60/G61)
// The values should not be less than 1
#define NUM_POSITON_SLOTS 2

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME 20000

//
// G2/G3 Arc Support
//
// Disable this feature to save ~3226 bytes
#define ARC_SUPPORT
#define MM_PER_ARC_SEGMENT 1    // Length of each arc segment
#define N_ARC_CORRECTION  25    // Number of intertpolated segments between corrections
//#define ARC_P_CIRCLES         // Enable the 'P' parameter to specify complete circles
//#define CNC_WORKSPACE_PLANES  // Allow G2/G3 to operate in XY, ZX, or YZ planes

// Moves with fewer segments than this will be ignored and joined with the next movement
#define MIN_STEPS_PER_SEGMENT 6

// Uncomment to add the M100 Free Memory Watcher for debug purpose
//#define M100_FREE_MEMORY_WATCHER

// Comment out to remove Dump sub-command
#define M100_FREE_MEMORY_DUMPER
// Comment out to remove Corrupt sub-command
#define M100_FREE_MEMORY_CORRUPTOR
/****************************************************************************************/


/****************************************************************************************
 ********************************* Nozzle Clean Feature *********************************
 ****************************************************************************************
 *                                                                                      *
 * When enabled allows the user to send G12 to start the nozzle cleaning                *
 * process, the G-Code accepts two parameters:                                          *
 *   "P" for pattern selection                                                          *
 *   "S" for defining the number of strokes/repetitions                                 *
 *   "T" for defining the number of triangles                                           *
 *   "R" for defining the center of circle                                              *
 *                                                                                      *
 * Available list of patterns:                                                          *
 *   P0: This is the default pattern, this process requires a sponge type               *
 *       material at a fixed bed location. S defines "strokes" i.e.                     *
 *       back-and-forth movements between the starting and end points.                  *
 *                                                                                      *
 *   P1: This starts a zig-zag pattern between (X0, Y0) and (X1, Y1), "T"               *
 *       defines the number of zig-zag triangles to be done. "S" defines the            *
 *       number of strokes aka one back-and-forth movement. As an example               *
 *       sending "G12 P1 S1 T3" will execute:                                           *
 *                                                                                      *
 *          --                                                                          *
 *         |  (X0, Y1) |     /\        /\        /\     | (X1, Y1)                      *
 *         |           |    /  \      /  \      /  \    |                               *
 *       A |           |   /    \    /    \    /    \   |                               *
 *         |           |  /      \  /      \  /      \  |                               *
 *         |  (X0, Y0) | /        \/        \/        \ | (X1, Y0)                      *
 *          --         +--------------------------------+                               *
 *                       |________|_________|_________|                                 *
 *                           T1        T2        T3                                     *
 *                                                                                      *
 *   P2: This starts a circular pattern with circle with middle in                      *
 *       NOZZLE CLEAN CIRCLE MIDDLE radius of R and stroke count of S.                  *
 *       Before starting the circle nozzle goes to NOZZLE CLEAN START POINT.            *
 *                                                                                      *
 * Caveats: End point Z should use the same value as Start point Z.                     *
 *                                                                                      *
 * Attention: This is an EXPERIMENTAL feature, in the future the G-code arguments       *
 * may change to add new functionality like different wipe patterns.                    *
 *                                                                                      *
 ****************************************************************************************/
//#define NOZZLE_CLEAN_FEATURE

// Default number of pattern repetitions
#define NOZZLE_CLEAN_STROKES 12

// Default number of triangles
#define NOZZLE_CLEAN_TRIANGLES 3

// Specify positions as { X, Y, Z }
#define NOZZLE_CLEAN_START_POINT { 30, 30, (Z_MIN_POS + 1)}
#define NOZZLE_CLEAN_END_POINT   {100, 60, (Z_MIN_POS + 1)}

// Circular pattern radius
#define NOZZLE_CLEAN_CIRCLE_RADIUS 6.5
// Circular pattern circle fragments number
#define NOZZLE_CLEAN_CIRCLE_FN 10
// Middle point of circle
#define NOZZLE_CLEAN_CIRCLE_MIDDLE NOZZLE_CLEAN_START_POINT

// Moves the nozzle to the initial position
#define NOZZLE_CLEAN_GOBACK
/****************************************************************************************/


/****************************************************************************************
 ********************************** Nozzle Park Feature *********************************
 ****************************************************************************************
 *                                                                                      *
 * When enabled allows the user to define a special XYZ position, inside the            *
 * machine's topology, to park the nozzle when idle or when receiving the G27           *
 * command.                                                                             *
 *                                                                                      *
 * The "P" paramenter controls what is the action applied to the Z axis:                *
 *    P0: (Default) If current Z-pos is lower than Z-park then the nozzle will          *
 *        be raised to reach Z-park height.                                             *
 *                                                                                      *
 *    P1: No matter the current Z-pos, the nozzle will be raised/lowered to             *
 *        reach Z-park height.                                                          *
 *                                                                                      *
 *    P2: The nozzle height will be raised by Z-park amount but never going over        *
 *        the machine's limit of Z_MAX_POS.                                             *
 *                                                                                      *
 ****************************************************************************************/
//#define NOZZLE_PARK_FEATURE

// Specify a park position as { X, Y, Z }
#define NOZZLE_PARK_POINT { (X_MIN_POS + 10), (Y_MAX_POS - 10), 20 }
/****************************************************************************************/


/**************************************************************************
 ************************ Advanced Pause Park *****************************
 **************************************************************************
 *                                                                        *
 * Advanced Pause Park feature for filament change support and for parking*
 * the nozzle when paused.                                                *
 * Add the GCode M600 for initiating filament change.                     *
 *                                                                        *
 * If PARK HEAD ON PAUSE enabled, adds the GCode M125 to pause printing   *
 * and park the nozzle.                                                   *
 *                                                                        *
 * Requires an LCD display.                                               *
 * This feature is required for the default FILAMENT RUNOUT SCRIPT.       *
 *                                                                        *
 **************************************************************************/
//#define ADVANCED_PAUSE_FEATURE

#define PAUSE_PARK_X_POS 3                  // X position of hotend
#define PAUSE_PARK_Y_POS 3                  // Y position of hotend
#define PAUSE_PARK_Z_ADD 10                 // Z addition of hotend (lift)
#define PAUSE_PARK_XY_FEEDRATE 100          // X and Y axes feedrate in mm/s (also used for delta printers Z axis)
#define PAUSE_PARK_Z_FEEDRATE 5             // Z axis feedrate in mm/s (not used for delta printers)
#define PAUSE_PARK_RETRACT_FEEDRATE 20      // Initial retract feedrate in mm/s
#define PAUSE_PARK_RETRACT_LENGTH 5         // Initial retract in mm
                                            // It is a short retract used immediately after print interrupt before move to filament exchange position
#define PAUSE_PARK_COOLDOWN_TEMP 0          // Cooling temperature, if this parameter is equal to 0 no cooling.
#define PAUSE_PARK_RETRACT_2_FEEDRATE 20    // Second retract filament feedrate in mm/s - filament retract post cool down
#define PAUSE_PARK_RETRACT_2_LENGTH 20      // Second retract filament length from hotend in mm
#define PAUSE_PARK_UNLOAD_FEEDRATE 100      // Unload filament feedrate in mm/s - filament unloading can be fast
#define PAUSE_PARK_UNLOAD_LENGTH 100        // Unload filament length from hotend in mm
                                            // Longer length for bowden printers to unload filament from whole bowden tube,
                                            // shorter length for printers without bowden to unload filament from extruder only,
                                            // 0 to disable unloading for manual unloading
#define PAUSE_PARK_LOAD_FEEDRATE 100        // Load filament feedrate in mm/s - filament loading into the bowden tube can be fast
#define PAUSE_PARK_LOAD_LENGTH 100          // Load filament length over hotend in mm
                                            // Longer length for bowden printers to fast load filament into whole bowden tube over the hotend,
                                            // Short or zero length for printers without bowden where loading is not used
#define PAUSE_PARK_EXTRUDE_FEEDRATE 5       // Extrude filament feedrate in mm/s - must be slower than load feedrate
#define PAUSE_PARK_EXTRUDE_LENGTH 50        // Extrude filament length in mm after filament is load over the hotend,
                                            // 0 to disable for manual extrusion
                                            // Filament can be extruded repeatedly from the filament exchange menu to fill the hotend,
                                            // or until outcoming filament color is not clear for filament color change
#define PAUSE_PARK_NOZZLE_TIMEOUT 45        // Turn off nozzle if user doesn't change filament within this time limit in seconds
#define PAUSE_PARK_PRINTER_OFF 5            // Turn off printer if user doesn't change filament within this time limit in Minutes
#define PAUSE_PARK_NUMBER_OF_ALERT_BEEPS 5  // Number of alert beeps before printer goes quiet
#define PAUSE_PARK_NO_STEPPER_TIMEOUT       // Enable to have stepper motors hold position during filament change
                                            // even if it takes longer than DEFAULT STEPPER DEACTIVE TIME.
//#define PARK_HEAD_ON_PAUSE                // Go to filament change position on pause, return to print position on resume
/**************************************************************************/


/*****************************************************************************************
 ****************************** G20/G21 Inch mode support ********************************
 *****************************************************************************************/
//#define INCH_MODE_SUPPORT
/****************************************************************************************/


/*****************************************************************************************
 ************************************* JSON OUTPUT ***************************************
 *****************************************************************************************
 *                                                                                       *
 * M408: Report JSON-style response                                                      *
 * Report a JSON-style response by specifying the desired type using the 'S' parameter.  *
 * The following response types are supported:                                           *
 * Type 0 is a short-form response.                                                      *
 * Type 1 is like type 0 except that static values are also included.                    *
 * Type 2 is similar to the response provided by the web server for Duet Web Control.    *
 * Type 3 is an extended version of type 2 which includes some additional parameters     *
 * that aren't expected to change very frequently.                                       *
 * Type 4 is an extended version of type 2 which may be used to poll for current         *
 * printer statistics.                                                                   *
 * Type 5 reports the current machine configuration.                                     *
 *                                                                                       *
 *****************************************************************************************/
//#define JSON_OUTPUT
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** M43 pins info ***************************************
 *****************************************************************************************
 *                                                                                       *
 * M43 Pins info and testing                                                             *
 *                                                                                       *
 *****************************************************************************************/
//#define PINS_DEBUGGING
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** Auto report temperatures *********************************
 *****************************************************************************************
 *                                                                                       *
 * Auto-report temperatures with M155 S<seconds>                                         *
 *                                                                                       *
 *****************************************************************************************/
//#define AUTO_REPORT_TEMPERATURES
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** Extend capabilities report *******************************
 *****************************************************************************************
 *                                                                                       *
 * Include capabilities in M115 output                                                   *
 *                                                                                       *
 *****************************************************************************************/
//#define EXTENDED_CAPABILITIES_REPORT
/*****************************************************************************************/


/*****************************************************************************************
 *************************************** Whatchdog ***************************************
 *****************************************************************************************
 *                                                                                       *
 * The hardware watchdog should reset the microcontroller disabling all outputs,         *
 * in case the firmware gets stuck and doesn't do temperature regulation.                *
 *                                                                                       *
 * Uncomment USE_WATCHDOG to enable this feature                                         *
 *                                                                                       *
 *****************************************************************************************/
//#define USE_WATCHDOG

// If you have a watchdog reboot in an ArduinoMega2560 then the device will hang forever, as a watchdog reset will leave the watchdog on.
// The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
// However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled. And the code could hang in an interrupt routine with interrupts disabled.
//#define WATCHDOG_RESET_MANUAL
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Start / Stop Gcode ************************************
 *****************************************************************************************
 *                                                                                       *
 * Start - Stop Gcode use when Start or Stop printing width M530 command                 *
 *                                                                                       *
 *****************************************************************************************/
//#define START_GCODE
#define START_PRINTING_SCRIPT "G28\nG1 Z10 F8000"

//#define STOP_GCODE
#define STOP_PRINTING_SCRIPT "G28\nM107\nM104 T0 S0\nM140 S0\nM84\nM81"
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Proportional Font ratio *********************************
 *****************************************************************************************
 * Set the number of proportional font spaces required to fill up a typical              *
 * character space.                                                                      *
 * This can help to better align the output of commands like `G29 O` Mesh Output.        *
 *                                                                                       *
 * For clients that use a fixed-width font (like OctoPrint), leave this set to 1.0.      *
 * Otherwise, adjust according to your client and font.                                  *
 *                                                                                       *
 *****************************************************************************************/
#define PROPORTIONAL_FONT_RATIO 1
/*****************************************************************************************/

 
/*****************************************************************************************
 *********************************** User menu items *************************************
 *****************************************************************************************
 *                                                                                       *
 * USer-defined menu items that execute custom GCode                                     *
 *                                                                                       *
 *****************************************************************************************/
//#define CUSTOM_USER_MENUS

#define USER_SCRIPT_DONE "M117 User Script Done"

#define USER_DESC_1 "Home & ABL"
#define USER_GCODE_1 "G28\nG29"

#define USER_DESC_2 "Preheat for PLA"
#define USER_GCODE_2 "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)

#define USER_DESC_3 "Preheat for ABS"
#define USER_GCODE_3 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)

#define USER_DESC_4 "Heat Bed/Home/Level"
#define USER_GCODE_4 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nG28\nG29"

#define USER_DESC_5 "Home & Info"
#define USER_GCODE_5 "G28\nM503"
/*****************************************************************************************/

#endif /* _CONFIGURATION_FEATURE_H_ */
