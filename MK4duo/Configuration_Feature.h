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
 * DRIVER FEATURES:
 * - Driver types
 * FAN FEATURES:
 * - Fan configuration
 * EXTRUDER FEATURES:
 * - Volumetric extrusion
 * - Filament Diameter
 * - Single nozzle
 * - BariCUDA paste extruder
 * - Solenoid extruder
 * - Color Mixing Extruder
 * - Multiextruder old MKR4
 * - Multiextruder MKR6
 * - Multiextruder MKR12
 * - Multiextruder MKSE6 (multiextruder with Servo)
 * - Multiextruder DONDOLO
 * - Extruder idle oozing prevention
 * - Extruder run-out prevention
 * - Extruder Advance Linear Pressure Control
 * MOTION FEATURES:
 * - Workspace offsets
 * - Stepper auto deactivation
 * - Software endstops
 * - Endstops only for homing
 * - Abort on endstop hit feature
 * - G38.2 and G38.3 Probe Target
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
 * - Z-axis two driver
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
 * - LCD Character Set
 * - LCD / Controller Selection
 * - LCD Options
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
 * - Double / Quad Stepping
 * - Junction Deviation
 * - Bézier Jerk Control
 * - Minimum stepper pulse
 * - Maximum stepper rate
 * - Direction Stepper Delay
 * - Adaptive Step Smoothing
 * - Microstepping
 * - Motor's current
 * - I2C DIGIPOT
 * ADVANCED FEATURES:
 * - Buffer stuff
 * - Nozzle Clean Feature
 * - Nozzle Park
 * - Advanced Pause Park
 * - G20/G21 Inch mode support
 * - Report JSON-style response
 * - Scad Mesh Output
 * - M43 command for pins info and testing
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

//===========================================================================
//============================= DRIVER FEATURES =============================
//===========================================================================

/****************************************************************************
 ******************************** Driver types ******************************
 ****************************************************************************
 *                                                                          *
 * Set driver type:                                                         *
 *  - A4988                                                                 *
 *  - A5984                                                                 *
 *  - DRV8825                                                               *
 *  - TMC26X                                                                *
 *  - L6470                                                                 *
 *  - TB6560                                                                *
 *  - TB6600                                                                *
 *  - TMC2100                                                               *
 *  - TMC2130                                                               *
 *  - TMC2208                                                               *
 *  - TMC2660                                                               *
 *  - TMC5130                                                               *
 *                                                                          *
 * See Configuration_Motor_Driver.h for configuration Motor Driver          *
 *                                                                          *
 ****************************************************************************/
#define  X_DRIVER_TYPE  A4988
#define  Y_DRIVER_TYPE  A4988
#define  Z_DRIVER_TYPE  A4988
#define X2_DRIVER_TYPE  A4988
#define Y2_DRIVER_TYPE  A4988
#define Z2_DRIVER_TYPE  A4988
#define E0_DRIVER_TYPE  A4988
#define E1_DRIVER_TYPE  A4988
#define E2_DRIVER_TYPE  A4988
#define E3_DRIVER_TYPE  A4988
#define E4_DRIVER_TYPE  A4988
#define E5_DRIVER_TYPE  A4988
/****************************************************************************/


//===========================================================================
//=============================== FAN FEATURES ==============================
//===========================================================================

/****************************************************************************
 ***************************** Fan configuration ****************************
 ****************************************************************************/
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

// This defines the minimal speed for the main fan
// set minimal speed for reliable running (1-255)
#define FAN_MIN_PWM 1

// To reverse the logic of fan pins
//#define INVERTED_FAN_PINS

// AUTO FAN - Fans for cooling Hotend or Controller Fan
// Put number Hotend in fan to automatically turn on/off when the associated
// hotend temperature is above/below HOTEND AUTO FAN TEMPERATURE.
// Or put 7 for controller fan
// -1 disables auto mode.
// Default fan 1 is auto fan for Hotend 0
#define AUTO_FAN { -1, 0, -1, -1, -1, -1 }
// Parameters for Hotend Fan
#define HOTEND_AUTO_FAN_TEMPERATURE  50
#define HOTEND_AUTO_FAN_SPEED       255 // 255 = full speed
#define HOTEND_AUTO_FAN_MIN_SPEED     0
// Parameters for Controller Fan
#define CONTROLLERFAN_SECS           60 // How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED         255 // 255 = full speed
#define CONTROLLERFAN_MIN_SPEED       0

// Add Tachometric option for fan ONLY FOR DUE. (Add TACHOMETRIC PIN in configuration pins)
//#define TACHOMETRIC
/****************************************************************************/


//===========================================================================
//============================ EXTRUDER FEATURES ============================
//===========================================================================

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
//#define VOLUMETRIC_EXTRUSION
//#define VOLUMETRIC_DEFAULT_ON
/***********************************************************************/


/***********************************************************************
 ************************* Filament Diameter ***************************
 ***********************************************************************
 *                                                                     *
 * Generally expected filament diameter (1.75, 2.85, 3.0, ...)         *
 * Used for Volumetric, Filament Width Sensor, etc.                    *
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
 * Setting for more extruders with relay system                        *
 * This is an old system for 4 extruders and 8 relays.                 *
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
 * Setting for more extruders with relay system                        *
 * This is a new system for 6 extruders with 2 drivers and 8 relays.   *
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
 * Setting for more extruders with relay system                        *
 * This is a new system for 12 extruders with 4 drivers and 16 relays. *
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
 * Setting for more extruders with servo system                        *
 * This is a new system for 6 extruders with 1 driver and 1 servo.     *
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


/*****************************************************************************************
 ****************** Extruder Advance Linear Pressure Control *****************************
 *****************************************************************************************
 *                                                                                       *
 * Linear Pressure Control v1.5                                                          *
 *                                                                                       *
 * Assumption: advance [steps] = k * (delta velocity [steps/s])                          *
 * K=0 means advance disabled.                                                           *
 *                                                                                       *
 * NOTE: K values for LIN_ADVANCE 1.5 differ from earlier versions!                      *
 *                                                                                       *
 * Set K around 0.22 for 3mm PLA Direct Drive with ~6.5cm between the                    *
 * drive gear and heatbreak.                                                             *
 * Larger K values will be needed for flexible filament and greater distances.           *
 * If this algorithm produces a higher speed offset than the                             *
 * extruder can handle (compared to E jerk)                                              *
 * print acceleration will be reduced during the affected moves to keep within the limit.*
 *                                                                                       *
 *****************************************************************************************/
//#define LIN_ADVANCE

// Unit: mm compression per 1mm/s extruder speed
#define LIN_ADVANCE_K 0.22

// If enabled, this will generate debug information output over Serial.
//#define LA_DEBUG
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
 ************************ Abort on endstop hit ****************************
 **************************************************************************
 *                                                                        *
 * This option allows you to abort printing when any endstop is triggered.*
 * This feature must be enabled with "M540 S1" or from the LCD menu or    *
 * by define ABORT ON ENDSTOP HIT DEFAULT true.                           *
 * With ENDSTOPS ONLY FOR HOMING you must send "M120" to enable endstops. *
 *                                                                        *
 **************************************************************************/
//#define ABORT_ON_ENDSTOP_HIT

#define ABORT_ON_ENDSTOP_HIT_DEFAULT true
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
#define BABYSTEP_MULTIPLICATOR 1
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
 * Note: Be sure to turn off auto-retract during filament change.         *
 * Note: Current Zlift reset by G28 or G28 Z.                             *
 *                                                                        *
 * Note that M207 / M208 / M209 settings are saved to EEPROM.             *
 *                                                                        *
 **************************************************************************/
//#define FWRETRACT

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
 * This section will allow you to use extra drivers to drive a second motor for X        *
 * Uncomment this define to utilize a separate stepper driver for each X axis motor.     *
 * If the motors need to spin in opposite directions set INVERT X2 VS X DIR.             *
 * If the second motor needs its own endstop set X TWO ENDSTOPS.                         *
 * Extra endstops will appear in the output of 'M119'.                                   *
 *                                                                                       *
 * ONLY Cartesian                                                                        *
 *                                                                                       *
 *****************************************************************************************/
//#define X_TWO_STEPPER_DRIVERS

#define INVERT_X2_VS_X_DIR false
//#define X_TWO_ENDSTOPS
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Y-axis two driver ************************************
 *****************************************************************************************
 *                                                                                       *
 * This section will allow you to use extra drivers to drive a second motor for Y        *
 * Uncomment this define to utilize a separate stepper driver for each Y axis motor.     *
 * If the motors need to spin in opposite directions set INVERT Y2 VS Y DIR.             *
 * If the second motor needs its own endstop set Y TWO ENDSTOPS.                         *
 * Extra endstops will appear in the output of 'M119'.                                   *
 *                                                                                       *
 * ONLY Cartesian                                                                        *
 *                                                                                       *
 *****************************************************************************************/
//#define Y_TWO_STEPPER_DRIVERS

#define INVERT_Y2_VS_Y_DIR false
//#define Y_TWO_ENDSTOPS
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Z-axis two driver ************************************
 *****************************************************************************************
 *                                                                                       *
 * This section will allow you to use extra drivers to drive a second motor for Z        *
 * Uncomment this define to utilize a separate stepper driver for each Z axis motor.     *
 * If the motors need to spin in opposite directions set INVERT Z2 VS Z DIR.             *
 * If the second motor needs its own endstop set Z TWO ENDSTOPS.                         *
 * Extra endstops will appear in the output of 'M119'.                                   *
 *                                                                                       *
 * Only Cartesian & Core                                                                 *
 *                                                                                       *
 *****************************************************************************************/
//#define Z_TWO_STEPPER_DRIVERS

#define INVERT_Z2_VS_Z_DIR false
//#define Z_TWO_ENDSTOPS
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

#define FILAMENT_SENSOR_EXTRUDER_NUM  0   // Index of the extruder that has the filament sensor. :[0,1,2,3,4,5]
#define MEASUREMENT_DELAY_CM         14   // (cm) The distance from the filament sensor to the melting chamber

#define FILWIDTH_ERROR_MARGIN        1.0  // (mm) If a measurement differs too much from nominal width ignore it
#define MAX_MEASUREMENT_DELAY        20   // (bytes) Buffer size for stored measurements (1 byte per cm). Must be larger than MEASUREMENT_DELAY_CM.

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
#define FIL_RUNOUT_LOGIC true
// Put true for use internal pullup for pin if the sensor is defined
#define PULLUP_FIL_RUNOUT false
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
 ************************** Door Open Sensor ******************************
 **************************************************************************
 *                                                                        *
 * A triggered door will prevent new commands from serial or sd card.     *
 * Setting DOOR OPEN PIN in Configuration_Pins.h                          *
 *                                                                        *
 **************************************************************************/
//#define DOOR_OPEN

// Set true or false should assigned
#define DOOR_OPEN_LOGIC false
// Put true for use internal pullup for pin if the sensor is defined.
#define PULLUP_DOOR_OPEN false
/**************************************************************************/


/**************************************************************************
 *************************** Power Check Sensor ***************************
 **************************************************************************
 *                                                                        *
 * A triggered when the pin detects lack of voltage                       *
 * Setting POWER CHECK PIN in Configuration_Pins.h                        *
 *                                                                        *
 **************************************************************************/
//#define POWER_CHECK

// Set true or false should assigned
#define POWER_CHECK_LOGIC false
// Put true for use internal pullup for pin if the sensor is defined.
#define PULLUP_POWER_CHECK false
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
 * Uncomment EEPROM I2C if your board mount I2C EEPROM (Already enabled for cards that mount this eeprom by default)    *
 * Uncomment EEPROM SPI if your board mount SPI EEPROM (Already enabled for cards that mount this eeprom by default)    *
 * Uncomment EEPROM SD for use writing EEPROM on SD  (Only for DUE)                                                     *
 * Uncomment EEPROM FLASH for use writing EEPROM on Flash Memory (Only for DUE)                                         *
 *                                                                                                                      *
 ************************************************************************************************************************/
//#define EEPROM_SETTINGS

// Uncomment this to enable EEPROM Serial responses.
//#define EEPROM_CHITCHAT

// Type EEPROM Hardware
//  Caution!!! The cards that mount the eeprom by default
//  have already enabled the correct define, do not touch this.
//#define EEPROM_I2C
//#define EEPROM_SPI
//#define EEPROM_SD
//#define EEPROM_FLASH

// Disabled M503 report
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

#define SD_FINISHED_STEPPERRELEASE true           // if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y Z E"  // You might want to keep the z enabled so your bed stays in place.

//#define MENU_ADDAUTOSTART

// Enable this option to scroll long filenames in the SD card menu
//#define SCROLL_LONG_FILENAMES

/**
 * Sort SD file listings in alphabetical order.
 *
 * With this option enabled, items on SD cards will be sorted
 * by name for easier navigation.
 *
 * By default...
 *
 *  - Use the slowest -but safest- method for sorting.
 *  - Folders are sorted to the top.
 *  - The sort key is statically allocated.
 *  - No added G-code (M36) support.
 *  - 40 item sorting limit. (Items after the first 40 are unsorted.)
 *
 * SD sorting uses static allocation (as set by SDSORT_LIMIT), allowing the
 * compiler to calculate the worst-case usage and throw an error if the SRAM
 * limit is exceeded.
 *
 *  - SDSORT_USES_RAM provides faster sorting via a static directory buffer.
 *  - SDSORT_USES_STACK does the same, but uses a local stack-based buffer.
 *  - SDSORT_CACHE_NAMES will retain the sorted file listing in RAM. (Expensive!)
 *  - SDSORT_DYNAMIC_RAM only uses RAM when the SD menu is visible. (Use with caution!)
 */
//#define SDCARD_SORT_ALPHA

// SD Card Sorting options
#define SDSORT_LIMIT       40     // Maximum number of sorted items (10-256). Costs 27 bytes each.
#define FOLDER_SORTING     -1     // -1=above  0=none  1=below
#define SDSORT_GCODE       false  // Allow turning sorting on/off with LCD and M36 g-code.
#define SDSORT_USES_RAM    false  // Pre-allocate a static array for faster pre-sorting.
#define SDSORT_USES_STACK  false  // Prefer the stack for pre-sorting to give back some SRAM. (Negated by next 2 options.)
#define SDSORT_CACHE_NAMES false  // Keep sorted items in RAM longer for speedy performance. Most expensive option.
#define SDSORT_DYNAMIC_RAM false  // Use dynamic allocation (within SD menus). Least expensive option. Set SDSORT_LIMIT before use!
#define SDSORT_CACHE_VFATS 2      // Maximum number of 13-byte VFAT entries to use for sorting.
                                  // Note: Only affects SCROLL_LONG_FILENAMES with SDSORT_CACHE_NAMES but not SDSORT_DYNAMIC_RAM.

// This function enable the firmware write restart.bin file for restart print when power loss
//#define SD_RESTART_FILE           // Uncomment to enable
#define SD_RESTART_FILE_SAVE_TIME 1 // seconds between update

// This enable the firmware to write statistics, that require frequent update on the SD card.
//#define SD_SETTINGS             // Uncomment to enable
#define SD_CFG_SECONDS 300        // seconds between update
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** LCD Language ****************************************
 *****************************************************************************************
 *                                                                                       *
 * Here you may choose the language used by MK4duo on the LCD menus,                     *
 * the following list of languages are available:                                        *
 *  en, an, bg, ca, cn, cz, de, el, el-gr, es, eu, fi, fr,                               *
 *  gl, hr, it, jp-kana, nl, pl, pt, pt-br, ru, sk,                                      *
 *  tr, uk, zh_CN, zh_TW                                                                 *
 *                                                                                       *
 * 'en':'English',          'an':'Aragonese', 'bg':'Bulgarian',       'ca':'Catalan',    *
 * 'cn':'Chinese',          'cz':'Czech',     'de':'German',          'el':'Greek',      *
 * 'el-gr':'Greek (Greece)' 'es':'Spanish',   'eu':'Basque-Euskera',  'fi':'Finnish',    *
 * 'fr':'French',           'gl':'Galician',  'hr':'Croatian',        'it':'Italian',    *
 * 'jp-kana':'Japanese',    'nl':'Dutch',     'pl':'Polish',          'pt':'Portuguese', *
 * 'ru':'Russian',          'sk':'Slovak',    'tr':'Turkish',         'uk':'Ukrainian',  *
 * 'pt-br':'Portuguese (Brazilian)',                                                     *
 * 'zh_CN':'Chinese (Simplified)'                                                        *
 * 'zh_TW':'Chinese (Traditional)'                                                       *
 *                                                                                       *
 *****************************************************************************************/
#define LCD_LANGUAGE en
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** LCD Character Set **************************************
 *****************************************************************************************
 *                                                                                       *
 * Note: This option is NOT applicable to Graphical Displays.                            *
 *                                                                                       *
 * All character-based LCDs provide ASCII plus one of these                              *
 * language extensions:                                                                  *
 *                                                                                       *
 *  - JAPANESE ... the most common                                                       *
 *  - WESTERN  ... with more accented characters                                         *
 *  - CYRILLIC ... for the Russian language                                              *
 *                                                                                       *
 *****************************************************************************************/
#define DISPLAY_CHARSET_HD44780 JAPANESE
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ******************************   (Character-based LCDs)   *******************************
 *****************************************************************************************/
// RepRapDiscount Smart Controller.
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//
// Note: Usually sold with a white PCB.
//
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

//
// ULTIMAKER Controller.
//
//#define ULTIMAKERCONTROLLER

//
// Ultipanel as seen on Thingiverse.
//
//#define ULTIPANEL

//
// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)
// http://reprap.org/wiki/PanelOne
//
//#define PANEL_ONE

//
// GADGETS3D G3D LCD/SD Controller
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//
// Note: Usually sold with a blue PCB.
//
//#define G3D_PANEL

//
// RigidBot Panel V1.0
// http://www.inventapart.com/
//
//#define RIGIDBOT_PANEL

//
// Makeboard 3D Printer Parts 3D Printer Mini Display 1602 Mini Controller
// https://www.aliexpress.com/item/Micromake-Makeboard-3D-Printer-Parts-3D-Printer-Mini-Display-1602-Mini-Controller-Compatible-with-Ramps-1/32765887917.html
//
//#define MAKEBOARD_MINI_2_LINE_DISPLAY_1602

//
// ANET and Tronxy 20x4 Controller
//
//#define ZONESTAR_LCD            // Requires ADC_KEYPAD_PIN to be assigned to an analog pin.
                                  // This LCD is known to be susceptible to electrical interference
                                  // which scrambles the display.  Pressing any button clears it up.
                                  // This is a LCD2004 display with 5 analog buttons.

//
// Generic 16x2, 16x4, 20x2, or 20x4 character-based LCD.
//
//#define ULTRA_LCD
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ***************************** (I2C and Shift-Register LCDs) *****************************
 *****************************************************************************************/
//
// CONTROLLER TYPE: I2C
//
// Note: These controllers require the installation of Arduino's LiquidCrystal_I2C
// library. For more info: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//

//
// Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
//
//#define RA_CONTROL_PANEL

//
// Sainsmart (YwRobot) LCD Displays
//
// These require F.Malpartida's LiquidCrystal_I2C library
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
//
//#define LCD_SAINSMART_I2C_1602
//#define LCD_SAINSMART_I2C_2004

//
// Generic LCM1602 LCD adapter
//
//#define LCM1602

//
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

//
// Panucatt VIKI LCD with status LEDs,
// integrated click & L/R/U/D buttons, separate encoder inputs.
//
//#define LCD_I2C_VIKI

// Original RADDS Display from Willy
// http://max3dshop.org/index.php/default/elektronik/radds-lcd-sd-display-with-reset-and-back-buttom.html
//#define RADDS_DISPLAY

//
// CONTROLLER TYPE: Shift register panels
//

//
// 2 wire Non-latching LCD SR from https://goo.gl/aJJ4sH
// LCD configuration: http://reprap.org/wiki/SAV_3D_LCD
//
//#define SAV_3DLCD
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ******************************      (Graphical LCDs)      *******************************
 *****************************************************************************************/
//
// CONTROLLER TYPE: Graphical 128x64 (DOGM)
//
// IMPORTANT: The U8glib library is required for Graphical Display!
//            https://github.com/olikraus/U8glib_Arduino
//

//
// RepRapDiscount FULL GRAPHIC Smart Controller
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

//
// ReprapWorld Graphical LCD
// https://reprapworld.com/?products_details&products_id/1218
//
//#define REPRAPWORLD_GRAPHICAL_LCD

//
// Activate one of these if you have a Panucatt Devices
// Viki 2.0 or mini Viki with Graphic LCD
// http://panucatt.com
//
//#define VIKI2
//#define miniVIKI

//
// MakerLab Mini Panel with graphic
// controller and SD support - http://reprap.org/wiki/Mini_panel
//
//#define MINIPANEL

//
// MaKr3d Makr-Panel with graphic controller and SD support.
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//
//#define MAKRPANEL

//
// Adafruit ST7565 Full Graphic Controller.
// https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/
//
//#define ELB_FULL_GRAPHIC_CONTROLLER

//
// BQ LCD Smart Controller shipped by
// default with the BQ Hephestos 2 and Witbox 2.
//
//#define BQ_LCD_SMART_CONTROLLER

//
// WANHAO D6 SSD1309 OLED full graphics
//
//#define WANHAO_D6_OLED

//
// Cartesio UI
// http://mauk.cc/webshop/cartesio-shop/electronics/user-interface
//
//#define CARTESIO_UI

//
// LCD for Melzi Card with Graphical LCD
//
//#define LCD_FOR_MELZI

//
// SSD1306 OLED full graphics generic display
//
//#define U8GLIB_SSD1306

//
// SAV OLEd LCD module support using either SSD1306 or SH1106 based LCD modules
//
//#define SAV_3DGLCD
//#define U8GLIB_SSD1306
//#define U8GLIB_SH1106

//
// Original Ulticontroller from Ultimaker 2 printer with SSD1309 I2C display and encoder
// https://github.com/Ultimaker/Ultimaker2/tree/master/1249_Ulticontroller_Board_(x1)
//
//#define ULTI_CONTROLLER

//
// TinyBoy2 128x64 OLED / Encoder Panel
//
//#define OLED_PANEL_TINYBOY2

//
// MKS MINI12864 with graphic controller and SD support
// http://reprap.org/wiki/MKS_MINI_12864
//
//#define MKS_MINI_12864

//
// Factory display for Creality CR-10
// https://www.aliexpress.com/item/Universal-LCD-12864-3D-Printer-Display-Screen-With-Encoder-For-CR-10-CR-7-Model/32833148327.html
//
// This is RAMPS-compatible using a single 10-pin connector.
// (For CR-10 owners who want to replace the Melzi Creality board but retain the display)
//
//#define CR10_STOCKDISPLAY

//
// ANET and Tronxy Graphical Controller
//
//#define ANET_FULL_GRAPHICS_LCD  // Anet 128x64 full graphics lcd with rotary encoder as used on Anet A6
                                  // A clone of the RepRapDiscount full graphics display but with
                                  // different pins/wiring (see pins_ANET_10.h).

//
// MKS OLED 1.3" 128 × 64 FULL GRAPHICS CONTROLLER
// http://reprap.org/wiki/MKS_12864OLED
//
// Tiny, but very sharp OLED display
//
//#define MKS_12864OLED          // Uses the SH1106 controller (default)
//#define MKS_12864OLED_SSD1306  // Uses the SSD1306 controller

//
// AZSMZ 12864 LCD with SD
// https://www.aliexpress.com/store/product/3D-printer-smart-controller-SMART-RAMPS-OR-RAMPS-1-4-LCD-12864-LCD-control-panel-green/2179173_32213636460.html
//
//#define AZSMZ_12864

//
// Silvergate GLCD controller
// http://github.com/android444/Silvergate
//
//#define SILVER_GATE_GLCD_CONTROLLER
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ******************************      Other Controllers     *******************************
 *****************************************************************************************/
//
// Nextion 4.3" - 5" Enanched - 7" Enanched HMI panel
//
//#define NEXTION
// Define Serial it use
#define NEXTION_SERIAL 1
// For GFX preview visualization enable NEXTION GFX
//#define NEXTION_GFX
// Define name firmware file for Nextion on SD
#define NEXTION_FIRMWARE_FILE "mk4duo.tft"
/*****************************************************************************************/


/*****************************************************************************************
 ************************************** LCD Options **************************************
 *****************************************************************************************/

#define SHOW_BOOTSCREEN
#define STRING_SPLASH_LINE1 "v" SHORT_BUILD_VERSION   // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE  // will be shown during bootup in line 2
#define BOOTSCREEN_TIMEOUT 2500

//
// *** VENDORS PLEASE READ ***
//
// MK4duo allows you to add a custom boot image for Graphical LCDs.
// With this option MK4duo will first show your custom screen followed
// by the standard MK4duo logo with version number and web URL.
// We encourage you to take advantage of this new feature and we also
// respectfully request that you retain the unmodified MK4duo boot screen.
//

// Enable to show the bitmap in MK4duo/src/lcd/custom_bootscreen.h on startup.
//#define SHOW_CUSTOM_BOOTSCREEN

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

// Swap the CW/CCW indicators in the graphics overlay
//#define OVERLAY_GFX_REVERSE

//
// LCD Menu Items
//
// Disable all menus and only display the Status Screen, or
// just remove some extraneous menu items to recover space.
//
//#define NO_LCD_MENUS
//#define SLIM_LCD_MENUS

//
// ENCODER SETTINGS
//
// This option overrides the default number of encoder pulses needed to
// produce one step. Should be increased for high-resolution encoders.
#define ENCODER_PULSES_PER_STEP 5

// Use this option to override the number of step signals required to
// move between next/prev menu items.
#define ENCODER_STEPS_PER_MENU_ITEM 1

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

// Comment to disable setting feedrate multiplier via encoder
#define ULTIPANEL_FEEDMULTIPLY

// SPEAKER/BUZZER
// If you have a speaker that can produce tones, enable it here.
// By default MK4duo assumes you have a buzzer with a fixed frequency.
//#define SPEAKER

// The duration and frequency for the UI feedback sound.
// Set these to 0 to disable audio feedback in the LCD menus.

// Note: Test audio output with the G-Code:
//  M300 S<frequency Hz> P<duration ms>
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
//#define LCD_FEEDBACK_FREQUENCY_HZ 5000

// Display Voltage Logic Selector on Alligator Board
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

// LED Control Menu
// Enable this feature to add LED Control to the LCD menu
//#define LED_CONTROL_MENU
//#define LED_COLOR_PRESETS             // Enable the Preset Color menu option
//#define LED_USER_PRESET_STARTUP       // Have the printer display the user preset color on startup
#define LED_USER_PRESET_RED        255  // User defined RED value
#define LED_USER_PRESET_GREEN      255  // User defined GREEN value
#define LED_USER_PRESET_BLUE       255  // User defined BLUE value
#define LED_USER_PRESET_WHITE      255  // User defined WHITE value
#define LED_USER_PRESET_BRIGHTNESS 255  // User defined intensity

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
/*****************************************************************************************/


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
 * Support RFID module card reader with UART interface.                   *
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
 * Enable support for an RGB LED connected to 5V digital pins, or         *
 * an RGB Strip connected to MOSFETs controlled by digital pins.          *
 *                                                                        *
 * Adds the M150 command to set the LED (or LED strip) color.             *
 * If pins are PWM capable (e.g., 4, 5, 6, 11) then a range of            *
 * luminance values can be set from 0 to 255.                             *
 *                                                                        *
 * *** CAUTION ***                                                        *
 *  LED Strips require a MOSFET Chip between PWM lines and LEDs,          *
 *  as the Arduino cannot handle the current the LEDs will require.       *
 *  Failure to follow this precaution can destroy your Arduino!           *
 * *** CAUTION ***                                                        *
 *                                                                        *
 * LED type. These options are mutually-exclusive. Uncomment only one.    *
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
//#define NEOPIXEL_LED

// NEO_GRBW / NEO_GRB - four/three channel driver type
// (defined in Adafruit_NeoPixel.h)
#define NEOPIXEL_TYPE   NEO_GRB
// Number of LEDs on strip
#define NEOPIXEL_PIXELS 16
// Sequential display for temperature change - LED by LED.
// Comment out for all LEDs to change at once.
#define NEOPIXEL_IS_SEQUENTIAL
// Initial brightness 0-255
#define NEOPIXEL_BRIGHTNESS 127
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
// Use Neopixel LED as case light, requires NEOPIXEL LED.
//#define CASE_LIGHT_USE_NEOPIXEL
// { Red, Green, Blue, White }
#define CASE_LIGHT_NEOPIXEL_COLOR { 255, 255, 255, 255 }
/**************************************************************************/


//===========================================================================
//========================= ADVANCED MOTION FEATURES ========================
//===========================================================================

/***********************************************************************
 ********************** Double / Quad Stepping *************************
 ***********************************************************************
 *                                                                     *
 * Disable double / quad stepping                                      *
 *                                                                     *
 ***********************************************************************/
//#define DISABLE_DOUBLE_QUAD_STEPPING
/***********************************************************************/


/**************************************************************************
 ************************* Junction Deviation *****************************
 **************************************************************************
 *                                                                        *
 * Use Junction Deviation instead of traditional Jerk limiting            *
 *                                                                        *
 * By Scott Latherine @Thinkyhead  and @ejtagle                           *
 *                                                                        *
 **************************************************************************/
//#define JUNCTION_DEVIATION

// (mm) Distance from real junction edge
#define JUNCTION_DEVIATION_MM 0.02
/**************************************************************************/


/****************************************************************************
 ************************** Bézier Jerk Control *****************************
 ****************************************************************************
 *                                                                          *
 * This option eliminates vibration during printing by fitting a Bézier     *
 * curve to move acceleration, producing much smoother direction changes.   *
 *                                                                          *
 * https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained *
 *                                                                          *
 ****************************************************************************/
//#define BEZIER_JERK_CONTROL
/****************************************************************************/


/***************************************************************************************
 ******************************** Minimum stepper pulse ********************************
 ***************************************************************************************
 *                                                                                     *
 * Minimum stepper driver pulse width (in µs)                                          *
 *  0 : Smallest possible width the MCU can produce, compatible with TMC2xxx drivers   *
 *  1 : Minimum for A4988, A5984, and LV8729 stepper drivers                           *
 *  2 : Minimum for DRV8825 stepper drivers                                            *
 *  3 : Minimum for TB6600 stepper drivers                                             *
 * 30 : Minimum for TB6560 stepper drivers                                             *
 *                                                                                     *
 ***************************************************************************************/
#define MINIMUM_STEPPER_PULSE 0UL
/***************************************************************************************/


/***************************************************************************************
 ********************************* Maximum stepper rate ********************************
 ***************************************************************************************
 *                                                                                     *
 * The maximum stepping rate (in Hz) the motor stepper driver allows                   *
 * If non defined, it defaults to 1000000 / (2 * MINIMUM STEPPER PULSE)                *
 *  500000 : Maximum for A4988 stepper driver                                          *
 *  400000 : Maximum for TMC2xxx stepper driver                                        *
 *  250000 : Maximum for DRV8825 stepper driver                                        *
 *  150000 : Maximum for TB6600 stepper driver                                         *
 *  130000 : Maximum for LV8729 stepper driver                                         *
 *   15000 : Maximum for TB6560 stepper driver                                         *
 *                                                                                     *
 ***************************************************************************************/
#define MAXIMUM_STEPPER_RATE 500000
/***************************************************************************************/


/***********************************************************************
 ********************** Direction Stepper Delay ************************
 ***********************************************************************
 *                                                                     *
 * Minimum delay after setting the stepper DIR (in ns)                 *
 *      0 : No delay at all - But, at least 10uS are expected          *
 *     50 : Minimum for TMC2xxx drivers                                *
 *    200 : Minimum for A4988 drivers                                  *
 *    400 : Minimum for A5984 drivers                                  *
 *    500 : Minimum for LV8729 drivers (guess, no info in datasheet)   *
 *    650 : Minimum for DRV8825 drivers                                *
 *   1500 : Minimum for TB6600 drivers (guess, no info in datasheet)   *
 *  15000 : Minimum for TB6560 drivers (guess, no info in datasheet)   *
 *                                                                     *
 ***********************************************************************/
#define DIRECTION_STEPPER_DELAY 0
/***********************************************************************/


/***********************************************************************
 ********************** Adaptive Step Smoothing ************************
 ***********************************************************************
 *                                                                     *
 * Adaptive Step Smoothing increases the resolution of multiaxis moves,*
 * particularly at step frequencies below 1kHz (for AVR) or            *
 * 10kHz (for ARM), where aliasing between axes in multiaxis moves     *
 * causes audible vibration and surface artifacts.                     *
 * The algorithm adapts to provide the best possible step smoothing    *
 * at the lowest stepping frequencies.                                 *
 *                                                                     *
 ***********************************************************************/
//#define ADAPTIVE_STEP_SMOOTHING
/***********************************************************************/


/***********************************************************************
 *************************** Microstepping *****************************
 ***********************************************************************
 *                                                                     *
 * Microstep setting - Only functional when stepper driver microstep   *
 * pins are connected to MCU.                                          *
 *                                                                     *
 * Alligator Board support 16 or 32 only value                         *
 *                                                                     *
 * [1,2,4,8,16,32]                                                     *
 *                                                                     *
 ***********************************************************************/
#define X_MICROSTEPS  16
#define X2_MICROSTEPS 16
#define Y_MICROSTEPS  16
#define Y2_MICROSTEPS 16
#define Z_MICROSTEPS  16
#define Z2_MICROSTEPS 16
#define E0_MICROSTEPS 16
#define E1_MICROSTEPS 16
#define E2_MICROSTEPS 16
#define E3_MICROSTEPS 16
#define E4_MICROSTEPS 16
#define E5_MICROSTEPS 16
/***********************************************************************/


/***********************************************************************
 ************************** Motor's current ****************************
 ***********************************************************************/
// Motor Current setting
// X Y Z E0 E1 E2 E3 - Values 100 - 3000 in mA
#define X_CURRENT   800
#define X2_CURRENT  800
#define Y_CURRENT   800
#define Y2_CURRENT  800
#define Z_CURRENT   800
#define Z2_CURRENT  800
#define E0_CURRENT  800
#define E1_CURRENT  800
#define E2_CURRENT  800
#define E3_CURRENT  800
#define E4_CURRENT  800
#define E5_CURRENT  800

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


//===========================================================================
//============================= ADVANCED FEATURES ===========================
//===========================================================================


/****************************************************************************************
 ************************************** Buffer stuff ************************************
 ****************************************************************************************/
// The number of linear motions that can be in the plan at any give time.
// THE BLOCK BUFFER SIZE NEEDS TO BE A POWER OF 2 (i.g. 8, 16, 32) because shifts
// and ors are used to do the ring-buffering.
// For Arduino DUE setting BLOCK BUFFER SIZE to 32
#define BLOCK_BUFFER_SIZE 16

// The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
// For Arduino DUE setting to 8
#define BUFSIZE 4

/** START Function only for 8 bit proccesor */
// Transmission to Host Buffer Size
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
// To buffer a simple "ok" you need 4 bytes.
// For ADVANCED_OK (M105) you need 32 bytes.
// For debug-echo: 128 bytes for the optimal speed.
// Other output doesn't need to be that speedy.
// 0, 2, 4, 8, 16, 32, 64, 128, 256
#define TX_BUFFER_SIZE 0

// Host Receive Buffer Size
// Without XON/XOFF flow control (see SERIAL XON XOFF below) 32 bytes should be enough.
// To use flow control, set this buffer size to at least 1024 bytes.
// 0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048
#define RX_BUFFER_SIZE 128

// Enable to have the controller send XON/XOFF control characters to
// the host to signal the RX buffer is becoming full.
//#define SERIAL_XON_XOFF

// Enable this option to collect and display the maximum
// RX queue usage after transferring a file to SD.
//#define SERIAL_STATS_MAX_RX_QUEUED

// Enable this option to collect and display the number
// of dropped bytes after a file transfer to SD.
//#define SERIAL_STATS_DROPPED_RX
/** END Function only for 8 bit proccesor */

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
#define NOZZLE_PARK_POINT { 10, 10, 20 }
#define NOZZLE_PARK_XY_FEEDRATE 100   // X and Y axes feedrate in mm/s (also used for delta printers Z axis)
#define NOZZLE_PARK_Z_FEEDRATE    5   // Z axis feedrate in mm/s (not used for delta printers)
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
 * Requires NOZZLE PARK FEATURE                                           *
 * This feature is required for the default FILAMENT RUNOUT SCRIPT.       *
 *                                                                        *
 **************************************************************************/
//#define ADVANCED_PAUSE_FEATURE

#define PAUSE_PARK_RETRACT_FEEDRATE       20  // (mm/s) Initial retract feedrate.
#define PAUSE_PARK_RETRACT_LENGTH          5  // (mm) Initial retract.
                                              // This short retract is done immediately, before parking the nozzle.
#define PAUSE_PARK_UNLOAD_FEEDRATE        50  // (mm/s) Unload filament feedrate. This can be pretty fast.
#define PAUSE_PARK_UNLOAD_LENGTH         100  // (mm) The length of filament for a complete unload.
                                              //   For Bowden, the full length of the tube and nozzle.
                                              //   For direct drive, the full length of the nozzle.
                                              //   Set to 0 for manual unloading.
#define PAUSE_PARK_SLOW_LOAD_FEEDRATE     50  // (mm/s) Slow move when starting load.
#define PAUSE_PARK_SLOW_LOAD_LENGTH      100  // (mm) Slow length, to allow time to insert material.
                                              // 0 to disable start loading and skip to fast load only
#define PAUSE_PARK_FAST_LOAD_FEEDRATE      6  // (mm/s) Load filament feedrate. This can be pretty fast.
#define PAUSE_PARK_FAST_LOAD_LENGTH        5  // (mm) Load length of filament, from extruder gear to nozzle.
                                              //   For Bowden, the full length of the tube and nozzle.
                                              //   For direct drive, the full length of the nozzle.
#define PAUSE_PARK_EXTRUDE_FEEDRATE        5  // (mm/s) Extrude feedrate (after loading). Should be slower than load feedrate.
#define PAUSE_PARK_EXTRUDE_LENGTH         50  // (mm) Length to extrude after loading.
                                              //   Set to 0 for manual extrusion.
                                              //   Filament can be extruded repeatedly from the Filament Change menu
                                              //   until extrusion is consistent, and to purge old filament.

                                              // Filament Unload does a Retract, Delay, and Purge first:
#define FILAMENT_UNLOAD_RETRACT_LENGTH    10  // (mm) Unload initial retract length.
#define FILAMENT_UNLOAD_DELAY           5000  // (ms) Delay for the filament to cool after retract.
#define FILAMENT_UNLOAD_PURGE_LENGTH       8  // (mm) An unretract is done, then this length is purged.

#define PAUSE_PARK_NOZZLE_TIMEOUT         45  // (seconds) Time limit before the nozzle is turned off for safety.
#define PAUSE_PARK_PRINTER_OFF 5              // (minute) Time limit before turn off printer if user doesn't change filament.
#define PAUSE_PARK_NUMBER_OF_ALERT_BEEPS  10  // Number of alert beeps before printer goes quiet
#define PAUSE_PARK_NO_STEPPER_TIMEOUT         // Enable for XYZ steppers to stay powered on during filament change.

//#define PARK_HEAD_ON_PAUSE                  // Park the nozzle during pause and filament change.
//#define HOME_BEFORE_FILAMENT_CHANGE         // Ensure homing has been completed prior to parking for filament change

//#define FILAMENT_LOAD_UNLOAD_GCODES         // Add M701/M702 Load/Unload G-codes, plus Load/Unload in the LCD Prepare menu.
//#define FILAMENT_UNLOAD_ALL_EXTRUDERS       // Allow M702 to unload all extruders above a minimum target temp (as set by M302)
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

// If you have a watchdog reboot in an ArduinoMega2560 then the device will hang forever,
// as a watchdog reset will leave the watchdog on.
// The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
// However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled.
// And the code could hang in an interrupt routine with interrupts disabled.
//#define WATCHDOG_RESET_MANUAL
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Start / Stop Gcode ************************************
 *****************************************************************************************
 *                                                                                       *
 * Start - Stop Gcode use when Start or Stop printing with M530 command                  *
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
