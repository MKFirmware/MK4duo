/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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
 * - Default nominal filament diameter
 * - Single nozzle
 * - BariCUDA paste extruder
 * - Color Mixing Extruder
 * - Multiextruder old MKR4
 * - Multiextruder new MKR6
 * - Multiextruder new MKSE6 (multiextruder with Servo)
 * - Multiextruder NPr2
 * - Multiextruder DONDOLO
 * - Extruder idle oozing prevention
 * - Extruder run-out prevention
 * - Bowden Filament management
 * - Extruder advance constant
 * - Filament exchange
 * MOTION FEATURES:
 * - Software endstops
 * - Endstops only for homing
 * - Abort on endstop hit feature
 * - Mesh Level Area
 * - R/C Servo
 * - Late Z axis
 * - Ahead slowdown
 * - Quick home
 * - Home Y before X
 * - Force Home XY before Home Z
 * - Babystepping
 * - Firmware retract
 * - Dual X-carriage
 * - X-axis dual driver
 * - Y-axis dual driver
 * - Z-axis dual driver
 * - XY Frequency limit
 * - Skeinforge arc fix
 * SENSORS FEATURES:
 * - Filament diameter sensor
 * - Filament Runout sensor
 * - Power consumption sensor
 * - Flow sensor
 * ADDON FEATURES:
 * - EEPROM
 * - SDCARD
 * - LCD Language
 * - LCD
 * - Canon RC-1 Remote
 * - Camera trigger
 * - RFID card reader
 * - BLINKM
 * - Laser beam
 * ADVANCED MOTION FEATURES:
 * - Stepper auto deactivation
 * - Low speed stepper
 * - High speed stepper
 * - Microstepping
 * - Motor's current
 * - I2C DIGIPOT
 * - Toshiba steppers
 * - TMC26X motor drivers
 * - L6470 motor drivers
 * ADVANCED FEATURES:
 * - Buffer stuff
 * - Clean Nozzle Feature
 * - Nozzle Park
 * - G20/G21 Inch mode support
 * - Report JSON-style response
 * - Whatchdog
 * - Start / Stop Gcode
 *
 * Basic-settings can be found in Configuration_Basic.h
 * Mechanisms-settings can be found in Configuration_Xxxxxx.h (where Xxxxxx can be: Cartesian - Delta - Core - Scara)
 * Pins-settings can be found in "Configuration_Pins.h"
 * 
 */

#ifndef CONFIGURATION_FEATURE_H
#define CONFIGURATION_FEATURE_H

/**************************************************************************
 **************************** Fan configuration ***************************
 **************************************************************************/
// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
// Only 8 bit boards
//#define FAST_PWM_FAN

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT PWM SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// When first starting the main fan, run it at full speed for the
// given number of milliseconds.  This gets the fan spinning reliably
// before setting a PWM value. (Does not work with software PWM for fan on Sanguinololu)
//#define FAN_KICKSTART_TIME 100

// This defines the minimal speed for the main fan, run in PWM mode
// to enable uncomment and set minimal PWM speed for reliable running (1-255)
// if fan speed is [1 - (FAN_MIN_PWM-1)] it is set to FAN_MIN_PWM
//#define FAN_MIN_PWM 50

// This is for controlling a fan to cool down the stepper drivers
// it will turn on when any driver is enabled
// and turn off after the set amount of seconds from last driver being disabled again
// You need to set CONTROLLERFAN_PIN in Configuration_pins.h
//#define CONTROLLERFAN
#define CONTROLLERFAN_SECS       60   // How many seconds, after all motors were disabled, the fan should run
#define CONTROLLERFAN_SPEED     255   // 255 = full speed
#define CONTROLLERFAN_MIN_SPEED   0

// Extruder cooling fans
// Configure fan pin outputs to automatically turn on/off when the associated
// extruder temperature is above/below EXTRUDER AUTO FAN TEMPERATURE.
// Multiple extruders can be assigned to the same pin in which case
// the fan will turn on when any selected extruder is above the threshold.
// You need to set _AUTO_FAN_PIN in Configuration_pins.h
//#define EXTRUDER_AUTO_FAN
#define EXTRUDER_AUTO_FAN_TEMPERATURE  50
#define EXTRUDER_AUTO_FAN_SPEED       255  // 255 = full speed
#define EXTRUDER_AUTO_FAN_MIN_SPEED     0
/**************************************************************************/


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
 ********************** COLOR MIXING EXTRUDER **************************
 ***********************************************************************
 *                                                                     *
 * Extends G0/G1 with mixing factors ABCDHI for up to 6 steppers.      *
 * Adds a new code, M165, to set the current mix factors.              *
 * Optional support for Repetier Host M163, M164, and virtual tools.   *
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
 * This is new system for 6 extruder width 2 driver and 6 relay.       *
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
 * before shoudown the printer if you enable this feature.             *
 *                                                                     *
 * Uncomment IDLE_OOZING_PREVENT to enable this feature                *
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
 * Uncomment EXTRUDER_RUNOUT_PREVENT to enable this feature                              *
 *                                                                                       *
 *****************************************************************************************/
//#define EXTRUDER_RUNOUT_PREVENT
#define EXTRUDER_RUNOUT_MINTEMP 190
#define EXTRUDER_RUNOUT_SECONDS  30
#define EXTRUDER_RUNOUT_ESTEPS   14  //mm filament
#define EXTRUDER_RUNOUT_SPEED  1500  //extrusion speed
#define EXTRUDER_RUNOUT_EXTRUDE 100
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
 * K=0 means advance disabled. A good value for a gregs wade extruder will be around K=75*
 *                                                                                       *
 *****************************************************************************************/
//#define LIN_ADVANCE
#define LIN_ADVANCE_K 75
/*****************************************************************************************/


/**************************************************************************
 *************************** Filament exchange ****************************
 **************************************************************************
 *                                                                        *
 * Add support for filament exchange support M600                         *
 *                                                                        *
 * Uncomment FILAMENT CHANGE FEATURE to enable this feature               *
 * Requires display                                                       *
 *                                                                        *
 **************************************************************************/
//#define FILAMENT_CHANGE_FEATURE

#define FILAMENT_CHANGE_X_POS 3             // X position of hotend
#define FILAMENT_CHANGE_Y_POS 3             // Y position of hotend
#define FILAMENT_CHANGE_Z_ADD 10            // Z addition of hotend (lift)
#define FILAMENT_CHANGE_XY_FEEDRATE 100     // X and Y axes feedrate in mm/s (also used for delta printers Z axis)
#define FILAMENT_CHANGE_Z_FEEDRATE 5        // Z axis feedrate in mm/s (not used for delta printers)
#define FILAMENT_CHANGE_RETRACT_LENGTH 2    // Initial retract in mm
                                            // It is a short retract used immediately after print interrupt before move to filament exchange position
#define FILAMENT_CHANGE_RETRACT_FEEDRATE 50 // Initial retract feedrate in mm/s
#define FILAMENT_CHANGE_UNLOAD_LENGTH 100   // Unload filament length from hotend in mm
                                            // Longer length for bowden printers to unload filament from whole bowden tube,
                                            // shorter lenght for printers without bowden to unload filament from extruder only,
                                            // 0 to disable unloading for manual unloading
#define FILAMENT_CHANGE_UNLOAD_FEEDRATE 100 // Unload filament feedrate in mm/s - filament unloading can be fast
#define FILAMENT_CHANGE_LOAD_LENGTH 100     // Load filament length over hotend in mm
                                            // Longer length for bowden printers to fast load filament into whole bowden tube over the hotend,
                                            // Short or zero length for printers without bowden where loading is not used
#define FILAMENT_CHANGE_LOAD_FEEDRATE 100   // Load filament feedrate in mm/s - filament loading into the bowden tube can be fast
#define FILAMENT_CHANGE_EXTRUDE_LENGTH 50   // Extrude filament length in mm after filament is load over the hotend,
                                            // 0 to disable for manual extrusion
                                            // Filament can be extruded repeatedly from the filament exchange menu to fill the hotend,
                                            // or until outcoming filament color is not clear for filament color change
#define FILAMENT_CHANGE_EXTRUDE_FEEDRATE 5  // Extrude filament feedrate in mm/s - must be slower than load feedrate
#define FILAMENT_CHANGE_PRINTER_OFF 5       // Minutes
/**************************************************************************/


//===========================================================================
//============================= MOTION FEATURES =============================
//===========================================================================

/**************************************************************************
 *************************** Software endstops ****************************
 **************************************************************************/
#define SOFTWARE_MIN_ENDSTOPS true  // If true, axis won't move to coordinates less than HOME_POS.
#define SOFTWARE_MAX_ENDSTOPS true  // If true, axis won't move to coordinates greater than the defined lengths below.
/**************************************************************************/


/**************************************************************************
 *********************** Endstops only for homing *************************
 **************************************************************************
 *                                                                        *
 * If defined the endstops will only be used for homing                   *
 *                                                                        *
 **************************************************************************/
#define ENDSTOPS_ONLY_FOR_HOMING
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
 *************************** Mesh Level Area ******************************
 **************************************************************************
 *                                                                        *
 * Default mesh area is an area with an inset margin on the print area.   *
 * Below are the macros that are used to define the borders for the mesh  *
 * area, made available here for specialized needs.                       *
 *                                                                        *
 **************************************************************************/
#define MESH_MIN_X (X_MIN_POS + MESH_INSET)
#define MESH_MAX_X (X_MAX_POS - (MESH_INSET))
#define MESH_MIN_Y (Y_MIN_POS + MESH_INSET)
#define MESH_MAX_Y (Y_MAX_POS - (MESH_INSET))
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

// Delay (in microseconds) before turning the servo off. This depends on the servo speed.
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
 * Babystepping enables the user to control the axis in tiny amounts,     *
 * independently from the normal printing process.                        *
 * It can e.g. be used to change z-positions in the print startup         *
 * phase in real-time.                                                    *
 * Does not respect endstops!                                             *
 *                                                                        *
 **************************************************************************/
//#define BABYSTEPPING
#define BABYSTEP_XY  // not only z, but also XY in the menu. more clutter, more functions
                     // not implemented for CoreXY and deltabots!
#define BABYSTEP_INVERT_Z false   // true for inverse movements in Z
#define BABYSTEP_MULTIPLICATOR 2  // faster z movements
/**************************************************************************/


/**************************************************************************
 *************************** Firmware retract *****************************
 **************************************************************************
 *                                                                        *
 * Firmware based and LCD controlled retract                              *
 * M207 and M208 can be used to define parameters for the retraction.     *
 * The retraction can be called by the slicer using G10 and G11           *
 * until then, intended retractions can be detected by moves that only    *
 * extrude and the direction.                                             *
 * the moves are than replaced by the firmware controlled ones.           *
 *                                                                        *
 **************************************************************************/
//#define FWRETRACT                     //ONLY PARTIALLY TESTED

#define MIN_RETRACT                 0.1 //minimum extruded mm to accept a automatic gcode retraction attempt
#define RETRACT_LENGTH              3   //default retract length (positive mm)
#define RETRACT_LENGTH_SWAP        13   //default swap retract length (positive mm), for extruder change
#define RETRACT_FEEDRATE           45   //default feedrate for retracting (mm/s)
#define RETRACT_ZLIFT               0   //default retract Z-lift
#define RETRACT_RECOVER_LENGTH      0   //default additional recover length (mm, added to retract length when recovering)
#define RETRACT_RECOVER_LENGTH_SWAP 0   //default additional swap recover length (mm, added to retract length when recovering from extruder change)
#define RETRACT_RECOVER_FEEDRATE    8   //default feedrate for recovering from retraction (mm/s)
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
//    Mode 0: Full control. The slicer has full control over both x-carriages and can achieve optimal travel results
//                           as long as it supports dual x-carriages. (M605 S0)
//    Mode 1: Auto-park mode. The firmware will automatically park and unpark the x-carriages on tool changes so
//                           that additional slicer support is not required. (M605 S1)
//    Mode 2: Duplication mode. The firmware will transparently make the second x-carriage and extruder copy all
//                           actions of the first x-carriage. This allows the printer to print 2 arbitrary items at
//                           once. (2nd extruder x offset and temp offset are set using: M605 S2 [Xnnn] [Rmmm])

// This is the default power-up mode which can be later using M605.
#define DEFAULT_DUAL_X_CARRIAGE_MODE 0

// Default settings in "Auto-park Mode"
#define TOOLCHANGE_PARK_ZLIFT   0.2      // the distance to raise Z axis when parking an extruder
#define TOOLCHANGE_UNPARK_ZLIFT 1        // the distance to raise Z axis when unparking an extruder

// Default x offset in duplication mode (typically set to half print bed width)
#define DEFAULT_DUPLICATION_X_OFFSET 100
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** X-axis dual driver ***********************************
 *****************************************************************************************
 *                                                                                       *
 * A single X stepper driver is usually used to drive 2 stepper motors.                  *
 * Uncomment this define to utilize a separate stepper driver for each X axis motor.     *
 * Only a few motherboards support this, like RAMPS,                                     *
 * which have dual extruder support (the 2nd, often unused, extruder driver is used      *
 * to control the 2nd X axis stepper motor).                                             *
 * The pins are currently only defined for a RAMPS motherboards.                         *
 * On a RAMPS (or other 5 driver) motherboard, using this feature will limit you         *
 * to using 1 extruder.                                                                  *
 *                                                                                       *
 *****************************************************************************************/
//#define X_DUAL_STEPPER_DRIVERS

// Define if the two X drives need to rotate in opposite directions
#define INVERT_X2_VS_X_DIR false
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Y-axis dual driver ***********************************
 *****************************************************************************************
 *                                                                                       *
 * A single Y stepper driver is usually used to drive 2 stepper motors.                  *
 * Uncomment this define to utilize a separate stepper driver for each Y axis motor.     *
 * Only a few motherboards support this, like RAMPS,                                     *
 * which have dual extruder support (the 2nd, often unused, extruder driver is used      *
 * to control the 2nd Y axis stepper motor).                                             *
 * The pins are currently only defined for a RAMPS motherboards.                         *
 * On a RAMPS (or other 5 driver) motherboard, using this feature will limit you         *
 * to using 1 extruder.                                                                  *
 *                                                                                       *
 *****************************************************************************************/
//#define Y_DUAL_STEPPER_DRIVERS

// Define if the two Y drives need to rotate in opposite directions
#define INVERT_Y2_VS_Y_DIR false
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Z-axis dual driver ***********************************
 *****************************************************************************************
 *                                                                                       *
 * A single Z stepper driver is usually used to drive 2 stepper motors.                  *
 * Uncomment this define to utilize a separate stepper driver for each Z axis motor.     *
 * Only a few motherboards support this, like RAMPS,                                     *
 * which have dual extruder support (the 2nd, often unused, extruder driver is used      *
 * to control the 2nd Z axis stepper motor).                                             *
 * The pins are currently only defined for a RAMPS motherboards.                         *
 * On a RAMPS (or other 5 driver) motherboard, using this feature will limit you         *
 * to using 1 extruder.                                                                  *
 *                                                                                       *
 *****************************************************************************************/
//#define Z_DUAL_STEPPER_DRIVERS

// Z DUAL ENDSTOPS is a feature to enable the use of 2 endstops for both Z steppers - Let's call them Z stepper and Z2 stepper.
// That way the machine is capable to align the bed during home, since both Z steppers are homed. 
// There is also an implementation of M666 (software endstops adjustment) to this feature.
// After Z homing, this adjustment is applied to just one of the steppers in order to align the bed.
// One just need to home the Z axis and measure the distance difference between both Z axis and apply the math: Z adjust = Z - Z2.
// If the Z stepper axis is closer to the bed, the measure Z > Z2 (yes, it is.. think about it) and the Z adjust would be positive.
// Play a little bit with small adjustments (0.5mm) and check the behaviour.
// The M119 (endstops report) will start reporting the Z2 Endstop as well.
//#define Z_DUAL_ENDSTOPS
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
 * It is assumed that when logic high = filament available and when logic         *
 * low = filament run out                                                         *
 * Single extruder only at this point (extruder 0)                                *
 *                                                                                *
 * You also need to set FILRUNOUT_PIN in Configuration_pins.h                     *
 *                                                                                *
 **********************************************************************************/
//#define FILAMENT_RUNOUT_SENSOR

#define FILRUNOUT_PIN_INVERTING true  // Should be uncommented and true or false should assigned
#define ENDSTOPPULLUP_FIL_RUNOUT      // Uncomment to use internal pullup for filament runout pins if the sensor is defined.
#define FILAMENT_RUNOUT_SCRIPT "M600" // Script execute when filament run out
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


//===========================================================================
//============================= ADDON FEATURES ==============================
//===========================================================================

/************************************************************************************************************************
 ***************************************************** EEPROM ***********************************************************
 ************************************************************************************************************************
 *                                                                                                                      *
 * The microcontroller can store settings in the EEPROM, e.g. max velocity...                                           *
 * M500 - stores parameters in EEPROM                                                                                   *
 * M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).                     *
 * M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to. *
 *                                                                                                                      *
 * Uncomment EEPROM SETTINGS to enable this feature.                                                                    *
 * Uncomment EEPROM CHITCHAT to enable EEPROM Serial responses.                                                         *
 *                                                                                                                      *
 ************************************************************************************************************************/
//#define EEPROM_SETTINGS
//#define EEPROM_CHITCHAT // Uncomment this to enable EEPROM Serial responses.
//#define DISABLE_M503
/************************************************************************************************************************/


/*****************************************************************************************
 *************************************** SDCARD *******************************************
 ****************************************************************************************/
//#define SDSUPPORT           // Enable SD Card Support in Hardware Console
//#define SDSLOW              // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SDEXTRASLOW         // Use even slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY  // Use CRC checks and retries on the SD communication
//#define SD_EXTENDED_DIR     // Show extended directory including file length. Don't use this with Pronterface

// Decomment this if you are external SD without DETECT_PIN
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
#define CFG_SD_FILE           "INFO.CFG"  // name of the configuration file
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** LCD Language ****************************************
 *****************************************************************************************
 *                                                                                       *
 * Here you may choose the language used by MK4duo on the LCD menus,                     *
 * the following list of languages are available:                                        *
 *    en, an, bg, ca, cn, cz, de, es, eu, fi, fr, gl, hr, it, kana, kana_utf8, nl, pl,   *
 *    pt, pt_utf8, pt-br, pt-br_utf8, ru                                                 *
 *                                                                                       *
 * 'en':'English',        'an':'Aragonese',   'bg':'Bulgarian',   'ca':'Catalan',        *
 * 'cn':'Chinese',        'cz':'Czech',       'de':'German',      'es':'Spanish',        *
 * 'eu':'Basque-Euskera', 'fi':'Finnish',     'fr':'French',      'gl':'Galician',       *
 * 'hr':'Croatian',       'it':'Italian',     'kana':'Japanese',  'nl':'Dutch',          *
 * 'pl':'Polish',         'pt':'Portuguese',  'ru':'Russian',                            *
 * 'kana_utf8':'Japanese (UTF8)',                                                        *
 * 'pt_utf8':'Portuguese (UTF8)',                                                        *
 * 'pt-br':'Portuguese (Brazilian)',                                                     *
 * 'pt-br_utf8':'Portuguese (Brazilian UTF8)',                                           *
 *                                                                                       *
 *****************************************************************************************/
#define LCD_LANGUAGE en
/*****************************************************************************************/


/***********************************************************************
 ******************************* LCD ***********************************
 ***********************************************************************/

//Charset type
//Choose ONE of these 3 charsets. This has to match your hardware.
//Ignored for full graphic display.
//To find out what type you have - compile with (test) - upload - click to get the menu.
//See: https://github.com/MagoKimbra/MK4duo/blob/master/Documentation/LCDLanguageFont.md
#define DISPLAY_CHARSET_HD44780_JAPAN        // this is the most common hardware
//#define DISPLAY_CHARSET_HD44780_WESTERN
//#define DISPLAY_CHARSET_HD44780_CYRILLIC
 
#define SHOW_BOOTSCREEN
//#define SHOW_CUSTOM_BOOTSCREEN
#define STRING_SPLASH_LINE1 "v" SHORT_BUILD_VERSION       // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE      // will be shown during bootup in line 2
#define SPLASH_SCREEN_DURATION 5000                       // SPLASH SCREEN duration in millisecond

//#define LCD_SCREEN_ROT_90    // Rotate screen orientation for graphics display by 90 degree clockwise
//#define LCD_SCREEN_ROT_180   // Rotate screen orientation for graphics display by 180 degree clockwise
//#define LCD_SCREEN_ROT_270   // Rotate screen orientation for graphics display by 270 degree clockwise

//#define INVERT_CLICK_BUTTON           // Option for invert encoder button logic
//#define INVERT_BACK_BUTTON            // Option for invert back button logic if avaible
//#define INVERT_ROTARY_SWITCH          // Option for reverses the encoder direction for navigating LCD menus.
#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC   75  // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value
#define ULTIPANEL_FEEDMULTIPLY          // Comment to disable setting feedrate multiplier via encoder

//#define ULTRA_LCD                              // general LCD support, also 16x2
//#define DOGLCD                                 // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
//#define ENCODER_PULSES_PER_STEP 1              // Increase if you have a high resolution encoder
//#define ENCODER_STEPS_PER_MENU_ITEM 5          // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER                    // As available from the Ultimaker online store.
//#define ULTIPANEL                              // The UltiPanel as on Thingiverse
//#define SPEAKER                                // The sound device is a speaker - not a buzzer. A buzzer resonates with his own frequency.
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 // the duration the buzzer plays the UI feedback sound. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000         // this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
                                                 // 0 to disable buzzer feedback. Test with M300 S<frequency Hz> P<duration ms>

//Display Voltage Logic Selector on Alligator Board
//#define UI_VOLTAGE_LEVEL 0 // 3.3 V
#define UI_VOLTAGE_LEVEL 1   // 5 V

// Include a page of printer information in the LCD Main Menu
#define LCD_INFO_MENU

// Original RADDS Display from Willy
// http://max3dshop.org/index.php/default/elektronik/radds-lcd-sd-display-with-reset-and-back-buttom.html
//#define RADDS_DISPLAY

// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)
// http://reprap.org/wiki/PanelOne
//#define PANEL_ONE

// The MaKr3d Makr-Panel with graphic controller and SD support
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//#define MAKRPANEL

// The Panucatt Devices Viki 2.0 and mini Viki with Graphic LCD
// http://panucatt.com
// REMEMBER TO INSTALL U8glib to your ARDUINO library folder: https://github.com/olikraus/U8glib_Arduino
//#define VIKI2
//#define miniVIKI

// This is a new controller currently under development.  https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/
//
// REMEMBER TO INSTALL U8glib to your ARDUINO library folder: https://github.com/olikraus/U8glib_Arduino
//#define ELB_FULL_GRAPHIC_CONTROLLER
//#define SD_DETECT_INVERTED

// The RepRapWorld Graphical LCD
// https://reprapworld.com/?products_details&products_id/1218
//#define REPRAPWORLD_GRAPHICAL_LCD

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// REMEMBER TO INSTALL U8glib to your ARDUINO library folder: https://github.com/olikraus/U8glib_Arduino
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

// The RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // how much should be moved when a key is pressed, eg 10.0 means 10mm per click

// The Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
// REMEMBER TO INSTALL LiquidCrystal_I2C.h in your ARDUINO library folder: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//#define RA_CONTROL_PANEL

// The MakerLab Mini Panel with graphic controller and SD support
// http://reprap.org/wiki/Mini_panel
//#define MINIPANEL

// Nextion HMI panel Serial
//#define NEXTION
#define NEXTION_SERIAL 1
// For GFX Visualization enable Nextion GFX
//#define NEXTION_GFX

// I2C Panels
//#define LCD_I2C_SAINSMART_YWROBOT

// PANELOLU2 LCD with status LEDs, separate encoder and click inputs
//
// This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
// Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
// (v1.2.3 no longer requires you to define PANELOLU in the LiquidTWI2.h library header file)
// Note: The PANELOLU2 encoder click input can either be directly connected to a pin
//       (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
//#define LCD_I2C_PANELOLU2

// Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
//#define LCD_I2C_VIKI
  
// SSD1306 OLED generic display support
// REMEMBER TO INSTALL U8glib to your ARDUINO library folder: https://github.com/olikraus/U8glib_Arduino
//#define U8GLIB_SSD1306

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection
// LCD configuration: http://reprap.org/wiki/SAV_3D_LCD
//#define SAV_3DLCD

// For dogm lcd displays you can choose some additional fonts:
//#define USE_BIG_EDIT_FONT //We don't have a big font for Cyrillic, Kana (Needs 3120 bytes of PROGMEM)
//#define USE_SMALL_INFOFONT //Smaller font on the Info-screen (Needs 2300 bytes of PROGMEM)

// Show a progress bar on HD44780 LCDs for SD printing
//#define LCD_PROGRESS_BAR
#define PROGRESS_BAR_BAR_TIME 5000 // Amount of time (ms) to show the bar
#define PROGRESS_BAR_MSG_TIME 1500 // Amount of time (ms) to show the status message
#define PROGRESS_MSG_EXPIRE   0    // Amount of time (ms) to retain the status message (0=forever)
//#define PROGRESS_MSG_ONCE        // Uncomment this to show messages for MSG_TIME then hide them
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
 ******************************* Laser beam *******************************
 **************************************************************************
 *                                                                        *
 * Support for laser beam                                                 *
 * Check also Configuration_Laser.h                                       *
 *                                                                        *
 **************************************************************************/
//#define LASERBEAM
/**************************************************************************/


//===========================================================================
//========================= ADVANCED MOTION FEATURES ========================
//===========================================================================

/***********************************************************************
 ********************* Stepper auto deactivation ***********************
 ***********************************************************************
 *                                                                     *
 * Default stepper release if idle. Set to 0 to deactivate.            *
 *                                                                     *
 ***********************************************************************/
#define DEFAULT_STEPPER_DEACTIVE_TIME 60
/***********************************************************************/


/***********************************************************************
 ************************* Low speed stepper ***************************
 ***********************************************************************
 *                                                                     *
 * Use it if you have low speed stepper driver                         *
 *                                                                     *
 ***********************************************************************/
//#define STEPPER_HIGH_LOW

// Delay in microseconds
#define STEPPER_HIGH_LOW_DELAY 1u
/***********************************************************************/


/***********************************************************************
 ************************* High speed stepper **************************
 ***********************************************************************
 *                                                                     *
 * Activate for very high stepping rates, normally only needed for 1/64*
 * or more micro steps (AXIS_STEPS_PER_UNIT * MAX_FEEDRATE > 150,000)  *
 *                                                                     *
 ***********************************************************************/
//#define ENABLE_HIGH_SPEED_STEPPING
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


/***********************************************************************
 ************************* TMC26X motor drivers ************************
 ***********************************************************************
 *                                                                     *
 * Support for TMC26X motor drivers                                    *
 *                                                                     *
 ***********************************************************************/
//#define HAVE_TMCDRIVER

//#define X_IS_TMC
#define X_MAX_CURRENT 1000  //in mA
#define X_SENSE_RESISTOR 91 //in mOhms
#define X_MICROSTEPS 16     //number of microsteps

//#define X2_IS_TMC
#define X2_MAX_CURRENT 1000  //in mA
#define X2_SENSE_RESISTOR 91 //in mOhms
#define X2_MICROSTEPS 16     //number of microsteps

//#define Y_IS_TMC
#define Y_MAX_CURRENT 1000  //in mA
#define Y_SENSE_RESISTOR 91 //in mOhms
#define Y_MICROSTEPS 16     //number of microsteps

//#define Y2_IS_TMC
#define Y2_MAX_CURRENT 1000  //in mA
#define Y2_SENSE_RESISTOR 91 //in mOhms
#define Y2_MICROSTEPS 16     //number of microsteps 

//#define Z_IS_TMC
#define Z_MAX_CURRENT 1000  //in mA
#define Z_SENSE_RESISTOR 91 //in mOhms
#define Z_MICROSTEPS 16     //number of microsteps

//#define Z2_IS_TMC
#define Z2_MAX_CURRENT 1000  //in mA
#define Z2_SENSE_RESISTOR 91 //in mOhms
#define Z2_MICROSTEPS 16     //number of microsteps

//#define E0_IS_TMC
#define E0_MAX_CURRENT 1000  //in mA
#define E0_SENSE_RESISTOR 91 //in mOhms
#define E0_MICROSTEPS 16     //number of microsteps

//#define E1_IS_TMC
#define E1_MAX_CURRENT 1000  //in mA
#define E1_SENSE_RESISTOR 91 //in mOhms
#define E1_MICROSTEPS 16     //number of microsteps 

//#define E2_IS_TMC
#define E2_MAX_CURRENT 1000  //in mA
#define E2_SENSE_RESISTOR 91 //in mOhms
#define E2_MICROSTEPS 16     //number of microsteps 

//#define E3_IS_TMC
#define E3_MAX_CURRENT 1000  //in mA
#define E3_SENSE_RESISTOR 91 //in mOhms
#define E3_MICROSTEPS 16     //number of microsteps   
/***********************************************************************/


/**********************************************************************************
 ****************************** L6470 motor drivers *******************************
 **********************************************************************************
 *                                                                                *
 * Support for L6470 motor drivers                                                *
 * You need to import the L6470 library into the arduino IDE for this.            *
 *                                                                                *
 **********************************************************************************/
//#define HAVE_L6470DRIVER

//#define X_IS_L6470
#define X_MICROSTEPS 16     //number of microsteps
#define X_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define X_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define X_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define X2_IS_L6470
#define X2_MICROSTEPS 16     //number of microsteps
#define X2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define X2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define X2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define Y_IS_L6470
#define Y_MICROSTEPS 16     //number of microsteps
#define Y_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Y_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define Y_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define Y2_IS_L6470
#define Y2_MICROSTEPS 16     //number of microsteps 
#define Y2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Y2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define Y2_STALLCURRENT 1500 //current in mA where the driver will detect a stall 

//#define Z_IS_L6470
#define Z_MICROSTEPS 16     //number of microsteps
#define Z_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Z_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define Z_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define Z2_IS_L6470
#define Z2_MICROSTEPS 16     //number of microsteps
#define Z2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define Z2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define Z2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define E0_IS_L6470
#define E0_MICROSTEPS 16     //number of microsteps
#define E0_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E0_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define E0_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define E1_IS_L6470
#define E1_MICROSTEPS 16     //number of microsteps
#define E1_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E1_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define E1_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define E2_IS_L6470
#define E2_MICROSTEPS 16     //number of microsteps
#define E2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define E2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

//#define E3_IS_L6470
#define E3_MICROSTEPS 16     //number of microsteps
#define E3_K_VAL 50          // 0 - 255, Higher values, are higher power. Be carefull not to go too high    
#define E3_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
#define E3_STALLCURRENT 1500 //current in mA where the driver will detect a stall
/**********************************************************************************/  


//===========================================================================
//============================= ADVANCED FEATURES ===========================
//===========================================================================

/****************************************************************************************
 ************************************** Buffer stuff ************************************
 ****************************************************************************************/
// The number of linear motions that can be in the plan at any give time.
// THE BLOCK BUFFER SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.
// For Arduino DUE setting BLOCK BUFFER SIZE to 32
#define BLOCK_BUFFER_SIZE 16

// The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
// For Arduino DUE setting to 8
#define BUFSIZE 4

// Defines the number of memory slots for saving/restoring position (G60/G61)
// The values should not be less than 1
#define NUM_POSITON_SLOTS 2

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME 20000

// Arc interpretation settings:
// Disabling this saves ~2738 bytes
#define ARC_SUPPORT
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

// Moves with fewer segments than this will be ignored and joined with the next movement
#define MIN_SEGMENTS_FOR_MOVE 6

// Uncomment to add the M100 Free Memory Watcher for debug purpose
//#define M100_FREE_MEMORY_WATCHER

// Comment out to remove Dump sub-command
#define M100_FREE_MEMORY_DUMPER
// Comment out to remove Corrupt sub-command
#define M100_FREE_MEMORY_CORRUPTOR
/****************************************************************************************/


/****************************************************************************************
 ********************************* Clean Nozzle Feature *********************************
 ****************************************************************************************
 *                                                                                      *
 * When enabled allows the user to send G12 to start the nozzle cleaning                *
 * process, the G-Code accepts two parameters:                                          *
 *   "P" for pattern selection                                                          *
 *   "S" for defining the number of strokes/repetitions                                 *
 *                                                                                      *
 * Available list of patterns:                                                          *
 *   P0: This is the default pattern, this process requires a sponge type               *
 *       material at a fixed bed location, the cleaning process is based on             *
 *       "strokes" i.e. back-and-forth movements between the starting and end           *
 *       points.                                                                        *
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
 * Caveats: End point Z should use the same value as Start point Z.                     *
 *                                                                                      *
 * Attention: This is an EXPERIMENTAL feature, in the future the G-code arguments       *
 * may change to add new functionality like different wipe patterns.                    *
 *                                                                                      *
 ****************************************************************************************/
//#define NOZZLE_CLEAN_FEATURE

// Number of pattern repetitions
#define NOZZLE_CLEAN_STROKES  12

// Specify positions as { X, Y, Z }
#define NOZZLE_CLEAN_START_POINT { 30, 30, (Z_MIN_POS + 1)}
#define NOZZLE_CLEAN_END_POINT   {100, 60, (Z_MIN_POS + 1)}

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
 ********************************* Start - Stop Gcode ************************************
 *****************************************************************************************
 *                                                                                       *
 * Start - Stop Gcode use when Start or Stop printing width M11 command                  *
 *                                                                                       *
 *****************************************************************************************/
//#define START_GCODE
#define START_PRINTING_SCRIPT "G28\nG1 Z10 F8000"

//#define STOP_GCODE
#define STOP_PRINTING_SCRIPT "G28\nM107\nM104 T0 S0\nM140 S0\nM84\nM81"
/*****************************************************************************************/
 
#endif
