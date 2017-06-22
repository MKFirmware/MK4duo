/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * Configuration_Laser.h
 *
 * This configuration file contains all laser functions.
 *
 */

#ifndef CONFIGURATION_LASER
#define CONFIGURATION_LASER

//===========================================================================
//============================= Laser Settings ==============================
//===========================================================================

// The following define selects how to control the laser.
// Please choose the one that matches your setup.
// 1 = Single pin control - LASER_PWR_PIN LOW when off, HIGH when on, PWM to adjust intensity
// 2 = Two pin control    - LASER_PWR_PIN for which LOW = off, HIGH = on, and a seperate LASER_PWM_PIN which carries a constant PWM signal and adjusts duty cycle to control intensity
#define LASER_CONTROL 1

// If your machine has laser focuser, set this to true and it will use Z axis for focus or disable it.
#define LASER_HAS_FOCUS false // (Removed from current code)

// In the case that the laserdriver need at least a certain level "LASER_REMAP_INTENSITY"
// to give anything, the intensity can be remapped to start at "LASER_REMAP_INTENSITY"
// At least some CO2-drivers need it, not sure about laserdiode drivers.
// comment this line if your do not want to remap the intensity. Saves 154 bytes and a few CPU cycles
//#define LASER_REMAP_INTENSITY 7

// Uncomment the following if your laser firing pin (not the PWM pin) for two pin control requires a HIGH signal to fire rather than a low (eg Red Sail M300 RS 3040)
#define HIGH_TO_FIRE

// Uncomment the following if your laser pwm pin (not the power pin) needs to be inverted.
#define LASER_PWM_INVERT

// The following defines select which G codes tell the laser to fire. It's OK to uncomment more than one.
#define LASER_FIRE_G1 10      // fire the laser on a G1 move, extinguish when the move ends
#define LASER_FIRE_SPINDLE 11 // fire the laser on M3, extinguish on M5
#define LASER_FIRE_E 12       // fire the laser when the E axis moves

// Raster mode enables the laser to etch bitmap data at high speeds. Increases command buffer size substantially.
#define LASER_RASTER
#define LASER_MAX_RASTER_LINE 68      // Maximum number of base64 encoded pixels per raster gcode command
#define LASER_RASTER_ASPECT_RATIO 1   // pixels aren't square on most displays, 1.33 == 4:3 aspect ratio. 
#define LASER_RASTER_MM_PER_PULSE 0.1 // Can be overridden by providing an R value in M649 command : M649 S17 B2 D0 R0.1 F4000

#define LASER_RASTER_MANUAL_Y_FEED // Do not perform any X or Y movements on a G7 @ direction change. Manual Moves must be made between each line. Has no effect on $ direction changes to stay compatible with turnkey plugin.

// Uncomment the following if the laser cutter is equipped with a peripheral relay board
// to control power to an exhaust fan, cooler pump, laser power supply, etc.
//#define LASER_PERIPHERALS
//#define LASER_PERIPHERALS_TIMEOUT 30000  // Number of milliseconds to wait for status signal from peripheral control board

// Uncomment the following line to enable cubic bezier curve movement with the G5 code
// #define G5_BEZIER

// Uncomment these options for the Buildlog.net laser cutter, and other similar models
//#define LASER_WATTS 40.0
//#define LASER_DIAMETER 0.1        // milimeters
//#define LASER_PWM 50000           // hertz
//#define LASER_FOCAL_HEIGHT 74.50  // z axis position at which the laser is focused


// Uncomment these options for the mUVe 1 3D printer
//#ifdef CUSTOM_MACHINE_NAME
// #undef CUSTOM_MACHINE_NAME
// #define CUSTOM_MACHINE_NAME "mUVe1 Printer"
//#endif
//#define LASER_WATTS 0.05
//#define LASER_DIAMETER 0.1 // milimeters
//#define LASER_PWM 8000 // hertz
//#define MUVE_Z_PEEL // The mUVe 1 uses a special peel maneuver between each layer, it requires independent control of each Z motor

// Uncomment these options for the Buildlog.net laser cutter, and other similar models
//#ifdef CUSTOM_MACHINE_NAME
// #undef CUSTOM_MACHINE_NAME
// #define CUSTOM_MACHINE_NAME "Laser Cutter"
//#endif
//#define LASER_WATTS 40.0
//#define LASER_DIAMETER 0.1 // milimeters
//#define LASER_PWM 25000 // hertz
//#define LASER_FOCAL_HEIGHT 74.50 // z axis position at which the laser is focused

// Uncomment these options for the K40 laser cutter, and other similar models
//#ifdef CUSTOM_MACHINE_NAME
// #undef CUSTOM_MACHINE_NAME
// #define CUSTOM_MACHINE_NAME "K40 Laser"
//#endif
//#define LASER_WATTS 40.0
//#define LASER_DIAMETER 0.1 // milimeters
//#define LASER_PWM 50000 // hertz
//#define LASER_FOCAL_HEIGHT 50 // z axis position at which the laser is focused
//  Maximum start speed for accelerated moves.    X,    Y,  Z,   E0...(per extruder)
//#ifdef DEFAULT_MAX_ACCELERATION
//  #undef DEFAULT_MAX_ACCELERATION
//  #define DEFAULT_MAX_ACCELERATION              {5000, 5000, 50, 1000, 1000, 1000, 1000}
//#endif
//  X, Y, Z and E* maximum acceleration in mm/s^2 for printing moves
//#ifdef DEFAULT_ACCELERATION
//  #undef DEFAULT_ACCELERATION
//  #define DEFAULT_ACCELERATION          5000
//#endif
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
//#ifdef DEFAULT_TRAVEL_ACCELERATION
//  #undef DEFAULT_TRAVEL_ACCELERATION
//  #define DEFAULT_TRAVEL_ACCELERATION   20000
//#endif



// Uncomment these options for the All Things RC A5 laser Engraver, and other similar models
#ifdef CUSTOM_MACHINE_NAME //if the macro CUSTOM_MACHINE_NAME is defined 
  #undef CUSTOM_MACHINE_NAME //un-define it
  #define CUSTOM_MACHINE_NAME "A5 Laser"//redefine it with the new value
#endif
#define LASER_WATTS 5.0
#define LASER_DIAMETER 0.1 // milimeters
#define LASER_PWM 16000 // hertz
#define LASER_FOCAL_HEIGHT 74.50 // z axis position at which the laser is focused
//  Maximum start speed for accelerated moves.    X,    Y,  Z,   E0...(per extruder)
#ifdef DEFAULT_MAX_ACCELERATION
  #undef DEFAULT_MAX_ACCELERATION
  #define DEFAULT_MAX_ACCELERATION              {1000, 500, 50, 1000, 1000, 1000, 1000}
#endif
//  X, Y, Z and E* maximum acceleration in mm/s^2 for printing moves
#ifdef DEFAULT_ACCELERATION
  #undef DEFAULT_ACCELERATION
  #define DEFAULT_ACCELERATION          1000
#endif
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
#ifdef DEFAULT_TRAVEL_ACCELERATION
  #undef DEFAULT_TRAVEL_ACCELERATION
  #define DEFAULT_TRAVEL_ACCELERATION   1000
#endif

// Uncomment these options for the All Things RC A3 laser Engraver, and other similar models
//#ifdef CUSTOM_MACHINE_NAME
// #undef CUSTOM_MACHINE_NAME
// #define CUSTOM_MACHINE_NAME "A3 Laser"
//#endif
//#define LASER_WATTS 5.0
//#define LASER_DIAMETER 0.1 // milimeters
//#define LASER_PWM 10000 // hertz
//#define LASER_FOCAL_HEIGHT 74.50 // z axis position at which the laser is focused
//  Maximum start speed for accelerated moves.    X,    Y,  Z,   E0...(per extruder)
//#ifdef DEFAULT_MAX_ACCELERATION
//  #undef DEFAULT_MAX_ACCELERATION
//  #define DEFAULT_MAX_ACCELERATION              {4000, 4000, 50, 1000, 1000, 1000, 1000}
//#endif
//  X, Y, Z and E* maximum acceleration in mm/s^2 for printing moves
//#ifdef DEFAULT_ACCELERATION
//  #undef DEFAULT_ACCELERATION
//  #define DEFAULT_ACCELERATION          2000
//#endif
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
//#ifdef DEFAULT_TRAVEL_ACCELERATION
//  #undef DEFAULT_TRAVEL_ACCELERATION
//  #define DEFAULT_TRAVEL_ACCELERATION   15000
//#endif

// Uncomment these options for the All Things RC Pocket laser Engraver, and other similar models
//#ifdef CUSTOM_MACHINE_NAME
// #undef CUSTOM_MACHINE_NAME
// #define CUSTOM_MACHINE_NAME "Pocket Laser"
//#endif
//#define LASER_WATTS 0.5
//#define LASER_DIAMETER 0.07 // milimeters
//#define LASER_PWM 8000 // hertz
//#define LASER_FOCAL_HEIGHT 74.50 // z axis position at which the laser is focused
//  Maximum start speed for accelerated moves.    X,    Y,  Z,   E0...(per extruder)
//#ifdef DEFAULT_MAX_ACCELERATION
//  #undef DEFAULT_MAX_ACCELERATION
//  #define DEFAULT_MAX_ACCELERATION              {500, 500, 50, 1000, 1000, 1000, 1000}
//#endif
//  X, Y, Z and E* maximum acceleration in mm/s^2 for printing moves
//#ifdef DEFAULT_ACCELERATION
//  #undef DEFAULT_ACCELERATION
//  #define DEFAULT_ACCELERATION          200
//#endif
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
//#ifdef DEFAULT_TRAVEL_ACCELERATION
//  #undef DEFAULT_TRAVEL_ACCELERATION
//  #define DEFAULT_TRAVEL_ACCELERATION   200
//#endif


#endif
