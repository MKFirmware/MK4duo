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

#ifndef _CONFIGURATION_LASER_H_
#define _CONFIGURATION_LASER_H_

//===========================================================================
//============================= Laser Settings ==============================
//===========================================================================

// The following define selects how to control the laser.
// Please choose the one that matches your setup.
// 1 = Single pin control - LASER_PWR_PIN LOW when off, HIGH when on, PWM to adjust intensity
// 2 = Two pin control    - LASER_PWR_PIN for which LOW = off, HIGH = on, and a seperate LASER_PWM_PIN which carries a constant PWM signal and adjusts duty cycle to control intensity
#define LASER_CONTROL 1

// In the case that the laserdriver need at least a certain level "LASER_REMAP_INTENSITY"
// to give anything, the intensity can be remapped to start at "LASER_REMAP_INTENSITY"
// At least some CO2-drivers need it, not sure about laserdiode drivers.
// comment this line if your do not want to remap the intensity. Saves 154 bytes and a few CPU cycles
#define LASER_REMAP_INTENSITY 7

// Normally the intensity is a value between 0 and 100 percent.
// If you want a value in bytes (0 to 255), activate the following function.
//#define INTENSITY_IN_BYTE

// Uncomment the following if your laser firing pin (not the PWM pin) for two pin control requires a HIGH signal to fire rather than a low (eg Red Sail M300 RS 3040)
// #define HIGH_TO_FIRE

// Uncomment the following if your laser pwm pin (not the power pin) needs to be inverted.
//#define LASER_PWM_INVERT

// The following defines select which G codes tell the laser to fire. It's OK to uncomment more than one.
#define LASER_FIRE_SPINDLE    // fire the laser on M3, extinguish on M5
//#define LASER_FIRE_G1       // fire the laser on a G1 move, extinguish when the move ends
//#define LASER_FIRE_E        // fire the laser when the E axis moves

// Raster mode enables the laser to etch bitmap data at high speeds. Increases command buffer size substantially.
#define LASER_RASTER
#define LASER_MAX_RASTER_LINE 68      // Maximum number of base64 encoded pixels per raster gcode command
#define LASER_RASTER_ASPECT_RATIO 1   // pixels aren't square on most displays, 1.33 == 4:3 aspect ratio. 
#define LASER_RASTER_MM_PER_PULSE 0.2 // Can be overridden by providing an R value in M649 command : M649 S17 B2 D0 R0.1 F4000

//#define LASER_RASTER_MANUAL_Y_FEED // Do not perform any X or Y movements on a G7 $ direction change. Manual Moves must be made between each line.

// Uncomment the following if the laser cutter is equipped with a peripheral relay board
// to control power to an exhaust fan, cooler pump, laser power supply, etc.
//#define LASER_PERIPHERALS
//#define LASER_PERIPHERALS_TIMEOUT 30000  // Number of milliseconds to wait for status signal from peripheral control board

// Uncomment the following line to enable cubic bezier curve movement with the G5 code
// #define G5_BEZIER

#define LASER_WATTS 40.0
#define LASER_DIAMETER 0.1        // milimeters
#define LASER_PWM 25000           // hertz
#define LASER_FOCAL_HEIGHT 74.50  // z axis position at which the laser is focused

#endif /* _CONFIGURATION_LASER_H_ */
