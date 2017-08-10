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
 * Configuration_CNCRouter.h - CNC Router control for MK4duo Version 1
 * Copyright (c) 2017 Franco (nextime) Lanza
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef _CONFIGURATION_CNCROUTER_H_
#define _CONFIGURATION_CNCROUTER_H_

//===========================================================================
//=========================== CNCRouter Settings ============================
//===========================================================================

// Enable speed control by PWM
//#define FAST_PWM_CNCROUTER 

// router speed settings in rpm
#define MAX_CNCROUTER_SPEED 32000
#define MIN_CNCROUTER_SPEED 300

// Router safe Z position
#define CNCROUTER_SAFE_Z 25.0

// Work around for bad PWM drivers for spindles
#define MAX_CNCROUTER_PWM_VAL 245

// Some routers power supply have issues with motor starting current,
// enable slowstart to avoid that
#define CNCROUTER_SLOWSTART
#define CNCROUTER_SLOWSTART_STEP 5000     // rpm of every step of the slow start accelleration
#define CNCROUTER_SLOWSTART_INTERVAL 0.5  // seconds between change of speed 

// Router have inverted rotation support (not yet supported)
//#define CNCROUTER_ANTICLOCKWISE

// Router enable pin is inverted polarity
//#define INVERTED_CNCROUTER_PIN

// Automatic tool change (not yet supported)
//#define CNCROUTER_AUTO_TOOL_CHANGE

// CNC ROUTERS must have EMERGENCY_PARSER
#define EMERGENCY_PARSER

#endif /* _CONFIGURATION_CNCROUTER_H_ */
