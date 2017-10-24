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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _HEATER_SANITYCHECK_H_
#define _HEATER_SANITYCHECK_H_

// Test required HEATER defines
#if HOTENDS > 3
  #if !HAS_HEATER_3
    #error DEPENDENCY ERROR: HEATER_3_PIN not EXIST for this board
  #endif
#elif HOTENDS > 2
  #if !HAS_HEATER_2
    #error DEPENDENCY ERROR: HEATER_2_PIN not EXIST for this board
  #endif
#elif HOTENDS > 1
  #if !HAS_HEATER_1
    #error DEPENDENCY ERROR: HEATER_1_PIN not EXIST for this board
  #endif
#elif HOTENDS > 0
  #if !HAS_HEATER_0
    #error DEPENDENCY ERROR: HEATER_0_PIN not EXIST for this board
  #endif
#endif

#endif /* _HEATER_SANITYCHECK_H_ */
