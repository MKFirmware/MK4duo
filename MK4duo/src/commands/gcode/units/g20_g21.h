/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * gcode.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#define CODE_G20
#define CODE_G21

#define INCH_FACTOR 25.4
#define MM_FACTOR    1.0

/**
 * G20: Set input mode to inches
 */
inline void gcode_G20(void) {
  #if ENABLED(INCH_MODE_SUPPORT)
    parser.set_input_linear_units(INCH_FACTOR);
  #else
    NOOP;
  #endif
}

/**
 * G21: Set input mode to millimeters
 */
inline void gcode_G21(void) {
  #if ENABLED(INCH_MODE_SUPPORT)
    parser.set_input_linear_units(MM_FACTOR);
  #else
    NOOP;
  #endif
}
