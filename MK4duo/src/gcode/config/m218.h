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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define CODE_M218

/**
 * M218 - set hotend offset (in linear units)
 *
 *   T<tool>
 *   X<xoffset>
 *   Y<yoffset>
 *   Z<zoffset>
 */
inline void gcode_M218(void) {

  GET_TARGET_HOTEND(218);
  if (TARGET_EXTRUDER == 0) return;

  if (parser.seenval('X')) tools.hotend_offset[X_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();
  if (parser.seenval('Y')) tools.hotend_offset[Y_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();
  if (parser.seenval('Z')) tools.hotend_offset[Z_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();

  SERIAL_SM(ECHO, MSG_HOTEND_OFFSET);
  LOOP_HOTEND() {
    SERIAL_MV(" ", tools.hotend_offset[X_AXIS][h]);
    SERIAL_MV(",", tools.hotend_offset[Y_AXIS][h]);
    SERIAL_MV(",", tools.hotend_offset[Z_AXIS][h]);
  }
  SERIAL_EOL();
}
