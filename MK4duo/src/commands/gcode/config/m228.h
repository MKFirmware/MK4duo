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
 * mcode
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if NOMECH(DELTA)

#define CODE_M228

/**
 * M228 - Set axis min/max travel
 *
 *   Snnn 0 = set axis maximum (default), 1 = set axis minimum
 *   Xnnn X axis limit
 *   Ynnn Y axis limit
 *   Znnn Z axis limit
 */
inline void gcode_M228(void) {

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M201 report.
    if (!parser.seen("XYZ")) {
      mechanics.print_M228();
      return;
    }
  #endif

  const bool minimum = parser.boolval('S');

  if (minimum) {
    if (parser.seenval('X')) mechanics.data.base_pos[X_AXIS].min = parser.value_linear_units();
    if (parser.seenval('Y')) mechanics.data.base_pos[Y_AXIS].min = parser.value_linear_units();
    if (parser.seenval('Z')) mechanics.data.base_pos[Z_AXIS].min = parser.value_linear_units();
  }
  else {
    if (parser.seenval('X')) mechanics.data.base_pos[X_AXIS].max = parser.value_linear_units();
    if (parser.seenval('Y')) mechanics.data.base_pos[Y_AXIS].max = parser.value_linear_units();
    if (parser.seenval('Z')) mechanics.data.base_pos[Z_AXIS].max = parser.value_linear_units();
  }

  LOOP_XYZ(axis) endstops.update_software_endstops(AxisEnum(axis));

}

#endif
