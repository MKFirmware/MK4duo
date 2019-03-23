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

#if ENABLED(NOZZLE_CLEAN_FEATURE)

  #define CODE_G12

  /**
   * G12: Clean the nozzle
   */
  inline void gcode_G12(void) {
    // Don't allow nozzle cleaning without homing first
    if (mechanics.axis_unhomed_error()) { return; }

    const uint8_t pattern = parser.seen('P') ? parser.value_ushort() : 0,
                  strokes = parser.seen('S') ? parser.value_ushort() : NOZZLE_CLEAN_STROKES,
                  objects = parser.seen('T') ? parser.value_ushort() : NOZZLE_CLEAN_TRIANGLES;
    const float   radius  = parser.seen('R') ? parser.value_float()  : NOZZLE_CLEAN_CIRCLE_RADIUS;

    Nozzle::clean(pattern, strokes, radius, objects);
  }

#endif
