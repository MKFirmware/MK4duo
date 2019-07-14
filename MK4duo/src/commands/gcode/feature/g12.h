/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(NOZZLE_CLEAN_FEATURE)

#define CODE_G12

/**
 * G12: Clean the nozzle
 */
inline void gcode_G12(void) {

  // Don't allow nozzle cleaning without homing first
  if (mechanics.axis_unhomed_error()) return;

  const bool seenxyz = parser.seen("XYZ"),
             clean_x = !seenxyz || parser.boolval('X'),
             clean_y = !seenxyz || parser.boolval('Y');

  #if ENABLED(NOZZLE_CLEAN_NO_Z)
    static constexpr bool clean_z = false;
  #else
    const bool clean_z = !seenxyz || parser.boolval('Z');
  #endif

  const uint8_t pattern = parser.ushortval('P', 0),
                strokes = parser.ushortval('S', NOZZLE_CLEAN_STROKES),
                objects = parser.ushortval('T', NOZZLE_CLEAN_TRIANGLES);
  const float   radius  = parser.floatval('R', NOZZLE_CLEAN_CIRCLE_RADIUS);

  #if HAS_LEVELING
    const bool was_enabled = bedlevel.flag.leveling_active;
    if (clean_z) bedlevel.set_bed_leveling_enabled(false);
  #endif

  Nozzle::clean(pattern, strokes, radius, objects, clean_x, clean_y, clean_z);

  // Re-enable bed level correction if it had been on
  #if HAS_LEVELING
    if (clean_z) bedlevel.set_bed_leveling_enabled(was_enabled);
  #endif

}

#endif
