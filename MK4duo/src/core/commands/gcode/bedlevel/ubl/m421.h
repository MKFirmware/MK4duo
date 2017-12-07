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

#if ENABLED(AUTO_BED_LEVELING_UBL)

  #define CODE_M421

  /**
   * M421: Set a single Mesh Bed Leveling Z coordinate
   *
   * Usage:
   *   M421 I<xindex> J<yindex> Z<linear>
   *   M421 I<xindex> J<yindex> Q<offset>
   *   M421 C Z<linear>
   *   M421 C Q<offset>
   */
  inline void gcode_M421(void) {
    int8_t ix = parser.intval('I', -1), iy = parser.intval('J', -1);
    const bool  hasI = ix >= 0,
                hasJ = iy >= 0,
                hasC = parser.seen('C'),
                hasZ = parser.seen('Z'),
                hasQ = !hasZ && parser.seen('Q');

    if (hasC) {
      const mesh_index_pair location = ubl.find_closest_mesh_point_of_type(REAL, mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], USE_NOZZLE_AS_REFERENCE, NULL);
      ix = location.x_index;
      iy = location.y_index;
    }

    if (int(hasC) + int(hasI && hasJ) != 1 || !(hasZ || hasQ))
      SERIAL_LM(ER, MSG_ERR_M421_PARAMETERS);
    else if (!WITHIN(ix, 0, GRID_MAX_POINTS_X - 1) || !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1))
      SERIAL_LM(ER, MSG_ERR_MESH_XY);
    else
      ubl.z_values[ix][iy] = parser.value_linear_units() + (hasQ ? ubl.z_values[ix][iy] : 0);
  }

#endif // ENABLED(MESH_BED_LEVELING)
