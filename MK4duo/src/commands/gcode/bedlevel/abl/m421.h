/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

#define CODE_M421

/**
 * M421: Set one or more Mesh Bed Leveling Z coordinates
 *
 * Usage:
 *   M421 I<xindex> J<yindex> Z<linear>
 *   M421 I<xindex> J<yindex> Q<offset>
 *
 *  - If I is omitted, set the entire row
 *  - If J is omitted, set the entire column
 *  - If both I and J are omitted, set all
 */
inline void gcode_M421() {

  int8_t  ix = parser.intval('I', -1),
          iy = parser.intval('J', -1);

  const bool  hasZ = parser.seen('Z'),
              hasQ = !hasZ && parser.seen('Q');

  if (hasZ || hasQ) {
    if (WITHIN(ix, -1, GRID_MAX_POINTS_X - 1) && WITHIN(iy, -1, GRID_MAX_POINTS_Y - 1)) {
      const float zval = parser.value_linear_units();
      uint8_t sx = ix >= 0 ? ix : 0, ex = ix >= 0 ? ix : GRID_MAX_POINTS_X - 1,
              sy = iy >= 0 ? iy : 0, ey = iy >= 0 ? iy : GRID_MAX_POINTS_Y - 1;
      LOOP_S_LE_N(x, sx, ex) {
        LOOP_S_LE_N(y, sy, ey) {
          abl.data.z_values[x][y] = zval + (hasQ ? abl.data.z_values[x][y] : 0);
        }
      }
      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        abl.virt_interpolate();
      #endif
    }
    else {
      SERIAL_LM(ER, STR_ERR_MESH_XY);
    }
  }
  else {
    SERIAL_LM(ER, STR_ERR_M421_PARAMETERS);
  }

}

#endif // ENABLED(AUTO_BED_LEVELING_BILINEAR)
