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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

  #define CODE_M321

  /**
   * M321: Set Level bilinear manual
   *
   * Usage:
   *   M321 I<xindex> J<yindex> Z<linear>
   *   M321 I<xindex> J<yindex> Q<offset>
   */
  inline void gcode_M321(void) {
    const bool hasI = parser.seen('I');
    const int8_t ix = hasI ? parser.value_int() : -1;
    const bool hasJ = parser.seen('J');
    const int8_t iy = hasJ ? parser.value_int() : -1;
    const bool hasZ = parser.seen('Z'), hasQ = !hasZ && parser.seen('Q');

    if (!hasI || !hasJ || !(hasZ || hasQ)) {
      SERIAL_LM(ER, MSG_ERR_M321_PARAMETERS);
    }
      else if (!WITHIN(ix, 0, GRID_MAX_POINTS_X - 1) || !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1)) {
      SERIAL_LM(ER, MSG_ERR_MESH_XY);
    }

    if (hasI && hasJ && !(hasZ || hasQ)) {
      SERIAL_MV("Level value in ix", ix);
      SERIAL_MV(" iy", iy);
      SERIAL_EMV(" Z", bedlevel.z_values[ix][iy]);
      return;
    }
    else {
      bedlevel.z_values[ix][iy] = parser.value_linear_units() + (hasQ ? bedlevel.z_values[ix][iy] : 0);
      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        bedlevel.bed_level_virt_interpolate();
      #endif
    }
  }

#endif // ENABLED(AUTO_BED_LEVELING_BILINEAR)
