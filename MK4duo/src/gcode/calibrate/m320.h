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

#if HAS_ABL

  #define CODE_M320

  /**
   * M320: Enable/Disable Bed Leveling and/or set the Z fade height.
   *
   *       S[bool]   Turns leveling on or off
   *       Z[height] Sets the Z fade height (0 or none to disable)
   *       V[bool]   Verbose - Print the leveling grid
   */
  inline void gcode_M320(void) {

    // V to print the matrix
    if (parser.seen('V')) {
      #if ABL_PLANAR
        bedlevel.matrix.debug(PSTR("Bed Level Correction Matrix:"));
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (bedlevel.leveling_is_valid()) {
          bedlevel.print_bilinear_leveling_grid();
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bedlevel.print_bilinear_leveling_grid_virt();
          #endif
        }
      #endif
    }

    bool to_enable = false;
    if (parser.seen('S')) {
      to_enable = parser.value_bool();
      bedlevel.set_bed_leveling_enabled(to_enable);
    }

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (parser.seen('Z')) {
        bedlevel.set_z_fade_height(parser.value_linear_units());
        SERIAL_LMV(ECHO, "ABL Fade Height = ", parser.value_linear_units(), 2);
      }
    #endif

    const bool new_status = bedlevel.leveling_is_active();
    if (to_enable && !new_status)
      SERIAL_LM(ER, MSG_ERR_M320_M420_FAILED);

    SERIAL_LMV(ECHO, "ABL: ", new_status ? MSG_ON : MSG_OFF);
  }

#endif // HAS_ABL
