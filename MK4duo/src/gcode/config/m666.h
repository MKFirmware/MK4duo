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

#if HAS_BED_PROBE && NOMECH(DELTA)

  #define CODE_M666

  // M666: Set Z probe offset
  inline void gcode_M666(void) {

    SERIAL_SM(ECHO, MSG_ZPROBE_ZOFFSET);
    SERIAL_CHR(' ');

    if (parser.seen('P')) {
      float p_val = parser.value_linear_units();
      if (Z_PROBE_OFFSET_RANGE_MIN <= p_val && p_val <= Z_PROBE_OFFSET_RANGE_MAX) {

        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          // Correct bilinear grid for new probe offset
          const float diff = p_val - probe.zprobe_zoffset;
          if (diff) {
            for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
              for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
                bedlevel.z_values[x][y] += diff;
          }
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bedlevel.bed_level_virt_interpolate();
          #endif
        #endif

        probe.zprobe_zoffset = p_val;
        SERIAL_VAL(probe.zprobe_zoffset);
      }
      else {
        SERIAL_MT(MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
        SERIAL_CHR(' ');
        SERIAL_MT(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
      }
    }
    else {
      SERIAL_MV(": ", probe.zprobe_zoffset, 3);
    }

    SERIAL_EOL();
  }

#endif // HAS_BED_PROBE && NOMECH(DELTA)
