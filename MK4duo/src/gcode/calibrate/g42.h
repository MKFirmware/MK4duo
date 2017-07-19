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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

  #define CODE_G42

  /**
   * G42: Move X & Y axes to mesh coordinates (I & J)
   */
  inline void gcode_G42(void) {
    if (printer.IsRunning()) {
      const bool hasI = parser.seenval('I');
      const int8_t ix = hasI ? parser.value_int() : 0;
      const bool hasJ = parser.seenval('J');
      const int8_t iy = hasJ ? parser.value_int() : 0;

      if ((hasI && !WITHIN(ix, 0, GRID_MAX_POINTS_X - 1)) || (hasJ && !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1))) {
        SERIAL_EM(MSG_ERR_MESH_XY);
        return;
      }

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        #define _GET_MESH_X(I) bedlevel.bilinear_start[X_AXIS] + I * bedlevel.bilinear_grid_spacing[X_AXIS]
        #define _GET_MESH_Y(J) bedlevel.bilinear_start[Y_AXIS] + J * bedlevel.bilinear_grid_spacing[Y_AXIS]
      #elif ENABLED(MESH_BED_LEVELING)
        #define _GET_MESH_X(I) mbl.index_to_xpos[I]
        #define _GET_MESH_Y(J) mbl.index_to_ypos[J]
      #endif

      mechanics.set_destination_to_current();
      if (hasI) mechanics.destination[X_AXIS] = LOGICAL_X_POSITION(_GET_MESH_X(ix));
      if (hasJ) mechanics.destination[Y_AXIS] = LOGICAL_Y_POSITION(_GET_MESH_Y(iy));
      if (parser.boolval('P')) {
        if (hasI) mechanics.destination[X_AXIS] -= X_PROBE_OFFSET_FROM_NOZZLE;
        if (hasJ) mechanics.destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_NOZZLE;
      }

      const float fval = parser.linearval('F');
      if (fval > 0.0) mechanics.feedrate_mm_s = MMM_TO_MMS(fval);

      mechanics.prepare_move_to_destination();

    }
  }

#endif // ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)
