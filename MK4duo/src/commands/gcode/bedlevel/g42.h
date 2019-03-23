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

#if HAS_MESH

  #define CODE_G42

  /**
   * G42: Move X & Y axes to mesh coordinates (I & J)
   */
  inline void gcode_G42(void) {
    if (printer.isRunning()) {
      const bool hasI = parser.seenval('I');
      const int8_t ix = hasI ? parser.value_int() : 0;
      const bool hasJ = parser.seenval('J');
      const int8_t iy = hasJ ? parser.value_int() : 0;

      if ((hasI && !WITHIN(ix, 0, GRID_MAX_POINTS_X - 1)) || (hasJ && !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1))) {
        SERIAL_EM(MSG_ERR_MESH_XY);
        return;
      }

      mechanics.set_destination_to_current();
      if (hasI) mechanics.destination[X_AXIS] = _GET_MESH_X(ix);
      if (hasJ) mechanics.destination[Y_AXIS] = _GET_MESH_Y(iy);
      if (parser.boolval('P')) {
        if (hasI) mechanics.destination[X_AXIS] -= probe.data.offset[X_AXIS];
        if (hasJ) mechanics.destination[Y_AXIS] -= probe.data.offset[Y_AXIS];
      }

      const float fval = parser.linearval('F');
      if (fval > 0.0) mechanics.feedrate_mm_s = MMM_TO_MMS(fval);

      // SCARA kinematic has "safe" XY raw moves
      #if IS_SCARA
        mechanics.prepare_uninterpolated_move_to_destination();
      #else
        mechanics.prepare_move_to_destination();
      #endif

    }
  }

#endif // ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)
