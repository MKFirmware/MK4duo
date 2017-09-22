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

#if IS_SCARA

  #define CODE_M665

  /**
   * M665: Set SCARA settings
   *
   * Parameters:
   *
   *   S[segments-per-second] - Segments-per-second
   *   P[theta-psi-offset]    - Theta-Psi offset, added to the shoulder (A/X) angle
   *   T[theta-offset]        - Theta     offset, added to the elbow    (B/Y) angle
   *
   *   A, P, and X are all aliases for the shoulder angle
   *   B, T, and Y are all aliases for the elbow angle
   */
  inline void gcode_M665() {
    if (parser.seen('S')) mechanics.delta_segments_per_second = parser.value_float();

    #if ENABLED(WORKSPACE_OFFSETS)
      const bool hasA = parser.seen('A'), hasP = parser.seen('P'), hasX = parser.seen('X');
      const uint8_t sumAPX = hasA + hasP + hasX;
      if (sumAPX == 1)
        mechanics.home_offset[A_AXIS] = parser.value_float();
      else if (sumAPX > 1) {
        SERIAL_EM("Only one of A, P, or X is allowed.");
        return;
      }

      const bool hasB = parser.seen('B'), hasT = parser.seen('T'), hasY = parser.seen('Y');
      const uint8_t sumBTY = hasB + hasT + hasY;
      if (sumBTY == 1)
        mechanics.home_offset[B_AXIS] = parser.value_float();
      else if (sumBTY > 1) {
        SERIAL_EM("Only one of B, T, or Y is allowed.");
        return;
      }
    #endif // WORKSPACE_OFFSETS
  }
#endif // IS_SCARA
