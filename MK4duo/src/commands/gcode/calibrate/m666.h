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
 * mcode
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_MULTI_ENDSTOP

  #define CODE_M666

  /**
   * M666: Set Two Endstops offsets for X, Y, and/or Z.
   *
   *        X = X: Endstop Adjust
   *        Y = Y: Endstop Adjust
   *        Z = Z: Endstop Adjust
   *
   * For Triple Z Endstops:
   *        Set Z2 Only: M666 S2 Z<offset>
   *        Set Z3 Only: M666 S3 Z<offset>
   *           Set Both: M666 Z<offset>
   */
  inline void gcode_M666(void) {

    SERIAL_MSG("Dual Endstop Adjustment (mm): ");
    #if ENABLED(X_TWO_ENDSTOPS)
      if (parser.seen('X')) endstops.x2_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" X2:", endstops.x2_endstop_adj);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      if (parser.seen('Y')) endstops.y2_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" Y2:", endstops.y2_endstop_adj);
    #endif
    #if ENABLED(Z_THREE_ENDSTOPS)
      if (parser.seen('Z')) {
        const int ind = parser.intval('S');
        const float z_adj = parser.value_linear_units();
        if (!ind || ind == 2) endstops.z2_endstop_adj = z_adj;
        if (!ind || ind == 3) endstops.z3_endstop_adj = z_adj;
      }
      SERIAL_MV(" Z2:", endstops.z2_endstop_adj);
      SERIAL_MV(" Z3:", endstops.z3_endstop_adj);
    #elif ENABLED(Z_TWO_ENDSTOPS)
      if (parser.seen('Z')) endstops.z2_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" Z2:", endstops.z2_endstop_adj);
    #endif
    SERIAL_EOL();
  }

#endif // ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
