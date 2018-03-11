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

#if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)

  #define CODE_M666

  /**
   * M666: Set Two Endstops offsets for X, Y, and/or Z.
   *
   *    X = X: Endstop Adjust
   *    Y = Y: Endstop Adjust
   *    Z = Z: Endstop Adjust
   */
  inline void gcode_M666(void) {

    SERIAL_MSG("Dual Endstop Adjustment (mm): ");
    #if ENABLED(X_TWO_ENDSTOPS)
      if (parser.seen('X')) endstops.x_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" X", endstops.x_endstop_adj);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      if (parser.seen('Y')) endstops.y_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" Y", endstops.y_endstop_adj);
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS)
      if (parser.seen('Z')) endstops.z_endstop_adj = parser.value_linear_units();
      SERIAL_MV(" Z", endstops.z_endstop_adj);
    #endif
    SERIAL_EOL();
  }

#endif // ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
