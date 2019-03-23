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

#if MECH(DELTA)

  #define CODE_M666

  /**
   * M666: Set delta endstop and geometry adjustment
   *
   *    D = Diagonal Rod
   *    R = Delta Radius
   *    S = Segments per Second
   *    L = Segments per Line
   *    A = Tower A: Diagonal Rod Adjust
   *    B = Tower B: Diagonal Rod Adjust
   *    C = Tower C: Diagonal Rod Adjust
   *    I = Tower A: Tower Angle Adjust
   *    J = Tower B: Tower Angle Adjust
   *    K = Tower C: Tower Angle Adjust
   *    U = Tower A: Tower Radius Adjust
   *    V = Tower B: Tower Radius Adjust
   *    W = Tower C: Tower Radius Adjust
   *    X = Tower A: Endstop Adjust
   *    Y = Tower B: Endstop Adjust
   *    Z = Tower C: Endstop Adjust
   *    O = Print radius
   *    P = Probe radius
   *    H = Z Height
   */
  inline void gcode_M666(void) {

    #if DISABLED(DISABLE_M503)
      // No arguments? Show M666 report.
      if (!parser.seen("XYZHDRSLAB") && !parser.seen("CIJKUVWOP")) {
        mechanics.print_M666();
        return;
      }
    #endif

    if (parser.seen('H')) mechanics.data.height                   = parser.value_linear_units();
    if (parser.seen('D')) mechanics.data.diagonal_rod             = parser.value_linear_units();
    if (parser.seen('R')) mechanics.data.radius                   = parser.value_linear_units();
    if (parser.seen('S')) mechanics.data.segments_per_second      = parser.value_ushort();
    if (parser.seen('L')) mechanics.data.segments_per_line        = parser.value_byte();
    if (parser.seen('A')) mechanics.data.diagonal_rod_adj[A_AXIS] = parser.value_linear_units();
    if (parser.seen('B')) mechanics.data.diagonal_rod_adj[B_AXIS] = parser.value_linear_units();
    if (parser.seen('C')) mechanics.data.diagonal_rod_adj[C_AXIS] = parser.value_linear_units();
    if (parser.seen('I')) mechanics.data.tower_angle_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('J')) mechanics.data.tower_angle_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('K')) mechanics.data.tower_angle_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('U')) mechanics.data.tower_radius_adj[A_AXIS] = parser.value_linear_units();
    if (parser.seen('V')) mechanics.data.tower_radius_adj[B_AXIS] = parser.value_linear_units();
    if (parser.seen('W')) mechanics.data.tower_radius_adj[C_AXIS] = parser.value_linear_units();
    if (parser.seen('O')) mechanics.data.print_radius             = parser.value_linear_units();
    if (parser.seen('P')) mechanics.data.probe_radius             = parser.value_linear_units();

    NOLESS(mechanics.data.segments_per_line, 10);
    NOMORE(mechanics.data.segments_per_line, 255);

    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) {
        const float v = parser.value_linear_units();
        if (v <= 0) mechanics.data.endstop_adj[i] = v;
      }
    }

    mechanics.recalc_delta_settings();

  }

#endif // MECH DELTA
