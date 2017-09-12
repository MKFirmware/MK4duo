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

#if MECH(DELTA)

  #define CODE_M666

  /**
   * M666: Set delta endstop and geometry adjustment
   *
   *    D = Diagonal Rod
   *    R = Delta Radius
   *    S = Segments per Second
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

    if (parser.seen('H')) {
      const float old_delta_height = mechanics.delta_height;
      mechanics.delta_height = parser.value_linear_units();
      mechanics.current_position[Z_AXIS] += mechanics.delta_height - old_delta_height;
    }

    if (parser.seen('D')) mechanics.delta_diagonal_rod              = parser.value_linear_units();
    if (parser.seen('R')) mechanics.delta_radius                    = parser.value_linear_units();
    if (parser.seen('S')) mechanics.delta_segments_per_second       = parser.value_float();
    if (parser.seen('A')) mechanics.delta_diagonal_rod_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('B')) mechanics.delta_diagonal_rod_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('C')) mechanics.delta_diagonal_rod_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('I')) mechanics.delta_tower_angle_adj[A_AXIS]   = parser.value_linear_units();
    if (parser.seen('J')) mechanics.delta_tower_angle_adj[B_AXIS]   = parser.value_linear_units();
    if (parser.seen('K')) mechanics.delta_tower_angle_adj[C_AXIS]   = parser.value_linear_units();
    if (parser.seen('U')) mechanics.delta_tower_radius_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('V')) mechanics.delta_tower_radius_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('W')) mechanics.delta_tower_radius_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('O')) mechanics.delta_print_radius              = parser.value_linear_units();
    if (parser.seen('P')) mechanics.delta_probe_radius              = parser.value_linear_units();

    mechanics.recalc_delta_settings();

    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) mechanics.delta_endstop_adj[i] = parser.value_linear_units();
    }

    if (parser.seen('L')) {
      SERIAL_LM(CFG, "Current Delta geometry values:");
      LOOP_XYZ(i) {
        SERIAL_SV(CFG, axis_codes[i]);
        SERIAL_EMV(" (Endstop Adj): ", mechanics.delta_endstop_adj[i], 3);
      }

      SERIAL_LMV(CFG, "A (Tower A Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[0], 3);
      SERIAL_LMV(CFG, "B (Tower B Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[1], 3);
      SERIAL_LMV(CFG, "C (Tower C Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[2], 3);
      SERIAL_LMV(CFG, "I (Tower A Angle Correction): ",         mechanics.delta_tower_angle_adj[0], 3);
      SERIAL_LMV(CFG, "J (Tower B Angle Correction): ",         mechanics.delta_tower_angle_adj[1], 3);
      SERIAL_LMV(CFG, "K (Tower C Angle Correction): ",         mechanics.delta_tower_angle_adj[2], 3);
      SERIAL_LMV(CFG, "U (Tower A Radius Correction): ",        mechanics.delta_tower_radius_adj[0], 3);
      SERIAL_LMV(CFG, "V (Tower B Radius Correction): ",        mechanics.delta_tower_radius_adj[1], 3);
      SERIAL_LMV(CFG, "W (Tower C Radius Correction): ",        mechanics.delta_tower_radius_adj[2], 3);
      SERIAL_LMV(CFG, "R (Delta Radius): ",                     mechanics.delta_radius, 4);
      SERIAL_LMV(CFG, "D (Diagonal Rod Length): ",              mechanics.delta_diagonal_rod, 4);
      SERIAL_LMV(CFG, "S (Delta Segments per second): ",        mechanics.delta_segments_per_second);
      SERIAL_LMV(CFG, "O (Delta Print Radius): ",               mechanics.delta_print_radius);
      SERIAL_LMV(CFG, "P (Delta Probe Radius): ",               mechanics.delta_probe_radius);
      SERIAL_LMV(CFG, "H (Z-Height): ",                         mechanics.delta_height, 3);
    }
  }

#endif // MECH DELTA
