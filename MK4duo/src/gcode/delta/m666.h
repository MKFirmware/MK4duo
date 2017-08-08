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

#if MECH(DELTA)

  #define CODE_M666

  /**
   * M666: Set delta endstop and geometry adjustment
   *
   *    D = Diagonal Rod
   *    R = Delta Radius
   *    S = Segments per Second
   *    A = Alpha (Tower 1) Diagonal Rod Adjust
   *    B = Beta  (Tower 2) Diagonal Rod Adjust
   *    C = Gamma (Tower 3) Diagonal Rod Adjust
   *    I = Alpha (Tower 1) Tower Radius Adjust
   *    J = Beta  (Tower 2) Tower Radius Adjust
   *    K = Gamma (Tower 3) Tower Radius Adjust
   *    U = Alpha (Tower 1) Tower Position Adjust
   *    V = Beta  (Tower 2) Tower Position Adjust
   *    W = Gamma (Tower 3) Tower Position Adjust
   *    X = Alpha (Tower 1) Endstop Adjust
   *    Y = Beta  (Tower 2) Endstop Adjust
   *    Z = Gamma (Tower 3) Endstop Adjust
   *    O = Print radius
   *    Q = Probe radius
   *    P = Z probe offset
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
    if (parser.seen('I')) mechanics.delta_tower_radius_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('J')) mechanics.delta_tower_radius_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('K')) mechanics.delta_tower_radius_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('U')) mechanics.delta_tower_pos_adj[A_AXIS]     = parser.value_linear_units();
    if (parser.seen('V')) mechanics.delta_tower_pos_adj[B_AXIS]     = parser.value_linear_units();
    if (parser.seen('W')) mechanics.delta_tower_pos_adj[C_AXIS]     = parser.value_linear_units();
    if (parser.seen('O')) mechanics.delta_print_radius              = parser.value_linear_units();
    if (parser.seen('Q')) mechanics.delta_probe_radius              = parser.value_linear_units();

    mechanics.recalc_delta_settings();

    #if HAS_BED_PROBE

      if (parser.seen('P')) {

        SERIAL_SM(ECHO, MSG_ZPROBE_ZOFFSET);
        SERIAL_CHR(' ');

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

        SERIAL_EOL();
      }

    #endif // HAS_BED_PROBE

    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) mechanics.delta_endstop_adj[i] = parser.value_linear_units();
    }

    if (parser.seen('L')) {
      SERIAL_LM(CFG, "Current Delta geometry values:");
      LOOP_XYZ(i) {
        SERIAL_SV(CFG, axis_codes[i]);
        SERIAL_EMV(" (Endstop Adj): ", mechanics.delta_endstop_adj[i], 3);
      }

      #if HAS_BED_PROBE
        SERIAL_LMV(CFG, "P (ZProbe ZOffset): ", probe.zprobe_zoffset, 3);
      #endif

      SERIAL_LMV(CFG, "A (Tower A Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[0], 3);
      SERIAL_LMV(CFG, "B (Tower B Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[1], 3);
      SERIAL_LMV(CFG, "C (Tower C Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[2], 3);
      SERIAL_LMV(CFG, "I (Tower A Radius Correction): ",        mechanics.delta_tower_radius_adj[0], 3);
      SERIAL_LMV(CFG, "J (Tower B Radius Correction): ",        mechanics.delta_tower_radius_adj[1], 3);
      SERIAL_LMV(CFG, "K (Tower C Radius Correction): ",        mechanics.delta_tower_radius_adj[2], 3);
      SERIAL_LMV(CFG, "U (Tower A Position Correction): ",      mechanics.delta_tower_pos_adj[0], 3);
      SERIAL_LMV(CFG, "V (Tower B Position Correction): ",      mechanics.delta_tower_pos_adj[1], 3);
      SERIAL_LMV(CFG, "W (Tower C Position Correction): ",      mechanics.delta_tower_pos_adj[2], 3);
      SERIAL_LMV(CFG, "R (Delta Radius): ",                     mechanics.delta_radius, 4);
      SERIAL_LMV(CFG, "D (Diagonal Rod Length): ",              mechanics.delta_diagonal_rod, 4);
      SERIAL_LMV(CFG, "S (Delta Segments per second): ",        mechanics.delta_segments_per_second);
      SERIAL_LMV(CFG, "O (Delta Print Radius): ",               mechanics.delta_print_radius);
      SERIAL_LMV(CFG, "Q (Delta Probe Radius): ",               mechanics.delta_probe_radius);
      SERIAL_LMV(CFG, "H (Z-Height): ",                         mechanics.delta_height, 3);
    }
  }

#endif // MECH DELTA
