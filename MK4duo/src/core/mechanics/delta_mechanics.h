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
 * delta_mechanics.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#pragma once

// Struct Delta Settings
typedef struct : public generic_data_t {

  float     diagonal_rod,
            radius,
            print_radius,
            probe_radius,
            height,
            diagonal_rod_adj[ABC],
            endstop_adj[ABC],
            tower_angle_adj[ABC],
            tower_radius_adj[ABC];

  uint16_t  segments_per_second;

  uint8_t   segments_per_line;

} mechanics_data_t;

class Delta_Mechanics : public Mechanics {

  public: /** Constructor */

    Delta_Mechanics() {}

  public: /** Public Parameters */

    static mechanics_data_t data;

    static float  delta[ABC],
                  delta_clip_start_height;

  private: /** Private Parameters */

    static float  delta_diagonal_rod_2[ABC],
                  towerX[ABC],
                  towerY[ABC],
                  Xbc,
                  Xca,
                  Xab,
                  Ybc,
                  Yca,
                  Yab,
                  coreFa,
                  coreFb,
                  coreFc,
                  Q,
                  Q2,
                  D2;

  public: /** Public Function */

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * Get the stepper positions in the cartesian_position[] array.
     * Forward kinematics are applied for DELTA.
     *
     * The result is in the current coordinate space with
     * leveling applied. The coordinates need to be run through
     * unapply_leveling to obtain the "ideal" coordinates
     * suitable for current_position, etc.
     */
    static void get_cartesian_from_steppers();

    #if DISABLED(AUTO_BED_LEVELING_UBL)
      /**
       * Prepare a linear move in a DELTA setup.
       *
       * This calls buffer_line several times, adding
       * small incremental moves for DELTA.
       */
      static bool prepare_move_to_destination_mech_specific();
    #endif

    /**
     *  Plan a move to (X, Y, Z) and set the current_position
     *  The final current_position may not be the one that was requested
     */
    static void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s=0.0);
    static void do_blocking_move_to_x(const float &rx, const float &fr_mm_s=0.0);
    static void do_blocking_move_to_z(const float &rz, const float &fr_mm_s=0.0);
    static void do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s=0.0);

    FORCE_INLINE static void do_blocking_move_to(const float (&raw)[XYZ], const float &fr_mm_s=0.0) {
      do_blocking_move_to(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS], fr_mm_s);
    }

    FORCE_INLINE static void do_blocking_move_to(const float (&raw)[XYZE], const float &fr_mm_s=0.0) {
      do_blocking_move_to(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS], fr_mm_s);
    }

    /**
     * Delta function
     */
    static void InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[XYZ]);
    static void InverseTransform(const float point[XYZ], float cartesian[XYZ]) { InverseTransform(point[X_AXIS], point[Y_AXIS], point[Z_AXIS], cartesian); }
    static void Transform(const float (&raw)[XYZ]);
    static void Transform(const float (&raw)[XYZE]);
    static void recalc_delta_settings();

    /**
     * Home Delta
     */
    static void home(const bool report_position=true);

    /**
     * Home an individual linear axis
     */
    static void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0);

    /**
     * Set an axis' current position to its home position (after homing).
     *
     * DELTA should wait until all homing is done before setting the XYZ
     * current_position to home, because homing is a single operation.
     * In the case where the axis positions are already known and previously
     * homed, DELTA could home to X or Y individually by moving either one
     * to the center. However, homing Z always homes XY and Z.
     *
     * Callers must sync the planner position after calling this!
     */
    static void set_axis_is_at_home(const AxisEnum axis);

    static bool position_is_reachable(const float &rx, const float &ry);
    static bool position_is_reachable_by_probe(const float &rx, const float &ry);

    /**
     * Report current position to host
     */
    static void report_current_position_detail();

    /**
     * Plan an arc in 2 dimensions
     *
     * The arc is approximated by generating many small linear segments.
     * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
     * Arcs should only be made relatively large (over 5mm), as larger arcs with
     * larger segments will tend to be more efficient. Your slicer should have
     * options for G2/G3 arc generation. In future these options may be GCode tunable.
     */
    #if ENABLED(ARC_SUPPORT)
      static void plan_arc(const float (&cart)[XYZE], const float (&offset)[2], const uint8_t clockwise);
    #endif

    #if ENABLED(DELTA_AUTO_CALIBRATION_1)
      static float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);
    #endif

    /**
     * Print mechanics parameters in memory
     */
    #if DISABLED(DISABLE_M503)
      static void print_parameters();
      static void print_M92();
      static void print_M201();
      static void print_M203();
      static void print_M204();
      static void print_M205();
      static void print_M666();
    #endif

    #if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)
      static void Nextion_gfx_clear();
    #endif

  private: /** Private Function */

    /**
     *  Home axis
     */
    static void homeaxis(const AxisEnum axis);

    /**
     * Calculate delta, start a line, and set current_position to destination
     */
    static void prepare_uninterpolated_move_to_destination(const float &fr_mm_s=0.0);

    /**
     * Calculate the highest Z position where the
     * effector has the full range of XY motion.
     */
    static void Set_clip_start_height();

    #if ENABLED(DELTA_FAST_SQRT) && ENABLED(__AVR__)
      static float Q_rsqrt(float number);
    #endif

};

extern Delta_Mechanics mechanics;
