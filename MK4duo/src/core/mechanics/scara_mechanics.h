/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * scara_mechanics.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#pragma once

// Struct Scara Settings
typedef struct : public generic_data_t {

  float_limits_t base_pos[XYZ];

  float         base_home_pos[XYZ],
                segments_per_second;

} mechanics_data_t;

class Scara_Mechanics : public Mechanics {

  public: /** Constructor */

    Scara_Mechanics() {}

  public: /** Public Parameters */

    static mechanics_data_t data;

    static const float  max_length[XYZ],
                        L1, L2,
                        L1_2, L1_2_2,
                        L2_2;

    static float  delta[ABC];

  public: /** Public Function */

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * Get the stepper positions in the cartesian_position[] array.
     * Forward kinematics are applied for SCARA.
     *
     * The result is in the current coordinate space with
     * leveling applied. The coordinates need to be run through
     * unapply_leveling to obtain the "ideal" coordinates
     * suitable for position.x, etc.
     */
    static void get_cartesian_from_steppers();

    #if DISABLED(AUTO_BED_LEVELING_UBL)
      /**
       * Prepare a linear move in a SCARA setup.
       *
       * This calls buffer_line several times, adding
       * small incremental moves for SCARA.
       */
      static bool prepare_move_to_destination_mech_specific();
    #endif

    /**
     *  Plan a move to (X, Y, Z) and set the position.x
     *  The final position.x may not be the one that was requested
     */
    static void do_blocking_move_to(const float rx, const float ry, const float rz, const feedrate_t &fr_mm_s=0.0);
    static void do_blocking_move_to_x(const float &rx, const feedrate_t &fr_mm_s=0.0);
    static void do_blocking_move_to_z(const float &rz, const feedrate_t &fr_mm_s=0.0);
    static void do_blocking_move_to_xy(const float &rx, const float &ry, const feedrate_t &fr_mm_s=0.0);

    FORCE_INLINE static void do_blocking_move_to(const float (&raw)[XYZ], const feedrate_t &fr_mm_s=0.0) {
      do_blocking_move_to(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS], fr_mm_s);
    }

    FORCE_INLINE static void do_blocking_move_to(const float (&raw)[XYZE], const feedrate_t &fr_mm_s=0.0) {
      do_blocking_move_to(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS], fr_mm_s);
    }

    /**
     * SCARA function
     */
    static void InverseTransform(const float Ha, const float Hb, float cartesian[XYZ]);
    static void InverseTransform(const float point[XYZ], float cartesian[XYZ]) { InverseTransform(point[X_AXIS], point[Y_AXIS], cartesian); }
    static void Transform(const float raw[XYZ]);

    /**
     * MORGAN SCARA function
     */
    #if MECH(MORGAN_SCARA)
      static bool move_to_cal(uint8_t delta_a, uint8_t delta_b);
    #endif

    /**
     * Home Scara
     */
    static void home();

    /**
     * Home an individual linear axis
     */
    static void do_homing_move(const AxisEnum axis, const float distance, const feedrate_t fr_mm_s=0.0f);

    /**
     * Set an axis' current position to its home position (after homing).
     *
     * SCARA should wait until all XY homing is done before setting the XY
     * position.x to home, because neither X nor Y is at home until
     * both are at home. Z can however be homed individually.
     *
     * Callers must sync the planner position after calling this!
     */
    static void set_axis_is_at_home(const AxisEnum axis);

    static bool position_is_reachable(const float &rx, const float &ry);
    static bool position_is_reachable_by_probe(const float &rx, const float &ry);

    /**
     * Calculate delta, start a line, and set position.x to destination
     */
    static void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0);

    /**
     * Report current position to host
     */
    static void report_detail_position();

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
      static void print_M206();
    #endif

  private: /** Private Function */

    /**
     *  Home axis
     */
    static void homeaxis(const AxisEnum axis);

};

extern Scara_Mechanics mechanics;
