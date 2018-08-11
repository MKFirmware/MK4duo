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
 * scara_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _SCARA_MECHANICS_H_
#define _SCARA_MECHANICS_H_

#if IS_SCARA

  class Scara_Mechanics : public Mechanics {

    public: /** Constructor */

      Scara_Mechanics() {}

    public: /** Public Parameters */

      static const float  base_max_pos[XYZ],
                          base_min_pos[XYZ],
                          base_home_pos[XYZ],
                          max_length[XYZ],
                          L1, L2,
                          L1_2, L1_2_2,
                          L2_2;

      static float  delta[ABC],
                    delta_segments_per_second;

    public: /** Public Function */

      /**
       * Initialize Factory parameters
       */
      static void factory_parameters();

      /**
       * sync_plan_position_mech_specific
       *
       * Set the planner/stepper positions directly from current_position with
       * no kinematic translation. Used for homing axes.
       */
      static void sync_plan_position_mech_specific();

      /**
       * Get the stepper positions in the cartesian_position[] array.
       * Forward kinematics are applied for SCARA.
       *
       * The result is in the current coordinate space with
       * leveling applied. The coordinates need to be run through
       * unapply_leveling to obtain the "ideal" coordinates
       * suitable for current_position, etc.
       */
      void get_cartesian_from_steppers() override;

      #if DISABLED(AUTO_BED_LEVELING_UBL)
        /**
         * Prepare a linear move in a SCARA setup.
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
      void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s=0.0) override;

      /**
       * Calculate delta, start a line, and set current_position to destination
       */
      static void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0);

      /**
       * SCARA function
       */
      static bool move_to_cal(uint8_t delta_a, uint8_t delta_b);
      static void InverseTransform(const float Ha, const float Hb, float cartesian[XYZ]);
      static void InverseTransform(const float point[XYZ], float cartesian[XYZ]) { InverseTransform(point[X_AXIS], point[Y_AXIS], cartesian); }
      static void Transform(const float raw[XYZ]);

      /**
       * Home Scara
       */
      static void home();

      /**
       * Set an axis' current position to its home position (after homing).
       *
       * SCARA should wait until all XY homing is done before setting the XY
       * current_position to home, because neither X nor Y is at home until
       * both are at home. Z can however be homed individually.
       *
       * Callers must sync the planner position after calling this!
       */
      static void set_axis_is_at_home(const AxisEnum axis);

      bool position_is_reachable(const float &rx, const float &ry) override;
      bool position_is_reachable_by_probe(const float &rx, const float &ry) override;

      /**
       * Report current position to host
       */
      void report_current_position_detail() override;

      /**
       * Home an individual linear axis
       */
      void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0) override;

      /**
       * Print mechanics parameters in memory
       */
      #if DISABLED(DISABLE_M503)
        static void print_parameters();
      #endif

    private: /** Private Function */

      /**
       *  Home axis
       */
      static void homeaxis(const AxisEnum axis);

      /**
       * Set sensorless homing.
       */
      #if ENABLED(SENSORLESS_HOMING)
        static void sensorless_homing(const bool on=true);
      #endif

  };

  extern Scara_Mechanics mechanics;

#endif // IS_SCARA

#endif /* _SCARA_MECHANICS_H_ */
