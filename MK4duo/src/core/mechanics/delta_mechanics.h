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
 * delta_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _DELTA_MECHANICS_H_
#define _DELTA_MECHANICS_H_

#if IS_DELTA

  class Delta_Mechanics : public Mechanics {

    public: /** Constructor */

      Delta_Mechanics() {}

    public: /** Public Parameters */

      static float  delta[ABC],
                    delta_diagonal_rod,
                    delta_radius,
                    delta_segments_per_second,
                    delta_print_radius,
                    delta_probe_radius,
                    delta_height,
                    delta_clip_start_height,
                    delta_diagonal_rod_adj[ABC],
                    delta_endstop_adj[ABC],
                    delta_tower_angle_adj[ABC],
                    delta_tower_radius_adj[ABC];

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
       * sync_plan_position_mech_specific
       *
       * Set the planner/stepper positions directly from current_position with
       * kinematic translation. Used for homing axes and cartesian/core syncing.
       */
      static void sync_plan_position_mech_specific();

      /**
       * Get the stepper positions in the cartesian_position[] array.
       * Forward kinematics are applied for DELTA.
       *
       * The result is in the current coordinate space with
       * leveling applied. The coordinates need to be run through
       * unapply_leveling to obtain the "ideal" coordinates
       * suitable for current_position, etc.
       */
      void get_cartesian_from_steppers() override;

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
      void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s=0.0) override;

      /**
       * Delta function
       */
      static void InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[XYZ]);
      static void InverseTransform(const float point[XYZ], float cartesian[XYZ]) { InverseTransform(point[X_AXIS], point[Y_AXIS], point[Z_AXIS], cartesian); }
      static void Transform(const float raw[]);
      static void Transform_buffer_segment(const float raw[], const float fr);
      static void recalc_delta_settings();

      /**
       * Home Delta
       */
      static void home();

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

      bool position_is_reachable(const float &rx, const float &ry) override;
      bool position_is_reachable_by_probe(const float &rx, const float &ry) override;

      /**
       * Report current position to host
       */
      void report_current_position_detail() override;

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        static float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);
      #endif

      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        void Nextion_gfx_clear() override;
      #endif

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
       * Calculate delta, start a line, and set current_position to destination
       */
      static void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0);

      /**
       * Calculate the highest Z position where the
       * effector has the full range of XY motion.
       */
      static void Set_clip_start_height();

      #if ENABLED(DELTA_FAST_SQRT) && ENABLED(__AVR__)
        static float Q_rsqrt(float number);
      #endif

      /**
       * Set sensorless homing.
       */
      #if ENABLED(SENSORLESS_HOMING)
        static void sensorless_homing(const bool on=true);
      #endif

  };

  extern Delta_Mechanics mechanics;

#endif // IS_DELTA

#endif /* _DELTA_MECHANICS_H_ */
