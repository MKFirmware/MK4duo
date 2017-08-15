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

      Delta_Mechanics() {};

    public: /** Public Parameters */

      float delta[ABC],
            delta_diagonal_rod,
            delta_radius,
            delta_segments_per_second,
            delta_print_radius,
            delta_probe_radius,
            delta_height,
            delta_clip_start_height,
            delta_diagonal_rod_adj[ABC],
            delta_endstop_adj[ABC],
            delta_tower_radius_adj[ABC],
            delta_tower_pos_adj[ABC];

      #if HAS_DELTA_AUTO_CALIBRATION
        bool g33_in_progress = false;
      #endif

    public: /** Public Function */

      /**
       * Initialize Delta parameters
       */
      void Init();

      /**
       * Set the planner.position and individual stepper positions.
       * Used by G92, G28, G29, and other procedures.
       */
      void set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e) override;
      void set_position_mm(const float position[NUM_AXIS]) override;

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

      /**
       * Prepare a linear move in a DELTA setup.
       *
       * This calls buffer_line several times, adding
       * small incremental moves for DELTA.
       */
      bool prepare_move_to_destination_mech_specific() override;

      /**
       *  Plan a move to (X, Y, Z) and set the current_position
       *  The final current_position may not be the one that was requested
       */
      void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s=0.0) override;

      void manual_goto_xy(const float &x, const float &y) override;

      /**
       * Delta function
       */
      void InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[ABC]);
      void InverseTransform(const float point[ABC], float cartesian[ABC]) { InverseTransform(point[A_AXIS], point[B_AXIS], point[C_AXIS], cartesian); }
      void Transform(const float logical[ABC]);
      void Transform_segment_raw(const float rx, const float ry, const float rz, const float le, const float fr);
      void recalc_delta_settings();

      /**
       * Home Delta
       */
      bool Home(const bool always_home_all=true);

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
      void set_axis_is_at_home(const AxisEnum axis);

      //bool position_is_reachable_raw_xy(const float &rx, const float &ry) override;
      //bool position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) override;

      #if HAS_DELTA_AUTO_CALIBRATION
        void auto_calibration();
      #endif

      /**
       * Report current position to host
       */
      void report_current_position_detail() override;

    private: /** Private Parameters */

      // Derived values
      float  delta_diagonal_rod_2[ABC],  // Diagonal rod 2
                    towerX[ABC],                // The X coordinate of each tower
                    towerY[ABC],                // The Y coordinate of each tower
                    homed_Height,
                    Xbc, Xca, Xab, Ybc, Yca, Yab,
                    coreFa, coreFb, coreFc,
                    Q, Q2, D2;

    private: /** Private Function */
      
      /**
       *  Home axis
       */
      void homeaxis(const AxisEnum axis);

      /**
       * Calculate delta, start a line, and set current_position to destination
       */
      void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0);

      /**
       * Calculate the highest Z position where the
       * effector has the full range of XY motion.
       */
      void Set_clip_start_height();

      void Adjust(const uint8_t numFactors, const float v[]);
      void Convert_endstop_adj();
      void NormaliseEndstopAdjustments();
      float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);

      void Calibration_cleanup(
        #if HOTENDS > 1
          const uint8_t old_tool_index
        #endif
      );

      /**
       * Print data
       */
      void print_signed_float(const char * const prefix, const float &f);
      void print_G33_settings(const bool end_stops, const bool tower_angles);

      #if ENABLED(DELTA_FAST_SQRT) && DISABLED(MATH_USE_HAL)
        float Q_rsqrt(float number);
      #endif

  };

  extern Delta_Mechanics mechanics;

#endif // IS_DELTA

#endif /* _DELTA_MECHANICS_H_ */
