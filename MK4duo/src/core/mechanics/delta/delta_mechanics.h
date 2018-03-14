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

      float delta[ABC]                  = { 0.0 },
            delta_diagonal_rod          = 0.0,
            delta_radius                = 0.0,
            delta_segments_per_second   = 0.0,
            delta_print_radius          = 0.0,
            delta_probe_radius          = 0.0,
            delta_height                = 0.0,
            delta_clip_start_height     = 0.0,
            delta_diagonal_rod_adj[ABC] = { 0.0 },
            delta_endstop_adj[ABC]      = { 0.0 },
            delta_tower_angle_adj[ABC]  = { 0.0 },
            delta_tower_radius_adj[ABC] = { 0.0 };

    private: /** Private Parameters */

      float delta_diagonal_rod_2[ABC] = { 0.0 },  // Diagonal rod 2
            towerX[ABC]               = { 0.0 },  // The X coordinate of each tower
            towerY[ABC]               = { 0.0 },  // The Y coordinate of each tower
            Xbc                       = 0.0,
            Xca                       = 0.0,
            Xab                       = 0.0,
            Ybc                       = 0.0,
            Yca                       = 0.0,
            Yab                       = 0.0,
            coreFa                    = 0.0,
            coreFb                    = 0.0,
            coreFc                    = 0.0,
            Q                         = 0.0,
            Q2                        = 0.0,
            D2                        = 0.0;

    public: /** Public Function */

      /**
       * Initialize Delta parameters
       */
      void init();

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

      #if DISABLED(AUTO_BED_LEVELING_UBL)
        /**
         * Prepare a linear move in a DELTA setup.
         *
         * This calls buffer_line several times, adding
         * small incremental moves for DELTA.
         */
        bool prepare_move_to_destination_mech_specific();
      #endif

      /**
       *  Plan a move to (X, Y, Z) and set the current_position
       *  The final current_position may not be the one that was requested
       */
      void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s=0.0) override;

      /**
       * Delta function
       */
      void InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[XYZ]);
      void InverseTransform(const float point[XYZ], float cartesian[XYZ]) { InverseTransform(point[X_AXIS], point[Y_AXIS], point[Z_AXIS], cartesian); }
      void Transform(const float raw[]);
      void Transform_buffer_segment(const float raw[], const float fr);
      void recalc_delta_settings();

      /**
       * Home Delta
       */
      bool home(const bool always_home_all=true);

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

      bool position_is_reachable(const float &rx, const float &ry) override;
      bool position_is_reachable_by_probe(const float &rx, const float &ry) override;

      /**
       * Report current position to host
       */
      void report_current_position_detail() override;

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);
      #endif

      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        void Nextion_gfx_clear() override;
      #endif

      /**
       * Set sensorless homing if the axis has it.
       */
      #if ENABLED(SENSORLESS_HOMING)
        void sensorless_homing_per_axis(const AxisEnum axis, const bool enable=true);
      #endif

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

      #if ENABLED(DELTA_FAST_SQRT) && DISABLED(MATH_USE_HAL)
        float Q_rsqrt(float number);
      #endif

  };

  extern Delta_Mechanics mechanics;

#endif // IS_DELTA

#endif /* _DELTA_MECHANICS_H_ */
