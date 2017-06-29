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
 * delta_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _DELTA_MECHANICS_H_
#define _DELTA_MECHANICS_H_

#if IS_DELTA

  class Delta_Mechanics {

    public: /** Constructor */

      Delta_Mechanics() {};

    public: /** Public Parameters */

      float delta_diagonal_rod,
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

      /**
       * Feedrate, min, max, travel
       */
      float min_feedrate_mm_s,
            max_feedrate_mm_s[XYZE_N],        // Max speeds in mm per second
            min_travel_feedrate_mm_s;

      /**
       * Step per unit
       */
      float axis_steps_per_mm[XYZE_N],
            steps_to_mm[XYZE_N];

      /**
       * Acceleration and Jerk
       */
      float     acceleration,                         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
                retract_acceleration[EXTRUDERS],      // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
                travel_acceleration,                  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
                max_jerk[XYZE_N];                     // The largest speed change requiring no acceleration
      uint32_t  max_acceleration_steps_per_s2[XYZE_N],
                max_acceleration_mm_per_s2[XYZE_N];   // Use M201 to override by software

      /**
       * Min segment time
       */
      millis_t  min_segment_time;

      /**
       * Cartesian Current Position
       *   Used to track the logical position as moves are queued.
       *   Used by 'line_to_current_position' to do a move after changing it.
       *   Used by 'sync_plan_position' to update 'planner.position'.
       */
      float current_position[ABCE];

      /**
       * Cartesian Stored Position
       *   Used to save logical position as moves are queued.
       *   Used by G60 for stored.
       *   Used by G61 for move to.
       */
      float stored_position[NUM_POSITON_SLOTS][ABCE];

      /**
       * Cartesian position
       */
      float cartesian_position[ABC];

      /**
       * Delta Position
       *   Used to move tower.
       */
      float delta[ABC];

      /**
       * Cartesian Destination
       *   A temporary position, usually applied to 'current_position'.
       *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
       *   'line_to_destination' sets 'current_position' to 'destination'.
       */
      float destination[ABCE];

      /**
       * axis_homed
       *   Flags that each linear axis was homed.
       *   ABC on delta.
       *
       * axis_known_position
       *   Flags that the position is known in each linear axis. Set when homed.
       *   Cleared whenever a stepper powers off, potentially losing its position.
       */
      bool axis_homed[ABC], axis_known_position[ABC];

      /**
       * Workspace Offset
       */
      #if ENABLED(WORKSPACE_OFFSETS)
        // The distance that XYZ has been offset by G92. Reset by G28.
        float position_shift[XYZ] = { 0 };

        // This offset is added to the configured home position.
        // Set by M206, M428, or menu item. Saved to EEPROM.
        float home_offset[XYZ] = { 0 };

        // The above two are combined to save on computes
        float workspace_offset[XYZ] = { 0 };
      #endif

      /**
       * Feed rates are often configured with mm/m
       * but the planner and stepper like mm/s units.
       */
      float   feedrate_mm_s             = MMM_TO_MMS(1500.0),
              saved_feedrate_mm_s       = MMM_TO_MMS(1500.0);
      int16_t feedrate_percentage       = 100,
              saved_feedrate_percentage = 100;

      /**
       * Homing feed rates are often configured with mm/m
       * but the planner and stepper like mm/s units.
       */
      const float homing_feedrate_mm_s[ABC] = { MMM_TO_MMS(HOMING_FEEDRATE_XYZ), MMM_TO_MMS(HOMING_FEEDRATE_XYZ), MMM_TO_MMS(HOMING_FEEDRATE_XYZ) },
                  home_bump_mm[ABC]         = { X_HOME_BUMP_MM, Y_HOME_BUMP_MM, Z_HOME_BUMP_MM };

      const signed char home_dir[ABC]       = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };

      #if HAS_DELTA_AUTO_CALIBRATION
        bool g33_in_progress = false;
      #endif

    public: /** Public Function */

      /**
       * Initialize Delta parameters
       */
      void Init();

      /**
       * Get the position (mm) of an axis based on stepper position(s)
       */
      float get_axis_position_mm(AxisEnum axis);

      /**
       * Set the planner.position and individual stepper positions.
       * Used by G92, G28, G29, and other procedures.
       *
       * Multiplies by axis_steps_per_mm[] and does necessary conversion
       *
       * Clears previous speed values.
       */
      void set_position_mm(const float &a, const float &b, const float &c, const float &e);
      void set_position_mm(const AxisEnum axis, const float &v);
      void set_position_mm_kinematic(const float position[NUM_AXIS]);
      FORCE_INLINE void set_z_position_mm(const float &z) { set_position_mm(AxisEnum(Z_AXIS), z); }
      FORCE_INLINE void set_e_position_mm(const float &e) { set_position_mm(AxisEnum(E_AXIS), e); }

      /**
       * Get the stepper positions in the cartesian_position[] array.
       * Forward kinematics are applied for DELTA.
       *
       * The result is in the current coordinate space with
       * leveling applied. The coordinates need to be run through
       * unapply_leveling to obtain the "ideal" coordinates
       * suitable for current_position, etc.
       */
      void get_cartesian_from_steppers();

      /**
       * Set the current_position for an axis based on
       * the stepper positions, removing any leveling that
       * may have been applied.
       */
      void set_current_from_steppers_for_axis(const AxisEnum axis);

      /**
       * Set current to destination and set destination to current
       */
      FORCE_INLINE void set_current_to_destination() { COPY_ARRAY(current_position, destination); }
      FORCE_INLINE void set_destination_to_current() { COPY_ARRAY(destination, current_position); }

      /**
       * line_to_current_position
       * Move the planner to the current position from wherever it last moved
       * (or from wherever it has been told it is located).
       */
      void line_to_current_position();

      /**
       * line_to_destination
       * Move the planner to the position stored in the destination array, which is
       * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
       */
      void line_to_destination(float fr_mm_s);
      void line_to_destination();

      /**
       * Prepare a linear move in a DELTA setup.
       *
       * This calls buffer_line several times, adding
       * small incremental moves for DELTA.
       */
      void prepare_move_to_destination();

      /**
       *  Plan a move to (X, Y, Z) and set the current_position
       *  The final current_position may not be the one that was requested
       */
      void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s = 0.0);
      void do_blocking_move_to(const float logical[ABC], const float &fr_mm_s = 0.0);
      void do_blocking_move_to_x(const float &lx, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_z(const float &lz, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s = 0.0);

      /**
       * sync_plan_position
       *
       * Set the planner/stepper positions kinematic
       */
      void sync_plan_position();
      void sync_plan_position_e();

      void InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[ABC]);
      void InverseTransform(const float point[ABC], float cartesian[ABC]) { InverseTransform(point[A_AXIS], point[B_AXIS], point[C_AXIS], cartesian); }
      void Transform(const float logical[ABC]);
      void Transform_segment_raw(const float rx, const float ry, const float rz, const float le, const float fr);
      void recalc_delta_settings();
      void reset_acceleration_rates();
      void refresh_positioning();

      /**
       * Home Delta
       */
      void Home(const bool always_home_all=true);

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

      bool axis_unhomed_error(const bool x=true, const bool y=true, const bool z=true);
      bool position_is_reachable_raw_xy(const float &rx, const float &ry);
      bool position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry);
      bool position_is_reachable_by_probe_xy(const float &lx, const float &ly);
      bool position_is_reachable_xy(const float &lx, const float &ly);

      #if HAS_DELTA_AUTO_CALIBRATION
        void auto_calibration();
      #endif

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);
        void Adjust(const uint8_t numFactors, const float v[]);
        void Convert_endstop_adj();
      #endif

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z);
        void print_xyz(const char* prefix, const char* suffix, const float xyz[]);
      #endif

    private: /** Private Parameters */

      // Derived values
      float  delta_diagonal_rod_2[ABC],  // Diagonal rod 2
                    towerX[ABC],                // The X coordinate of each tower
                    towerY[ABC],                // The Y coordinate of each tower
                    homed_Height,
                    printRadiusSquared,
                    Xbc, Xca, Xab, Ybc, Yca, Yab,
                    coreFa, coreFb, coreFc,
                    Q, Q2, D2;

    private: /** Private Function */

      /**
       * Home an individual linear axis
       */
      void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0);

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

      /**
       * Some planner shorthand inline functions
       */
      float get_homing_bump_feedrate(const AxisEnum axis);

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        void NormaliseEndstopAdjustments();
      #endif

      #if ENABLED(DELTA_FAST_SQRT) && DISABLED(MATH_USE_HAL)
        float Q_rsqrt(float number);
      #endif

  };

  extern Delta_Mechanics Mechanics;

#endif // IS_DELTA

#endif // _DELTA_MECHANICS_H_
