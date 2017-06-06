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

      float diagonal_rod,
            delta_radius,
            segments_per_second,
            print_radius,
            probe_radius,
            delta_height,
            clip_start_height,
            diagonal_rod_adj[ABC],
            endstop_adj[ABC],
            tower_radius_adj[ABC],
            tower_pos_adj[ABC];

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
       * Homing feed rates are often configured with mm/m
       * but the planner and stepper like mm/s units.
       */
      const float homing_feedrate_mm_s[ABC] = { MMM_TO_MMS(HOMING_FEEDRATE_XYZ), MMM_TO_MMS(HOMING_FEEDRATE_XYZ), MMM_TO_MMS(HOMING_FEEDRATE_XYZ) },
                  home_bump_mm[ABC]         = { X_HOME_BUMP_MM, Y_HOME_BUMP_MM, Z_HOME_BUMP_MM };

      const signed char home_dir[ABC]       = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };

    public: /** Public Function */

      /**
       * Initialize Delta parameters
       */
      void Init();

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
       * Move the planner, not necessarily synced with current_position
       */
      void line_to_destination(float fr_mm_s);
      void line_to_destination();

      /**
       * Prepare a single move and get ready for the next one.
       *
       * This calls buffer_line several times, adding
       * small incremental moves for DELTA.
       */
      void prepare_move_to_destination();

      /**
       * Blocking movement and shorthand functions
       */
      void do_blocking_move_to(const float &x, const float &y, const float &z, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_x(const float &x, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_z(const float &z, const float &fr_mm_s = 0.0);
      void do_blocking_move_to_xy(const float &x, const float &y, const float &fr_mm_s = 0.0);

      /**
       * sync_plan_position
       *
       * Set the planner/stepper positions kinematic
       */
      void sync_plan_position();
      void sync_plan_position_e();

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);
        void Adjust(const uint8_t numFactors, const float v[]);
        void Convert_endstop_adj();
      #endif

      void InverseTransform(const float Ha, const float Hb, const float Hc, float cartesian[ABC]);
      void InverseTransform(const float point[ABC], float cartesian[ABC]) { InverseTransform(point[A_AXIS], point[B_AXIS], point[C_AXIS], cartesian); }
      void Transform(const float logical[ABC]);
      void Recalc();
      void Set_clip_start_height();

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
      void set_axis_is_at_home(AxisEnum axis);

      bool axis_unhomed_error(const bool x=true, const bool y=true, const bool z=true);
      bool position_is_reachable_raw_xy(const float &rx, const float &ry);
      bool position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry);
      bool position_is_reachable_by_probe_xy(const float &lx, const float &ly);
      bool position_is_reachable_xy(const float &lx, const float &ly);

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
      void do_homing_move(AxisEnum axis, const float distance, float fr_mm_s=0.0);

      /**
       *  Home axis
       */
      void homeaxis(const AxisEnum axis);

      /**
       * Calculate delta, start a line, and set current_position to destination
       */
      void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0);

      /**
       * Some planner shorthand inline functions
       */
      float get_homing_bump_feedrate(AxisEnum axis);

      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        void NormaliseEndstopAdjustments();
      #endif

      #if ENABLED(DELTA_FAST_SQRT) && DISABLED(MATH_USE_HAL)
        float Q_rsqrt(float number);
      #endif

  };

  extern Delta_Mechanics Mechanics;

  // DEBUG LEVELING
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    #define DEBUG_POS(SUFFIX,VAR)       do{ \
      Mechanics.print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); } while(0)
  #endif

  // Workspace offsets
  #if ENABLED(WORKSPACE_OFFSETS)
    #define WORKSPACE_OFFSET(AXIS) Mechanics.workspace_offset[AXIS]
  #else
    #define WORKSPACE_OFFSET(AXIS) 0
  #endif

  #define LOGICAL_POSITION(POS, AXIS) ((POS) + WORKSPACE_OFFSET(AXIS))
  #define RAW_POSITION(POS, AXIS)     ((POS) - WORKSPACE_OFFSET(AXIS))

  #if ENABLED(WORKSPACE_OFFSETS)
    #define LOGICAL_X_POSITION(POS)   LOGICAL_POSITION(POS, X_AXIS)
    #define LOGICAL_Y_POSITION(POS)   LOGICAL_POSITION(POS, Y_AXIS)
    #define LOGICAL_Z_POSITION(POS)   LOGICAL_POSITION(POS, Z_AXIS)
    #define RAW_X_POSITION(POS)       RAW_POSITION(POS, X_AXIS)
    #define RAW_Y_POSITION(POS)       RAW_POSITION(POS, Y_AXIS)
    #define RAW_Z_POSITION(POS)       RAW_POSITION(POS, Z_AXIS)
  #else
    #define LOGICAL_X_POSITION(POS)   (POS)
    #define LOGICAL_Y_POSITION(POS)   (POS)
    #define LOGICAL_Z_POSITION(POS)   (POS)
    #define RAW_X_POSITION(POS)       (POS)
    #define RAW_Y_POSITION(POS)       (POS)
    #define RAW_Z_POSITION(POS)       (POS)
  #endif

  #define RAW_CURRENT_POSITION(A)     RAW_##A##_POSITION(Mechanics.current_position[A##_AXIS])

#endif // MECH(DELTA)

#endif // _DELTA_MECHANICS_H_
