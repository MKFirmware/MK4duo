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
 * core_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _CORE_MECHANICS_H_
#define _CORE_MECHANICS_H_

#if IS_CORE

  class Core_Mechanics {

    public: /** Constructor */

      Core_Mechanics() {};

    public: /** Public Parameters */

      /**
       * Cartesian Current Position
       *   Used to track the logical position as moves are queued.
       *   Used by 'line_to_current_position' to do a move after changing it.
       *   Used by 'sync_plan_position' to update 'planner.position'.
       */
      float current_position[XYZE];

      /**
       * Cartesian Stored Position
       *   Used to save logical position as moves are queued.
       *   Used by G60 for stored.
       *   Used by G61 for move to.
       */
      float stored_position[NUM_POSITON_SLOTS][XYZE];

      /**
       * Cartesian Destination
       *   A temporary position, usually applied to 'current_position'.
       *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
       *   'line_to_destination' sets 'current_position' to 'destination'.
       */
      float destination[XYZE];

      /**
       * axis_homed
       *   Flags that each linear axis was homed.
       *   XYZ on cartesian.
       *
       * axis_known_position
       *   Flags that the position is known in each linear axis. Set when homed.
       *   Cleared whenever a stepper powers off, potentially losing its position.
       */
      bool axis_homed[XYZ], axis_known_position[XYZ];

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
      const float homing_feedrate_mm_s[XYZ] = { MMM_TO_MMS(HOMING_FEEDRATE_X), MMM_TO_MMS(HOMING_FEEDRATE_Y), MMM_TO_MMS(HOMING_FEEDRATE_Z) },
                  base_max_pos[XYZ]         = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
                  base_min_pos[XYZ]         = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
                  base_home_pos[XYZ]        = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
                  max_length[XYZ]           = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH },
                  home_bump_mm[XYZ]         = { X_HOME_BUMP_MM, Y_HOME_BUMP_MM, Z_HOME_BUMP_MM };

      const signed char home_dir[XYZ]       = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };

    public: /** Public Function */

      /**
       * Initialize Cartesian parameters
       */
      void Init();

      void set_current_to_destination() { COPY_ARRAY(current_position, destination); }
      void set_destination_to_current() { COPY_ARRAY(destination, current_position); }

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
       * Prepare a single move and get ready for the next one
       * If Mesh Bed Leveling is enabled, perform a mesh move.
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
       * Set the planner/stepper positions directly from current_position with
       * no kinematic translation. Used for homing axes and cartesian/core syncing.
       */
      void sync_plan_position();
      void sync_plan_position_e();

      /**
       * Home Cartesian
       */
      void Home(const bool always_home_all);

      /**
       * Set an axis' current position to its home position (after homing).
       *
       * For Cartesian robots this applies one-to-one when an
       * individual axis has been homed.
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
        #if HAS_ABL
          void print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz);
        #endif
      #endif

    private: /** Private Parameters */

    private: /** Private Function */

      /**
       * Home an individual linear axis
       */
      void do_homing_move(AxisEnum axis, const float distance, float fr_mm_s=0.0);

      /**
       *  Home axis
       */
      void homeaxis(const AxisEnum axis);

      #if ENABLED(QUICK_HOME)
        void quick_home_xy();
      #endif

      #if ENABLED(Z_SAFE_HOMING)
        void home_z_safely();
      #endif

      #if ENABLED(DOUBLE_Z_HOMING)
        void double_home_z();
      #endif

      /**
       * Some planner shorthand inline functions
       */
      float get_homing_bump_feedrate(AxisEnum axis);

  };

  extern Core_Mechanics Mechanics;

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

#endif // MECH(CARTESIAN)

#endif // _CORE_MECHANICS_H_
