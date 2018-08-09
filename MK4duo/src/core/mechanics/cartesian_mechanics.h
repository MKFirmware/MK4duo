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
 * cartesian_mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _CARTESIAN_MECHANICS_H_
#define _CARTESIAN_MECHANICS_H_

#if IS_CARTESIAN

  class Cartesian_Mechanics : public Mechanics {

    public: /** Constructor */

      Cartesian_Mechanics() {}

    public: /** Public Parameters */

      static const float  base_max_pos[XYZ],
                          base_min_pos[XYZ],
                          base_home_pos[XYZ],
                          max_length[XYZ];

      #if ENABLED(DUAL_X_CARRIAGE)
        static DualXMode  dual_x_carriage_mode;
        static float      inactive_hotend_x_pos,            // used in mode 0 & 1
                          raised_parked_position[NUM_AXIS], // used in mode 1
                          duplicate_hotend_x_offset;        // used in mode 2
        static int16_t    duplicate_hotend_temp_offset;     // used in mode 2
        static millis_t   delayed_move_time;                // used in mode 1
        static bool       active_hotend_parked,             // used in mode 1 & 2
                          hotend_duplication_enabled;       // used in mode 2
      #endif

    public: /** Public Function */

      /**
       * Initialize Factory parameters
       */
      static void factory_parameters();

      /**
       * sync_plan_position_mech_specific
       *
       * Set the planner/stepper positions directly from current_position with
       * no kinematic translation. Used for homing axes and cartesian/core syncing.
       */
      static void sync_plan_position_mech_specific();

      /**
       * Home all axes according to settings
       */
      static void home(const bool homeX=false, const bool homeY=false, const bool homeZ=false);

      /**
       * Prepare a linear move in a Cartesian setup.
       *
       * When a mesh-based leveling system is active, moves are segmented
       * according to the configuration of the leveling system.
       *
       * Returns true if current_position[] was set to destination[]
       */
      static bool prepare_move_to_destination_mech_specific();

      /**
       * Set an axis' current position to its home position (after homing).
       *
       * For Cartesian robots this applies one-to-one when an
       * individual axis has been homed.
       *
       * Callers must sync the planner position after calling this!
       */
      static void set_axis_is_at_home(const AxisEnum axis);

      /**
       * Prepare a linear move in a dual X axis setup
       */
      #if ENABLED(DUAL_X_CARRIAGE)
        static float  x_home_pos(const int extruder);
        static bool   dual_x_carriage_unpark();
        FORCE_INLINE static int x_home_dir(const uint8_t extruder) { return extruder ? X2_HOME_DIR : X_HOME_DIR; }
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
       * Prepare a linear move in a Cartesian setup.
       * Bed Leveling will be applied to the move if enabled.
       *
       * Returns true if current_position[] was set to destination[]
       */
      static bool prepare_move_to_destination_cartesian();

      #if ENABLED(QUICK_HOME)
        static void quick_home_xy();
      #endif

      #if ENABLED(Z_SAFE_HOMING)
        static void home_z_safely();
      #endif

      #if ENABLED(DOUBLE_Z_HOMING)
        static void double_home_z();
      #endif

  };

  extern Cartesian_Mechanics mechanics;

#endif // IS_CARTESIAN

#endif /* _CARTESIAN_MECHANICS_H_ */
