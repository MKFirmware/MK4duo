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
 * cartesian_mechanics.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#pragma once

// Struct Cartesian Settings
typedef struct : public generic_data_t {
  xyz_limit_float_t base_pos;
} mechanics_data_t;

class Cartesian_Mechanics : public Mechanics {

  public: /** Constructor */

    Cartesian_Mechanics() {}

  public: /** Public Parameters */

    static mechanics_data_t data;

    #if ENABLED(DUAL_X_CARRIAGE)
      static DualXModeEnum  dual_x_carriage_mode;
      static float          inactive_extruder_x_pos,        // used in mode 0 & 1
                            duplicate_extruder_x_offset;    // used in mode 2 & 3
      static xyz_pos_t      raised_parked_position;         // used in mode 1
      static int16_t        duplicate_extruder_temp_offset; // used in mode 2 & 3
      static bool           active_extruder_parked,         // used in mode 1, 2 & 3
                            extruder_duplication_enabled,   // used in mode 2
                            mirrored_duplication_mode;      // used in mode 3
      static short_timer_t  delayed_move_timer;             // used in mode 1
    #endif

  public: /** Public Function */

    /**
     * Initialize Factory parameters
     */
    static void factory_parameters();

    /**
     * Get the stepper positions in the cartesian_position[] array.
     *
     * The result is in the current coordinate space with
     * leveling applied. The coordinates need to be run through
     * unapply_leveling to obtain the "ideal" coordinates
     * suitable for position.x, etc.
     */
    static void get_cartesian_from_steppers();

    /**
     * Move the planner to the current position from wherever it last moved
     * (or from wherever it has been told it is located).
     */
    static void internal_move_to_destination(const feedrate_t &fr_mm_s=0.0f);
    static inline void prepare_internal_move_to_destination(const feedrate_t &fr_mm_s=0.0f) {
      internal_move_to_destination(fr_mm_s);
    }

    /**
     *  Plan a move to (X, Y, Z) and set the position
     */
    static void do_blocking_move_to(const float rx, const float ry, const float rz, const feedrate_t &fr_mm_s=0.0f);
    static void do_blocking_move_to(const xy_pos_t &raw, const feedrate_t &fr_mm_s=0.0f);
    static void do_blocking_move_to(const xyz_pos_t &raw, const feedrate_t &fr_mm_s=0.0f);
    static void do_blocking_move_to(const xyze_pos_t &raw, const feedrate_t &fr_mm_s=0.0f);

    static void do_blocking_move_to_x(const float &rx, const feedrate_t &fr_mm_s=0.0f);
    static void do_blocking_move_to_y(const float &ry, const feedrate_t &fr_mm_s=0.0f);
    static void do_blocking_move_to_z(const float &rz, const feedrate_t &fr_mm_s=0.0f);

    static void do_blocking_move_to_xy(const float &rx, const float &ry, const feedrate_t &fr_mm_s=0.0f);
    static void do_blocking_move_to_xy(const xy_pos_t &raw, const feedrate_t &fr_mm_s=0.0f);
    FORCE_INLINE static void do_blocking_move_to_xy(const xyz_pos_t &raw, const feedrate_t &fr_mm_s=0.0f) {
      do_blocking_move_to(raw.x, raw.y, position.z, fr_mm_s);
    }
    FORCE_INLINE static void do_blocking_move_to_xy(const xyze_pos_t &raw, const feedrate_t &fr_mm_s=0.0f) {
      do_blocking_move_to(raw.x, raw.y, position.z, fr_mm_s);
    }

    static void do_blocking_move_to_xy_z(const xy_pos_t &raw, const float &z, const feedrate_t &fr_mm_s=0.0f);
    FORCE_INLINE static void do_blocking_move_to_xy_z(const xyz_pos_t &raw, const float &z, const feedrate_t &fr_mm_s=0.0f) {
      do_blocking_move_to_xy_z(xy_pos_t(raw), z, fr_mm_s);
    }
    FORCE_INLINE static void do_blocking_move_to_xy_z(const xyze_pos_t &raw, const float &z, const feedrate_t &fr_mm_s=0.0f) {
      do_blocking_move_to_xy_z(xy_pos_t(raw), z, fr_mm_s);
    }

    /**
     * Home all axes according to settings
     */
    static void home(uint8_t axis_bits=0);

    /**
     * Home an individual linear axis
     */
    static void do_homing_move(const AxisEnum axis, const float distance, const feedrate_t fr_mm_s=0.0f);

    /**
     * Prepare a linear move in a Cartesian setup.
     *
     * When a mesh-based leveling system is active, moves are segmented
     * according to the configuration of the leveling system.
     *
     * Returns true if position[] was set to destination[]
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
     * Return Home position
     */
    static float axis_home_pos(const AxisEnum axis);
    static float x_home_pos(const uint8_t extruder=0);
    static float y_home_pos();
    static float z_home_pos();

    /**
     * Check position is reachable
     */
    static bool position_is_reachable(const float &rx, const float &ry);
    static inline bool position_is_reachable(const xy_pos_t &pos) { return position_is_reachable(pos.x, pos.y); }

    /**
     * Return whether the given position is within the bed, and whether the nozzle
     * can reach the position required to put the probe at the given position.
     *
     * Example: For a probe offset of -10,+10, then for the probe to reach 0,0 the
     *          nozzle must be be able to reach +10,-10.
     */
    static bool position_is_reachable_by_probe(const float &rx, const float &ry);
    static inline bool position_is_reachable_by_probe(const xy_pos_t &pos) { return position_is_reachable_by_probe(pos.x, pos.y); }

    /**
     * Report position to host
     */
    static void report_real_position();
    static void report_detail_position();
    static void report_logical_position();
    static inline void report_position() { report_some_position(position.asLogical()); }

    /**
     * Prepare a linear move in a dual X axis setup
     */
    #if ENABLED(DUAL_X_CARRIAGE)
      static bool dual_x_carriage_unpark();
      FORCE_INLINE static bool dxc_is_duplicating() { return dual_x_carriage_mode == DXC_DUPLICATION_MODE; }
      FORCE_INLINE static int x_home_dir(const uint8_t extruder) { return extruder ? X2_HOME_DIR : X_HOME_DIR; }
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
      static void print_M206();
      static void print_M228();
    #endif

    #if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)
      static void nextion_gfx_clear();
    #endif

  private: /** Private Function */

    /**
     *  Home axis
     */
    static void homeaxis(const AxisEnum axis);

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
