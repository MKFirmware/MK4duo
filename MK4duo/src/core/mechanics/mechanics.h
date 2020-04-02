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
#pragma once

/**
 * mechanics.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

union home_flag_t {
  bool all;
  struct {
    bool  XHomed  : 1;
    bool  YHomed  : 1;
    bool  ZHomed  : 1;
    bool  bit3    : 1;
    bool  bit4    : 1;
    bool  bit5    : 1;
    bool  bit6    : 1;
    bool  bit7    : 1;
  };
  home_flag_t() { all = false; }
};

union dir_flag_t {
  int8_t dir;
  struct {
    int8_t x : 2;
    int8_t y : 2;
    int8_t z : 2;
    int8_t e : 2;
  };
  dir_flag_t(const int8_t dirx=0, const int8_t diry=0, const int8_t dirz=0) { x = dirx; y = diry; z = dirz; e = -1; }
};

union sensorless_flag_t {
  bool all;
  struct {
    bool x    : 1;
    bool y    : 1;
    bool z    : 1;
    bool x2   : 1;
    bool y2   : 1;
    bool z2   : 1;
    bool z3   : 1;
    bool bit7 : 1;
  };
  sensorless_flag_t() { all = false; }
};

// Struct Mechanics data
struct generic_data_t {

  xyz_float_t   axis_steps_per_mm,
                max_feedrate_mm_s;

  float         acceleration,
                travel_acceleration,
                min_feedrate_mm_s,
                min_travel_feedrate_mm_s;

  xyz_ulong_t   max_acceleration_mm_per_s2;

  uint32_t      min_segment_time_us;

  #if ENABLED(JUNCTION_DEVIATION)
    float     junction_deviation_mm;
  #endif

  #if HAS_CLASSIC_JERK
    xyz_float_t max_jerk;
  #endif

  #if ENABLED(WORKSPACE_OFFSETS)
    xyz_pos_t home_offset;
  #endif

};

class Mechanics {

  public: /** Constructor */

    Mechanics() {}

  public: /** Public Parameters */

    /**
     * Settings data
     */
    static generic_data_t     data;

    /**
     * Home flag
     */
    static home_flag_t        home_flag;

    /**
     * Home direction
     */
    static const dir_flag_t   home_dir;

    /**
     * Homing feed rates
     */
    static const xyz_float_t  homing_feedrate_mm_s;

    /**
     * Home bump in mm
     */
    static const xyz_float_t  home_bump_mm;

    /**
     * Feedrate
     */
    static feedrate_t         feedrate_mm_s;
    static int16_t            feedrate_percentage;

    /**
     * Step
     */
    static xyz_float_t        steps_to_mm;

    /**
     * Acceleration
     */
    static xyz_ulong_t        max_acceleration_steps_per_s2;

    /**
     * Current Position
     *   Used to track the native machine position as moves are queued.
     *   Used by 'line_to_position' to do a move after changing it.
     *   Used by 'sync_plan_position' to update 'planner.position'.
     */
    static xyze_pos_t position;

    /**
     * Stored Position
     *   Used to save native machine position as moves are queued.
     *   Used by G60 for stored.
     *   Used by G61 for move to.
     */
    static xyze_pos_t stored_position[NUM_POSITON_SLOTS];

    /**
     * Cartesian position
     */
    static xyz_pos_t cartesian_position;

    /**
     * Destination
     *   The destination for a move, filled in by G-code movement commands,
     *   and expected by functions like 'prepare_move_to_destination'.
     *   Set with 'get_destination' or 'set_destination_to_current'.
     */
    static xyze_pos_t destination;

    /**
     * Relative mode for axis
     */
    static uint8_t axis_relative_modes;

    /**
     * Workspace Offset
     */
    #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
      // The distance that XYZ has been offset by G92. Reset by G28.
      static xyz_pos_t position_shift;

      // The above two are combined to save on computes
      static xyz_pos_t workspace_offset;
    #endif

    #if ENABLED(CNC_WORKSPACE_PLANES)
      /**
       * Workspace planes only apply to G2/G3 moves
       * (and "canned cycles" - not a current feature)
       */
      static WorkspacePlaneEnum workspace_plane;
    #endif

  private: /** Private Parameters */

    static feedrate_t saved_feedrate_mm_s;
    static int16_t saved_feedrate_percentage;

  public: /** Public Function */

    /**
     * Get homedir for axis
     */
    static int8_t get_homedir(const AxisEnum axis);

    FORCE_INLINE static void setAxisHomed(const AxisEnum axis, const bool onoff) {
      switch (axis) {
        case X_AXIS: home_flag.XHomed = onoff; break;
        case Y_AXIS: home_flag.YHomed = onoff; break;
        case Z_AXIS: home_flag.ZHomed = onoff; break;
        default: break;
      }
    }
    FORCE_INLINE static bool isAxisHomed(const AxisEnum axis) {
      switch (axis) {
        case X_AXIS:  return home_flag.XHomed;  break;
        case Y_AXIS:  return home_flag.YHomed;  break;
        case Z_AXIS:  return home_flag.ZHomed;  break;
        default:      return false;             break;
      }
    }

    FORCE_INLINE static void unsetHomedAll() { home_flag.all = false; }
    FORCE_INLINE static bool isHomedAll() { return home_flag.XHomed && home_flag.YHomed && home_flag.ZHomed; }

    /**
     * Set the position for an axis based on
     * the stepper positions, removing any leveling that
     * may have been applied.
     *
     * To prevent small shifts in axis position always call
     * sync_plan_position_mech_specific after updating axes with this.
     *
     * To keep hosts in sync, always call report_position
     * after updating the position
     */
    static void set_position_from_steppers_for_axis(const AxisEnum axis);

    /**
     * Move the planner to the current position from wherever it last moved
     * (or from wherever it has been told it is located).
     */
    static void line_to_position(const feedrate_t &fr_mm_s=feedrate_mm_s);

    /**
     * Move the planner to the destination from wherever it last moved
     * (or from wherever it has been told it is located).
     */
    static void line_to_destination(const feedrate_t &fr_mm_s=feedrate_mm_s);

    /**
     * Prepare a single move and get ready for the next one
     *
     * This may result in several calls to planner.buffer_line to
     * do smaller moves for DELTA, SCARA, mesh moves, etc.
     */
    static void prepare_move_to_destination();

    /**
     * Prepare to do endstop or probe moves with custom feedrates.
     *  - Save / restore current feedrate and multiplier
     */
    static void setup_for_endstop_or_probe_move();
    static void clean_up_after_endstop_or_probe_move();

    /**
     * Compute a Bézier curve using the De Casteljau's algorithm (see
     * https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm), which is
     * easy to code and has good numerical stability (very important,
     * since Arudino works with limited precision real numbers).
     */
    #if ENABLED(G5_BEZIER)
      static void plan_cubic_move(const float offset[4]);
    #endif

    /**
     * sync_plan_position
     *
     * Set the planner/stepper positions directly from position with
     * no kinematic translation. Used for homing axes and cartesian/core syncing.
     */
    static void sync_plan_position();
    static void sync_plan_position_e();

    static void unscaled_e_move(const float &length, const feedrate_t &fr_mm_s);

    /**
     * Report position to host
     */
    static void report_some_position(const xyze_pos_t &pos);
    FORCE_INLINE static void report_xyz(const xyz_pos_t &pos) { xyze_pos_t abce = pos; report_xyze(abce, 3); }
    FORCE_INLINE static void report_xyz(const xyze_pos_t &pos) { report_xyze(pos, 3); }

    static uint8_t axis_need_homing(uint8_t axis_bits=0x07);
    static bool axis_unhomed_error(uint8_t axis_bits=0x07);

    /**
     * Axis relative function
     */
    FORCE_INLINE static bool axis_is_relative(const AxisEnum axis) {
      return TEST(axis_relative_modes, axis);
    }
    FORCE_INLINE static void set_relative_mode(const bool rel) {
      axis_relative_modes = rel ? _BV(X_AXIS) | _BV(Y_AXIS) | _BV(Z_AXIS) | _BV(E_AXIS) : 0;
    }
    FORCE_INLINE static void set_e_relative() {
      SBI(axis_relative_modes, E_AXIS);
    }
    FORCE_INLINE static void set_e_absolute() {
      CBI(axis_relative_modes, E_AXIS);
    }

    #if ENABLED(WORKSPACE_OFFSETS)
      /**
       * Change the home offset for an axis, update the current
       * position and the software endstops to retain the same
       * relative distance to the new home.
       *
       * Since this changes the position.x, code should
       * call sync_plan_position soon after this.
       */
      static void update_workspace_offset(const AxisEnum axis);
      static void set_home_offset(const AxisEnum axis, const float v);
    #endif

    #if HAS_LINEAR_E_JERK
      static void recalculate_max_e_jerk();
    #endif

  protected: /** Protected Function */

    /**
     * Set sensorless homing if the axis has it.
     */
    #if ENABLED(SENSORLESS_HOMING)
      static sensorless_flag_t start_sensorless_homing_per_axis(const AxisEnum axis);
      static void stop_sensorless_homing_per_axis(const AxisEnum axis, sensorless_flag_t enable_stealth);
    #endif

    static void report_xyze(const xyze_pos_t &pos, const uint8_t n=4);

    /**
     * Homing bump feedrate (mm/s)
     */
    static feedrate_t get_homing_bump_feedrate(const AxisEnum axis);

};

#if MECH(CARTESIAN)
  #include "cartesian_mechanics.h"
#elif IS_CORE
  #include "core_mechanics.h"
#elif MECH(DELTA)
  #include "delta_mechanics.h"
#elif IS_SCARA
  #include "scara_mechanics.h"
#endif
