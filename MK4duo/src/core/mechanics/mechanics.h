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
 * mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _MECHANICS_H_
#define _MECHANICS_H_

/**
 * Axis indices as enumerated constants
 *
 * Special axis:
 *  A_AXIS and B_AXIS are used by COREXY or COREYX printers
 *  A_AXIS and C_AXIS are used by COREXZ or COREZX printers
 *  X_HEAD and Y_HEAD and Z_HEAD is used for systems that don't have a 1:1 relationship between X_AXIS and X Head movement, like CoreXY bots.
 *  A_AXIS and B_AXIS and C_AXIS is used for DELTA system.
 */
enum AxisEnum {
  NO_AXIS = -1,
  X_AXIS  = 0,
  A_AXIS  = 0,
  Y_AXIS  = 1,
  B_AXIS  = 1,
  Z_AXIS  = 2,
  C_AXIS  = 2,
  E_AXIS  = 3,
  X_HEAD  = 4,
  Y_HEAD  = 5,
  Z_HEAD  = 6,
  ALL_AXES = 100
};

/**
 * DUAL X CARRIAGE
 */
#if ENABLED(DUAL_X_CARRIAGE)
  enum DualXMode {
    DXC_FULL_CONTROL_MODE,
    DXC_AUTO_PARK_MODE,
    DXC_DUPLICATION_MODE
  };
#endif

#define LOGICAL_X_POSITION(POS) mechanics.native_to_logical(POS, X_AXIS)
#define LOGICAL_Y_POSITION(POS) mechanics.native_to_logical(POS, Y_AXIS)
#define LOGICAL_Z_POSITION(POS) mechanics.native_to_logical(POS, Z_AXIS)
#define NATIVE_X_POSITION(POS)  mechanics.logical_to_native(POS, X_AXIS)
#define NATIVE_Y_POSITION(POS)  mechanics.logical_to_native(POS, Y_AXIS)
#define NATIVE_Z_POSITION(POS)  mechanics.logical_to_native(POS, Z_AXIS)

#if PLANNER_LEVELING || ENABLED(ZWOBBLE) || ENABLED(HYSTERESIS)
  #define ARG_X float rx
  #define ARG_Y float ry
  #define ARG_Z float rz
#else
  #define ARG_X const float &rx
  #define ARG_Y const float &ry
  #define ARG_Z const float &rz
#endif

class Mechanics {

  public: /** Constructor */

    Mechanics() {}

  public: /** Public Parameters */

    /**
     * Feedrate, min, max, travel
     */
    float   feedrate_mm_s             = MMM_TO_MMS(1500.0),
            min_feedrate_mm_s         = 0.0,
            max_feedrate_mm_s[XYZE_N] = { 0.0 },
            min_travel_feedrate_mm_s  = 0.0;
    int16_t feedrate_percentage       = 100;

    /**
     * Homing feed rates are often configured with mm/m
     * but the planner and stepper like mm/s units.
     */
    const float homing_feedrate_mm_s[XYZ] = { MMM_TO_MMS(HOMING_FEEDRATE_X), MMM_TO_MMS(HOMING_FEEDRATE_Y), MMM_TO_MMS(HOMING_FEEDRATE_Z) },
                home_bump_mm[XYZ]         = { X_HOME_BUMP_MM, Y_HOME_BUMP_MM, Z_HOME_BUMP_MM };

    /**
     * Step per unit
     */
    float axis_steps_per_mm[XYZE_N] = { 0.0 },
          steps_to_mm[XYZE_N]       = { 0.0 };

    /**
     * Acceleration and Jerk
     */
    float     acceleration                          = 0.0,
              travel_acceleration                   = 0.0,
              retract_acceleration[EXTRUDERS]       = { 0.0 },
              max_jerk[XYZE_N]                      = { 0.0 };
    uint32_t  max_acceleration_steps_per_s2[XYZE_N] = { 0 },
              max_acceleration_mm_per_s2[XYZE_N]    = { 0 };

    const signed char home_dir[XYZ]       = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };

    /**
     * Min segment time
     */
    millis_t  min_segment_time_us = 0;

    /**
     * Cartesian Current Position
     *   Used to track the native machine position as moves are queued.
     *   Used by 'line_to_current_position' to do a move after changing it.
     *   Used by 'sync_plan_position' to update 'planner.position'.
     */
    float current_position[XYZE] = { 0.0 };

    /**
     * Cartesian Stored Position
     *   Used to save native machine position as moves are queued.
     *   Used by G60 for stored.
     *   Used by G61 for move to.
     */
    float stored_position[NUM_POSITON_SLOTS][XYZE] = {{ 0.0 }};

    /**
     * Cartesian position
     */
    float cartesian_position[XYZ] = { 0.0 };

    /**
     * Cartesian Destination
     *   A temporary position, usually applied to 'current_position'.
     *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
     *   'line_to_destination' sets 'current_position' to 'destination'.
     */
    float destination[XYZE] = { 0.0 };

    /**
     * Workspace Offset
     */
    #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
      // The distance that XYZ has been offset by G92. Reset by G28.
      float position_shift[XYZ] = { 0.0 };

      // This offset is added to the configured home position.
      // Set by M206, M428, or menu item. Saved to EEPROM.
      float home_offset[XYZ] = { 0.0 };

      // The above two are combined to save on computes
      float workspace_offset[XYZ] = { 0.0 };
    #endif

    #if ENABLED(CNC_WORKSPACE_PLANES)
      /**
       * Workspace planes only apply to G2/G3 moves
       * (and "canned cycles" - not a current feature)
       */
      enum WorkspacePlane { PLANE_XY, PLANE_ZX, PLANE_YZ };
      WorkspacePlane workspace_plane = PLANE_XY;
    #endif

    #if ENABLED(BABYSTEPPING)
      int babystepsTodo[XYZ] = { 0 };
    #endif

  public: /** Public Function */

    /**
     * Set the planner.position and individual stepper positions.
     * Used by G92, G28, G29, and other procedures.
     *
     * Multiplies by axis_steps_per_mm[] and does necessary conversion
     *
     * Clears previous speed values.
     */
            void _set_position_mm(const float &a, const float &b, const float &c, const float &e);
            void set_position_mm(const AxisEnum axis, const float &v);
    virtual void set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e);
    virtual void set_position_mm(const float position[NUM_AXIS]);
    FORCE_INLINE void set_z_position_mm(const float &z) { set_position_mm(AxisEnum(Z_AXIS), z); }
    FORCE_INLINE void set_e_position_mm(const float &e) { set_position_mm(AxisEnum(E_AXIS), e); }

    /**
     * Get the stepper positions in the cartes[] array.
     * Forward kinematics are applied for DELTA and SCARA.
     *
     * The result is in the current coordinate space with
     * leveling applied. The coordinates need to be run through
     * unapply_leveling to obtain the "ideal" coordinates
     * suitable for current_position, etc.
     */
    virtual void get_cartesian_from_steppers();

    /**
     * Set the current_position for an axis based on
     * the stepper positions, removing any leveling that
     * may have been applied.
     *
     * To prevent small shifts in axis position always call
     * SYNC_PLAN_POSITION_KINEMATIC after updating axes with this.
     *
     * To keep hosts in sync, always call report_current_position
     * after updating the current_position.
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
    FORCE_INLINE void line_to_destination() { line_to_destination(feedrate_mm_s); }

    /**
     * Prepare a single move and get ready for the next one
     *
     * This may result in several calls to planner.buffer_line to
     * do smaller moves for DELTA, SCARA, mesh moves, etc.
     */
    void prepare_move_to_destination();

    /**
     * Compute a Bézier curve using the De Casteljau's algorithm (see
     * https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm), which is
     * easy to code and has good numerical stability (very important,
     * since Arudino works with limited precision real numbers).
     */
    #if ENABLED(G5_BEZIER)
      void plan_cubic_move(const float offset[4]);
    #endif

    /**
     * Plan a move to (X, Y, Z) and set the current_position
     * The final current_position may not be the one that was requested
     */
    virtual void do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s=0.0);
            void do_blocking_move_to_x(const float &rx, const float &fr_mm_s=0.0);
            void do_blocking_move_to_z(const float &rz, const float &fr_mm_s=0.0);
            void do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s=0.0);

    /**
     * sync_plan_position
     *
     * Set the planner/stepper positions directly from current_position with
     * no kinematic translation. Used for homing axes and cartesian/core syncing.
     */
    void sync_plan_position();
    void sync_plan_position_e();

    /**
     * Recalculate the steps/s^2 acceleration rates, based on the mm/s^2
     */
    void reset_acceleration_rates();

    /**
     * Recalculate position, steps_to_mm if axis_steps_per_mm changes!
     */
    void refresh_positioning();

    /**
     * Home an individual linear axis
     */
    virtual void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0);

    /**
     * Report current position to host
     */
    virtual void report_current_position();
    virtual void report_current_position_detail();

    FORCE_INLINE void report_xyz(const float pos[]) { report_xyze(pos, 3); }

    bool axis_unhomed_error(const bool x=true, const bool y=true, const bool z=true);

    /**
     * position_is_reachable family of functions
     */
    virtual bool position_is_reachable(const float &rx, const float &ry);
    virtual bool position_is_reachable_by_probe(const float &rx, const float &ry);

    /**
     * Plan an arc in 2 dimensions
     *
     * The arc is approximated by generating many small linear segments.
     * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
     * Arcs should only be made relatively large (over 5mm), as larger arcs with
     * larger segments will tend to be more efficient. Your slicer should have
     * options for G2/G3 arc generation. In future these options may be GCode tunable.
     */
    #if ENABLED(ARC_SUPPORT)
      void plan_arc(const float (&cart)[XYZE], const float (&offset)[2], const uint8_t clockwise);
    #endif

    #if ENABLED(WORKSPACE_OFFSETS)
      /**
       * Change the home offset for an axis, update the current
       * position and the software endstops to retain the same
       * relative distance to the new home.
       *
       * Since this changes the current_position, code should
       * call sync_plan_position soon after this.
       */
      void set_home_offset(const AxisEnum axis, const float v);

      float native_to_logical(const float pos, const AxisEnum axis);
      float logical_to_native(const float pos, const AxisEnum axis);
    #else
      FORCE_INLINE float native_to_logical(const float pos, const AxisEnum axis) { UNUSED(axis); return pos; }
      FORCE_INLINE float logical_to_native(const float pos, const AxisEnum axis) { UNUSED(axis); return pos; }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      void log_machine_info();
    #endif

    #if ENABLED(BABYSTEPPING)
      void babystep_axis(const AxisEnum axis, const int distance);
    #endif

    #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
      virtual void Nextion_gfx_clear();
    #endif

    /**
     * Set sensorless homing if the axis has it.
     */
    #if ENABLED(SENSORLESS_HOMING)
      virtual void sensorless_homing_per_axis(const AxisEnum axis, const bool enable=true);
    #endif

  protected: /** Protected Function */

    void report_xyze(const float pos[], const uint8_t n=4, const uint8_t precision=3);

    float get_homing_bump_feedrate(const AxisEnum axis);

};

#if IS_CARTESIAN
  #include "cartesian/cartesian_mechanics.h"
#elif IS_CORE
  #include "core/core_mechanics.h"
#elif IS_DELTA
  #include "delta/delta_mechanics.h"
#elif IS_SCARA
  #include "scara/scara_mechanics.h"
#endif

#endif /* _MECHANICS_H_ */
