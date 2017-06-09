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
 * core_mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"

#if IS_CORE

  Core_Mechanics Mechanics;

  void Core_Mechanics::Init() { }

  /**
   * Prepare a single move and get ready for the next one
   * If Mesh Bed Leveling is enabled, perform a mesh move.
   */
  void Core_Mechanics::prepare_move_to_destination() {

    endstops.clamp_to_software_endstops(destination);
    refresh_cmd_timeout();

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      if (!DEBUGGING(DRYRUN)) {
        if (destination[E_AXIS] != current_position[E_AXIS]) {
          if (thermalManager.tooColdToExtrude(active_extruder))
            current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
          #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
            if (labs(destination[E_AXIS] - current_position[E_AXIS]) > EXTRUDE_MAXLENGTH) {
              current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
              SERIAL_LM(ER, MSG_ERR_LONG_EXTRUDE_STOP);
            }
          #endif
        }
      }
    #endif

    #if ENABLED(DUAL_X_CARRIAGE)

      if (prepare_move_to_destination_dualx()) return;

    #else

      #if ENABLED(LASERBEAM) && ENABLED(LASER_FIRE_E)
        if (current_position[E_AXIS] != destination[E_AXIS] && ((current_position[X_AXIS] != destination [X_AXIS]) || (current_position[Y_AXIS] != destination [Y_AXIS]))){
          laser.status = LASER_ON;
          laser.fired = LASER_FIRE_E;
        }
        if (current_position[E_AXIS] == destination[E_AXIS] && laser.fired == LASER_FIRE_E)
          laser.status = LASER_OFF;
      #endif

      // Do not use feedrate_percentage for E or Z only moves
      if (destination[X_AXIS] == current_position[X_AXIS] && destination[Y_AXIS] == current_position[Y_AXIS])
        line_to_destination();
      else {
        const float fr_scaled = MMS_SCALED(feedrate_mm_s);
        #if ENABLED(MESH_BED_LEVELING)
          if (mbl.active()) { // direct used of mbl.active() for speed
            mesh_line_to_destination(fr_scaled);
            return;
          }
          else
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          if (planner.abl_enabled) { // direct use of abl_enabled for speed
            bilinear_line_to_destination(fr_scaled);
            return;
          }
          else
        #endif
            line_to_destination(fr_scaled);
      }

    #endif

    set_current_to_destination();
  }

  #if ENABLED(DUAL_X_CARRIAGE)

    /**
     * Prepare a linear move in a dual X axis setup
     */
    bool Core_Mechanics::prepare_move_to_destination_dualx() {
      if (active_hotend_parked) {
        switch (dual_x_carriage_mode) {
          case DXC_FULL_CONTROL_MODE:
            break;
          case DXC_AUTO_PARK_MODE:
            if (current_position[E_AXIS] == destination[E_AXIS]) {
              // This is a travel move (with no extrusion)
              // Skip it, but keep track of the current position
              // (so it can be used as the start of the next non-travel move)
              if (delayed_move_time != 0xFFFFFFFFUL) {
                set_current_to_destination();
                NOLESS(raised_parked_position[Z_AXIS], destination[Z_AXIS]);
                delayed_move_time = millis();
                return true;
              }
            }
            // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
            for (uint8_t i = 0; i < 3; i++)
              planner.buffer_line(
                i == 0 ? raised_parked_position[X_AXIS] : current_position[X_AXIS],
                i == 0 ? raised_parked_position[Y_AXIS] : current_position[Y_AXIS],
                i == 2 ? current_position[Z_AXIS] : raised_parked_position[Z_AXIS],
                current_position[E_AXIS],
                i == 1 ? PLANNER_XY_FEEDRATE() : planner.max_feedrate_mm_s[Z_AXIS],
                active_extruder,
                active_driver
              );
            delayed_move_time = 0;
            active_extruder_parked = false;
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) SERIAL_EM("Clear active_extruder_parked");
            #endif
            break;
          case DXC_DUPLICATION_MODE:
            if (active_extruder == 0) {
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) {
                  SERIAL_MV("Set planner X", LOGICAL_X_POSITION(inactive_extruder_x_pos));
                  SERIAL_EMV(" ... Line to X", current_position[X_AXIS] + duplicate_extruder_x_offset);
                }
              #endif
              // move duplicate extruder into correct duplication position.
              planner.set_position_mm(
                LOGICAL_X_POSITION(inactive_extruder_x_pos),
                current_position[Y_AXIS],
                current_position[Z_AXIS],
                current_position[E_AXIS]
              );
              planner.buffer_line(
                current_position[X_AXIS] + duplicate_extruder_x_offset,
                current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],
                planner.max_feedrate_mm_s[X_AXIS], 1, active_driver
              );
              sync_plan_position();
              stepper.synchronize();
              hotend_duplication_enabled = true;
              active_extruder_parked = false;
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) SERIAL_EM("Set hotend_duplication_enabled\nClear active_extruder_parked");
              #endif
            }
            else {
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) SERIAL_EM("Active extruder not 0");
              #endif
            }
            break;
        }
      }
      return false;
    }

  #endif

  /**
   * line_to_current_position
   * Move the planner to the current position from wherever it last moved
   * (or from wherever it has been told it is located).
   */
  void Core_Mechanics::line_to_current_position() {
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder, active_driver);
  }

  /**
   * line_to_destination
   * Move the planner to the position stored in the destination array, which is
   * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
   */
  void Core_Mechanics::line_to_destination(float fr_mm_s) {
    planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder, active_driver);
  }
  void Core_Mechanics::line_to_destination() { line_to_destination(feedrate_mm_s); }

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  void Core_Mechanics::do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s /*=0.0*/) {
    const float old_feedrate_mm_s = feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, lx, ly, lz);
    #endif

    // If Z needs to raise, do it before moving XY
    if (current_position[Z_AXIS] < lz) {
      feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];
      current_position[Z_AXIS] = lz;
      line_to_current_position();
    }

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    current_position[X_AXIS] = lx;
    current_position[Y_AXIS] = ly;
    line_to_current_position();

    // If Z needs to lower, do it after moving XY
    if (current_position[Z_AXIS] > lz) {
      feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];
      current_position[Z_AXIS] = lz;
      line_to_current_position();
    }

    stepper.synchronize();

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< do_blocking_move_to");
    #endif
  }
  void Core_Mechanics::do_blocking_move_to_x(const float &lx, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(lx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
  }
  void Core_Mechanics::do_blocking_move_to_z(const float &lz, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], lz, fr_mm_s);
  }
  void Core_Mechanics::do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(lx, ly, current_position[Z_AXIS], fr_mm_s);
  }

  void Core_Mechanics::sync_plan_position() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position_kinematic", current_position);
    #endif
    planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
  }

  void Core_Mechanics::sync_plan_position_e() {
    planner.set_e_position_mm(current_position[E_AXIS]);
  }

  /**
   * Home an individual linear axis
   */
  void Core_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
        SERIAL_MV(", ", distance);
        SERIAL_MV(", ", fr_mm_s);
        SERIAL_C(')'); SERIAL_E;
      }
    #endif

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      const bool deploy_bltouch = (axis == Z_AXIS && distance < 0);
      if (deploy_bltouch) set_bltouch_deployed(true);
    #endif

    // Tell the planner we're at Z=0
    current_position[axis] = 0;

    sync_plan_position();
    current_position[axis] = distance;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], active_extruder, active_driver);

    stepper.synchronize();

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      if (deploy_bltouch) set_bltouch_deployed(false);
    #endif

    endstops.hit_on_purpose();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
        SERIAL_C(')'); SERIAL_E;
      }
    #endif
  }

  void Core_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_C(')'); SERIAL_E;
      }
    #endif

    const int axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        (axis == X_AXIS) ? x_home_dir(active_extruder) :
      #endif
      home_dir[axis];

    // Homing Z towards the bed? Deploy the Z probe or endstop.
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && DEPLOY_PROBE()) return;
    #endif

    // Set a flag for Z motor locking
    #if ENABLED(Z_TWO_ENDSTOPS)
      if (axis == Z_AXIS) stepper.set_homing_flag(true);
    #endif

    // Fast move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("Home 1 Fast:");
    #endif

    // Fast move towards endstop until triggered
    do_homing_move(axis, 1.5 * max_length[axis] * axis_home_dir);

    // When homing Z with probe respect probe clearance
    const float bump = axis_home_dir * (
      #if HOMING_Z_WITH_PROBE
        (axis == Z_AXIS) ? max(Z_PROBE_BETWEEN_HEIGHT, home_bump_mm[Z_AXIS]) :
      #endif
      home_bump_mm[axis]
    );

    // If a second homing move is configured...
    if (bump) {
      // Move away from the endstop by the axis HOME_BUMP_MM
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("Move Away:");
      #endif
      do_homing_move(axis, -bump);

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("Home 2 Slow:");
      #endif
      do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
    }

    #if ENABLED(Z_TWO_ENDSTOPS)
      if (axis == Z_AXIS) {
        float adj = FABS(endstops.z2_endstop_adj);
        bool lockZ1;
        if (axis_home_dir > 0) {
          adj = -adj;
          lockZ1 = (endstops.z2_endstop_adj > 0);
        }
        else
          lockZ1 = (endstops.z2_endstop_adj < 0);

        if (lockZ1) stepper.set_z_lock(true); else stepper.set_z2_lock(true);

        // Move to the adjusted endstop height
        do_homing_move(axis, adj);

        if (lockZ1) stepper.set_z_lock(false); else stepper.set_z2_lock(false);
        stepper.set_homing_flag(false);
      } // Z_AXIS
    #endif

    // For cartesian machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];

    // Put away the Z probe
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && STOW_PROBE()) return;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_C(')'); SERIAL_E;
      }
    #endif
  }

  #if ENABLED(QUICK_HOME)

    void Core_Mechanics::quick_home_xy() {

      // Pretend the current position is 0,0
      current_position[X_AXIS] = current_position[Y_AXIS] = 0;
      sync_plan_position();

      #if ENABLED(DUAL_X_CARRIAGE)
        const int x_axis_home_dir = x_home_dir(active_extruder);
      #else
        const int x_axis_home_dir = home_dir[X_AXIS];
      #endif

      const float mlx = max_length[X_AXIS],
                  mly = max_length[Y_AXIS],
                  mlratio = mlx > mly ? mly / mlx : mlx / mly,
                  fr_mm_s = min(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * SQRT(sq(mlratio) + 1.0);

      do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir[Y_AXIS], fr_mm_s);
      endstops.hit_on_purpose(); // clear endstop hit flags
      current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
    }

  #endif // QUICK_HOME

  #if ENABLED(Z_SAFE_HOMING)

    void Core_Mechanics::home_z_safely() {

      // Disallow Z homing if X or Y are unknown
      if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
        LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
        SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
        return;
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("Z_SAFE_HOMING >>>");
      #endif

      sync_plan_position();

      /**
       * Move the Z probe (or just the nozzle) to the safe homing point
       */
      destination[X_AXIS] = LOGICAL_X_POSITION(Z_SAFE_HOMING_X_POINT);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(Z_SAFE_HOMING_Y_POINT);
      destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

      #if HOMING_Z_WITH_PROBE
        destination[X_AXIS] -= X_PROBE_OFFSET_FROM_NOZZLE;
        destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_NOZZLE;
      #endif

      if (position_is_reachable_xy(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("Z_SAFE_HOMING", destination);
        #endif

        // This causes the carriage on Dual X to unpark
        #if ENABLED(DUAL_X_CARRIAGE)
          active_extruder_parked = false;
        #endif

        do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
        homeaxis(Z_AXIS);
      }
      else {
        LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
        SERIAL_LM(ECHO, MSG_ZPROBE_OUT);
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("<<< Z_SAFE_HOMING");
      #endif
    }

  #endif // Z_SAFE_HOMING

  #if ENABLED(DOUBLE_Z_HOMING)

    void Core_Mechanics::double_home_z() {

      // Disallow Z homing if X or Y are unknown
      if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
        LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
        SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
        return;
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("DOUBLE_Z_HOMING >>>");
      #endif

      sync_plan_position();

      /**
       * Move the Z probe (or just the nozzle) to the safe homing point
       */
      destination[X_AXIS] = LOGICAL_X_POSITION(DOUBLE_Z_HOMING_X_POINT);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(DOUBLE_Z_HOMING_Y_POINT);
      destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

      #if HAS_BED_PROBE
        destination[X_AXIS] -= X_PROBE_OFFSET_FROM_NOZZLE;
        destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_NOZZLE;
      #endif

      if (position_is_reachable_xy(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("DOUBLE_Z_HOMING", destination);
        #endif

        const float newzero = probe_pt(destination[X_AXIS], destination[Y_AXIS], true, 1) - (2 * zprobe_zoffset);
        current_position[Z_AXIS] -= newzero;
        destination[Z_AXIS] = current_position[Z_AXIS];
        endstops.soft_endstop_max[Z_AXIS] = base_max_pos(Z_AXIS) - newzero;

        sync_plan_position();
        do_blocking_move_to_z(MIN_Z_HEIGHT_FOR_HOMING);
      }
      else {
        LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
        SERIAL_LM(ECHO, MSG_ZPROBE_OUT);
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("<<< DOUBLE_Z_HOMING");
      #endif
    }

  #endif

  /**
   * Home Cartesian
   */
  void Core_Mechanics::Home(const bool always_home_all) {

    #if ENABLED(FORCE_HOME_XY_BEFORE_Z)
      const bool  homeZ = always_home_all || parser.seen('Z'),
                  homeX = always_home_all || homeZ || parser.seen('X'),
                  homeY = always_home_all || homeZ || parser.seen('Y'),
                  homeE = always_home_all || parser.seen('E');
    #else
      const bool  homeX = always_home_all || parser.seen('X'),
                  homeY = always_home_all || parser.seen('Y'),
                  homeZ = always_home_all || parser.seen('Z'),
                  homeE = always_home_all || parser.seen('E');
    #endif
  
    const bool home_all = (!homeX && !homeY && !homeZ && !homeE) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (home_all || homeZ) {
        homeaxis(Z_AXIS);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeaxis(Z_AXIS)", current_position);
        #endif
      }

    #else

      if (home_all || homeX || homeY) {
        // Raise Z before homing any other axes and z is not already high enough (never lower z)
        destination[Z_AXIS] = LOGICAL_Z_POSITION(MIN_Z_HEIGHT_FOR_HOMING);
        if (destination[Z_AXIS] > current_position[Z_AXIS]) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING))
              SERIAL_EMV("Raise Z (before homing) to ", destination[Z_AXIS]);
          #endif
          do_blocking_move_to_z(destination[Z_AXIS]);
        }
      }

    #endif

    #if ENABLED(QUICK_HOME)
      if (home_all || (homeX && homeY)) quick_home_xy();
    #endif

    #if ENABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all || homeY) {
        homeaxis(Y_AXIS);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif

    // Home X
    if (home_all || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        // Always home the 2nd (right) extruder first
        active_extruder = 1;
        homeaxis(X_AXIS);

        // Remember this extruder's position for later tool change
        inactive_hotend_x_pos = RAW_X_POSITION(current_position[X_AXIS]);

        // Home the 1st (left) extruder
        active_extruder = 0;
        homeaxis(X_AXIS);

        // Consider the active extruder to be parked
        COPY_ARRAY(raised_parked_position, current_position);
        delayed_move_time = 0;
        active_hotend_parked = true;
      #else
        homeaxis(X_AXIS);
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> homeX", current_position);
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all || homeY) {
        homeaxis(Y_AXIS);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (home_all || homeZ) {
        #if ENABLED(Z_SAFE_HOMING)
          home_z_safely();
        #else
          homeaxis(Z_AXIS);
        #endif // !Z_SAFE_HOMING
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all || homeZ) > final", current_position);
        #endif
      }
    #elif ENABLED(DOUBLE_Z_HOMING)
      if (home_all || homeZ)
        double_home_z();
    #endif

    sync_plan_position();
  }

  /**
   * Set an axis' current position to its home position (after homing).
   *
   * For Cartesian this applies one-to-one when an
   * individual axis has been homed.
   *
   * Callers must sync the planner position after calling this!
   */
  void Core_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_C(')'); SERIAL_E;
      }
    #endif

    axis_known_position[axis] = axis_homed[axis] = true;

    #if ENABLED(WORKSPACE_OFFSETS)
      position_shift[axis] = 0;
      endstops.update_software_endstops(axis);
    #endif

    #if ENABLED(DUAL_X_CARRIAGE)
      if (axis == X_AXIS && (active_extruder == 1 || dual_x_carriage_mode == DXC_DUPLICATION_MODE)) {
        current_position[X_AXIS] = x_home_pos(active_extruder);
        return;
      }
    #endif

    current_position[axis] = LOGICAL_POSITION(base_home_pos[axis], axis);

    /**
     * Z Probe Z Homing? Account for the probe's Z offset.
     */
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS) {
        current_position[Z_AXIS] -= zprobe_zoffset;

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_EM("*** Z HOMED WITH PROBE ***");
            SERIAL_EMV("zprobe_zoffset = ", zprobe_zoffset);
          }
        #endif
      }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        #if ENABLED(WORKSPACE_OFFSETS)
          SERIAL_MV("> home_offset[", axis_codes[axis]);
          SERIAL_EMV("] = ", home_offset[axis]);
        #endif
        DEBUG_POS("", current_position);
        SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_C(')'); SERIAL_E;
      }
    #endif
  }

  float Core_Mechanics::get_homing_bump_feedrate(const AxisEnum axis) {
    const uint8_t homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
    uint8_t hbd = homing_bump_divisor[axis];
    if (hbd < 1) {
      hbd = 10;
      SERIAL_LM(ER, "Warning: Homing Bump Divisor < 1");
    }
    return homing_feedrate_mm_s[axis] / hbd;
  }

  bool Core_Mechanics::axis_unhomed_error(const bool x/*=true*/, const bool y/*=true*/, const bool z/*=true*/) {
    const bool  xx = x && !axis_homed[X_AXIS],
                yy = y && !axis_homed[Y_AXIS],
                zz = z && !axis_homed[Z_AXIS];

    if (xx || yy || zz) {
      SERIAL_SM(ECHO, MSG_HOME " ");
      if (xx) SERIAL_M(MSG_X);
      if (yy) SERIAL_M(MSG_Y);
      if (zz) SERIAL_M(MSG_Z);
      SERIAL_EM(" " MSG_FIRST);

      #if ENABLED(ULTRA_LCD)
        lcd_status_printf_P(0, PSTR(MSG_HOME " %s%s%s " MSG_FIRST), xx ? MSG_X : "", yy ? MSG_Y : "", zz ? MSG_Z : "");
      #endif
      return true;
    }
    return false;
  }
  bool Core_Mechanics::position_is_reachable_raw_xy(const float &rx, const float &ry) {
    // Add 0.001 margin to deal with float imprecision
    return WITHIN(rx, X_MIN_POS - 0.001, X_MAX_POS + 0.001)
        && WITHIN(ry, Y_MIN_POS - 0.001, Y_MAX_POS + 0.001);
  }
  bool Core_Mechanics::position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) {
    // Add 0.001 margin to deal with float imprecision
    return WITHIN(rx, MIN_PROBE_X - 0.001, MAX_PROBE_X + 0.001)
        && WITHIN(ry, MIN_PROBE_Y - 0.001, MAX_PROBE_Y + 0.001);
  }
  bool Core_Mechanics::position_is_reachable_by_probe_xy(const float &lx, const float &ly) {
    return position_is_reachable_by_probe_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
  }
  bool Core_Mechanics::position_is_reachable_xy(const float &lx, const float &ly) {
    return position_is_reachable_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
  }

  #if ENABLED(DUAL_X_CARRIAGE)

    float Core_Mechanics::x_home_pos(const int extruder) {
      if (extruder == 0)
        return LOGICAL_X_POSITION(base_home_pos[X_AXIS]);
      else
        // In dual carriage mode the extruder offset provides an override of the
        // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
        // This allow soft recalibration of the second extruder offset position without firmware reflash
        // (through the M218 command).
        return LOGICAL_X_POSITION(hotend_offset[X_AXIS][1] > 0 ? hotend_offset[X_AXIS][1] : X2_HOME_POS);
    }

  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)

    void Core_Mechanics::print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
      SERIAL_PS(prefix);
      SERIAL_C('(');
      SERIAL_V(x);
      SERIAL_MV(", ", y);
      SERIAL_MV(", ", z);
      SERIAL_C(")");

      if (suffix) SERIAL_PS(suffix);
      else SERIAL_E;
    }

    void Core_Mechanics::print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
      print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
    }

    #if HAS_ABL
      void Core_Mechanics::print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz) {
        print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
      }
    #endif

  #endif

#endif
