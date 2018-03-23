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
 * cartesian_mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../../../MK4duo.h"
#include "cartesian_mechanics.h"

#if IS_CARTESIAN

  Cartesian_Mechanics mechanics;

  void Cartesian_Mechanics::init() { 

    #if ENABLED(HYSTERESIS)
      const float hyst[] = DEFAULT_HYSTERESIS_MM;
      set_hysteresis(hyst[0], hyst[1], hyst[2], hyst[3]);
      m_hysteresis_prev_direction_bits = 0;
    #endif

    #if ENABLED(ZWOBBLE)
      const float wobble[] = DEFAULT_ZWOBBLE;
      m_zwobble_consistent = false;
      zwobble_lastZ = -1.0;
      zwobble_lastZRod = -1.0;
      m_zwobble_scalingFactor = 1.0;
      m_zwobble_sinusoidal = true;
      set_zwobble_amplitude(wobble[0]);
      set_zwobble_period(wobble[1]);
      set_zwobble_phase(wobble[2]);
    #endif

  }

  /**
   * Home Cartesian
   */
  void Cartesian_Mechanics::home(const bool always_home_all) {

    if (printer.debugSimulation()) {
      LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        Nextion_gfx_clear();
      #endif
      return;
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_EM(">>> gcode_G28");
        log_machine_info();
      }
    #endif

    #if HAS_POWER_SWITCH
      if (!powerManager.lastPowerOn) powerManager.power_on(); // Power On if power is off
    #endif

    // Wait for planner moves to finish!
    stepper.synchronize();

    // Cancel the active G29 session
    #if HAS_LEVELING && ENABLED(PROBE_MANUALLY)
      bedlevel.g29_in_progress = false;
      #if HAS_NEXTION_MANUAL_BED
        Nextion_ProbeOff();
      #endif
    #endif

    // Disable the leveling matrix before homing
    #if HAS_LEVELING
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        const bool ubl_state_at_entry = bedlevel.leveling_active;
      #endif
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    #if ENABLED(CNC_WORKSPACE_PLANES)
      workspace_plane = PLANE_XY;
    #endif

    // Always home with tool 0 active
    #if HOTENDS > 1
      const uint8_t old_tool_index = tools.active_extruder;
      tools.change(0, 0, true);
    #endif

    #if ENABLED(DUAL_X_CARRIAGE)
      hotend_duplication_enabled = false;
    #endif

    printer.setup_for_endstop_or_probe_move();
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("> endstops.setEnabled(true)");
    #endif
    endstops.setEnabled(true); // Enable endstops for next homing move

    bool come_back = parser.boolval('B');
    float lastpos[NUM_AXIS];
    float old_feedrate_mm_s;
    if (come_back) {
      old_feedrate_mm_s = feedrate_mm_s;
      COPY_ARRAY(lastpos, current_position);
    }

    #if ENABLED(FORCE_HOME_XY_BEFORE_Z)
      const bool  homeZ = always_home_all || parser.seen('Z'),
                  homeX = always_home_all || homeZ || parser.seen('X'),
                  homeY = always_home_all || homeZ || parser.seen('Y');
    #else
      const bool  homeX = always_home_all || parser.seen('X'),
                  homeY = always_home_all || parser.seen('Y'),
                  homeZ = always_home_all || parser.seen('Z');
    #endif

    const bool home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (home_all || homeZ) {
        homeaxis(Z_AXIS);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) DEBUG_POS("> homeaxis(Z_AXIS)", current_position);
        #endif
      }

    #else

      const float z_homing_height = printer.isZHomed() ? MIN_Z_HEIGHT_FOR_HOMING : 0;

      if (z_homing_height && (home_all || homeX || homeY)) {
        // Raise Z before homing any other axes and z is not already high enough (never lower z)
        destination[Z_AXIS] = z_homing_height;
        if (destination[Z_AXIS] > current_position[Z_AXIS]) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (printer.debugLeveling())
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
          if (printer.debugLeveling()) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif

    // Home X
    if (home_all || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        // Always home the 2nd (right) extruder first
        tools.active_extruder = 1;
        homeaxis(X_AXIS);

        // Remember this extruder's position for later tool change
        inactive_hotend_x_pos = current_position[X_AXIS];

        // Home the 1st (left) extruder
        tools.active_extruder = 0;
        homeaxis(X_AXIS);

        // Consider the active extruder to be parked
        COPY_ARRAY(raised_parked_position, current_position);
        delayed_move_time = 0;
        active_hotend_parked = true;
      #else
        homeaxis(X_AXIS);
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) DEBUG_POS("> homeX", current_position);
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all || homeY) {
        homeaxis(Y_AXIS);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) DEBUG_POS("> homeY", current_position);
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
          if (printer.debugLeveling()) DEBUG_POS("> (home_all || homeZ) > final", current_position);
        #endif
      } // home_all || homeZ
      #if HOMING_Z_WITH_PROBE && Z_PROBE_AFTER_PROBING > 0
        probe.move_z_after_probing();
      #endif
    #elif ENABLED(DOUBLE_Z_HOMING)
      if (home_all || homeZ)
        double_home_z();
    #endif

    sync_plan_position();
    endstops.setNotHoming();

    if (come_back) {
      feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
      COPY_ARRAY(destination, lastpos);
      prepare_move_to_destination();
      feedrate_mm_s = old_feedrate_mm_s;
    }

    #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
      Nextion_gfx_clear();
    #endif

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      bedlevel.set_bed_leveling_enabled(ubl_state_at_entry);
    #endif

    printer.clean_up_after_endstop_or_probe_move();

    stepper.synchronize();

    // Restore the active tool after homing
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif

    lcd_refresh();

    report_current_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("<<< gcode_G28");
    #endif

  }

  /**
   * Prepare a single move and get ready for the next one
   */
  bool Cartesian_Mechanics::prepare_move_to_destination_mech_specific() {
    #if ENABLED(DUAL_X_CARRIAGE)
      if (prepare_move_to_destination_dualx() || prepare_move_to_destination_cartesian()) return true;
    #else
      if (prepare_move_to_destination_cartesian()) return true;
    #endif

    set_current_to_destination();
    return false;
  }

  #if ENABLED(DUAL_X_CARRIAGE)

    /**
     * Prepare a linear move in a dual X axis setup
     */
    bool Cartesian_Mechanics::prepare_move_to_destination_dualx() {
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
                i == 1 ? PLANNER_XY_FEEDRATE() : max_feedrate_mm_s[Z_AXIS],
                tools.active_extruder
              );
            delayed_move_time = 0;
            active_hotend_parked = false;
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (printer.debugLeveling()) SERIAL_EM("Clear active_hotend_parked");
            #endif
            break;
          case DXC_DUPLICATION_MODE:
            if (tools.active_extruder == 0) {
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (printer.debugLeveling()) {
                  SERIAL_MV("Set planner X", inactive_hotend_x_pos);
                  SERIAL_EMV(" ... Line to X", current_position[X_AXIS] + duplicate_hotend_x_offset);
                }
              #endif
              // move duplicate extruder into correct duplication position.
              set_position_mm(
                inactive_hotend_x_pos,
                current_position[Y_AXIS],
                current_position[Z_AXIS],
                current_position[E_AXIS]
              );
              planner.buffer_line(
                current_position[X_AXIS] + duplicate_hotend_x_offset,
                current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],
                max_feedrate_mm_s[X_AXIS], 1
              );
              sync_plan_position();
              stepper.synchronize();
              hotend_duplication_enabled = true;
              active_hotend_parked = false;
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (printer.debugLeveling()) SERIAL_EM("Set hotend_duplication_enabled\nClear active_hotend_parked");
              #endif
            }
            else {
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (printer.debugLeveling()) SERIAL_EM("Active extruder not 0");
              #endif
            }
            break;
        }
      }
      return false;
    }

  #endif

  void Cartesian_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    const int axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        (axis == X_AXIS) ? x_home_dir(tools.active_extruder) :
      #endif
      home_dir[axis];

    // Homing Z towards the bed? Deploy the Z probe or endstop.
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && DEPLOY_PROBE()) return;
    #endif

    // Set flags for X, Y, Z motor locking
    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
      printer.setHoming(true);
    #endif

    // Fast move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) SERIAL_EM("Home 1 Fast:");
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
        if (printer.debugLeveling()) SERIAL_EM("Move Away:");
      #endif
      do_homing_move(axis, -bump);

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("Home 2 Slow:");
      #endif
      do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
    }

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
      const bool pos_dir = axis_home_dir > 0;
      #if ENABLED(X_TWO_ENDSTOPS)
        if (axis == X_AXIS) {
          const bool lock_x1 = pos_dir ? (endstops.x_endstop_adj > 0) : (endstops.x_endstop_adj < 0);
          float adj = FABS(endstops.x_endstop_adj);
          if (pos_dir) adj = -adj;
          if (lock_x1) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
          do_homing_move(axis, adj);
          if (lock_x1) stepper.set_x_lock(false); else stepper.set_x2_lock(false);
          printer.setHoming(false);
        }
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        if (axis == Y_AXIS) {
          const bool lock_y1 = pos_dir ? (endstops.y_endstop_adj > 0) : (endstops.y_endstop_adj < 0);
          float adj = FABS(endstops.y_endstop_adj);
          if (pos_dir) adj = -adj;
          if (lock_y1) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
          do_homing_move(axis, adj);
          if (lock_y1) stepper.set_y_lock(false); else stepper.set_y2_lock(false);
          printer.setHoming(false);
        }
      #endif
      #if ENABLED(Z_TWO_ENDSTOPS)
        if (axis == Z_AXIS) {
          const bool lock_z1 = pos_dir ? (endstops.z_endstop_adj > 0) : (endstops.z_endstop_adj < 0);
          float adj = FABS(endstops.z_endstop_adj);
          if (pos_dir) adj = -adj;
          if (lock_z1) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
          do_homing_move(axis, adj);
          if (lock_z1) stepper.set_z_lock(false); else stepper.set_z2_lock(false);
          printer.setHoming(false);
        }
      #endif
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

    // Clear z_lift if homing the Z axis
    #if ENABLED(FWRETRACT)
      if (axis == Z_AXIS) fwretract.hop_amount = 0.0;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  /**
   * Prepare a linear move in a Cartesian setup.
   * Bed Leveling will be applied to the move if enabled.
   *
   * Returns true if current_position[] was set to destination[]
   */
  bool Cartesian_Mechanics::prepare_move_to_destination_cartesian() {

    #if ENABLED(LASER) && ENABLED(LASER_FIRE_E)
      if (current_position[E_AXIS] < destination[E_AXIS] && ((current_position[X_AXIS] != destination [X_AXIS]) || (current_position[Y_AXIS] != destination [Y_AXIS])))
        laser.status = LASER_ON;
      else
        laser.status = LASER_OFF;
    #endif

    #if HAS_MESH
      if (bedlevel.leveling_active) {
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          ubl.line_to_destination_cartesian(MMS_SCALED(feedrate_mm_s), tools.active_extruder);
          return true;
        #else
          /**
           * For MBL and ABL-BILINEAR only segment moves when X or Y are involved.
           * Otherwise fall through to do a direct single move.
           */
          if (current_position[X_AXIS] != destination[X_AXIS] || current_position[Y_AXIS] != destination[Y_AXIS]) {
            #if ENABLED(MESH_BED_LEVELING)
              mbl.line_to_destination(MMS_SCALED(feedrate_mm_s));
            #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
              abl.bilinear_line_to_destination(MMS_SCALED(feedrate_mm_s));
            #endif
            return true;
          }
        #endif
      }
    #endif // HAS_MESH

    line_to_destination(MMS_SCALED(feedrate_mm_s));
    return false;
  }

  #if ENABLED(QUICK_HOME)

    void Cartesian_Mechanics::quick_home_xy() {

      // Pretend the current position is 0,0
      current_position[X_AXIS] = current_position[Y_AXIS] = 0;
      sync_plan_position();

      #if ENABLED(DUAL_X_CARRIAGE)
        const int x_axis_home_dir = x_home_dir(tools.active_extruder);
      #else
        const int x_axis_home_dir = home_dir[X_AXIS];
      #endif

      const float mlx = max_length[X_AXIS],
                  mly = max_length[Y_AXIS],
                  mlratio = mlx > mly ? mly / mlx : mlx / mly,
                  fr_mm_s = min(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * SQRT(sq(mlratio) + 1.0);

      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(X_AXIS);
        sensorless_homing_per_axis(Y_AXIS);
      #endif

      do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir[Y_AXIS], fr_mm_s);
      endstops.hit_on_purpose(); // clear endstop hit flags
      current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;

      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(X_AXIS, false);
        sensorless_homing_per_axis(Y_AXIS, false);
        printer.safe_delay(500);
      #endif
    }

  #endif // QUICK_HOME

  #if ENABLED(Z_SAFE_HOMING)

    void Cartesian_Mechanics::home_z_safely() {

      // Disallow Z homing if X or Y are unknown
      if (!printer.isXHomed() || !printer.isYHomed()) {
        LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
        SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
        return;
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("Z_SAFE_HOMING >>>");
      #endif

      sync_plan_position();

      /**
       * Move the Z probe (or just the nozzle) to the safe homing point
       */
      destination[X_AXIS] = Z_SAFE_HOMING_X_POINT;
      destination[Y_AXIS] = Z_SAFE_HOMING_Y_POINT;
      destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

      #if HOMING_Z_WITH_PROBE
        destination[X_AXIS] -= probe.offset[X_AXIS];
        destination[Y_AXIS] -= probe.offset[Y_AXIS];
      #endif

      if (position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) DEBUG_POS("Z_SAFE_HOMING", destination);
        #endif

        // This causes the carriage on Dual X to unpark
        #if ENABLED(DUAL_X_CARRIAGE)
          active_hotend_parked = false;
        #endif

        #if ENABLED(SENSORLESS_HOMING)
          printer.safe_delay(500);
        #endif

        do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
        homeaxis(Z_AXIS);
      }
      else {
        LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
        SERIAL_LM(ECHO, MSG_ZPROBE_OUT);
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("<<< Z_SAFE_HOMING");
      #endif
    }

  #endif // Z_SAFE_HOMING

  #if ENABLED(DOUBLE_Z_HOMING)

    void Cartesian_Mechanics::double_home_z() {

      // Disallow Z homing if X or Y are unknown
      if (!printer.isXHomed() || !printer.isYHomed()) {
        LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
        SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
        return;
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("DOUBLE_Z_HOMING >>>");
      #endif

      sync_plan_position();

      /**
       * Move the Z probe (or just the nozzle) to the safe homing point
       */
      destination[X_AXIS] = DOUBLE_Z_HOMING_X_POINT;
      destination[Y_AXIS] = DOUBLE_Z_HOMING_Y_POINT;
      destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

      #if HAS_BED_PROBE
        destination[X_AXIS] -= probe.offset[X_AXIS];
        destination[Y_AXIS] -= probe.offset[Y_AXIS];
      #endif

      if (position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) DEBUG_POS("DOUBLE_Z_HOMING", destination);
        #endif

        const float newzero = probe_pt(destination[X_AXIS], destination[Y_AXIS], true, 1) - (2 * probe.offset[Z_AXIS]);
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
        if (printer.debugLeveling()) SERIAL_EM("<<< DOUBLE_Z_HOMING");
      #endif
    }

  #endif

  /**
   * Set an axis' current position to its home position (after homing).
   *
   * For Cartesian this applies one-to-one when an
   * individual axis has been homed.
   *
   * Callers must sync the planner position after calling this!
   */
  void Cartesian_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    printer.setAxisHomed(axis, true);

    #if ENABLED(WORKSPACE_OFFSETS)
      position_shift[axis] = 0;
      endstops.update_software_endstops(axis);
    #endif

    #if ENABLED(DUAL_X_CARRIAGE)
      if (axis == X_AXIS && (tools.active_extruder == 1 || dual_x_carriage_mode == DXC_DUPLICATION_MODE)) {
        current_position[X_AXIS] = x_home_pos(tools.active_extruder);
        return;
      }
    #endif

    current_position[axis] = base_home_pos[axis];

    /**
     * Z Probe Z Homing? Account for the probe's Z offset.
     */
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS) {
        current_position[Z_AXIS] -= probe.offset[Z_AXIS];

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (printer.debugLeveling()) {
            SERIAL_EM("*** Z HOMED WITH PROBE ***");
            SERIAL_EMV("zprobe_zoffset = ", probe.offset[Z_AXIS]);
          }
        #endif
      }
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (printer.debugLeveling()) {
        #if ENABLED(WORKSPACE_OFFSETS)
          SERIAL_MV("> home_offset[", axis_codes[axis]);
          SERIAL_EMV("] = ", home_offset[axis]);
        #endif
        DEBUG_POS("", current_position);
        SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  #if ENABLED(DUAL_X_CARRIAGE)

    float Cartesian_Mechanics::x_home_pos(const int extruder) {
      if (extruder == 0)
        return base_home_pos[X_AXIS];
      else
        // In dual carriage mode the extruder offset provides an override of the
        // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
        // This allow soft recalibration of the second extruder offset position without firmware reflash
        // (through the M218 command).
        return tools.hotend_offset[X_AXIS][1] > 0 ? tools.hotend_offset[X_AXIS][1] : X2_HOME_POS;
    }

  #endif

  #if ENABLED(HYSTERESIS)

    void Cartesian_Mechanics::set_hysteresis(float x_mm, float y_mm, float z_mm, float e_mm) {
      m_hysteresis_mm[X_AXIS] = x_mm;
      m_hysteresis_mm[Y_AXIS] = y_mm;
      m_hysteresis_mm[Z_AXIS] = z_mm;
      m_hysteresis_mm[E_AXIS] = e_mm;

      m_hysteresis_bits = ((m_hysteresis_mm[X_AXIS] != 0.0f) ? (1 << X_AXIS) : 0)
                        | ((m_hysteresis_mm[Y_AXIS] != 0.0f) ? (1 << Y_AXIS) : 0)
                        | ((m_hysteresis_mm[Z_AXIS] != 0.0f) ? (1 << Z_AXIS) : 0)
                        | ((m_hysteresis_mm[E_AXIS] != 0.0f) ? (1 << E_AXIS) : 0);

      calc_hysteresis_steps();
    }

    void Cartesian_Mechanics::calc_hysteresis_steps() {
      for (uint8_t i = 0; i < NUM_AXIS; i++)
        m_hysteresis_steps[i] = (long)(m_hysteresis_mm[i] * axis_steps_per_mm[i]);
    }

    void Cartesian_Mechanics::set_hysteresis_axis(uint8_t axis, float mm) {
      m_hysteresis_mm[axis] = mm;

      if (mm != 0.0f) m_hysteresis_bits |=  ( 1 << axis);
      else            m_hysteresis_bits &= ~( 1 << axis);

      calc_hysteresis_steps();
      report_hysteresis();
    }

    void Cartesian_Mechanics::report_hysteresis() {
      SERIAL_MV("Hysteresis X", m_hysteresis_mm[X_AXIS]);
      SERIAL_MV(" Y", m_hysteresis_mm[Y_AXIS]);
      SERIAL_MV(" Z", m_hysteresis_mm[Z_AXIS]);
      SERIAL_MV(" E", m_hysteresis_mm[E_AXIS]);
      SERIAL_MV(" SHIFTS: x=", m_hysteresis_axis_shift[X_AXIS]);
      SERIAL_MV(" y=", m_hysteresis_axis_shift[Y_AXIS]);
      SERIAL_MV(" z=", m_hysteresis_axis_shift[Z_AXIS]);
      SERIAL_EMV(" e=", m_hysteresis_axis_shift[E_AXIS]);
    }

    void Cartesian_Mechanics::insert_hysteresis_correction(const float x, const float y, const float z, const float e) {
      long target[NUM_AXIS] = {x * axis_steps_per_mm[X_AXIS], y * axis_steps_per_mm[Y_AXIS], z * axis_steps_per_mm[Z_AXIS], e * axis_steps_per_mm[E_AXIS + tools.active_extruder]};
      uint8_t direction_bits = calc_direction_bits(planner.position, target);
      uint8_t move_bits = calc_move_bits(planner.position, target);

      // if the direction has changed in any of the axis that need hysteresis corrections...
      uint8_t direction_change_bits = (direction_bits ^ m_hysteresis_prev_direction_bits) & move_bits;

      if ((direction_change_bits & m_hysteresis_bits) != 0 ) {
        // calculate the position to move to that will fix the hysteresis
        for (uint8_t axis = 0; axis < NUM_AXIS; axis++) {
          // if this axis changed direction...
          if (direction_change_bits & (1 << axis)) {
            long fix = (((direction_bits & (1 << axis)) != 0) ? -m_hysteresis_steps[axis] : m_hysteresis_steps[axis]);
            //... add the hysteresis: move the current position in the opposite direction so that the next travel move is longer
            planner.position[axis] -= fix;
            m_hysteresis_axis_shift[axis] += fix;
          }
        }
      }
      m_hysteresis_prev_direction_bits = (direction_bits & move_bits) | (m_hysteresis_prev_direction_bits & ~move_bits);
    }
    
    // direction 0: positive, 1: negative
    uint8_t Cartesian_Mechanics::calc_direction_bits(const long *position, const long *target) {
      unsigned char direction_bits = 0;

      if (target[X_AXIS] < position[X_AXIS])
        direction_bits |= (1 << X_AXIS);
      if (target[Y_AXIS] < position[Y_AXIS])
        direction_bits |= (1 << Y_AXIS);
      if (target[Z_AXIS] < position[Z_AXIS])
        direction_bits |= (1 << Z_AXIS);
      if (target[E_AXIS] < position[E_AXIS])
        direction_bits |= (1 << E_AXIS);

      return direction_bits;
    }

    uint8_t Cartesian_Mechanics::calc_move_bits(const long *position, const long *target) {
      uint8_t move_bits = 0;

      if (target[X_AXIS] != position[X_AXIS])
        move_bits |= (1 << X_AXIS);
      if (target[Y_AXIS] != position[Y_AXIS])
        move_bits |= (1 << Y_AXIS);
      if (target[Z_AXIS] != position[Z_AXIS])
        move_bits |= (1 << Z_AXIS);
      if (target[E_AXIS] != position[E_AXIS])
        move_bits |= (1 << E_AXIS);

      return move_bits;
    }

  #endif // ENABLED(HYSTERESIS)

  #if ENABLED(ZWOBBLE)

    void Cartesian_Mechanics::report_zwobble() {
      if (!m_zwobble_sinusoidal)
        SERIAL_MSG("Custom wobble function");
      else
        SERIAL_MV("Cartesian_Mechanics Amp(A): ", m_zwobble_amplitude);

      SERIAL_MV(" phase(P): ", m_zwobble_phase); 
      SERIAL_MV(" period(W): ", TWOPI / m_zwobble_puls);
      SERIAL_MV(" puls: ", m_zwobble_puls);

      if (!areParametersConsistent())
        SERIAL_SM(WARNING, " Inconsistent parameters!");

      SERIAL_EOL();

      if (!m_zwobble_sinusoidal) {
        // print out the LUT
        for (int i = 0; i < zwobble_lutSize; i++) {
          SERIAL_MV("Rod: ", ZROD(i));
          SERIAL_MV(" Act: ", ZACTUAL(i));

          int delta = (ZACTUAL(i) - ZROD(i)) * 200 + 20;
          for (int j = 0; j < delta; j++) {
            SERIAL_MSG(" ");
          }
          SERIAL_EM("  +");
        }
      }
    }

    // insert a planner.buffer_line if required to handle any hysteresis
    void Cartesian_Mechanics::insert_zwobble_correction(const float targetZ) {

      if (!m_zwobble_consistent) return; // don't go through consistency checks all the time; just check one bool

      float originZ = (float)planner.position[Z_AXIS] / axis_steps_per_mm[Z_AXIS];

      if (originZ < ZWOBBLE_MIN_Z || targetZ < ZWOBBLE_MIN_Z) return;

      if (DEBUGGING(ALL)) {
        SERIAL_MV("Origin: ", originZ);
        SERIAL_MV(" Target: ", targetZ);
      }

      if (EQUAL_WITHIN_TOLERANCE(originZ, targetZ)) return; // if there is no Z move, do nothing

      float originZRod;

      // there is a high chance that the origin Z is the same as the last target Z: skip one iteration of the algorithm if possible
      if (originZ == zwobble_lastZ)
        originZRod = zwobble_lastZRod;
      else
        originZRod = findZRod(originZ);

      if (DEBUGGING(ALL))
        SERIAL_MV(" Origin rod: ", originZRod);

      float targetZRod = findZRod(targetZ);

      if (DEBUGGING(ALL))
        SERIAL_MV(" Target Rod: ", targetZRod);

      // difference in steps between the correct movement (originZRod->targetZRod) and the planned movement
      long stepDiff = LROUND((targetZRod - originZRod) * axis_steps_per_mm[Z_AXIS]) - (LROUND(targetZ * axis_steps_per_mm[Z_AXIS]) - planner.position[Z_AXIS]);

      if (DEBUGGING(ALL))
        SERIAL_EMV(" stepDiff: ", stepDiff);

      zwobble_lastZ = targetZ;
      zwobble_lastZRod = targetZRod;

      // don't adjust if target posizion is less than 0
      if (planner.position[Z_AXIS] - stepDiff > 0)
        planner.position[Z_AXIS] -= stepDiff;
    }

    void Cartesian_Mechanics::set_zwobble_amplitude(float _amplitude) {
      m_zwobble_amplitude = _amplitude;
      m_zwobble_sinusoidal = true; // set_zwobble_amplitude sets to sinusoidal by default
      calculateLut();
    }

    void Cartesian_Mechanics::set_zwobble_period(float _period) {
      if (_period <= 0) return;
      m_zwobble_puls = TWOPI / _period;
      calculateLut();
    }

    void Cartesian_Mechanics::set_zwobble_phase(float _phase ) {
      // poor man's modulo operation
      while (_phase > 0) _phase -= 360;
      while (_phase < 0) _phase += 360;

      // phase now will be between 0 and 360
      m_zwobble_phase = RADIANS(_phase); // convert phase to radians
    }

    void Cartesian_Mechanics::set_zwobble_sample(float zRod, float zActual) {
      if (DEBUGGING(ALL)) {
        SERIAL_MV("New sample Rod: ", zRod);
        SERIAL_EMV(" Act: ", zActual);
      }

      if (m_zwobble_puls <= 0) {
        SERIAL_EM("You must define a period first (M97 W...)");
        return;
      }

      if (m_zwobble_sinusoidal) {
        m_zwobble_sinusoidal = false;
        calculateLut(); // initializes the LUT to linear
      }
      insertInLut(zRod, zActual);
    }

    void Cartesian_Mechanics::set_zwobble_scaledsample(float zRod, float zScaledLength) {
      // We want to be able to correct scaling factor or set it before/after samples, so (ICK) store scaled samples as negative numbers  
      set_zwobble_sample(zRod, -zScaledLength);

      // Make sure we have a non-zero scaling factor
      if (!m_zwobble_scalingFactor) {
        m_zwobble_scalingFactor = 1.0;
      }

      // Find two scaled samples close to period
      float period = TWOPI / m_zwobble_puls;
      int s1 = -1, s2 = -1;

      for (int i = 0; i < zwobble_lutSize; i++) {
        if (ZACTUAL_IS_SCALED(i)) {
          s1 = s2;
          s2 = i;
          if (ZROD(s2) >= period) break;
        }
      }

      // Calculate scaling factor so zact[period] = zrod[period]
      if (s2 >= 0 && ZACTUAL(s2)) {
        // Case 1 - Only one sample
        if (s1 < 0) {
          m_zwobble_scalingFactor *= ZROD(s2) / ZACTUAL(s2);
        }

        // Case 2 - Samples bracketing period (s1 - p - s2): average zact
        else if (ZROD(s2) > period) {
          float gap1 = period - ZROD(s1);
          float gap2 = ZROD(s2) - period;
          float zActPeriod = ZACTUAL(s1) + (ZACTUAL(s2) - ZACTUAL(s1)) * gap1 / (gap1 + gap2);

          m_zwobble_scalingFactor *= period / zActPeriod;
        }

        // Case 3 - Both samples before period (s1 - s2 - p): extrapolate zact
        else {
          float gap1 = ZROD(s2) - ZROD(s1);
          float gap2 = period - ZROD(s2);
          float zActPeriod = ZACTUAL(s2) + (ZACTUAL(s2) - ZACTUAL(s1)) * gap2 / gap1;

          m_zwobble_scalingFactor *= period / zActPeriod;
        }
      }
    }

    void Cartesian_Mechanics::set_zwobble_scalingfactor(float zActualPerScaledLength) {
      m_zwobble_scalingFactor = zActualPerScaledLength;
    }

    // calculate the ZRod -> Zactual LUT using the model Zactual = Zrod + sin(w*Zrod) - this will actually only be used for one period
    void Cartesian_Mechanics::calculateLut() {
      zwobble_lastZ = -1.0;
      zwobble_lastZRod = -1.0; // reinitialize memorized Z values since we are changing the model
      if (!areParametersConsistent()) return;
      if (!m_zwobble_sinusoidal) {
        initLinearLut();
        return; // if the model is not sinusoidal, initializes LUT to linear
      }
      zwobble_lutSize = STEPS_IN_ZLUT;
      float period = TWOPI / m_zwobble_puls;
      // divide the period in STEPS_IN_ZLUT steps
      float lutStep = period / STEPS_IN_ZLUT;
      for (int i = 0; i < STEPS_IN_ZLUT; i++) {
        float zRod = lutStep * i;
        SET_ZROD(i, zRod);
        SET_ZACTUAL(i, zRod + m_zwobble_amplitude * sin(m_zwobble_puls * zRod));
      }
    }

    void Cartesian_Mechanics::initLinearLut() {
      float period = TWOPI / m_zwobble_puls;
      zwobble_lutSize = 2; // only 2 samples originally
      SET_ZROD(0, 0);
      SET_ZACTUAL(0, 0);
      SET_ZROD(1, period);
      SET_ZACTUAL(1, period);
    }

    void Cartesian_Mechanics::insertInLut(float zRod, float zActual) {
      // check if the given zRod alread exists in LUT
      for (int i = 0; i < zwobble_lutSize; i++) {
        if (EQUAL_WITHIN_TOLERANCE(zRod, ZROD(i))) {
          // replace value
          SET_ZROD(i, zRod);
          SET_ZACTUAL(i, zActual);
          return;
        }
      }

      // ok the value does not exist: is there still space in LUT? Insert it
      if (zwobble_lutSize < STEPS_IN_ZLUT) {
        int zPlace = -1;
        for (int i = 0; i < zwobble_lutSize; i++) {
          if (ZROD(i) > zRod) {
            zPlace = i;
            break;
          }
        }

        // shift samples after zPlace
        for (int i = zwobble_lutSize; i > zPlace; i--) {
          SET_ZROD(i, ZROD(i - 1));
          SET_ZACTUAL(i, ZACTUAL(i - 1));
        }
        zwobble_lutSize++; // increase zwobble_lutSize

        // insert sample
        SET_ZROD(zPlace, zRod);
        SET_ZACTUAL(zPlace, zActual);
        return;
      }
      else {
        // zwobble_lutSize == STEPS_IN_ZLUT: replace the closest point with the new sample
        int zPlace = 0;
        float dist = DISTANCE(zRod, ZROD(zPlace));
        for (int i = 1; i < zwobble_lutSize; i++) {
          if (DISTANCE(zRod, ZROD(i)) < dist) {
            zPlace = i;
            dist = DISTANCE(zRod, ZROD(i));
          }
        }

        SET_ZROD(zPlace, zRod);
        SET_ZACTUAL(zPlace, zActual);
      }
    }

    float Cartesian_Mechanics::findInLut(float z) {
      int i = 0;
      if (z >= ZACTUAL(zwobble_lutSize - 1))
        return ZROD(zwobble_lutSize - 1);

      if (z <= ZACTUAL(0))
        return ZROD(0);

      for (i = 0; i < zwobble_lutSize; i++) {
        if (ZACTUAL(i) > z)
          break;
      }

      float invZDist = 1 / (ZACTUAL(i) - ZACTUAL(i-1)); // distance between Z steps

      float interpZ = (ZROD(i - 1) * (ZACTUAL(i) - z) + ZROD(i) * (z - ZACTUAL(i - 1))) * invZDist; // linear interpolation between neighboring Z values

      return interpZ;
    }

    // Find the Z value to be given to the "rod" in order to obtain the desired Z
    float Cartesian_Mechanics::findZRod(float z) {
      int nCycle = 0;
      float identicalZ = -m_zwobble_phase / m_zwobble_puls;

      // find the last point in which the two Z are identical: this happens every (2kPI-phase)/w
      while (identicalZ <= z) identicalZ = (TWOPI * (++nCycle) - m_zwobble_phase) / m_zwobble_puls; 

      // find Z again using the previous cycle
      identicalZ = (TWOPI *(nCycle - 1) - m_zwobble_phase) / m_zwobble_puls;

      float deltaZa = z - identicalZ;

      // find deltaZRod by linear interpolation of the lookup table
      float deltaZrod = findInLut(deltaZa);
      return identicalZ + deltaZrod;
    }

    bool Cartesian_Mechanics::areParametersConsistent() {
      if (!m_zwobble_sinusoidal) {
        m_zwobble_consistent = true;  // parameters are always consistent if lut is not sinusoidal
        return true;          // if the model is not sinusoidal, then don't check for consistency
      }

       // m_zwobble_amplitude*m_zwobble_puls must be less than 1 in order for the function to be invertible (otherwise it would mean that the wobble is so much that the axis goes back)
      if (m_zwobble_puls <= 0 || m_zwobble_amplitude <= 0 || (m_zwobble_amplitude * m_zwobble_puls) >= 1) {
        m_zwobble_consistent = false;
        return false;
      }

      m_zwobble_consistent = true;
      return true;
    }

  #endif // ENABLED(ZWOBBLE)

#endif // IS_CARTESIAN
