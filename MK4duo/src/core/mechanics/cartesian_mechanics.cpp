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

#include "../../../MK4duo.h"

#if IS_CARTESIAN

  Cartesian_Mechanics mechanics;

  /** Public Parameters */
  mechanics_data_t Cartesian_Mechanics::data;

  const float Cartesian_Mechanics::base_max_pos[XYZ]  = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS },
              Cartesian_Mechanics::base_min_pos[XYZ]  = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
              Cartesian_Mechanics::base_home_pos[XYZ] = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS },
              Cartesian_Mechanics::max_length[XYZ]    = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };

  #if ENABLED(DUAL_X_CARRIAGE)
    DualXMode Cartesian_Mechanics::dual_x_carriage_mode           = DEFAULT_DUAL_X_CARRIAGE_MODE;
    float     Cartesian_Mechanics::inactive_extruder_x_pos        = X2_MAX_POS,
              Cartesian_Mechanics::raised_parked_position[XYZE],
              Cartesian_Mechanics::duplicate_extruder_x_offset    = DEFAULT_DUPLICATION_X_OFFSET;
    int16_t   Cartesian_Mechanics::duplicate_extruder_temp_offset = 0;
    millis_t  Cartesian_Mechanics::delayed_move_time              = 0;
    bool      Cartesian_Mechanics::active_extruder_parked         = false,
              Cartesian_Mechanics::extruder_duplication_enabled   = false,
              Cartesian_Mechanics::scaled_duplication_mode        = false;
  #endif

  /** Private Parameters */
  constexpr float slop = 0.0001;

  /** Public Function */
  void Cartesian_Mechanics::factory_parameters() {

    static const float    tmp_step[]          PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT,
                          tmp_maxfeedrate[]   PROGMEM = DEFAULT_MAX_FEEDRATE;

    static const uint32_t tmp_maxacc[]        PROGMEM = DEFAULT_MAX_ACCELERATION,
                          tmp_retract[]       PROGMEM = DEFAULT_RETRACT_ACCELERATION;

    LOOP_XYZE_N(i) {
      data.axis_steps_per_mm[i]           = pgm_read_float(&tmp_step[i < COUNT(tmp_step) ? i : COUNT(tmp_step) - 1]);
      data.max_feedrate_mm_s[i]           = pgm_read_float(&tmp_maxfeedrate[i < COUNT(tmp_maxfeedrate) ? i : COUNT(tmp_maxfeedrate) - 1]);
      data.max_acceleration_mm_per_s2[i]  = pgm_read_dword_near(&tmp_maxacc[i < COUNT(tmp_maxacc) ? i : COUNT(tmp_maxacc) - 1]);
    }

    LOOP_EXTRUDER()
      data.retract_acceleration[e]  = pgm_read_dword_near(&tmp_retract[e < COUNT(tmp_retract) ? e : COUNT(tmp_retract) - 1]);

    data.acceleration               = DEFAULT_ACCELERATION;
    data.travel_acceleration        = DEFAULT_TRAVEL_ACCELERATION;
    data.min_feedrate_mm_s          = DEFAULT_MIN_FEEDRATE;
    data.min_segment_time_us        = DEFAULT_MIN_SEGMENT_TIME;
    data.min_travel_feedrate_mm_s   = DEFAULT_MIN_TRAVEL_FEEDRATE;

    #if ENABLED(JUNCTION_DEVIATION)
      data.junction_deviation_mm = float(JUNCTION_DEVIATION_MM);
    #else
      static const float tmp_ejerk[] PROGMEM = DEFAULT_EJERK;
      data.max_jerk[X_AXIS]  = DEFAULT_XJERK;
      data.max_jerk[Y_AXIS]  = DEFAULT_YJERK;
      data.max_jerk[Z_AXIS]  = DEFAULT_ZJERK;
      #if DISABLED(LIN_ADVANCE)
        LOOP_EXTRUDER()
          data.max_jerk[E_AXIS + e] = pgm_read_float(&tmp_ejerk[e < COUNT(tmp_ejerk) ? e : COUNT(tmp_ejerk) - 1]);
      #endif
    #endif

    #if ENABLED(WORKSPACE_OFFSETS)
      ZERO(mechanics.data.home_offset);
    #endif

  }

  /**
   * Get the stepper positions in the cartesian_position[] array.
   *
   * The result is in the current coordinate space with
   * leveling applied. The coordinates need to be run through
   * unapply_leveling to obtain the "ideal" coordinates
   * suitable for current_position, etc.
   */
  void Cartesian_Mechanics::get_cartesian_from_steppers() {
    cartesian_position[X_AXIS] = planner.get_axis_position_mm(X_AXIS);
    cartesian_position[Y_AXIS] = planner.get_axis_position_mm(Y_AXIS);
    cartesian_position[Z_AXIS] = planner.get_axis_position_mm(Z_AXIS);
  }

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  void Cartesian_Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s /*=0.0*/) {
    const float old_feedrate_mm_s = feedrate_mm_s;

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) Com::print_xyz(PSTR(">>> do_blocking_move_to"), NULL, rx, ry, rz);
    #endif

    const float z_feedrate = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];

    // If Z needs to raise, do it before moving XY
    if (current_position[Z_AXIS] < rz) {
      feedrate_mm_s = z_feedrate;
      current_position[Z_AXIS] = rz;
      line_to_current_position();
    }

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    current_position[X_AXIS] = rx;
    current_position[Y_AXIS] = ry;
    line_to_current_position();

    // If Z needs to lower, do it after moving XY
    if (current_position[Z_AXIS] > rz) {
      feedrate_mm_s = z_feedrate;
      current_position[Z_AXIS] = rz;
      line_to_current_position();
    }

    feedrate_mm_s = old_feedrate_mm_s;

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) SERIAL_EM("<<< do_blocking_move_to");
    #endif

    planner.synchronize();

  }
  void Cartesian_Mechanics::do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(rx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
  }
  void Cartesian_Mechanics::do_blocking_move_to_z(const float &rz, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], rz, fr_mm_s);
  }
  void Cartesian_Mechanics::do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
    do_blocking_move_to(rx, ry, current_position[Z_AXIS], fr_mm_s);
  }

  /**
   * Home Cartesian
   */
  void Cartesian_Mechanics::home(const bool homeX/*=false*/, const bool homeY/*=false*/, const bool homeZ/*=false*/) {

    if (printer.debugSimulation()) {
      LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        mechanics.Nextion_gfx_clear();
      #endif
      return;
    }

    #if ENABLED(DUAL_X_CARRIAGE)
      const bool DXC_saved_duplication_state = extruder_duplication_enabled;
      DualXMode DXC_saved_mode = dual_x_carriage_mode;
    #endif

    #if HAS_POWER_SWITCH
      powerManager.power_on(); // Power On if power is off
    #endif

    // Wait for planner moves to finish!
    planner.synchronize();

    // Cancel the active G29 session
    #if HAS_LEVELING && ENABLED(PROBE_MANUALLY)
      bedlevel.g29_in_progress = false;
      #if HAS_NEXTION_MANUAL_BED
        Nextion_ProbeOff();
      #endif
    #endif

    // Disable the leveling matrix before homing
    #if HAS_LEVELING
      const bool leveling_was_active = bedlevel.leveling_active;
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
      extruder_duplication_enabled = false;
    #endif

    printer.setup_for_endstop_or_probe_move();
    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) SERIAL_EM("> endstops.setEnabled(true)");
    #endif
    endstops.setEnabled(true); // Enable endstops for next homing move

    bool come_back = parser.boolval('B');
    float lastpos[NUM_AXIS];
    float old_feedrate_mm_s;
    if (come_back) {
      old_feedrate_mm_s = feedrate_mm_s;
      COPY_ARRAY(lastpos, current_position);
    }

    const bool home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first
      if (home_all || homeZ) homeaxis(Z_AXIS);
    #endif

    const float z_homing_height = printer.isZHomed() ? MIN_Z_HEIGHT_FOR_HOMING : 0;

    if (z_homing_height && (home_all || homeX || homeY)) {
      // Raise Z before homing any other axes and z is not already high enough (never lower z)
      destination[Z_AXIS] = z_homing_height;
      if (destination[Z_AXIS] > current_position[Z_AXIS]) {
        #if ENABLED(DEBUG_FEATURE)
          if (printer.debugFeature())
            SERIAL_EMV("Raise Z (before homing) to ", destination[Z_AXIS]);
        #endif
        do_blocking_move_to_z(destination[Z_AXIS]);
      }
    }

    #if ENABLED(QUICK_HOME)
      if (home_all || (homeX && homeY)) quick_home_xy();
    #endif

    #if ENABLED(HOME_Y_BEFORE_X)
      // Home Y (before X)
      if (home_all || homeY) homeaxis(Y_AXIS);
    #endif

    // Home X
    if (home_all || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        // Always home the 2nd (right) extruder first
        tools.active_extruder = 1;
        homeaxis(X_AXIS);

        // Remember this extruder's position for later tool change
        inactive_extruder_x_pos = current_position[X_AXIS];

        // Home the 1st (left) extruder
        tools.active_extruder = 0;
        homeaxis(X_AXIS);

        // Consider the active extruder to be parked
        COPY_ARRAY(raised_parked_position, current_position);
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        homeaxis(X_AXIS);
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y (after X)
      if (home_all || homeY) homeaxis(Y_AXIS);
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (home_all || homeZ) {
        #if ENABLED(Z_SAFE_HOMING)
          home_z_safely();
        #else
          homeaxis(Z_AXIS);
        #endif // !Z_SAFE_HOMING

        #if HOMING_Z_WITH_PROBE && Z_PROBE_AFTER_PROBING > 0
          probe.move_z_after_probing();
        #endif

      } // home_all || homeZ

    #elif ENABLED(DOUBLE_Z_HOMING)
      if (home_all || homeZ) double_home_z();
    #endif

    sync_plan_position();

    #if ENABLED(DUAL_X_CARRIAGE)

      if (dxc_is_duplicating()) {

        // Always home the 2nd (right) extruder first
        tools.active_extruder = 1;
        homeaxis(X_AXIS);

        // Remember this extruder's position for later tool change
        inactive_extruder_x_pos = current_position[X_AXIS];

        // Home the 1st (left) extruder
        tools.active_extruder = 0;
        homeaxis(X_AXIS);

        // Consider the active extruder to be parked
        COPY_ARRAY(raised_parked_position, current_position);
        delayed_move_time = 0;
        active_extruder_parked = true;
        extruder_duplication_enabled  = false;
        dual_x_carriage_mode          = DXC_saved_mode;
        stepper.set_directions();
      }

    #endif // DUAL_X_CARRIAGE

    endstops.setNotHoming();

    if (come_back) {
      feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
      COPY_ARRAY(destination, lastpos);
      prepare_move_to_destination();
      feedrate_mm_s = old_feedrate_mm_s;
    }

    #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
      mechanics.Nextion_gfx_clear();
    #endif

    #if HAS_LEVELING
      bedlevel.set_bed_leveling_enabled(leveling_was_active);
    #endif

    printer.clean_up_after_endstop_or_probe_move();

    planner.synchronize();

    // Restore the active tool after homing
    #if HOTENDS > 1
      tools.change(old_tool_index, 0, true);
    #endif

    lcd_refresh();

    mechanics.report_current_position();

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) SERIAL_EM("<<< G28");
    #endif

  }

  /**
   * Home an individual linear axis
   */
  void Cartesian_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
        SERIAL_MV(", ", distance);
        SERIAL_MSG(", ");
        if (fr_mm_s)
          SERIAL_VAL(fr_mm_s);
        else {
          SERIAL_MV(" [", homing_feedrate_mm_s[axis]);
          SERIAL_CHR(']');
        }
        SERIAL_CHR(')');
        SERIAL_EOL();
      }
    #endif

    // Only do some things when moving towards an endstop
    const int8_t axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        (axis == X_AXIS) ? mechanics.x_home_dir(tools.active_extruder) :
      #endif
      get_homedir(axis);
    const bool is_home_dir = (axis_home_dir > 0) == (distance > 0);

    if (is_home_dir) {

      if (axis == Z_AXIS) {
        #if HOMING_Z_WITH_PROBE
          #if ENABLED(BLTOUCH)
            bltouch.set_deployed(true);
          #endif
          #if QUIET_PROBING
            probe.probing_pause(true);
          #endif
        #endif
      }

      // Disable stealthChop if used. Enable diag1 pin on driver.
      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(axis);
      #endif
    }

    float target[ABCE] = { planner.get_axis_position_mm(A_AXIS), planner.get_axis_position_mm(B_AXIS), planner.get_axis_position_mm(C_AXIS), planner.get_axis_position_mm(E_AXIS) };
    target[axis] = 0;
    planner.set_machine_position_mm(target);
    target[axis] = distance;

    // Set cartesian axes directly
    planner.buffer_segment(target, fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], tools.active_extruder);

    planner.synchronize();

    if (is_home_dir) {

      if (axis == Z_AXIS) {
        #if HOMING_Z_WITH_PROBE
          #if QUIET_PROBING
            probe.probing_pause(false);
          #endif
          #if ENABLED(BLTOUCH)
            bltouch.set_deployed(false);
          #endif
        #endif
      }

      endstops.validate_homing_move();

      // Re-enable stealthChop if used. Disable diag1 pin on driver.
      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(axis, false);
      #endif
    }

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

  }

  /**
   * Prepare a linear move in a Cartesian setup.
   *
   * When a mesh-based leveling system is active, moves are segmented
   * according to the configuration of the leveling system.
   *
   * Returns true if current_position[] was set to destination[]
   */
  bool Cartesian_Mechanics::prepare_move_to_destination_mech_specific() {

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

  /**
   * Set an axis' current position to its home position (after homing).
   *
   * For Cartesian this applies one-to-one when an
   * individual axis has been homed.
   *
   * Callers must sync the planner position after calling this!
   */
  void Cartesian_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
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
      if (axis == X_AXIS && (tools.active_extruder == 1 || dxc_is_duplicating())) {
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

        #if ENABLED(DEBUG_FEATURE)
          if (printer.debugFeature()) {
            SERIAL_EM("*** Z HOMED WITH PROBE ***");
            SERIAL_EMV("zprobe_zoffset = ", probe.offset[Z_AXIS]);
          }
        #endif
      }
    #endif

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        #if ENABLED(WORKSPACE_OFFSETS)
          SERIAL_MV("> data.home_offset[", axis_codes[axis]);
          SERIAL_EMV("] = ", data.home_offset[axis]);
        #endif
        DEBUG_POS("", current_position);
        SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }

  // Return true if the given position is within the machine bounds.
  bool Cartesian_Mechanics::position_is_reachable(const float &rx, const float &ry) {
    if (!WITHIN(ry, Y_MIN_POS - slop, Y_MAX_POS + slop)) return false;
    #if ENABLED(DUAL_X_CARRIAGE)
      if (tools.active_extruder)
        return WITHIN(rx, X2_MIN_POS - slop, X2_MAX_POS + slop);
      else
        return WITHIN(rx, X1_MIN_POS - slop, X1_MAX_POS + slop);
    #else
      return WITHIN(rx, X_MIN_POS - slop, X_MAX_POS + slop);
    #endif
  }
  // Return whether the given position is within the bed, and whether the nozzle
  //  can reach the position required to put the probe at the given position.
  bool Cartesian_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
    return position_is_reachable(rx - probe.offset[X_AXIS], ry - probe.offset[Y_AXIS])
        && WITHIN(rx, MIN_PROBE_X - slop, MAX_PROBE_X + slop)
        && WITHIN(ry, MIN_PROBE_Y - slop, MAX_PROBE_Y + slop);
  }

  // Report detail current position to host
  void Cartesian_Mechanics::report_current_position_detail() {

    SERIAL_MSG("\nLogical:");
    const float logical[XYZ] = {
      LOGICAL_X_POSITION(current_position[X_AXIS]),
      LOGICAL_Y_POSITION(current_position[Y_AXIS]),
      LOGICAL_Z_POSITION(current_position[Z_AXIS])
    };
    report_xyz(logical);

    SERIAL_MSG("Raw:    ");
    report_xyze(current_position);

    float leveled[XYZ] = { current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] };

    #if HAS_LEVELING
      SERIAL_MSG("Leveled:");
      bedlevel.apply_leveling(leveled);
      report_xyz(leveled);

      SERIAL_MSG("UnLevel:");
      float unleveled[XYZ] = { leveled[X_AXIS], leveled[Y_AXIS], leveled[Z_AXIS] };
      bedlevel.unapply_leveling(unleveled);
      report_xyz(unleveled);
    #endif

    planner.synchronize();

    SERIAL_MSG("Stepper:");
    LOOP_XYZE(i) {
      SERIAL_CHR(' ');
      SERIAL_CHR(axis_codes[i]);
      SERIAL_CHR(':');
      SERIAL_TXT(stepper.position((AxisEnum)i));
    }
    SERIAL_EOL();

    SERIAL_MSG("FromStp:");
    get_cartesian_from_steppers();  // writes cartesian_position[XYZ] (with forward kinematics)
    const float from_steppers[XYZE] = { cartesian_position[X_AXIS], cartesian_position[Y_AXIS], cartesian_position[Z_AXIS], planner.get_axis_position_mm(E_AXIS) };
    report_xyze(from_steppers);

    const float diff[XYZE] = {
      from_steppers[X_AXIS] - leveled[X_AXIS],
      from_steppers[Y_AXIS] - leveled[Y_AXIS],
      from_steppers[Z_AXIS] - leveled[Z_AXIS],
      from_steppers[E_AXIS] - current_position[E_AXIS]
    };

    SERIAL_MSG("Differ: ");
    report_xyze(diff);

  }

  #if ENABLED(ARC_SUPPORT)

    #if N_ARC_CORRECTION < 1
      #undef N_ARC_CORRECTION
      #define N_ARC_CORRECTION 1
    #endif

    /**
     * Plan an arc in 2 dimensions
     *
     * The arc is approximated by generating many small linear segments.
     * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
     * Arcs should only be made relatively large (over 5mm), as larger arcs with
     * larger segments will tend to be more efficient. Your slicer should have
     * options for G2/G3 arc generation. In future these options may be GCode tunable.
     */
    void Cartesian_Mechanics::plan_arc(
      const float (&cart)[XYZE],  // Destination position
      const float (&offset)[2],   // Center of rotation relative to current_position
      const uint8_t clockwise     // Clockwise?
    ) {

      #if ENABLED(CNC_WORKSPACE_PLANES)
        AxisEnum p_axis, q_axis, l_axis;
        switch (workspace_plane) {
          default:
          case PLANE_XY: p_axis = X_AXIS; q_axis = Y_AXIS; l_axis = Z_AXIS; break;
          case PLANE_ZX: p_axis = Z_AXIS; q_axis = X_AXIS; l_axis = Y_AXIS; break;
          case PLANE_YZ: p_axis = Y_AXIS; q_axis = Z_AXIS; l_axis = X_AXIS; break;
        }
      #else
        constexpr AxisEnum p_axis = X_AXIS, q_axis = Y_AXIS, l_axis = Z_AXIS;
      #endif

      // Radius vector from center to current location
      float r_P = -offset[0], r_Q = -offset[1];

      const float radius = HYPOT(r_P, r_Q),
                  center_P = current_position[p_axis] - r_P,
                  center_Q = current_position[q_axis] - r_Q,
                  rt_X = cart[p_axis] - center_P,
                  rt_Y = cart[q_axis] - center_Q,
                  linear_travel = cart[l_axis] - current_position[l_axis],
                  extruder_travel = cart[E_AXIS] - current_position[E_AXIS];

      // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
      float angular_travel = ATAN2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y);
      if (angular_travel < 0) angular_travel += RADIANS(360);
      if (clockwise) angular_travel -= RADIANS(360);

      // Make a circle if the angular rotation is 0
      if (angular_travel == 0 && current_position[p_axis] == cart[p_axis] && current_position[q_axis] == cart[q_axis])
        angular_travel += RADIANS(360);

      const float flat_mm = radius * angular_travel,
                  mm_of_travel = linear_travel ? HYPOT(flat_mm, linear_travel) : ABS(flat_mm);
      if (mm_of_travel < 0.001f) return;

      uint16_t segments = FLOOR(mm_of_travel / (MM_PER_ARC_SEGMENT));
      if (segments == 0) segments = 1;

      /**
       * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
       *     r_T = [cos(phi) -sin(phi);
       *            sin(phi)  cos(phi] * r ;
       *
       * For arc generation, the center of the circle is the axis of rotation and the radius vector is
       * defined from the circle center to the initial position. Each line segment is formed by successive
       * vector rotations. This requires only two cos() and sin() computations to form the rotation
       * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
       * all double numbers are single precision on the Arduino. (True double precision will not have
       * round off issues for CNC applications.) Single precision error can accumulate to be greater than
       * tool precision in some cases. Therefore, arc path correction is implemented.
       *
       * Small angle approximation may be used to reduce computation overhead further. This approximation
       * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
       * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       * issue for CNC machines with the single precision Arduino calculations.
       *
       * This approximation also allows plan_arc to immediately insert a line segment into the planner
       * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
       * This is important when there are successive arc motions.
       */
      // Vector rotation matrix values
      float raw[XYZE];
      const float theta_per_segment = angular_travel / segments,
                  linear_per_segment = linear_travel / segments,
                  extruder_per_segment = extruder_travel / segments,
                  sin_T = theta_per_segment,
                  cos_T = 1 - 0.5f * sq(theta_per_segment); // Small angle approximation

      // Initialize the linear axis
      raw[l_axis] = current_position[l_axis];

      // Initialize the extruder axis
      raw[E_AXIS] = current_position[E_AXIS];

      const float fr_mm_s = MMS_SCALED(feedrate_mm_s);

      millis_t next_idle_ms = millis() + 200UL;

      #if N_ARC_CORRECTION > 1
        int8_t arc_recalc_count = N_ARC_CORRECTION;
      #endif

      for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

        printer.check_periodical_actions();
        if (ELAPSED(millis(), next_idle_ms)) {
          next_idle_ms = millis() + 200UL;
          printer.idle();
        }

        #if N_ARC_CORRECTION > 1
          if (--arc_recalc_count) {
            // Apply vector rotation matrix to previous r_P / 1
            const float r_new_Y = r_P * sin_T + r_Q * cos_T;
            r_P = r_P * cos_T - r_Q * sin_T;
            r_Q = r_new_Y;
          }
          else
        #endif
        {
          #if N_ARC_CORRECTION > 1
            arc_recalc_count = N_ARC_CORRECTION;
          #endif

          // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
          // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
          // To reduce stuttering, the sin and cos could be computed at different times.
          // For now, compute both at the same time.
          const float cos_Ti = cos(i * theta_per_segment),
                      sin_Ti = sin(i * theta_per_segment);
          r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti;
          r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti;
        }

        // Update raw location
        raw[p_axis] = center_P + r_P;
        raw[q_axis] = center_Q + r_Q;
        raw[l_axis] += linear_per_segment;
        raw[E_AXIS] += extruder_per_segment;

        endstops.clamp_to_software(raw);

        #if HAS_LEVELING && !PLANNER_LEVELING
          bedlevel.apply_leveling(raw);
        #endif

        if (!planner.buffer_line(raw, fr_mm_s, tools.active_extruder, MM_PER_ARC_SEGMENT))
          break;
      }

      // Ensure last segment arrives at target location.
      COPY_ARRAY(raw, cart);

      #if HAS_LEVELING && !PLANNER_LEVELING
        bedlevel.apply_leveling(raw);
      #endif

      planner.buffer_line(raw, fr_mm_s, tools.active_extruder, MM_PER_ARC_SEGMENT);

      COPY_ARRAY(current_position, raw);

    }

  #endif // ENABLED(ARC_SUPPORT)

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

    /**
     * Prepare a linear move in a dual X axis setup
     *
     * Return true if current_position[] was set to destination[]
     */
    bool Cartesian_Mechanics::dual_x_carriage_unpark() {
      if (active_extruder_parked) {
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
            #define CUR_X    current_position[X_AXIS]
            #define CUR_Y    current_position[Y_AXIS]
            #define CUR_Z    current_position[Z_AXIS]
            #define CUR_E    current_position[E_AXIS]
            #define RAISED_X raised_parked_position[X_AXIS]
            #define RAISED_Y raised_parked_position[Y_AXIS]
            #define RAISED_Z raised_parked_position[Z_AXIS]

            if (  planner.buffer_line(RAISED_X, RAISED_Y, RAISED_Z, CUR_E, data.max_feedrate_mm_s[Z_AXIS], tools.active_extruder))
              if (planner.buffer_line(   CUR_X,    CUR_Y, RAISED_Z, CUR_E, PLANNER_XY_FEEDRATE(),     tools.active_extruder))
                  planner.buffer_line(   CUR_X,    CUR_Y,    CUR_Z, CUR_E, data.max_feedrate_mm_s[Z_AXIS], tools.active_extruder);
            delayed_move_time = 0;
            active_extruder_parked = false;
            #if ENABLED(DEBUG_FEATURE)
              if (printer.debugFeature()) SERIAL_EM("Clear active_extruder_parked");
            #endif
            break;
          case DXC_SCALED_DUPLICATION_MODE:
          case DXC_DUPLICATION_MODE:
            if (tools.active_extruder == 0) {
              #if ENABLED(DEBUG_FEATURE)
                if (printer.debugFeature()) {
                  SERIAL_MV("Set planner X", inactive_extruder_x_pos);
                  SERIAL_EMV(" ... Line to X", current_position[X_AXIS] + duplicate_extruder_x_offset);
                }
              #endif
              // move duplicate extruder into correct duplication position.
              planner.set_position_mm(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

              if (!planner.buffer_line(
                dual_x_carriage_mode == DXC_DUPLICATION_MODE ? duplicate_extruder_x_offset + current_position[X_AXIS] : inactive_extruder_x_pos,
                current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS],
                data.max_feedrate_mm_s[X_AXIS], 1
              )) break;
              planner.synchronize();
              sync_plan_position();
              extruder_duplication_enabled = true;
              active_extruder_parked = false;
              #if ENABLED(DEBUG_FEATURE)
                if (printer.debugFeature()) SERIAL_EM("Set extruder_duplication_enabled\nClear active_extruder_parked");
              #endif
            }
            else {
              #if ENABLED(DEBUG_FEATURE)
                if (printer.debugFeature()) SERIAL_EM("Active extruder not 0");
              #endif
            }
            break;
        }
      }
      stepper.set_directions();
      return false;
    }

  #endif // ENABLED(DUAL_X_CARRIAGE)

  #if DISABLED(DISABLE_M503)

    void Cartesian_Mechanics::print_parameters() {

      SERIAL_LM(CFG, "Steps per unit:");
      SERIAL_SMV(CFG, "  M92 X", LINEAR_UNIT(data.axis_steps_per_mm[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.axis_steps_per_mm[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(data.axis_steps_per_mm[Z_AXIS]), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(data.axis_steps_per_mm[E_AXIS]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M92 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(data.axis_steps_per_mm[E_AXIS + e]), 3);
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Maximum feedrates (units/s):");
      SERIAL_SMV(CFG, "  M203 X", LINEAR_UNIT(data.max_feedrate_mm_s[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.max_feedrate_mm_s[Y_AXIS]), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(data.max_feedrate_mm_s[Z_AXIS]), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(data.max_feedrate_mm_s[E_AXIS]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M203 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(data.max_feedrate_mm_s[E_AXIS + e]), 3);
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Maximum Acceleration (units/s2):");
      SERIAL_SMV(CFG, "  M201 X", LINEAR_UNIT(data.max_acceleration_mm_per_s2[X_AXIS]));
      SERIAL_MV(" Y", LINEAR_UNIT(data.max_acceleration_mm_per_s2[Y_AXIS]));
      SERIAL_MV(" Z", LINEAR_UNIT(data.max_acceleration_mm_per_s2[Z_AXIS]));
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(data.max_acceleration_mm_per_s2[E_AXIS]));
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M201 T", (int)e);
          SERIAL_EMV(" E", VOLUMETRIC_UNIT(data.max_acceleration_mm_per_s2[E_AXIS + e]));
        }
      #endif // EXTRUDERS > 1

      SERIAL_LM(CFG, "Acceleration (units/s2): P<DEFAULT_ACCELERATION> V<DEFAULT_TRAVEL_ACCELERATION> T* R<DEFAULT_RETRACT_ACCELERATION>");
      SERIAL_SMV(CFG,"  M204 P", LINEAR_UNIT(data.acceleration), 3);
      SERIAL_MV(" V", LINEAR_UNIT(data.travel_acceleration), 3);
      #if EXTRUDERS == 1
        SERIAL_MV(" T0 R", LINEAR_UNIT(data.retract_acceleration[0]), 3);
      #endif
      SERIAL_EOL();
      #if EXTRUDERS > 1
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M204 T", (int)e);
          SERIAL_EMV(" R", LINEAR_UNIT(data.retract_acceleration[e]), 3);
        }
      #endif

      SERIAL_LM(CFG, "Advanced: B<DEFAULT_MIN_SEGMENT_TIME> S<DEFAULT_MIN_FEEDRATE> V<DEFAULT_MIN_TRAVEL_FEEDRATE>");
      SERIAL_SMV(CFG, "  M205 B", data.min_segment_time_us);
      SERIAL_MV(" S", LINEAR_UNIT(data.min_feedrate_mm_s), 3);
      SERIAL_EMV(" V", LINEAR_UNIT(data.min_travel_feedrate_mm_s), 3);

      #if ENABLED(JUNCTION_DEVIATION)
        SERIAL_LM(CFG, "Junction Deviation: J<JUNCTION_DEVIATION_MM>");
        SERIAL_LMV(CFG, "  M205 J", data.junction_deviation_mm, 3);
      #else
        SERIAL_SM(CFG, "Jerk: X<DEFAULT_XJERK> Y<DEFAULT_YJERK> Z<max_z_jerk>");
        #if DISABLED(LIN_ADVANCE)
          SERIAL_MSG(" T* E<DEFAULT_EJERK>");
        #endif
        SERIAL_EOL();

        SERIAL_SMV(CFG, "  M205 X", LINEAR_UNIT(data.max_jerk[X_AXIS]), 3);
        SERIAL_MV(" Y", LINEAR_UNIT(data.max_jerk[Y_AXIS]), 3);
        SERIAL_MV(" Z", LINEAR_UNIT(data.max_jerk[Z_AXIS]), 3);

        #if DISABLED(LIN_ADVANCE)
          #if EXTRUDERS == 1
            SERIAL_MV(" T0 E", LINEAR_UNIT(data.max_jerk[E_AXIS]), 3);
          #endif
          SERIAL_EOL();
          #if (EXTRUDERS > 1)
            LOOP_EXTRUDER() {
              SERIAL_SMV(CFG, "  M205 T", (int)e);
              SERIAL_EMV(" E" , LINEAR_UNIT(data.max_jerk[E_AXIS + e]), 3);
            }
          #endif
        #endif
      #endif

      #if ENABLED(WORKSPACE_OFFSETS)
        SERIAL_LM(CFG, "Home offset:");
        SERIAL_SMV(CFG, "  M206 X", LINEAR_UNIT(data.home_offset[X_AXIS]), 3);
        SERIAL_MV(" Y", LINEAR_UNIT(data.home_offset[Y_AXIS]), 3);
        SERIAL_EMV(" Z", LINEAR_UNIT(data.home_offset[Z_AXIS]), 3);
      #endif

    }

  #endif // DISABLED(DISABLE_M503)

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)

    void Cartesian_Mechanics::Nextion_gfx_clear() {
      gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
      gfx_cursor_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
    }

  #endif

  /** Private Function */
  void Cartesian_Mechanics::homeaxis(const AxisEnum axis) {

    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    const int axis_home_dir = (
      #if ENABLED(DUAL_X_CARRIAGE)
        axis == X_AXIS ? x_home_dir(tools.active_extruder) :
      #endif
      get_homedir(axis)
    );

    // Homing Z towards the bed? Deploy the Z probe or endstop.
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && DEPLOY_PROBE()) return;
    #endif

    // Set flags for X, Y, Z motor locking
    #if ENABLED(X_TWO_ENDSTOPS)
      if (axis == X_AXIS) stepper.set_separate_multi_axis(true);
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      if (axis == Y_AXIS) stepper.set_separate_multi_axis(true);
    #endif
    #if ENABLED(Z_TWO_ENDSTOPS) || ENABLED(Z_THREE_ENDSTOPS)
      if (axis == Z_AXIS) stepper.set_separate_multi_axis(true);
    #endif

    // Fast move towards endstop until triggered
    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) SERIAL_EM("Home 1 Fast:");
    #endif

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS && bltouch.set_deployed(true)) return;
    #endif

    do_homing_move(axis, 1.5f * max_length[axis] * axis_home_dir);

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS) bltouch.set_deployed(false);
    #endif

    // When homing Z with probe respect probe clearance
    const float bump = axis_home_dir * (
      #if HOMING_Z_WITH_PROBE
        (axis == Z_AXIS) ? MAX(Z_PROBE_BETWEEN_HEIGHT, home_bump_mm[Z_AXIS]) :
      #endif
      home_bump_mm[axis]
    );

    // If a second homing move is configured...
    if (bump) {
      // Move away from the endstop by the axis HOME_BUMP_MM
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("Move Away:");
      #endif
      do_homing_move(axis, -bump
        #if HOMING_Z_WITH_PROBE
          , axis == Z_AXIS ? MMM_TO_MMS(Z_PROBE_SPEED_FAST) : 0.0
        #endif
      );

      // Slow move towards endstop until triggered
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("Home 2 Slow:");
      #endif

      #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
        // BLTOUCH needs to be deployed every time
        if (axis == Z_AXIS && bltouch.set_deployed(true)) return;
      #endif

      do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));

      #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
        // BLTOUCH needs to be deployed every time
        if (axis == Z_AXIS) bltouch.set_deployed(false);
      #endif
    }

    #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
      const bool pos_dir = axis_home_dir > 0;
      #if ENABLED(X_TWO_ENDSTOPS)
        if (axis == X_AXIS) {
          const float adj = ABS(endstops.x2_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.x2_endstop_adj > 0) : (endstops.x2_endstop_adj < 0)) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
            do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_x_lock(false);
            stepper.set_x2_lock(false);
          }
        }
      #endif
      #if ENABLED(Y_TWO_ENDSTOPS)
        if (axis == Y_AXIS) {
          const float adj = ABS(endstops.y2_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.y2_endstop_adj > 0) : (endstops.y2_endstop_adj < 0)) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
            do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_y_lock(false);
            stepper.set_y2_lock(false);
          }
        }
      #endif
      #if ENABLED(Z_THREE_ENDSTOPS)
        if (axis == Z_AXIS) {
          // we push the function pointers for the stepper lock function into an array
          void (*lock[3]) (bool)= { &stepper.set_z_lock, &stepper.set_z2_lock, &stepper.set_z3_lock };
          float adj[3] = { 0, endstops.z2_endstop_adj, endstops.z3_endstop_adj };

          void (*tempLock) (bool);
          float tempAdj;

          // manual bubble sort by adjust value
          if (adj[1] < adj[0]) {
            tempLock = lock[0], tempAdj = adj[0];
            lock[0] = lock[1], adj[0] = adj[1];
            lock[1] = tempLock, adj[1] = tempAdj;
          }
          if (adj[2] < adj[1]) {
            tempLock = lock[1], tempAdj = adj[1];
            lock[1] = lock[2], adj[1] = adj[2];
            lock[2] = tempLock, adj[2] = tempAdj;
          }
          if (adj[1] < adj[0]) {
            tempLock = lock[0], tempAdj = adj[0];
            lock[0] = lock[1], adj[0] = adj[1];
            lock[1] = tempLock, adj[1] = tempAdj;
          }

          if (pos_dir) {
            // normalize adj to smallest value and do the first move
            (*lock[0])(true);
            do_homing_move(axis, adj[1] - adj[0]);
            // lock the second stepper for the final correction
            (*lock[1])(true);
            do_homing_move(axis, adj[2] - adj[1]);
          }
          else {
            (*lock[2])(true);
            do_homing_move(axis, adj[1] - adj[2]);
            (*lock[1])(true);
            do_homing_move(axis, adj[0] - adj[1]);
          }

          stepper.set_z_lock(false);
          stepper.set_z2_lock(false);
          stepper.set_z3_lock(false);
        }
      #elif ENABLED(Z_TWO_ENDSTOPS)
        if (axis == Z_AXIS) {
          const float adj = ABS(endstops.z2_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.z2_endstop_adj > 0) : (endstops.z2_endstop_adj < 0)) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
            do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_z_lock(false);
            stepper.set_z2_lock(false);
          }
        }
      #endif

      stepper.set_separate_multi_axis(false);

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

    // Clear retracted status if homing the Z axis
    #if ENABLED(FWRETRACT)
      if (axis == Z_AXIS) fwretract.current_hop = 0.0;
    #endif

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
        SERIAL_CHR(')');
        SERIAL_EOL();
      }
    #endif
  }

  #if ENABLED(QUICK_HOME)

    void Cartesian_Mechanics::quick_home_xy() {

      // Pretend the current position is 0,0
      current_position[X_AXIS] = current_position[Y_AXIS] = 0;
      sync_plan_position();

      #if ENABLED(DUAL_X_CARRIAGE)
        const int x_axis_home_dir = x_home_dir(tools.active_extruder);
      #else
        const int x_axis_home_dir = home_dir.X;
      #endif

      const float mlx = max_length[X_AXIS],
                  mly = max_length[Y_AXIS],
                  mlratio = mlx > mly ? mly / mlx : mlx / mly,
                  fr_mm_s = MIN(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * SQRT(sq(mlratio) + 1.0);

      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(X_AXIS);
        sensorless_homing_per_axis(Y_AXIS);
      #endif

      do_blocking_move_to_xy(1.5f * mlx * x_axis_home_dir, 1.5f * mly * home_dir.Y, fr_mm_s);

      endstops.validate_homing_move();

      current_position[X_AXIS] = current_position[Y_AXIS] = 0.0f;

      #if ENABLED(SENSORLESS_HOMING)
        sensorless_homing_per_axis(X_AXIS, false);
        sensorless_homing_per_axis(Y_AXIS, false);
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

      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("Z_SAFE_HOMING >>>");
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

      if (mechanics.position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_FEATURE)
          if (printer.debugFeature()) DEBUG_POS("Z_SAFE_HOMING", destination);
        #endif

        // This causes the carriage on Dual X to unpark
        #if ENABLED(DUAL_X_CARRIAGE)
          active_extruder_parked = false;
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

      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("<<< Z_SAFE_HOMING");
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

      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("DOUBLE_Z_HOMING >>>");
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

      if (mechanics.position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

        #if ENABLED(DEBUG_FEATURE)
          if (printer.debugFeature()) DEBUG_POS("DOUBLE_Z_HOMING", destination);
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

      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM("<<< DOUBLE_Z_HOMING");
      #endif
    }

  #endif

#endif // IS_CARTESIAN
