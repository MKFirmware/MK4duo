/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if IS_CORE

Core_Mechanics mechanics;

/** Public Parameters */
mechanics_data_t Core_Mechanics::data;

/** Private Parameters */
constexpr float slop = 0.0001;

/** Public Function */
void Core_Mechanics::factory_parameters() {

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

  // Base min pos
  data.base_pos[X_AXIS].min       = X_MIN_POS;
  data.base_pos[Y_AXIS].min       = Y_MIN_POS;
  data.base_pos[Z_AXIS].min       = Z_MIN_POS;

  // Base max pos
  data.base_pos[X_AXIS].max       = X_MAX_POS;
  data.base_pos[Y_AXIS].max       = Y_MAX_POS;
  data.base_pos[Z_AXIS].max       = Z_MAX_POS;

  // Base home pos
  data.base_home_pos[X_AXIS]      = X_HOME_POS;
  data.base_home_pos[Y_AXIS]      = Y_HOME_POS;
  data.base_home_pos[Z_AXIS]      = Z_HOME_POS;

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
    ZERO(data.home_offset);
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
void Core_Mechanics::get_cartesian_from_steppers() {
  cartesian_position[X_AXIS] = planner.get_axis_position_mm(X_AXIS);
  cartesian_position[Y_AXIS] = planner.get_axis_position_mm(Y_AXIS);
  cartesian_position[Z_AXIS] = planner.get_axis_position_mm(Z_AXIS);
}

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void Core_Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s /*=0.0*/) {

  if (printer.debugFeature()) DEBUG_XYZ(PSTR(">>> do_blocking_move_to"), nullptr, rx, ry, rz);

  const float z_feedrate  = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS],
              xy_feedrate = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

  // If Z needs to raise, do it before moving XY
  if (current_position[Z_AXIS] < rz) {
    current_position[Z_AXIS] = rz;
    line_to_current_position(z_feedrate);
  }

  current_position[X_AXIS] = rx;
  current_position[Y_AXIS] = ry;
  line_to_current_position(xy_feedrate);

  // If Z needs to lower, do it after moving XY
  if (current_position[Z_AXIS] > rz) {
    current_position[Z_AXIS] = rz;
    line_to_current_position(z_feedrate);
  }

  if (printer.debugFeature()) DEBUG_EM("<<< do_blocking_move_to");

  planner.synchronize();

}
void Core_Mechanics::do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void Core_Mechanics::do_blocking_move_to_z(const float &rz, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], rz, fr_mm_s);
}
void Core_Mechanics::do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, ry, current_position[Z_AXIS], fr_mm_s);
}

/**
 * Home Core
 */
void Core_Mechanics::home(const bool homeX/*=false*/, const bool homeY/*=false*/, const bool homeZ/*=false*/) {

  if (printer.debugSimulation()) {
    LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
    #if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)
      Nextion_gfx_clear();
    #endif
    return;
  }

  #if HAS_POWER_SWITCH
    powerManager.power_on(); // Power On if power is off
  #endif

  // Wait for planner moves to finish!
  planner.synchronize();

  // Cancel the active G29 session
  #if HAS_LEVELING && ENABLED(PROBE_MANUALLY)
    bedlevel.flag.g29_in_progress = false;
  #endif

  // Disable the leveling matrix before homing
  #if HAS_LEVELING
    const bool leveling_was_active = bedlevel.flag.leveling_active;
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

  setup_for_endstop_or_probe_move();
  endstops.setEnabled(true); // Enable endstops for next homing move

  bool come_back = parser.boolval('B');
  REMEMBER(fr, feedrate_mm_s);
  COPY_ARRAY(stored_position[1], current_position);

  const bool  home_all  = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ),
              doX       = home_all || homeX,
              doY       = home_all || homeY,
              doZ       = home_all || homeZ;

  set_destination_to_current();

  #if Z_HOME_DIR > 0  // If homing away from BED do Z first
    if (doZ) homeaxis(Z_AXIS);
  #endif

  const float z_homing_height = home_flag.ZHomed ? MIN_Z_HEIGHT_FOR_HOMING : 0;

  if (z_homing_height && (doX || doY)) {
    // Raise Z before homing any other axes and z is not already high enough (never lower z)
    destination[Z_AXIS] = z_homing_height;
    if (destination[Z_AXIS] > current_position[Z_AXIS]) {
      if (printer.debugFeature()) DEBUG_EMV("Raise Z (before homing) to ", destination[Z_AXIS]);
      do_blocking_move_to_z(destination[Z_AXIS]);
    }
  }

  #if ENABLED(QUICK_HOME)
    if (doX && doY) quick_home_xy();
  #endif

  #if ENABLED(HOME_Y_BEFORE_X)
    // Home Y (before X)
    if (doY) homeaxis(Y_AXIS);
  #endif

  // Home X
  if (doX) homeaxis(X_AXIS);

  #if DISABLED(HOME_Y_BEFORE_X)
    // Home Y (after X)
    if (doY) homeaxis(Y_AXIS);
  #endif

  // Home Z last if homing towards the bed
  #if Z_HOME_DIR < 0
    if (doZ) {
      #if ENABLED(BLTOUCH)
        bltouch.init();
      #endif
      #if ENABLED(Z_SAFE_HOMING)
        home_z_safely();
      #else
        homeaxis(Z_AXIS);
      #endif // !Z_SAFE_HOMING

      #if HOMING_Z_WITH_PROBE && Z_PROBE_AFTER_PROBING > 0
        probe.move_z_after_probing();
      #endif

    } // doZ

  #elif ENABLED(DOUBLE_Z_HOMING)
    if (doZ) double_home_z();
  #endif

  sync_plan_position();
  endstops.setNotHoming();

  if (come_back) {
    feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
    COPY_ARRAY(destination, stored_position[1]);
    prepare_move_to_destination();
    RESTORE(fr);
  }

  #if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)
    Nextion_gfx_clear();
  #endif

  #if HAS_LEVELING
    bedlevel.set_bed_leveling_enabled(leveling_was_active);
  #endif

  clean_up_after_endstop_or_probe_move();

  planner.synchronize();

  // Restore the active tool after homing
  #if HOTENDS > 1
    tools.change(old_tool_index, 0, true);
  #endif

  lcdui.refresh();

  report_current_position();

  if (printer.debugFeature()) DEBUG_EM("<<< G28");

}

/**
 * Home an individual linear axis
 */
void Core_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

  if (printer.debugFeature()) {
    DEBUG_MV(">>> do_homing_move(", axis_codes[axis]);
    DEBUG_MV(", ", distance);
    DEBUG_MSG(", ");
    if (fr_mm_s)
      DEBUG_VAL(fr_mm_s);
    else {
      DEBUG_MV(" [", homing_feedrate_mm_s[axis]);
      DEBUG_CHR(']');
    }
    DEBUG_CHR(')');
    DEBUG_EOL();
  }

  // Only do some things when moving towards an endstop
  const bool is_home_dir = (get_homedir(axis) > 0) == (distance > 0);

  #if ENABLED(SENSORLESS_HOMING)
    sensorless_t stealth_states;
  #endif

  if (is_home_dir) {

    if (axis == Z_AXIS) {
      #if HOMING_Z_WITH_PROBE
        #if ENABLED(BLTOUCH)
          bltouch.deploy();
        #endif
        #if QUIET_PROBING
          probe.probing_pause(true);
        #endif
      #endif
    }

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      stealth_states = start_sensorless_homing_per_axis(axis);
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
          bltouch.stow();
        #endif
      #endif
    }

    endstops.validate_homing_move();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      stop_sensorless_homing_per_axis(axis, stealth_states);
    #endif
  }

  if (printer.debugFeature()) {
    DEBUG_MV("<<< do_homing_move(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

}

/**
 * Prepare a linear move in a Cartesian setup.
 *
 * When a mesh-based leveling system is active, moves are segmented
 * according to the configuration of the leveling system.
 *
 * Returns true if current_position[] was set to destination[]
 */
bool Core_Mechanics::prepare_move_to_destination_mech_specific() {

  #if ENABLED(LASER) && ENABLED(LASER_FIRE_E)
    if (current_position[E_AXIS] < destination[E_AXIS] && ((current_position[X_AXIS] != destination [X_AXIS]) || (current_position[Y_AXIS] != destination [Y_AXIS])))
      laser.status = LASER_ON;
    else
      laser.status = LASER_OFF;
  #endif

  #if HAS_MESH
    if (bedlevel.flag.leveling_active && bedlevel.leveling_active_at_z(destination[Z_AXIS])) {
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

  buffer_line_to_destination(MMS_SCALED(feedrate_mm_s));
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
void Core_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

  if (printer.debugFeature()) {
    DEBUG_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

  setAxisHomed(axis, true);

  #if ENABLED(WORKSPACE_OFFSETS)
    position_shift[axis] = 0;
    endstops.update_software_endstops(axis);
  #endif

  current_position[axis] = data.base_home_pos[axis];

  /**
   * Z Probe Z Homing? Account for the probe's Z offset.
   */
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS) {
      current_position[Z_AXIS] -= probe.data.offset[Z_AXIS];
      if (printer.debugFeature()) {
        DEBUG_EM("*** Z HOMED WITH PROBE ***");
        DEBUG_EMV("zprobe_zoffset = ", probe.data.offset[Z_AXIS]);
      }
    }
  #endif

  #if ENABLED(BABYSTEPPING) && ENABLED(BABYSTEP_DISPLAY_TOTAL)
    babystep.reset_total(axis);
  #endif

  if (printer.debugFeature()) {
    #if ENABLED(WORKSPACE_OFFSETS)
      DEBUG_MV("> data.home_offset[", axis_codes[axis]);
      DEBUG_EMV("] = ", data.home_offset[axis]);
    #endif
    DEBUG_POS("", current_position);
    DEBUG_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

}

// Return true if the given position is within the machine bounds.
bool Core_Mechanics::position_is_reachable(const float &rx, const float &ry) {
  if (!WITHIN(ry, data.base_pos[Y_AXIS].min - slop, data.base_pos[Y_AXIS].max + slop)) return false;
  return WITHIN(rx, data.base_pos[X_AXIS].min - slop, data.base_pos[X_AXIS].max + slop);
}
// Return whether the given position is within the bed, and whether the nozzle
//  can reach the position required to put the probe at the given position.
bool Core_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
  return position_is_reachable(rx - probe.data.offset[X_AXIS], ry - probe.data.offset[Y_AXIS])
      && WITHIN(rx, MIN_PROBE_X - slop, MAX_PROBE_X + slop)
      && WITHIN(ry, MIN_PROBE_Y - slop, MAX_PROBE_Y + slop);
}

// Report detail current position to host
void Core_Mechanics::report_current_position_detail() {

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
    SERIAL_VAL(stepper.position((AxisEnum)i));
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

#if DISABLED(DISABLE_M503)

  void Core_Mechanics::print_parameters() {
    print_M92();
    print_M203();
    print_M201();
    print_M204();
    print_M205();
    print_M206();
    print_M228();
  }

  void Core_Mechanics::print_M92() {
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
  }

  void Core_Mechanics::print_M201() {
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
  }

  void Core_Mechanics::print_M203() {
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
  }

  void Core_Mechanics::print_M204() {
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
  }

  void Core_Mechanics::print_M205() {
    SERIAL_LM(CFG, "Advanced: B<DEFAULT_MIN_SEGMENT_TIME> S<DEFAULT_MIN_FEEDRATE> V<DEFAULT_MIN_TRAVEL_FEEDRATE>");
    SERIAL_SMV(CFG, "  M205 B", data.min_segment_time_us);
    SERIAL_MV(" S", LINEAR_UNIT(data.min_feedrate_mm_s), 3);
    SERIAL_EMV(" V", LINEAR_UNIT(data.min_travel_feedrate_mm_s), 3);

    #if ENABLED(JUNCTION_DEVIATION)
      SERIAL_LM(CFG, "Junction Deviation: J<JUNCTION_DEVIATION_MM>");
      SERIAL_LMV(CFG, "  M205 J", data.junction_deviation_mm, 2);
    #else
      SERIAL_SM(CFG, "Jerk: X<DEFAULT_XJERK> Y<DEFAULT_YJERK> Z<DEFAULT_ZJERK>");
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
  }

  void Core_Mechanics::print_M206() {
    #if ENABLED(WORKSPACE_OFFSETS)
      SERIAL_LM(CFG, "Home offset:");
      SERIAL_SMV(CFG, "  M206 X", LINEAR_UNIT(data.home_offset[X_AXIS]), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.home_offset[Y_AXIS]), 3);
      SERIAL_EMV(" Z", LINEAR_UNIT(data.home_offset[Z_AXIS]), 3);
    #endif
  }

  void Core_Mechanics::print_M228() {
    SERIAL_LM(CFG, "Set axis max travel:");
    SERIAL_SMV(CFG, "  M228 S0 X", LINEAR_UNIT(data.base_pos[X_AXIS].max), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.base_pos[Y_AXIS].max), 3);
    SERIAL_EMV(" Z", LINEAR_UNIT(data.base_pos[Z_AXIS].max), 3);
    SERIAL_LM(CFG, "Set axis min travel:");
    SERIAL_SMV(CFG, "  M228 S1 X", LINEAR_UNIT(data.base_pos[X_AXIS].min), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.base_pos[Y_AXIS].min), 3);
    SERIAL_EMV(" Z", LINEAR_UNIT(data.base_pos[Z_AXIS].min), 3);
  }

#endif // DISABLED(DISABLE_M503)

#if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)

  void Core_Mechanics::Nextion_gfx_clear() {
    gfx_clear(X_MAX_BED, Y_MAX_BED, Z_MAX_BED);
    gfx_cursor_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
  }

#endif

/** Private Function */
void Core_Mechanics::homeaxis(const AxisEnum axis) {

  #define CAN_HOME(A) \
    (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
  if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

  if (printer.debugFeature()) {
    DEBUG_MV(">>> homeaxis(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

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
  if (printer.debugFeature()) DEBUG_EM("Home 1 Fast:");

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    // BLTOUCH needs to be deployed every time
    if (axis == Z_AXIS && bltouch.deploy()) return;
  #endif

  do_homing_move(axis, 1.5f * data.base_pos[axis].max * get_homedir(axis));

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    // BLTOUCH needs to be deployed every time
    if (axis == Z_AXIS) bltouch.stow();
  #endif

  // When homing Z with probe respect probe clearance
  const float bump = get_homedir(axis) * (
    #if HOMING_Z_WITH_PROBE
      (axis == Z_AXIS) ? MAX(Z_PROBE_BETWEEN_HEIGHT, home_bump_mm[Z_AXIS]) :
    #endif
    home_bump_mm[axis]
  );

  // If a second homing move is configured...
  if (bump) {
    // Move away from the endstop by the axis HOME_BUMP_MM
    if (printer.debugFeature()) DEBUG_EM("Move Away:");
    do_homing_move(axis, -bump
      #if HOMING_Z_WITH_PROBE
        , axis == Z_AXIS ? MMM_TO_MMS(Z_PROBE_SPEED_FAST) : 0.0
      #endif
    );

    // Slow move towards endstop until triggered
    if (printer.debugFeature()) DEBUG_EM("Home 2 Slow:");

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS && bltouch.deploy()) return;
    #endif

    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));

    #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS) bltouch.stow();
    #endif
  }

  #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
    const bool pos_dir = get_homedir(axis) > 0;
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

  if (printer.debugFeature()) {
    DEBUG_MV("<<< homeaxis(", axis_codes[axis]);
    DEBUG_CHR(')');
    DEBUG_EOL();
  }

}

#if ENABLED(QUICK_HOME)

  void Core_Mechanics::quick_home_xy() {

    // Pretend the current position is 0,0
    current_position[X_AXIS] = current_position[Y_AXIS] = 0;
    sync_plan_position();

    const float mlratio = data.base_pos[X_AXIS].max > data.base_pos[Y_AXIS].max ? data.base_pos[Y_AXIS].max / data.base_pos[X_AXIS].max : data.base_pos[X_AXIS].max / data.base_pos[Y_AXIS].max,
                fr_mm_s = MIN(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * SQRT(sq(mlratio) + 1.0);

    #if ENABLED(SENSORLESS_HOMING)
      sensorless_t stealth_states;
      stealth_states.x = tmc.enable_stallguard(stepperX);
      stealth_states.y = tmc.enable_stallguard(stepperY);
    #endif

    do_blocking_move_to_xy(1.5f * data.base_pos[X_AXIS].max * x_axis_home_dir, 1.5f * data.base_pos[Y_AXIS].max * home_dir.Y, fr_mm_s);

    endstops.validate_homing_move();

    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0f;

    #if ENABLED(SENSORLESS_HOMING)
      tmc.disable_stallguard(stepperX, stealth_states.x);
      tmc.disable_stallguard(stepperY, stealth_states.y);
    #endif
  }

#endif // QUICK_HOME

#if ENABLED(Z_SAFE_HOMING)

  void Core_Mechanics::home_z_safely() {

    // Disallow Z homing if X or Y are unknown
    if (!home_flag.XHomed || !home_flag.YHomed) {
      LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
      SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
      return;
    }

    if (printer.debugFeature()) DEBUG_EM("Z_SAFE_HOMING >>>");

    sync_plan_position();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     */
    destination[X_AXIS] = Z_SAFE_HOMING_X_POINT;
    destination[Y_AXIS] = Z_SAFE_HOMING_Y_POINT;
    destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

    #if HOMING_Z_WITH_PROBE
      destination[X_AXIS] -= probe.data.offset[X_AXIS];
      destination[Y_AXIS] -= probe.data.offset[Y_AXIS];
    #endif

    if (position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

      if (printer.debugFeature()) DEBUG_POS("Z_SAFE_HOMING", destination);

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

    if (printer.debugFeature()) DEBUG_EM("<<< Z_SAFE_HOMING");
  }

#endif // Z_SAFE_HOMING

#if ENABLED(DOUBLE_Z_HOMING)

  void Core_Mechanics::double_home_z() {

    // Disallow Z homing if X or Y are unknown
    if (!home_flag.XHomed || !home_flag.YHomed) {
      LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
      SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
      return;
    }

    if (printer.debugFeature()) DEBUG_EM("DOUBLE_Z_HOMING >>>");

    sync_plan_position();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     */
    destination[X_AXIS] = DOUBLE_Z_HOMING_X_POINT;
    destination[Y_AXIS] = DOUBLE_Z_HOMING_Y_POINT;
    destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

    #if HAS_BED_PROBE
      destination[X_AXIS] -= probe.data.offset[X_AXIS];
      destination[Y_AXIS] -= probe.data.offset[Y_AXIS];
    #endif

    if (position_is_reachable(destination[X_AXIS], destination[Y_AXIS])) {

      if (printer.debugFeature()) DEBUG_POS("DOUBLE_Z_HOMING", destination);

      const float newzero = probe_pt(destination[X_AXIS], destination[Y_AXIS], true, 1) - (2 * probe.data.offset[Z_AXIS]);
      current_position[Z_AXIS] -= newzero;
      destination[Z_AXIS] = current_position[Z_AXIS];
      endstops.soft_endstop[Z_AXIS].max = data.base_pos[Z_AXIS].max - newzero;

      sync_plan_position();
      do_blocking_move_to_z(MIN_Z_HEIGHT_FOR_HOMING);
    }
    else {
      LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      SERIAL_LM(ECHO, MSG_ZPROBE_OUT);
    }

    if (printer.debugFeature()) DEBUG_EM("<<< DOUBLE_Z_HOMING");
  }

#endif // ENABLED(DOUBLE_Z_HOMING)

#endif // IS_CORE
