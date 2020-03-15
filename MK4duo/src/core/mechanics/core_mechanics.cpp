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
 * core_mechanics.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if IS_CORE

Core_Mechanics mechanics;

/** Public Parameters */
mechanics_data_t Core_Mechanics::data;

/** Private Parameters */
constexpr float fslop = 0.0001;

/** Public Function */
void Core_Mechanics::factory_parameters() {

  static const float    tmp_step[]          PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT,
                        tmp_maxfeedrate[]   PROGMEM = DEFAULT_MAX_FEEDRATE;

  static const uint32_t tmp_maxacc[]        PROGMEM = DEFAULT_MAX_ACCELERATION;

  LOOP_XYZ(axis) {
    data.axis_steps_per_mm[axis]          = pgm_read_float(&tmp_step[axis < COUNT(tmp_step) ? axis : COUNT(tmp_step) - 1]);
    data.max_feedrate_mm_s[axis]          = pgm_read_float(&tmp_maxfeedrate[axis < COUNT(tmp_maxfeedrate) ? axis : COUNT(tmp_maxfeedrate) - 1]);
    data.max_acceleration_mm_per_s2[axis] = pgm_read_dword_near(&tmp_maxacc[axis < COUNT(tmp_maxacc) ? axis : COUNT(tmp_maxacc) - 1]);
  }

  // Base pos
  data.base_pos.min.set(X_MIN_POS, Y_MIN_POS, Z_MIN_POS);
  data.base_pos.max.set(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);

  data.acceleration               = DEFAULT_ACCELERATION;
  data.travel_acceleration        = DEFAULT_TRAVEL_ACCELERATION;
  data.min_feedrate_mm_s          = DEFAULT_MIN_FEEDRATE;
  data.min_segment_time_us        = DEFAULT_MIN_SEGMENT_TIME;
  data.min_travel_feedrate_mm_s   = DEFAULT_MIN_TRAVEL_FEEDRATE;

  #if ENABLED(JUNCTION_DEVIATION)
    data.junction_deviation_mm = float(JUNCTION_DEVIATION_MM);
  #else
    data.max_jerk.set(DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK);
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
 * suitable for position.x, etc.
 */
void Core_Mechanics::get_cartesian_from_steppers() {
  cartesian_position.set(planner.get_axis_position_mm(X_AXIS), planner.get_axis_position_mm(Y_AXIS), planner.get_axis_position_mm(Z_AXIS));
}

void Core_Mechanics::internal_move_to_destination(const feedrate_t &fr_mm_s/*=0.0f*/) {

  REMEMBER(old_fr, feedrate_mm_s);
  if (fr_mm_s) feedrate_mm_s = fr_mm_s;

  REMEMBER(old_pct, feedrate_percentage, 100);
  REMEMBER(old_fac, extruders[toolManager.extruder.active]->e_factor, 1.0f);

  prepare_move_to_destination();
}

/**
 *  Plan a move to (X, Y, Z) and set the position
 */
void Core_Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const feedrate_t &fr_mm_s/*=0.0f*/) {

  if (printer.debugFeature()) DEBUG_XYZ(">>> do_blocking_move_to", rx, ry, rz);

  const feedrate_t  z_feedrate  = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s.z,
                    xy_feedrate = fr_mm_s ? fr_mm_s : feedrate_t(XY_PROBE_FEEDRATE_MM_S);

  // If Z needs to raise, do it before moving XY
  if (position.z < rz) {
    position.z = rz;
    line_to_position(z_feedrate);
  }

  position.set(rx, ry);
  line_to_position(xy_feedrate);

  // If Z needs to lower, do it after moving XY
  if (position.z > rz) {
    position.z = rz;
    line_to_position(z_feedrate);
  }

  if (printer.debugFeature()) DEBUG_EM("<<< do_blocking_move_to");

  planner.synchronize();

}
void Core_Mechanics::do_blocking_move_to(const xy_pos_t &raw, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(raw.x, raw.y, position.z, fr_mm_s);
}
void Core_Mechanics::do_blocking_move_to(const xyz_pos_t &raw, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(raw.x, raw.y, raw.z, fr_mm_s);
}
void Core_Mechanics::do_blocking_move_to(const xyze_pos_t &raw, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(raw.x, raw.y, raw.z, fr_mm_s);
}

void Core_Mechanics::do_blocking_move_to_x(const float &rx, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(rx, position.y, position.z, fr_mm_s);
}
void Core_Mechanics::do_blocking_move_to_y(const float &ry, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(position.x, ry, position.z, fr_mm_s);
}
void Core_Mechanics::do_blocking_move_to_z(const float &rz, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(position.x, position.y, rz, fr_mm_s);
}

void Core_Mechanics::do_blocking_move_to_xy(const float &rx, const float &ry, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(rx, ry, position.z, fr_mm_s);
}
void Core_Mechanics::do_blocking_move_to_xy(const xy_pos_t &raw, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(raw.x, raw.y, position.z, fr_mm_s);
}

void Core_Mechanics::do_blocking_move_to_xy_z(const xy_pos_t &raw, const float &z, const feedrate_t &fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(raw.x, raw.y, z, fr_mm_s);
}

/**
 * Home Core
 */
void Core_Mechanics::home(uint8_t axis_bits/*=0*/) {

  if (printer.debugSimulation()) {
    LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
    #if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)
      nextion_gfx_clear();
    #endif
    return;
  }

  #if HAS_POWER_SWITCH
    powerManager.power_on(); // Power On if power is off
  #endif

  // Wait for planner moves to finish!
  planner.synchronize();

  // Cancel the active G29 session
  #if HAS_LEVELING && HAS_PROBE_MANUALLY
    bedlevel.flag.g29_in_progress = false;
  #endif

  // Disable the leveling matrix before homing
  #if HAS_LEVELING
    bedlevel.set_bed_leveling_enabled(false);
  #endif

  #if ENABLED(CNC_WORKSPACE_PLANES)
    workspace_plane = PLANE_XY;
  #endif

  // Reduce Acceleration and Jerk for Homing
  #if ENABLED(SLOW_HOMING) || ENABLED(IMPROVE_HOMING_RELIABILITY)
    REMEMBER(accel_x, data.max_acceleration_mm_per_s2.x, 100);
    REMEMBER(accel_y, data.max_acceleration_mm_per_s2.y, 100);
    #if HAS_CLASSIC_JERK
      REMEMBER(jerk_x, data.max_jerk.x, 0);
      REMEMBER(jerk_y, data.max_jerk.y, 0);
    #endif
    planner.reset_acceleration_rates();
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    const uint8_t old_tool_index = toolManager.extruder.active;
    toolManager.change(0, true);
  #endif

  setup_for_endstop_or_probe_move();
  endstops.setEnabled(true); // Enable endstops for next homing move

  bool come_back = parser.boolval('B');
  REMEMBER(fr, feedrate_mm_s);
  stored_position[0] = position;

  const bool  homeX = TEST(axis_bits, X_AXIS),
              homeY = TEST(axis_bits, Y_AXIS),
              homeZ = TEST(axis_bits, Z_AXIS),
              home_all  = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ),
              doX       = home_all || homeX,
              doY       = home_all || homeY,
              doZ       = home_all || homeZ;

  destination = position;

  #if Z_HOME_DIR > 0  // If homing away from BED do Z first
    if (doZ) homeaxis(Z_AXIS);
  #endif

  const float z_homing_height = !home_flag.ZHomed ? 0 :
    (parser.seenval('R') ? parser.value_linear_units() : MIN_Z_HEIGHT_FOR_HOMING);

  if (z_homing_height && (doX || doY)) {
    // Raise Z before homing any other axes and z is not already high enough (never lower z)
    destination.z = z_homing_height;
    if (destination.z > position.z) {
      if (printer.debugFeature()) DEBUG_EMV("Raise Z (before homing) to ", destination.z);
      do_blocking_move_to_z(destination.z);
    }
  }

  #if ENABLED(QUICK_HOME)
    if (doX && doY) quick_home_xy();
  #endif

  #if ENABLED(HOME_Y_BEFORE_X)
    // Home Y (before X)
    if (doY || doX) homeaxis(Y_AXIS);
  #endif

  // Home X
  if (doX) homeaxis(X_AXIS);

  #if DISABLED(HOME_Y_BEFORE_X)
    // Home Y (after X)
    if (doY) homeaxis(Y_AXIS);
  #endif

  #if ENABLED(SLOW_HOMING) || ENABLED(IMPROVE_HOMING_RELIABILITY)
    RESTORE(accel_x);
    RESTORE(accel_y);
    #if HAS_CLASSIC_JERK
      RESTORE(jerk_x);
      RESTORE(jerk_y);
    #endif
    planner.reset_acceleration_rates();
  #endif

  // Home Z last if homing towards the bed
  #if Z_HOME_DIR < 0
    if (doZ) {
      #if HAS_BLTOUCH
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
    feedrate_mm_s = homing_feedrate_mm_s.x;
    destination = stored_position[0];
    prepare_move_to_destination();
  }

  #if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)
    nextion_gfx_clear();
  #endif

  // Re-enable bed level correction if it had been on
  #if HAS_LEVELING
    bedlevel.restore_bed_leveling_state();
  #endif

  clean_up_after_endstop_or_probe_move();

  planner.synchronize();

  // Restore the active tool after homing
  #if HOTENDS > 1
    toolManager.change(old_tool_index, true);
  #endif

  lcdui.refresh();

  report_position();

  if (printer.debugFeature()) DEBUG_EM("<<< G28");

}

/**
 * Home an individual linear axis
 */
void Core_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const feedrate_t fr_mm_s/*=0.0f*/) {

  if (printer.debugFeature()) {
    DEBUG_MC(">>> do_homing_move(", axis_codes[axis]);
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
    sensorless_flag_t stealth_states;
  #endif

  if (is_home_dir) {

    #if HOMING_Z_WITH_PROBE && QUIET_PROBING
      if (axis == Z_AXIS) probe.set_paused(true);
    #endif

    // Disable stealthChop if used. Enable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      stealth_states = start_sensorless_homing_per_axis(axis);
    #endif
  }

  abce_pos_t target = { planner.get_axis_position_mm(A_AXIS), planner.get_axis_position_mm(B_AXIS), planner.get_axis_position_mm(C_AXIS), planner.get_axis_position_mm(E_AXIS) };
  target[axis] = 0;
  planner.set_machine_position_mm(target);
  target[axis] = distance;

  // Set cartesian axes directly
  planner.buffer_segment(target, fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], toolManager.extruder.active);

  planner.synchronize();

  if (is_home_dir) {

    #if HOMING_Z_WITH_PROBE && QUIET_PROBING
      if (axis == Z_AXIS) probe.set_paused(false);
    #endif

    endstops.validate_homing_move();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.
    #if ENABLED(SENSORLESS_HOMING)
      stop_sensorless_homing_per_axis(axis, stealth_states);
    #endif
  }

  if (printer.debugFeature()) {
    DEBUG_MC("<<< do_homing_move(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

}

/**
 * Prepare a linear move in a Cartesian setup.
 *
 * When a mesh-based leveling system is active, moves are segmented
 * according to the configuration of the leveling system.
 *
 * Returns true if position[] was set to destination[]
 */
bool Core_Mechanics::prepare_move_to_destination_mech_specific() {

  const float scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);

  #if ENABLED(LASER) && ENABLED(LASER_FIRE_E)
    if (position.e < destination.e && ((position.x != destination.x) || (position.y != destination.y)))
      laser.status = LASER_ON;
    else
      laser.status = LASER_OFF;
  #endif

  #if HAS_MESH
    if (bedlevel.flag.leveling_active && bedlevel.leveling_active_at_z(destination.z)) {
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        ubl.line_to_destination_cartesian(scaled_fr_mm_s, toolManager.extruder.active);
        return true;
      #else
        /**
         * For MBL and ABL-BILINEAR only segment moves when X or Y are involved.
         * Otherwise fall through to do a direct single move.
         */
        if (position.x != destination.x || position.y != destination.y) {
          #if ENABLED(MESH_BED_LEVELING)
            mbl.line_to_destination(scaled_fr_mm_s);
          #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
            abl.line_to_destination(scaled_fr_mm_s);
          #endif
          return true;
        }
      #endif
    }
  #endif // HAS_MESH

  planner.buffer_line(destination, scaled_fr_mm_s, toolManager.extruder.active);
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
    DEBUG_MC(">>> set_axis_is_at_home(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

  setAxisHomed(axis, true);

  #if ENABLED(WORKSPACE_OFFSETS)
    position_shift[axis] = 0;
    endstops.update_software_endstops(axis);
  #endif

  position[axis] = axis_home_pos(axis);

  /**
   * Z Probe Z Homing? Account for the probe's Z offset.
   */
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS) {
      position.z -= probe.data.offset.z;
      if (printer.debugFeature()) {
        DEBUG_EM("*** Z HOMED WITH PROBE ***");
        DEBUG_EMV("zprobe_zoffset = ", probe.data.offset.z);
      }
    }
  #endif

  #if ENABLED(BABYSTEPPING) && ENABLED(BABYSTEP_DISPLAY_TOTAL)
    babystep.reset_total(axis);
  #endif

  if (printer.debugFeature()) {
    #if ENABLED(WORKSPACE_OFFSETS)
      DEBUG_MC("> data.home_offset[", axis_codes[axis]);
      DEBUG_EMV("] = ", data.home_offset[axis]);
    #endif
    DEBUG_POS("", position);
    DEBUG_MC("<<< set_axis_is_at_home(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

}

float Core_Mechanics::axis_home_pos(const AxisEnum axis) {
  switch (axis) {
    case X_AXIS: return x_home_pos(); break;
    case Y_AXIS: return y_home_pos(); break;
    case Z_AXIS: return z_home_pos(); break;
    default: break;
  }
}

float Core_Mechanics::x_home_pos() {
  #if ENABLED(MANUAL_X_HOME_POS)
    return MANUAL_X_HOME_POS;
  #elif ENABLED(BED_CENTER_AT_0_0)
    return ((mechanics.data.base_pos.max.x - mechanics.data.base_pos.min.x) * (mechanics.get_homedir(X_AXIS)) * 0.5);
  #else
    return (mechanics.get_homedir(X_AXIS) < 0 ? mechanics.data.base_pos.min.x : mechanics.data.base_pos.max.x);
  #endif
}

float Core_Mechanics::y_home_pos() {
  #if ENABLED(MANUAL_Y_HOME_POS)
    return MANUAL_Y_HOME_POS;
  #elif ENABLED(BED_CENTER_AT_0_0)
    return ((mechanics.data.base_pos.max.y - mechanics.data.base_pos.min.y) * (mechanics.get_homedir(Y_AXIS)) * 0.5);
  #else
    return (mechanics.get_homedir(Y_AXIS) < 0 ? mechanics.data.base_pos.min.y : mechanics.data.base_pos.max.y);
  #endif
}

float Core_Mechanics::z_home_pos() {
  #if ENABLED(MANUAL_Z_HOME_POS)
    return MANUAL_Z_HOME_POS;
  #else
    return (mechanics.get_homedir(Z_AXIS) < 0 ? mechanics.data.base_pos.min.z : mechanics.data.base_pos.max.z);
  #endif
}

// Return true if the given position is within the machine bounds.
bool Core_Mechanics::position_is_reachable(const float &rx, const float &ry) {
  if (!WITHIN(ry, data.base_pos.min.y - fslop, data.base_pos.max.y + fslop)) return false;
  return WITHIN(rx, data.base_pos.min.x - fslop, data.base_pos.max.x + fslop);
}
// Return whether the given position is within the bed, and whether the nozzle
//  can reach the position required to put the probe at the given position.
bool Core_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
  return position_is_reachable(rx - probe.data.offset.x, ry - probe.data.offset.y)
      && WITHIN(rx, probe.min_x() - fslop, probe.max_x() + fslop)
      && WITHIN(ry, probe.min_y() - fslop, probe.max_y() + fslop);
}

// Report the real current position according to the steppers
void Core_Mechanics::report_real_position() {

  get_cartesian_from_steppers();

  #if HAS_POSITION_MODIFIERS
    xyze_pos_t npos = cartesian_position;
    planner.unapply_modifiers(npos
      #if HAS_LEVELING
        , true
      #endif
    );
  #else
    const xyze_pos_t &npos = cartesian_position;
  #endif

  xyze_pos_t lpos = npos.asLogical();
  lpos.e = planner.get_axis_position_mm(E_AXIS);
  report_some_position(lpos);

}

// Report detail current position to host
void Core_Mechanics::report_detail_position() {

  SERIAL_MSG("\nLogical:");
  report_xyz(position.asLogical());

  SERIAL_MSG("Raw:    ");
  report_xyz(position);

  xyze_pos_t leveled = position;

  #if HAS_LEVELING
    SERIAL_MSG("Leveled:");
    bedlevel.apply_leveling(leveled);
    report_xyz(leveled);

    SERIAL_MSG("UnLevel:");
    xyze_pos_t unleveled = leveled;
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
  get_cartesian_from_steppers();
  xyze_pos_t from_steppers = { cartesian_position.x, cartesian_position.y, cartesian_position.z, planner.get_axis_position_mm(E_AXIS) };
  report_xyze(from_steppers);

  const xyze_float_t diff = from_steppers - leveled;
  SERIAL_MSG("Differ: ");
  report_xyze(diff);

}

// Report the logical current position according to the most recent G-code command.
void Core_Mechanics::report_logical_position() {

  xyze_pos_t rpos = position;

  #if HAS_POSITION_MODIFIERS
    planner.apply_modifiers(rpos);
  #endif

  const abc_long_t spos = {
    int32_t(LROUND(rpos.a * mechanics.steps_to_mm.a)),
    int32_t(LROUND(rpos.b * mechanics.steps_to_mm.b)),
    int32_t(LROUND(rpos.c * mechanics.steps_to_mm.c))
  };

  report_some_position(position.asLogical());
  stepper.report_positions(spos);

}

#if DISABLED(DISABLE_M503)

  void Core_Mechanics::print_parameters() {
    print_M92();
    print_M201();
    print_M203();
    print_M204();
    print_M205();
    print_M206();
    print_M228();
  }

  void Core_Mechanics::print_M92() {
    SERIAL_LM(CFG, "Steps per unit:");
    SERIAL_SMV(CFG, "  M92 X", LINEAR_UNIT(data.axis_steps_per_mm.x), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.axis_steps_per_mm.y), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(data.axis_steps_per_mm.z), 3);
    SERIAL_EOL();
    LOOP_EXTRUDER() {
      SERIAL_SMV(CFG, "  M92 T", (int)e);
      SERIAL_EMV(" E", VOLUMETRIC_UNIT(extruders[e]->data.axis_steps_per_mm), 3);
    }
  }

  void Core_Mechanics::print_M201() {
    SERIAL_LM(CFG, "Maximum Acceleration (units/s2):");
    SERIAL_SMV(CFG, "  M201 X", LINEAR_UNIT(data.max_acceleration_mm_per_s2.x));
    SERIAL_MV(" Y", LINEAR_UNIT(data.max_acceleration_mm_per_s2.y));
    SERIAL_MV(" Z", LINEAR_UNIT(data.max_acceleration_mm_per_s2.z));
    SERIAL_EOL();
    LOOP_EXTRUDER() {
      SERIAL_SMV(CFG, "  M201 T", (int)e);
      SERIAL_EMV(" E", VOLUMETRIC_UNIT(extruders[e]->data.max_acceleration_mm_per_s2));
    }
  }

  void Core_Mechanics::print_M203() {
    SERIAL_LM(CFG, "Maximum feedrates (units/s):");
    SERIAL_SMV(CFG, "  M203 X", LINEAR_UNIT(data.max_feedrate_mm_s.x), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.max_feedrate_mm_s.y), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(data.max_feedrate_mm_s.z), 3);
    SERIAL_EOL();
    LOOP_EXTRUDER() {
      SERIAL_SMV(CFG, "  M203 T", (int)e);
      SERIAL_EMV(" E", VOLUMETRIC_UNIT(extruders[e]->data.max_feedrate_mm_s), 3);
    }
  }

  void Core_Mechanics::print_M204() {
    SERIAL_LM(CFG, "Acceleration (units/s2): P<DEFAULT_ACCELERATION> V<DEFAULT_TRAVEL_ACCELERATION> T* R<DEFAULT_RETRACT_ACCELERATION>");
    SERIAL_SMV(CFG,"  M204 P", LINEAR_UNIT(data.acceleration), 3);
    SERIAL_MV(" V", LINEAR_UNIT(data.travel_acceleration), 3);
    SERIAL_EOL();
    LOOP_EXTRUDER() {
      SERIAL_SMV(CFG, "  M204 T", (int)e);
      SERIAL_EMV(" R", LINEAR_UNIT(extruders[e]->data.retract_acceleration), 3);
    }
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

      SERIAL_SMV(CFG, "  M205 X", LINEAR_UNIT(data.max_jerk.x), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.max_jerk.y), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(data.max_jerk.z), 3);
      SERIAL_EOL();

      #if DISABLED(LIN_ADVANCE)
        LOOP_EXTRUDER() {
          SERIAL_SMV(CFG, "  M205 T", (int)e);
          SERIAL_EMV(" E" , LINEAR_UNIT(extruders[e]->data.max_jerk), 3);
        }
      #endif
    #endif
  }

  void Core_Mechanics::print_M206() {
    #if ENABLED(WORKSPACE_OFFSETS)
      SERIAL_LM(CFG, "Home offset:");
      SERIAL_SMV(CFG, "  M206 X", LINEAR_UNIT(data.home_offset.x), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.home_offset.y), 3);
      SERIAL_EMV(" Z", LINEAR_UNIT(data.home_offset.z), 3);
    #endif
  }

  void Core_Mechanics::print_M228() {
    SERIAL_LM(CFG, "Set axis max travel:");
    SERIAL_SMV(CFG, "  M228 S0 X", LINEAR_UNIT(data.base_pos.max.x), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.base_pos.max.y), 3);
    SERIAL_EMV(" Z", LINEAR_UNIT(data.base_pos.max.z), 3);
    SERIAL_LM(CFG, "Set axis min travel:");
    SERIAL_SMV(CFG, "  M228 S1 X", LINEAR_UNIT(data.base_pos.min.x), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.base_pos.min.y), 3);
    SERIAL_EMV(" Z", LINEAR_UNIT(data.base_pos.min.z), 3);
  }

#endif // DISABLED(DISABLE_M503)

#if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)

  void Core_Mechanics::nextion_gfx_clear() {
    nexlcd.gfx_clear(X_MAX_BED, Y_MAX_BED, Z_MAX_BED);
    nexlcd.gfx_cursor_to(position);
  }

#endif

/** Private Function */
void Core_Mechanics::homeaxis(const AxisEnum axis) {

  #define CAN_HOME(A) \
    (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
  if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

  if (printer.debugFeature()) {
    DEBUG_MC(">>> homeaxis(", axis_codes[axis]);
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

  #if HOMING_Z_WITH_PROBE && HAS_BLTOUCH
    if (axis == Z_AXIS && bltouch.deploy()) return; // The initial DEPLOY
  #endif

  do_homing_move(axis, 1.5f * data.base_pos.max[axis] * get_homedir(axis));

  #if HOMING_Z_WITH_PROBE && HAS_BLTOUCH && DISABLED(BLTOUCH_HIGH_SPEED_MODE)
    if (axis == Z_AXIS) bltouch.stow(); // Intermediate STOW (in LOW SPEED MODE)
  #endif

  // When homing Z with probe respect probe clearance
  const float bump = get_homedir(axis) * (
    #if HOMING_Z_WITH_PROBE
      (axis == Z_AXIS) ? MAX(Z_PROBE_BETWEEN_HEIGHT, home_bump_mm.z) :
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

    #if HOMING_Z_WITH_PROBE && HAS_BLTOUCH && DISABLED(BLTOUCH_HIGH_SPEED_MODE)
      // BLTOUCH needs to be deployed every time
      if (axis == Z_AXIS && bltouch.deploy()) return; // Intermediate DEPLOY (in LOW SPEED MODE)
    #endif

    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));

    #if HOMING_Z_WITH_PROBE && HAS_BLTOUCH
      if (axis == Z_AXIS) bltouch.stow(); // The final STOW
    #endif
  }

  #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
    const bool pos_dir = get_homedir(axis) > 0;
    #if ENABLED(X_TWO_ENDSTOPS)
      if (axis == X_AXIS) {
        const float adj = ABS(endstops.data.x2_endstop_adj);
        if (adj) {
          if (pos_dir ? (endstops.data.x2_endstop_adj > 0) : (endstops.data.x2_endstop_adj < 0)) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
          do_homing_move(axis, pos_dir ? -adj : adj);
          stepper.set_x_lock(false);
          stepper.set_x2_lock(false);
        }
      }
    #endif
    #if ENABLED(Y_TWO_ENDSTOPS)
      if (axis == Y_AXIS) {
        const float adj = ABS(endstops.data.y2_endstop_adj);
        if (adj) {
          if (pos_dir ? (endstops.data.y2_endstop_adj > 0) : (endstops.data.y2_endstop_adj < 0)) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
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
        float adj[3] = { 0, endstops.data.z2_endstop_adj, endstops.data.z3_endstop_adj };

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
        const float adj = ABS(endstops.data.z2_endstop_adj);
        if (adj) {
          if (pos_dir ? (endstops.data.z2_endstop_adj > 0) : (endstops.data.z2_endstop_adj < 0)) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
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

  destination[axis] = position[axis];

  // Put away the Z probe
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && STOW_PROBE()) return;
  #endif

  // Clear retracted status if homing the Z axis
  #if ENABLED(FWRETRACT)
    if (axis == Z_AXIS) fwretract.current_hop = 0.0;
  #endif

  if (printer.debugFeature()) {
    DEBUG_MC("<<< homeaxis(", axis_codes[axis]);
    DEBUG_CHR(')');
    DEBUG_EOL();
  }

}

#if ENABLED(QUICK_HOME)

  void Core_Mechanics::quick_home_xy() {

    // Pretend the current position is 0,0
    position.x = position.y = 0;
    sync_plan_position();

    const float mlratio = data.base_pos.max.x > data.base_pos.max.y ? data.base_pos.max.y / data.base_pos.max.x : data.base_pos.max.x / data.base_pos.max.y,
                fr_mm_s = MIN(homing_feedrate_mm_s.x, homing_feedrate_mm_s.y) * SQRT(sq(mlratio) + 1.0);

    #if ENABLED(SENSORLESS_HOMING)
      sensorless_flag_t stealth_states;
      stealth_states.x = tmcManager.enable_stallguard(driver.x);
      stealth_states.y = tmcManager.enable_stallguard(driver.y);
    #endif

    do_blocking_move_to_xy(1.5f * data.base_pos.max.x * home_dir.x, 1.5f * data.base_pos.max.y * home_dir.y, fr_mm_s);

    endstops.validate_homing_move();

    position.x = position.y = 0.0f;

    #if ENABLED(SENSORLESS_HOMING)
      tmcManager.disable_stallguard(driver.x, stealth_states.x);
      tmcManager.disable_stallguard(driver.y, stealth_states.y);
    #endif
  }

#endif // QUICK_HOME

#if ENABLED(Z_SAFE_HOMING)

  void Core_Mechanics::home_z_safely() {

    // Disallow Z homing if X or Y are unknown
    if (!home_flag.XHomed || !home_flag.YHomed) {
      LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
      SERIAL_LM(ECHO, MSG_HOST_ERR_Z_HOMING);
      return;
    }

    if (printer.debugFeature()) DEBUG_EM("home_z_safely >>>");

    sync_plan_position();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     * (Z is already at the right height)
     */
    destination.set(Z_SAFE_HOMING_X_POINT, Z_SAFE_HOMING_Y_POINT, position.z);

    #if HOMING_Z_WITH_PROBE
      destination -= probe.data.offset;
    #endif

    if (position_is_reachable(destination)) {

      if (printer.debugFeature()) DEBUG_POS("home_z_safely", destination);

      #if ENABLED(SENSORLESS_HOMING)
        HAL::delayMilliseconds(500);
      #endif

      do_blocking_move_to_xy(destination);
      homeaxis(Z_AXIS);
    }
    else {
      LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      SERIAL_LM(ECHO, MSG_HOST_ZPROBE_OUT);
    }

    if (printer.debugFeature()) DEBUG_EM("<<< home_z_safely");
  }

#endif // Z_SAFE_HOMING

#if ENABLED(DOUBLE_Z_HOMING)

  void Core_Mechanics::double_home_z() {

    // Disallow Z homing if X or Y are unknown
    if (!home_flag.XHomed || !home_flag.YHomed) {
      LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
      SERIAL_LM(ECHO, MSG_HOST_ERR_Z_HOMING);
      return;
    }

    if (printer.debugFeature()) DEBUG_EM("DOUBLE_Z_HOMING >>>");

    sync_plan_position();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     * Z is already at the right height
     */
    destination.set(DOUBLE_Z_HOMING_X_POINT, DOUBLE_Z_HOMING_Y_POINT, position.z);

    #if HAS_BED_PROBE
      destination -= probe.data.offset;
    #endif

    if (position_is_reachable(destination)) {

      if (printer.debugFeature()) DEBUG_POS("DOUBLE_Z_HOMING", destination);

      const float newzero = probe_pt(destination.x, destination.y, true, 1) - (2 * probe.data.offset.z);
      position.z -= newzero;
      destination.z = position.z;
      endstops.soft_endstop.max.z = data.base_pos.max.z - newzero;

      sync_plan_position();
      do_blocking_move_to_z(MIN_Z_HEIGHT_FOR_HOMING);
    }
    else {
      LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      SERIAL_LM(ECHO, MSG_HOST_ZPROBE_OUT);
    }

    if (printer.debugFeature()) DEBUG_EM("<<< DOUBLE_Z_HOMING");
  }

#endif // ENABLED(DOUBLE_Z_HOMING)

#endif // IS_CORE
