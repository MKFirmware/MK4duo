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
 * scara_mechanics.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if IS_SCARA

Scara_Mechanics mechanics;

/** Public Parameters */
mechanics_data_t Scara_Mechanics::data;

const float Scara_Mechanics::L1                   = SCARA_LINKAGE_1,
            Scara_Mechanics::L2                   = SCARA_LINKAGE_2,
            Scara_Mechanics::L1_2                 = sq(float(L1)),
            Scara_Mechanics::L1_2_2               = 2.0f * L1_2,
            Scara_Mechanics::L2_2                 = sq(float(L2));

float Scara_Mechanics::delta[ABC]                 = { 0.0 };

/** Public Function */
void Scara_Mechanics::factory_parameters() {

  static const float    tmp_step[]          PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT,
                        tmp_maxfeedrate[]   PROGMEM = DEFAULT_MAX_FEEDRATE;

  static const uint32_t tmp_maxacc[]        PROGMEM = DEFAULT_MAX_ACCELERATION;

  LOOP_XYZE_N(i) {
    data.axis_steps_per_mm[i]           = pgm_read_float(&tmp_step[i < COUNT(tmp_step) ? i : COUNT(tmp_step) - 1]);
    data.max_feedrate_mm_s[i]           = pgm_read_float(&tmp_maxfeedrate[i < COUNT(tmp_maxfeedrate) ? i : COUNT(tmp_maxfeedrate) - 1]);
    data.max_acceleration_mm_per_s2[i]  = pgm_read_dword_near(&tmp_maxacc[i < COUNT(tmp_maxacc) ? i : COUNT(tmp_maxacc) - 1]);
  }

  // Base min pos
  data.base_pos.min.x       = X_MIN_POS;
  data.base_pos.min.y       = Y_MIN_POS;
  data.base_pos.min.z       = Z_MIN_POS;

  // Base max pos
  data.base_pos.max.x       = X_MAX_POS;
  data.base_pos.max.y       = Y_MAX_POS;
  data.base_pos.max.z       = Z_MAX_POS;

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
  #endif

  static const float tmp_ejerk[] PROGMEM = DEFAULT_EJERK;
  data.max_jerk.x  = DEFAULT_XJERK;
  data.max_jerk.y  = DEFAULT_YJERK;
  data.max_jerk.z  = DEFAULT_ZJERK;
  #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
    LOOP_EXTRUDER()
      data.max_jerk[E_AXIS + e] = pgm_read_float(&tmp_ejerk[e < COUNT(tmp_ejerk) ? e : COUNT(tmp_ejerk) - 1]);
  #endif

  data.segments_per_second = SCARA_SEGMENTS_PER_SECOND;

  #if ENABLED(WORKSPACE_OFFSETS)
    ZERO(data.home_offset);
  #endif

}

/**
 * Get the stepper positions in the cartesian_position[] array.
 * Forward kinematics are applied for SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for position.x, etc.
 */
void Scara_Mechanics::get_cartesian_from_steppers() {
  InverseTransform(
    planner.get_axis_position_mm(A_AXIS),
    planner.get_axis_position_mm(B_AXIS),
    cartesian_position
  );
  cartesian_position.z = planner.get_axis_position_mm(Z_AXIS);
}

#if DISABLED(AUTO_BED_LEVELING_UBL)

  /**
   * Prepare a linear move in a SCARA setup.
   *
   * This calls buffer_line several times, adding
   * small incremental moves for SCARA.
   */
   bool Scara_Mechanics::prepare_move_to_destination_mech_specific() {

    // Get the top feedrate of the move in the XY plane
    const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    // Get the cartesian distances moved in XYZE
    const float difference[XYZE] = {
      destination.x - position.x,
      destination.y - position.y,
      destination.z - position.z,
      destination.e - position.e
    };

    // If the move is only in Z/E don't split up the move
    if (!difference[X_AXIS] && !difference[Y_AXIS]) {
      planner.buffer_line(destination, _feedrate_mm_s, toolManager.extruder.active);
      return false; // caller will update position.x
    }

    // Fail if attempting move outside printable radius
    if (endstops.isSoftEndstop() && !position_is_reachable(destination.x, destination.y)) return true;

    // Get the linear distance in XYZ
    float cartesian_mm = SQRT(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

    // If the move is very short, check the E move distance
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(difference[E_AXIS]);

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return true;

    // Minimum number of seconds to move the given distance
    const float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments we should produce
    uint16_t segments = data.segments_per_second * seconds;

    // For SCARA minimum segment size is 0.5mm
    NOMORE(segments, cartesian_mm * 2);

    // At least one segment is required
    NOLESS(segments, 1U);

    // The approximate length of each segment
    const float inv_segments = 1.0f / float(segments),
                segment_distance[XYZE] = {
                  difference[X_AXIS] * inv_segments,
                  difference[Y_AXIS] * inv_segments,
                  difference[Z_AXIS] * inv_segments,
                  difference[E_AXIS] * inv_segments
                };

    #if DISABLED(SCARA_FEEDRATE_SCALING)
      const float cartesian_segment_mm = cartesian_mm * inv_segments;
    #endif

    //DEBUG_MV("mm=", cartesian_mm);
    //DEBUG_MV(" seconds=", seconds);
    //DEBUG_MV(" segments=", segments);
    #if DISABLED(SCARA_FEEDRATE_SCALING)
      //DEBUG_MV(" segment_mm=", cartesian_segment_mm);
    #endif
    //DEBUG_EOL();

    #if ENABLED(SCARA_FEEDRATE_SCALING)
      // SCARA needs to scale the feed rate from mm/s to degrees/s
      // i.e., Complete the angular vector in the given time.
      const float segment_length = cartesian_mm * inv_segments,
                  inv_segment_length = 1.0f / segment_length, // 1/mm/segs
                  inverse_secs = inv_segment_length * _feedrate_mm_s;

      float oldA = planner.position_float[A_AXIS],
            oldB = planner.position_float[B_AXIS];
    #endif

    // Get the current position as starting point
    float raw[XYZE];
    COPY_ARRAY(raw, position.x);

    // Calculate and execute the segments
    while (--segments) {

      LOOP_XYZE(i) raw[i] += segment_distance[i];
      Transform(raw);

      // Adjust Z if bed leveling is enabled
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (bedlevel.flag.leveling_active) {
          const float zadj = abl.bilinear_z_offset(raw);
          delta[A_AXIS] += zadj;
          delta[B_AXIS] += zadj;
          delta[C_AXIS] += zadj;
        }
      #endif

      #if ENABLED(SCARA_FEEDRATE_SCALING)
        // For SCARA scale the feed rate from mm/s to degrees/s
        // i.e., Complete the angular vector in the given time.
        if (!planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], raw[Z_AXIS], raw[E_AXIS], HYPOT(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB) * inverse_secs, toolManager.extruder.active))
          break;
        oldA = delta[A_AXIS];
        oldB = delta[B_AXIS];
      #else
        if (!planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], raw[E_AXIS], _feedrate_mm_s, toolManager.extruder.active, cartesian_segment_mm))
          break;
      #endif

    }

    // Ensure last segment arrives at target location.
    #if ENABLED(SCARA_FEEDRATE_SCALING)
      // For SCARA scale the feed rate from mm/s to degrees/s
      // With segments > 1 length is 1 segment, otherwise total length
      Transform(destination);

      // Adjust Z if bed leveling is enabled
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (bedlevel.flag.leveling_active) {
          const float zadj = abl.bilinear_z_offset(raw);
          delta[A_AXIS] += zadj;
          delta[B_AXIS] += zadj;
          delta[C_AXIS] += zadj;
        }
      #endif
      const float diff2 = HYPOT2(delta[A_AXIS] - oldA, delta[B_AXIS] - oldB);
      if (diff2)
        planner.buffer_segment(delta[A_AXIS], delta[B_AXIS], destination.z, destination.e, SQRT(diff2) * inverse_secs, toolManager.extruder.active);
    #else
      planner.buffer_line(destination, _feedrate_mm_s, toolManager.extruder.active);
    #endif

    return false; // caller will update position.x
  }

#endif // DISABLED(AUTO_BED_LEVELING_UBL)

/**
 *  Plan a move to (X, Y, Z) and set the position.x
 *  The final position.x may not be the one that was requested
 */
void Scara_Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const feedrate_t &fr_mm_s /*=0.0*/) {

  if (printer.debugFeature()) DEBUG_XYZ(">>> do_blocking_move_to", rx, ry, rz);

  const feedrate_t  z_feedrate  = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s.z,
                    xy_feedrate = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

  if (!position_is_reachable(rx, ry)) return;

  destination = position;

  // If Z needs to raise, do it before moving XY
  if (destination.z < rz) {
    destination.z = rz;
    prepare_uninterpolated_move_to_destination(z_feedrate);
  }

  destination.x = rx;
  destination.y = ry;
  prepare_uninterpolated_move_to_destination(xy_feedrate);

  // If Z needs to lower, do it after moving XY
  if (destination.z > rz) {
    destination.z = rz;
    prepare_uninterpolated_move_to_destination(z_feedrate);
  }

  if (printer.debugFeature()) DEBUG_EM("<<< do_blocking_move_to");

  planner.synchronize();

}
void Scara_Mechanics::do_blocking_move_to_x(const float &rx, const feedrate_t &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, position.y, position.z, fr_mm_s);
}
void Scara_Mechanics::do_blocking_move_to_z(const float &rz, const feedrate_t &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(position.x, position.y, rz, fr_mm_s);
}
void Scara_Mechanics::do_blocking_move_to_xy(const float &rx, const float &ry, const feedrate_t &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, ry, position.z, fr_mm_s);
}

/**
 * SCARA InverseTransform. Results in cartesian[].
 * Maths and first version by QHARLEY.
 * Integrated into Marlin and slightly restructured by Joachim Cerny.
 */
void Scara_Mechanics::InverseTransform(const float Ha, const float Hb, float cartesian[XYZ]) {

  const float a_sin = sin(RADIANS(Ha)) * L1,
              a_cos = cos(RADIANS(Ha)) * L1,
              b_sin = sin(RADIANS(Hb)) * L2,
              b_cos = cos(RADIANS(Hb)) * L2;

  cartesian[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
  cartesian[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

}

/**
 * SCARA Transform. Results in delta[].
 *
 * See http://forums.reprap.org/read.php?185,283327
 *
 * Maths and first version by QHARLEY.
 */
void Scara_Mechanics::Transform(const float raw[XYZ]) {

  static float C2, S2, SK1, SK2, THETA, PSI;

  float sx = raw[X_AXIS] - SCARA_OFFSET_X,  // Translate SCARA to standard X Y
        sy = raw[Y_AXIS] - SCARA_OFFSET_Y;  // With scaling factor.

  if (L1 == L2)
    C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
  else
    C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0f * L1 * L2);

  S2 = SQRT(1 - sq(C2));

  // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
  SK1 = L1 + L2 * C2;

  // Rotated Arm2 gives the distance from Arm1 to Arm2
  SK2 = L2 * S2;

  // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
  THETA = ATAN2(SK1, SK2) - ATAN2(sx, sy);

  // Angle of Arm2
  PSI = ATAN2(S2, C2);

  delta[A_AXIS] = DEGREES(THETA);        // theta is support arm angle
  delta[B_AXIS] = DEGREES(THETA + PSI);  // equal to sub arm angle (inverted motor)
  delta[C_AXIS] = raw[Z_AXIS];

}

#if MECH(MORGAN_SCARA)
  bool Scara_Mechanics::move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (printer.isRunning()) {
      InverseTransform(delta_a, delta_b, cartesian_position);
      destination.x = cartesian_position.x;
      destination.y = cartesian_position.y;
      destination.z = position.z;
      prepare_move_to_destination();
      return true;
    }
    return false;
  }
#endif

/**
 * Home Scara
 */
void Scara_Mechanics::home() {

  if (printer.debugSimulation()) {
    LOOP_XYZ(axis) set_axis_is_at_home((AxisEnum)axis);
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

  // Always home with tool 0 active
  #if HOTENDS > 1
    const uint8_t old_tool_index = toolManager.extruder.active;
    toolManager.change(0, true);
  #endif

  setup_for_endstop_or_probe_move();
  if (printer.debugFeature()) DEBUG_EM("> endstops.setEnabled(true)");
  endstops.setEnabled(true); // Enable endstops for next homing move

  bool come_back = parser.boolval('B');
  REMEMBER(fr, feedrate_mm_s);
  COPY_ARRAY(stored_position[0], position.x);

  if (printer.debugFeature()) DEBUG_POS(">>> home_scara", position);

  const bool  homeA = parser.seen('X'),
              homeB = parser.seen('Y'),
              homeZ = parser.seen('Z');

  const bool home_all = (!homeA && !homeB && !homeZ) || (homeA && homeB && homeZ);

  // Home A
  if (home_all || homeA) {
    homeaxis(A_AXIS);
    if (printer.debugFeature()) DEBUG_POS("> homeA", position);
  }

  // Home B
  if (home_all || homeB) {
    homeaxis(B_AXIS);
    if (printer.debugFeature()) DEBUG_POS("> homeB", position);
  }
  
  // Home Z
  if (home_all || homeZ) {
    homeaxis(Z_AXIS);
    if (printer.debugFeature()) DEBUG_POS("> homeZ", position);
  }

  if (printer.debugFeature()) DEBUG_POS("<<< home_scara", position);
  sync_plan_position();
  endstops.setNotHoming();

  if (come_back) {
    feedrate_mm_s = homing_feedrate_mm_s.x;
    COPY_ARRAY(destination, stored_position[0]);
    prepare_move_to_destination();
  }

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
void Scara_Mechanics::do_homing_move(const AxisEnum axis, const float distance, const feedrate_t fr_mm_s/*=0.0f*/) {

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

  if (is_home_dir) {
    #if HOMING_Z_WITH_PROBE && HAS_BLTOUCH
      const bool deploy_bltouch = (axis == Z_AXIS && distance < 0.0);
      if (deploy_bltouch) bltouch.deploy();
    #endif

    #if QUIET_PROBING
      if (axis == Z_AXIS) probe.set_paused(true);
    #endif
  }

  // Tell the planner we're at Z=0
  position[axis] = 0;
  sync_plan_position();
  position[axis] = distance;
  planner.buffer_line(position.x, fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], toolManager.extruder.active);

  planner.synchronize();

  if (is_home_dir) {
    #if QUIET_PROBING
      if (axis == Z_AXIS) probe.set_paused(false);
    #endif

    #if HOMING_Z_WITH_PROBE && HAS_BLTOUCH
      if (deploy_bltouch) bltouch.stow();
    #endif

    endstops.validate_homing_move();
  }

  if (printer.debugFeature()) {
    DEBUG_MV("<<< do_homing_move(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * position.x to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */ 
void Scara_Mechanics::set_axis_is_at_home(const AxisEnum axis) {

  if (printer.debugFeature()) {
    DEBUG_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
    DEBUG_CHR(')');
    DEBUG_EOL();
  }

  setAxisHomed(axis, true);

  #if MECH(MORGAN_SCARA)

    /**
     * Morgan SCARA homes XY at the same time
     */
    if (axis == X_AXIS || axis == Y_AXIS) {

      float homeposition[XYZ];
      LOOP_XYZ(i) homeposition[i] = base_home_pos[(AxisEnum)i];

      /**
       * Get Home position SCARA arm angles using inverse kinematics,
       * and calculate homing offset using forward kinematics
       */
      Transform(homeposition);
      InverseTransform(delta[A_AXIS], delta[B_AXIS], cartesian_position);

      // DEBUG_ECHOPAIR("Cartesian X:", cartes[X_AXIS]);
      // DEBUG_ECHOLNPAIR(" Y:", cartes[Y_AXIS]);

      position[axis] = cartesian_position[axis];

      /**
       * SCARA home positions are based on configuration since the actual
       * limits are determined by the inverse kinematic transform.
       */
      endstops.soft_endstop[axis].min = data.base_pos[axis].min; // + (cartes[axis] - base_home_pos(axis));
      endstops.soft_endstop[axis].max = data.base_pos[axis].max; // + (cartes[axis] - base_home_pos(axis));
    }
    else
  #endif
  {
    position[axis] = base_home_pos[axis];
  }

  /**
   * Z Probe Z Homing? Account for the probe's Z offset.
   */
  #if HAS_BED_PROBE && Z_HOME_DIR < 0
    if (axis == Z_AXIS) {
      #if HOMING_Z_WITH_PROBE

        position.z -= probe.data.offset.z;

        if (printer.debugFeature()) {
          DEBUG_EM("*** Z HOMED WITH PROBE ***");
          DEBUG_EMV("> zprobe_zoffset = ", probe.data.offset.z);
        }

      #else

        if (printer.debugFeature()) DEBUG_EM("*** Z HOMED TO ENDSTOP ***");

      #endif
    }
  #endif

  if (printer.debugFeature()) {
    DEBUG_POS("", position);
    DEBUG_MT("<<< set_axis_is_at_home(", axis_codes[axis]);
    DEBUG_CHR(')');
    DEBUG_EOL();
  }

}

// Return true if the given point is within the printable area
bool Scara_Mechanics::position_is_reachable(const float &rx, const float &ry) {
  #if MIDDLE_DEAD_ZONE_R > 0
    const float R2 = HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y);
    return R2 >= sq(float(MIDDLE_DEAD_ZONE_R)) && R2 <= sq(L1 + L2);
  #else
    return HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y) <= sq(L1 + L2);
  #endif
}
// Return true if the both nozzle and the probe can reach the given point.
bool Scara_Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
  return position_is_reachable(rx, ry)
      && position_is_reachable(rx - probe.data.offset.x, ry - probe.data.offset.y);
}

/**
 * Calculate delta, start a line, and set position.x to destination
 */
void Scara_Mechanics::prepare_uninterpolated_move_to_destination(const feedrate_t fr_mm_s=0.0) {

  if (printer.debugFeature()) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);

  if ( position.x == destination.x
    && position.y == destination.y
    && position.z == destination.z
    && position.e == destination.e
  ) return;

  planner.buffer_line(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), toolManager.extruder.active);

  position = destination;
}

// Report detail current position to host
void Scara_Mechanics::report_detail_position() {

  SERIAL_MSG("\nLogical:");
  const float logical[XYZ] = {
    LOGICAL_X_POSITION(position.x),
    LOGICAL_Y_POSITION(position.y),
    LOGICAL_Z_POSITION(position.z)
  };
  report_xyz(logical);

  SERIAL_MSG("Raw:    ");
  report_xyze(position.x);

  float leveled[XYZ] = { position.x, position.y, position.z };

  #if HAS_LEVELING
    SERIAL_MSG("Leveled:");
    bedlevel.apply_leveling(leveled);
    report_xyz(leveled);

    SERIAL_MSG("UnLevel:");
    float unleveled[XYZ] = { leveled[X_AXIS], leveled[Y_AXIS], leveled[Z_AXIS] };
    bedlevel.unapply_leveling(unleveled);
    report_xyz(unleveled);
  #endif

  SERIAL_MSG("ScaraK: ");
  Transform(leveled);
  report_xyz(delta);

  planner.synchronize();

  SERIAL_MSG("Stepper:");
  LOOP_XYZE(i) {
    SERIAL_CHR(' ');
    SERIAL_CHR(axis_codes[i]);
    SERIAL_CHR(':');
    SERIAL_VAL(stepper.position((AxisEnum)i));
  }
  SERIAL_EOL();

  const float deg[XYZ] = {
    planner.get_axis_position_degrees(A_AXIS),
    planner.get_axis_position_degrees(B_AXIS)
  };
  SERIAL_MSG("Degrees:");
  report_xyze(deg, 2);

  SERIAL_MSG("FromStp:");
  get_cartesian_from_steppers();  // writes cartesian_position[XYZ] (with forward kinematics)
  const float from_steppers[XYZE] = { cartesian_position.x, cartesian_position.y, cartesian_position.z, planner.get_axis_position_mm(E_AXIS) };
  report_xyze(from_steppers);

  const float diff[XYZE] = {
    from_steppers[X_AXIS] - leveled[X_AXIS],
    from_steppers[Y_AXIS] - leveled[Y_AXIS],
    from_steppers[Z_AXIS] - leveled[Z_AXIS],
    from_steppers[E_AXIS] - position.e
  };

  SERIAL_MSG("Differ: ");
  report_xyze(diff);

}

#if DISABLED(DISABLE_M503)

  void Scara_Mechanics::print_parameters() {
    print_M92();
    print_M201();
    print_M203();
    print_M204();
    print_M205();
    print_M206();
  }

  void Scara_Mechanics::print_M92() {
    SERIAL_LM(CFG, "Steps per unit:");
    SERIAL_SMV(CFG, "  M92 X", LINEAR_UNIT(data.axis_steps_per_mm.x), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.axis_steps_per_mm.y), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(data.axis_steps_per_mm.z), 3);
    #if EXTRUDERS == 1
      SERIAL_MV(" T0 E", VOLUMETRIC_UNIT(data.axis_steps_per_mm.e[0]), 3);
    #endif
    SERIAL_EOL();
    #if EXTRUDERS > 1
      LOOP_EXTRUDER() {
        SERIAL_SMV(CFG, "  M92 T", (int)e);
        SERIAL_EMV(" E", VOLUMETRIC_UNIT(data.axis_steps_per_mm.e[e]), 3);
      }
    #endif // EXTRUDERS > 1
  }

  void Scara_Mechanics::print_M201() {
    SERIAL_LM(CFG, "Maximum Acceleration (units/s2):");
    SERIAL_SMV(CFG, "  M201 X", LINEAR_UNIT(data.max_acceleration_mm_per_s2.x));
    SERIAL_MV(" Y", LINEAR_UNIT(data.max_acceleration_mm_per_s2.y));
    SERIAL_MV(" Z", LINEAR_UNIT(data.max_acceleration_mm_per_s2.z));
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

  void Scara_Mechanics::print_M203() {
    SERIAL_LM(CFG, "Maximum feedrates (units/s):");
    SERIAL_SMV(CFG, "  M203 X", LINEAR_UNIT(data.max_feedrate_mm_s.x), 3);
    SERIAL_MV(" Y", LINEAR_UNIT(data.max_feedrate_mm_s.y), 3);
    SERIAL_MV(" Z", LINEAR_UNIT(data.max_feedrate_mm_s.z), 3);
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

  void Scara_Mechanics::print_M204() {
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

  void Scara_Mechanics::print_M205() {
    SERIAL_LM(CFG, "Advanced: B<DEFAULT_MIN_SEGMENT_TIME> S<DEFAULT_MIN_FEEDRATE> V<DEFAULT_MIN_TRAVEL_FEEDRATE>");
    SERIAL_SMV(CFG, "  M205 B", data.min_segment_time_us);
    SERIAL_MV(" S", LINEAR_UNIT(data.min_feedrate_mm_s), 3);
    SERIAL_EMV(" V", LINEAR_UNIT(data.min_travel_feedrate_mm_s), 3);

    #if ENABLED(JUNCTION_DEVIATION)
      SERIAL_LM(CFG, "Junction Deviation: J<JUNCTION_DEVIATION_MM>");
      SERIAL_LMV(CFG, "  M205 J", data.junction_deviation_mm, 2);
    #endif

    SERIAL_SM(CFG, "Jerk: X<DEFAULT_XJERK>");
    #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
      SERIAL_MSG(" T* E<DEFAULT_EJERK>");
    #endif
    SERIAL_EOL();

    SERIAL_SMV(CFG, "  M205 X", LINEAR_UNIT(data.max_jerk.x), 3);

    #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
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
    #else
      SERIAL_EOL();
    #endif
  }

  void Scara_Mechanics::print_M206() {
    #if ENABLED(WORKSPACE_OFFSETS)
      SERIAL_LM(CFG, "Home offset P<theta-psi-offset> T<theta-offset> Z<Z offset>:");
      SERIAL_SM(CFG, "  M206");
      SERIAL_MV(" P", data.home_offset[X_AXIS], 3);
      SERIAL_MV(" T", data.home_offset[Y_AXIS], 3);
      SERIAL_MV(" Z", LINEAR_UNIT(data.home_offset[Z_AXIS]), 3);
      SERIAL_EOL();
    #endif
  }

#endif // DISABLED(DISABLE_M503)

/** Private Function */
void Scara_Mechanics::homeaxis(const AxisEnum axis) {

  // Only Z homing (with probe) is permitted
  if (axis != Z_AXIS) { sound.playtone(100, NOTE_A5); return; }

  if (printer.debugFeature()) {
    DEBUG_MV(">>> homeaxis(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

  // Homing Z towards the bed? Deploy the Z probe or endstop.
  #if HOMING_Z_WITH_PROBE
    if (DEPLOY_PROBE()) return;
  #endif

  // Fast move towards endstop until triggered
  if (printer.debugFeature()) DEBUG_EM("Home 1 Fast:");

  // Fast move towards endstop until triggered
  do_homing_move(axis, 1.5f * data.base_pos[axis].max * get_homedir(axis));

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
    do_homing_move(axis, -bump);

    // Slow move towards endstop until triggered
    if (printer.debugFeature()) DEBUG_EM("Home 2 Slow:");
    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
  }

  set_axis_is_at_home(axis);
  sync_plan_position();

  // Put away the Z probe
  #if HOMING_Z_WITH_PROBE
    if (probe.STOW_PROBE()) return;
  #endif

  // Clear retracted status if homing the Z axis
  #if ENABLED(FWRETRACT)
    fwretract.current_hop = 0.0;
  #endif

  if (printer.debugFeature()) {
    DEBUG_MV("<<< homeaxis(", axis_codes[axis]);
    DEBUG_CHR(')'); DEBUG_EOL();
  }

}

#endif // IS_SCARA
