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
 * mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "mechanics.h"

/**
 * Directly set the planner XYZ position (and stepper positions)
 * converting mm into steps.
 */
void Mechanics::_set_position_mm(const float &a, const float &b, const float &c, const float &e) {

  planner.position[X_AXIS] = LROUND(a * axis_steps_per_mm[X_AXIS]),
  planner.position[Y_AXIS] = LROUND(b * axis_steps_per_mm[Y_AXIS]),
  planner.position[Z_AXIS] = LROUND(c * axis_steps_per_mm[Z_AXIS]),
  planner.position[E_AXIS] = LROUND(e * axis_steps_per_mm[E_INDEX]);

  #if ENABLED(LIN_ADVANCE)
    planner.position_float[X_AXIS] = a;
    planner.position_float[Y_AXIS] = b;
    planner.position_float[Z_AXIS] = c;
    planner.position_float[E_AXIS] = e;
  #endif

  stepper.set_position(planner.position[X_AXIS], planner.position[Y_AXIS], planner.position[Z_AXIS], planner.position[E_AXIS]);
  planner.zero_previous_nominal_speed();
  planner.zero_previous_speed();

}
void Mechanics::set_position_mm(const AxisEnum axis, const float &v) {

  #if EXTRUDERS > 1
    const uint8_t axis_index = axis + (axis == E_AXIS ? tools.active_extruder : 0);
  #else
    const uint8_t axis_index = axis;
  #endif

  planner.position[axis] = LROUND(v * axis_steps_per_mm[axis_index]);
  #if ENABLED(LIN_ADVANCE)
    planner.position_float[axis] = v;
  #endif
  stepper.set_position(axis, v);
  planner.zero_previous_speed(axis);

}
void Mechanics::set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e) {
  #if PLANNER_LEVELING
    bedlevel.apply_leveling(rx, ry, rz);
  #endif
  _set_position_mm(rx, ry, rz, e);
}
void Mechanics::set_position_mm(const float position[NUM_AXIS]) {
  #if PLANNER_LEVELING
    float lpos[XYZ] = { position[X_AXIS], position[Y_AXIS], position[Z_AXIS] };
    bedlevel.apply_leveling(lpos);
  #else
    const float * const lpos = position;
  #endif
  _set_position_mm(lpos[X_AXIS], lpos[Y_AXIS], lpos[Z_AXIS], position[E_AXIS]);
}

/**
 * Get the stepper positions in the cartesian_position[] array.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void Mechanics::get_cartesian_from_steppers() {
  cartesian_position[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
  cartesian_position[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);
  cartesian_position[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
}

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
void Mechanics::set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  #if PLANNER_LEVELING
    bedlevel.unapply_leveling(cartesian_position);
  #endif
  if (axis == ALL_AXES)
    COPY_ARRAY(current_position, cartesian_position);
  else
    current_position[axis] = cartesian_position[axis];
}

/**
 * line_to_current_position
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void Mechanics::line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, tools.active_extruder);
}

/**
 * line_to_destination
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
void Mechanics::line_to_destination(float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, tools.active_extruder);
}

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 */
void Mechanics::prepare_move_to_destination() {
  endstops.clamp_to_software_endstops(destination);
  commands.refresh_cmd_timeout();

  if (!printer.debugSimulation()) { // Simulation Mode no movement
    if (
      #if UBL_DELTA
        ubl.prepare_segmented_line_to(destination, feedrate_mm_s)
      #else
        mechanics.prepare_move_to_destination_mech_specific()
      #endif
    ) return;
  }

  set_current_to_destination();
}

#if ENABLED(G5_BEZIER)

  /**
   * Compute a Bézier curve using the De Casteljau's algorithm (see
   * https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm), which is
   * easy to code and has good numerical stability (very important,
   * since Arduino works with limited precision real numbers).
   */
  void Mechanics::plan_cubic_move(const float offset[4]) {
    Bezier::cubic_b_spline(current_position, destination, offset, MMS_SCALED(feedrate_mm_s), tools.active_extruder);

    // As far as the parser is concerned, the position is now == destination. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }

#endif // G5_BEZIER


/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void Mechanics::do_blocking_move_to(const float rx, const float ry, const float rz, const float &fr_mm_s /*=0.0*/) {
  const float old_feedrate_mm_s = feedrate_mm_s;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, rx, ry, rz);
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

  stepper.synchronize();

  feedrate_mm_s = old_feedrate_mm_s;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) SERIAL_EM("<<< do_blocking_move_to");
  #endif
}
void Mechanics::do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void Mechanics::do_blocking_move_to_z(const float &rz, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], rz, fr_mm_s);
}
void Mechanics::do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, ry, current_position[Z_AXIS], fr_mm_s);
}

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void Mechanics::sync_plan_position() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) DEBUG_POS("sync_plan_position", current_position);
  #endif
  set_position_mm(current_position);
}
void Mechanics::sync_plan_position_e() {
  set_e_position_mm(current_position[E_AXIS]);
}

/**
 * Recalculate the steps/s^2 acceleration rates, based on the mm/s^2
 */
void Mechanics::reset_acceleration_rates() {
  #if EXTRUDERS > 1
    #define HIGHEST_CONDITION (i < E_AXIS || i == E_INDEX)
  #else
    #define HIGHEST_CONDITION true
  #endif
  uint32_t highest_rate = 1;
  LOOP_XYZE_N(i) {
    max_acceleration_steps_per_s2[i] = max_acceleration_mm_per_s2[i] * axis_steps_per_mm[i];
    if (HIGHEST_CONDITION) NOLESS(highest_rate, max_acceleration_steps_per_s2[i]);
  }
  planner.cutoff_long = 4294967295UL / highest_rate;
}

/**
 * Recalculate position, steps_to_mm if axis_steps_per_mm changes!
 */
void Mechanics::refresh_positioning() {
  LOOP_XYZE_N(i) steps_to_mm[i] = 1.0 / axis_steps_per_mm[i];
  set_position_mm(current_position);
  reset_acceleration_rates();
}

void Mechanics::do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s/*=0.0*/) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) {
      SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
      SERIAL_MV(", ", distance);
      SERIAL_MV(", ", fr_mm_s);
      SERIAL_CHR(')');
      SERIAL_EOL();
    }
  #endif

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    const bool deploy_bltouch = (axis == Z_AXIS && distance < 0.0);
    if (deploy_bltouch) probe.set_bltouch_deployed(true);
  #endif

  #if QUIET_PROBING
    if (axis == Z_AXIS) probe.probing_pause(true);
  #endif

  // Disable stealthChop if used. Enable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    mechanics.sensorless_homing_per_axis(axis);
  #endif

  // Tell the planner we're at Z=0
  current_position[axis] = 0;

  set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
  current_position[axis] = distance;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], tools.active_extruder);

  stepper.synchronize();

  #if QUIET_PROBING
    if (axis == Z_AXIS) probe.probing_pause(false);
  #endif

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    if (deploy_bltouch) probe.set_bltouch_deployed(false);
  #endif

  endstops.hit_on_purpose();

  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    mechanics.sensorless_homing_per_axis(axis, false);
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) {
      SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
      SERIAL_CHR(')'); SERIAL_EOL();
    }
  #endif
}

/**
 * Report current position to host
 */
void Mechanics::report_current_position() {
  SERIAL_MV( "X:", LOGICAL_X_POSITION(current_position[X_AXIS]), 2);
  SERIAL_MV(" Y:", LOGICAL_Y_POSITION(current_position[Y_AXIS]), 2);
  SERIAL_MV(" Z:", LOGICAL_Z_POSITION(current_position[Z_AXIS]), 3);
  SERIAL_EMV(" E:", current_position[E_AXIS], 4);
}
void Mechanics::report_current_position_detail() {

  stepper.synchronize();

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

  #if PLANNER_LEVELING
    SERIAL_MSG("Leveled:");
    bedlevel.apply_leveling(leveled);
    report_xyz(leveled);

    SERIAL_MSG("UnLevel:");
    float unleveled[XYZ] = { leveled[X_AXIS], leveled[Y_AXIS], leveled[Z_AXIS] };
    bedlevel.unapply_leveling(unleveled);
    report_xyz(unleveled);
  #endif

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
  const float from_steppers[XYZE] = { cartesian_position[X_AXIS], cartesian_position[Y_AXIS], cartesian_position[Z_AXIS], stepper.get_axis_position_mm(E_AXIS) };
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

void Mechanics::report_xyze(const float pos[], const uint8_t n/*=4*/, const uint8_t precision/*=3*/) {
  char str[12];
  for (uint8_t i = 0; i < n; i++) {
    SERIAL_CHR(' ');
    SERIAL_CHR(axis_codes[i]);
    SERIAL_CHR(':');
    SERIAL_VAL(dtostrf(pos[i], 9, precision, str));
  }
  SERIAL_EOL();
}

float Mechanics::get_homing_bump_feedrate(const AxisEnum axis) {
  const uint8_t homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  uint8_t hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
    SERIAL_LM(ER, "Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate_mm_s[axis] / hbd;
}

bool Mechanics::axis_unhomed_error(const bool x/*=true*/, const bool y/*=true*/, const bool z/*=true*/) {
  const bool  xx = x && !printer.isXHomed(),
              yy = y && !printer.isYHomed(),
              zz = z && !printer.isZHomed();

  if (xx || yy || zz) {
    SERIAL_SM(ECHO, MSG_HOME " ");
    if (xx) SERIAL_MSG(MSG_X);
    if (yy) SERIAL_MSG(MSG_Y);
    if (zz) SERIAL_MSG(MSG_Z);
    SERIAL_EM(" " MSG_FIRST);

    #if ENABLED(ULTRA_LCD)
      lcd_status_printf_P(0, PSTR(MSG_HOME " %s%s%s " MSG_FIRST), xx ? MSG_X : "", yy ? MSG_Y : "", zz ? MSG_Z : "");
    #endif
    return true;
  }
  return false;
}

// Return true if the given position is within the machine bounds.
bool Mechanics::position_is_reachable(const float &rx, const float &ry) {
  // Add 0.001 margin to deal with float imprecision
  return WITHIN(rx, X_MIN_POS - 0.001, X_MAX_POS + 0.001)
      && WITHIN(ry, Y_MIN_POS - 0.001, Y_MAX_POS + 0.001);
}
// Return whether the given position is within the bed, and whether the nozzle
//  can reach the position required to put the probe at the given position.
bool Mechanics::position_is_reachable_by_probe(const float &rx, const float &ry) {
  const float nx = rx - probe.offset[X_AXIS],
              ny = ry - probe.offset[Y_AXIS];
  return position_is_reachable(nx, ny)
      && WITHIN(rx, X_MIN_BED - 0.001, X_MAX_BED + 0.001)
      && WITHIN(ry, Y_MIN_BED - 0.001, Y_MAX_BED + 0.001);
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
  void Mechanics::plan_arc(
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
                mm_of_travel = linear_travel ? HYPOT(flat_mm, linear_travel) : FABS(flat_mm);
    if (mm_of_travel < 0.001) return;

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
    float arc_target[XYZE];
    const float theta_per_segment = angular_travel / segments,
                linear_per_segment = linear_travel / segments,
                extruder_per_segment = extruder_travel / segments,
                sin_T = theta_per_segment,
                cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation

    // Initialize the linear axis
    arc_target[l_axis] = current_position[l_axis];

    // Initialize the extruder axis
    arc_target[E_AXIS] = current_position[E_AXIS];

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

      // Update arc_target location
      arc_target[p_axis] = center_P + r_P;
      arc_target[q_axis] = center_Q + r_Q;
      arc_target[l_axis] += linear_per_segment;
      arc_target[E_AXIS] += extruder_per_segment;

      endstops.clamp_to_software_endstops(arc_target);

      planner.buffer_line_kinematic(arc_target, fr_mm_s, tools.active_extruder);
    }

    // Ensure last segment arrives at target location.
    planner.buffer_line_kinematic(cart, fr_mm_s, tools.active_extruder);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }

#endif

#if ENABLED(WORKSPACE_OFFSETS)

  /**
   * Change the home offset for an axis.
   * Also refreshes the workspace offset.
   */
  void Mechanics::set_home_offset(const AxisEnum axis, const float v) {
    home_offset[axis] = v;
    endstops.update_software_endstops(axis);
  }

  float Mechanics::native_to_logical(const float pos, const AxisEnum axis) {
    if (axis == E_AXIS)
      return pos;
    else
      return pos + workspace_offset[axis];
  }

  float Mechanics::logical_to_native(const float pos, const AxisEnum axis) {
    if (axis == E_AXIS)
      return pos;
    else
      return pos - workspace_offset[axis];
  }

#endif

#if ENABLED(DEBUG_LEVELING_FEATURE)

  void Mechanics::log_machine_info() {
    SERIAL_MSG("Machine Type: ");
    SERIAL_EM(MACHINE_TYPE);

    SERIAL_MSG("Probe: ");
    #if ENABLED(PROBE_MANUALLY)
      SERIAL_EM("PROBE_MANUALLY");
    #elif ENABLED(Z_PROBE_FIX_MOUNTED)
      SERIAL_EM("Z_PROBE_FIX_MOUNTED");
    #elif ENABLED(BLTOUCH)
      SERIAL_EM("BLTOUCH");
    #elif ENABLED(Z_PROBE_SLED)
      SERIAL_EM("Z_PROBE_SLED");
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      SERIAL_EM("ALLEN KEY");
    #elif HAS_Z_SERVO_PROBE
      SERIAL_EM("SERVO PROBE");
    #else
      SERIAL_EM("NONE");
    #endif

    SERIAL_SM(ECHO, "Probe Offset");
    SERIAL_MV(" X:", probe.offset[X_AXIS]);
    SERIAL_MV(" Y:", probe.offset[Y_AXIS]);
    SERIAL_MV(" Z:", probe.offset[Z_AXIS]);

    if (probe.offset[X_AXIS] > 0)
      SERIAL_MSG(" (Right");
    else if (probe.offset[X_AXIS] < 0)
      SERIAL_MSG(" (Left");
    else if (probe.offset[Y_AXIS] != 0)
      SERIAL_MSG(" (Middle");
    else
      SERIAL_MSG(" (Aligned With");

    if (probe.offset[Y_AXIS] > 0)
      SERIAL_MSG("-Back");
    else if (probe.offset[Y_AXIS] < 0)
      SERIAL_MSG("-Front");
    else if (probe.offset[X_AXIS] != 0)
      SERIAL_MSG("-Center");

    if (probe.offset[Z_AXIS] < 0)
      SERIAL_MSG(" & Below");
    else if (probe.offset[Z_AXIS] > 0)
      SERIAL_MSG(" & Above");
    else
      SERIAL_MSG(" & Same Z as");
    SERIAL_EM(" Nozzle)");

    #if HAS_ABL
      SERIAL_MSG("Auto Bed Leveling: ");
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        SERIAL_MSG("LINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        SERIAL_MSG("BILINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        SERIAL_MSG("3POINT");
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        SERIAL_MSG("UBL");
      #endif
      if (bedlevel.leveling_active) {
        SERIAL_EM(" (enabled)");
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          if (bedlevel.z_fade_height)
            SERIAL_MV("Z Fade: ", bedlevel.z_fade_height);
        #endif
        #if ABL_PLANAR
          const float diff[XYZ] = {
            stepper.get_axis_position_mm(X_AXIS) - current_position[X_AXIS],
            stepper.get_axis_position_mm(Y_AXIS) - current_position[Y_AXIS],
            stepper.get_axis_position_mm(Z_AXIS) - current_position[Z_AXIS]
          };
          SERIAL_MSG("ABL Adjustment X");
          if (diff[X_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[X_AXIS]);
          SERIAL_MSG(" Y");
          if (diff[Y_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[Y_AXIS]);
          SERIAL_MSG(" Z");
          if (diff[Z_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[Z_AXIS]);
        #else
          #if ENABLED(AUTO_BED_LEVELING_UBL)
            SERIAL_MSG("UBL Adjustment Z");
            const float rz = ubl.get_z_correction(current_position[X_AXIS], current_position[Y_AXIS]);
          #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
            SERIAL_MSG("ABL Adjustment Z");
            const float rz = abl.bilinear_z_offset(current_position);
          #endif
          SERIAL_VAL(ftostr43sign(rz, '+'));
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            if (bedlevel.z_fade_height) {
              SERIAL_MV(" (", ftostr43sign(rz * bedlevel.fade_scaling_factor_for_z(current_position[Z_AXIS])));
              SERIAL_MSG("+)");
            }
          #endif
        #endif
      }
      else
        SERIAL_MSG(" (disabled)");

      SERIAL_EOL();

    #elif ENABLED(MESH_BED_LEVELING)

      SERIAL_MSG("Mesh Bed Leveling");
      if (bedlevel.leveling_active) {
        SERIAL_EM(" (enabled)");
        SERIAL_MV("MBL Adjustment Z", ftostr43sign(mbl.get_z(current_position[X_AXIS], current_position[Y_AXIS])));
        SERIAL_CHR('+');
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          if (bedlevel.z_fade_height) {
            SERIAL_MV(" (", ftostr43sign(
              mbl.get_z(current_position[X_AXIS], current_position[Y_AXIS], bedlevel.fade_scaling_factor_for_z(current_position[Z_AXIS]))));
            SERIAL_MSG("+)");
          }
        #endif
      }
      else
        SERIAL_MSG(" (disabled)");

      SERIAL_EOL();

    #endif

  }

#endif // DEBUG_LEVELING_FEATURE

#if ENABLED(BABYSTEPPING)

  void Mechanics::babystep_axis(const AxisEnum axis, const int distance) {

    if (printer.isAxisHomed(axis)) {
      #if IS_CORE
        #if ENABLED(BABYSTEP_XY)
          switch (axis) {
            case CORE_AXIS_1: // X on CoreXY and CoreXZ, Y on CoreYZ
              babystepsTodo[CORE_AXIS_1] += distance * 2;
              babystepsTodo[CORE_AXIS_2] += distance * 2;
              break;
            case CORE_AXIS_2: // Y on CoreXY, Z on CoreXZ and CoreYZ
              babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
              babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
              break;
            case NORMAL_AXIS: // Z on CoreXY, Y on CoreXZ, X on CoreYZ
              babystepsTodo[NORMAL_AXIS] += distance;
              break;
          }
        #elif CORE_IS_XZ || CORE_IS_YZ
          // Only Z stepping needs to be handled here
          babystepsTodo[CORE_AXIS_1] += CORESIGN(distance * 2);
          babystepsTodo[CORE_AXIS_2] -= CORESIGN(distance * 2);
        #else
          babystepsTodo[Z_AXIS] += distance;
        #endif
      #else
        babystepsTodo[axis] += distance;
      #endif
    }
  }

#endif // BABYSTEPPING

#if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)

  void Mechanics::Nextion_gfx_clear() {
    gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
    gfx_cursor_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
  }

#endif

