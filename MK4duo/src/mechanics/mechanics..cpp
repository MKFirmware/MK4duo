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
 * mechanics.cpp
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"
#include "mechanics.h"

/**
 * Get an axis position according to stepper position(s)
 */
float Mechanics::get_axis_position_mm(AxisEnum axis) {
  return stepper.position(axis) * steps_to_mm[axis];
}

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
    const uint8_t axis_index = axis + (axis == E_AXIS ? active_extruder : 0);
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
  #if HAS_LEVELING
    bedlevel.apply_leveling(lx, ly, lz);
  #endif
  _set_position_mm(lx, ly, lz, e);
}
void Mechanics::set_position_mm(const float position[NUM_AXIS]) {
  #if HAS_LEVELING
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
  cartesian_position[X_AXIS] = get_axis_position_mm(X_AXIS);
  cartesian_position[Y_AXIS] = get_axis_position_mm(Y_AXIS);
  cartesian_position[Z_AXIS] = get_axis_position_mm(Z_AXIS);
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 */
void Mechanics::set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  #if HAS_LEVELING
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
  planner.buffer_line(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder);
}

/**
 * line_to_destination
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
void Mechanics::line_to_destination(float fr_mm_s) {
  planner.buffer_line(destination[A_AXIS], destination[B_AXIS], destination[C_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
}

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void Mechanics::do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s /*=0.0*/) {
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
void Mechanics::do_blocking_move_to(const float logical[XYZ], const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(logical[A_AXIS], logical[B_AXIS], logical[C_AXIS], fr_mm_s);
}
void Mechanics::do_blocking_move_to_x(const float &lx, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(lx, current_position[B_AXIS], current_position[C_AXIS], fr_mm_s);
}
void Mechanics::do_blocking_move_to_z(const float &lz, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(current_position[A_AXIS], current_position[B_AXIS], lz, fr_mm_s);
}
void Mechanics::do_blocking_move_to_xy(const float &lx, const float &ly, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(lx, ly, current_position[C_AXIS], fr_mm_s);
}

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void Mechanics::sync_plan_position() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position_kinematic", current_position);
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
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
      SERIAL_MV(", ", distance);
      SERIAL_MV(", ", fr_mm_s);
      SERIAL_CHR(')'); SERIAL_EOL();
    }
  #endif

  // Tell the planner we're at Z=0
  current_position[axis] = 0;

  set_position_mm(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS]);
  current_position[axis] = distance;
  planner.buffer_line(current_position[A_AXIS], current_position[B_AXIS], current_position[C_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], active_extruder);

  stepper.synchronize();

  endstops.hit_on_purpose();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
      SERIAL_CHR(')'); SERIAL_EOL();
    }
  #endif
}

/**
 * Report current position to host
 */
void Mechanics::report_current_position() {
  SERIAL_MV( "X:", current_position[X_AXIS]);
  SERIAL_MV(" Y:", current_position[Y_AXIS]);
  SERIAL_MV(" Z:", current_position[Z_AXIS]);
  SERIAL_MV(" E:", current_position[E_AXIS]);

  stepper.report_positions();
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
  const bool  xx = x && !axis_homed[A_AXIS],
              yy = y && !axis_homed[B_AXIS],
              zz = z && !axis_homed[C_AXIS];

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

bool Mechanics::position_is_reachable_raw_xy(const float &rx, const float &ry) {
  // Add 0.001 margin to deal with float imprecision
  return WITHIN(rx, X_MIN_POS - 0.001, X_MAX_POS + 0.001)
      && WITHIN(ry, Y_MIN_POS - 0.001, Y_MAX_POS + 0.001);
}
bool Mechanics::position_is_reachable_by_probe_raw_xy(const float &rx, const float &ry) {
  // Add 0.001 margin to deal with float imprecision
  return WITHIN(rx, MIN_PROBE_X - 0.001, MAX_PROBE_X + 0.001)
      && WITHIN(ry, MIN_PROBE_Y - 0.001, MAX_PROBE_Y + 0.001);
}
bool Mechanics::position_is_reachable_by_probe_xy(const float &lx, const float &ly) {
  return position_is_reachable_by_probe_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
}
bool Mechanics::position_is_reachable_xy(const float &lx, const float &ly) {
  return position_is_reachable_raw_xy(RAW_X_POSITION(lx), RAW_Y_POSITION(ly));
}

#if ENABLED(MESH_BED_LEVELING) || ENABLED(PROBE_MANUALLY)

  void Mechanics::manual_goto_xy(const float &x, const float &y) {

    const float old_feedrate_mm_s = feedrate_mm_s;

    #if MANUAL_PROBE_HEIGHT > 0
      feedrate_mm_s = homing_feedrate_mm_s[Z_AXIS];
      current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS) + MANUAL_PROBE_HEIGHT;
      line_to_current_position();
    #endif

    feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
    current_position[X_AXIS] = LOGICAL_X_POSITION(x);
    current_position[Y_AXIS] = LOGICAL_Y_POSITION(y);
    line_to_current_position();

    #if MANUAL_PROBE_HEIGHT > 0
      feedrate_mm_s = homing_feedrate_mm_s[Z_AXIS];
      current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS); // just slightly over the bed
      line_to_current_position();
    #endif

    feedrate_mm_s = old_feedrate_mm_s;
    stepper.synchronize();

    #if ENABLED(PROBE_MANUALLY) && ENABLED(LCD_BED_LEVELING) && ENABLED(ULTRA_LCD)
      lcd_wait_for_move = false;
    #endif

  }

#endif

#if ENABLED(DEBUG_LEVELING_FEATURE)

  void Mechanics::print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
    SERIAL_PS(prefix);
    SERIAL_CHR('(');
    SERIAL_VAL(x);
    SERIAL_MV(", ", y);
    SERIAL_MV(", ", z);
    SERIAL_CHR(")");

    if (suffix) SERIAL_PS(suffix);
    else SERIAL_EOL();
  }

  void Mechanics::print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }

  #if ABL_PLANAR
    void Mechanics::print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz) {
      print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
    }
  #endif

#endif
