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
 * mechanics.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "mechanics.h"

/** Public Parameters */
generic_data_t  Mechanics::data;

flaghome_t      Mechanics::home_flag;

const flagdir_t Mechanics::home_dir(X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR);

const float Mechanics::homing_feedrate_mm_s[XYZ]          = { MMM_TO_MMS(HOMING_FEEDRATE_X), MMM_TO_MMS(HOMING_FEEDRATE_Y), MMM_TO_MMS(HOMING_FEEDRATE_Z) },
            Mechanics::home_bump_mm[XYZ]                  = { X_HOME_BUMP_MM, Y_HOME_BUMP_MM, Z_HOME_BUMP_MM };

float Mechanics::feedrate_mm_s                            = MMM_TO_MMS(1500.0),
      Mechanics::steps_to_mm[XYZE_N]                      = { 0.0 },
      Mechanics::current_position[XYZE]                   = { 0.0 },
      Mechanics::cartesian_position[XYZ]                  = { 0.0 },
      Mechanics::destination[XYZE]                        = { 0.0 },
      Mechanics::stored_position[NUM_POSITON_SLOTS][XYZE] = { { 0.0 } };

int16_t Mechanics::feedrate_percentage                    = 100;

uint32_t Mechanics::max_acceleration_steps_per_s2[XYZE_N] = { 0 };

#if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
  // The distance that XYZ has been offset by G92. Reset by G28.
  float Mechanics::position_shift[XYZ] = { 0.0 };

  // The above two are combined to save on computes
  float Mechanics::workspace_offset[XYZ] = { 0.0 };
#endif

/** Private Parameters */
float   Mechanics::saved_feedrate_mm_s = 0.0;
int16_t Mechanics::saved_feedrate_percentage = 0;

/**
 * Get homedir for axis
 */
int8_t Mechanics::get_homedir(const AxisEnum axis) {
  switch (axis) {
    case X_AXIS:  return home_dir.X; break;
    case Y_AXIS:  return home_dir.Y; break;
    case Z_AXIS:  return home_dir.Z; break;
    default:      return 0;
  }
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * sync_plan_position after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void Mechanics::set_current_from_steppers_for_axis(const AxisEnum axis) {

  mechanics.get_cartesian_from_steppers();

  #if HAS_POSITION_MODIFIERS
    float pos[XYZE] = { cartesian_position[X_AXIS], cartesian_position[Y_AXIS], cartesian_position[Z_AXIS], current_position[E_AXIS] };
    planner.unapply_modifiers(pos
      #if HAS_LEVELING
        , true
      #endif
    );
    const float (&cartesian_position)[XYZE] = pos;
  #endif

  if (axis == ALL_AXES)
    COPY_ARRAY(current_position, cartesian_position);
  else
    current_position[axis] = cartesian_position[axis];
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void Mechanics::line_to_current_position(const float &fr_mm_s/*=feedrate_mm_s*/) {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s, tools.active_extruder);
}

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
void Mechanics::buffer_line_to_destination(const float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, tools.active_extruder);
}

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 */
void Mechanics::prepare_move_to_destination() {
  endstops.apply_motion_limits(destination);

  #if ENABLED(DUAL_X_CARRIAGE)
    if (mechanics.dual_x_carriage_unpark()) return;
  #endif

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

void Mechanics::setup_for_endstop_or_probe_move() {
  saved_feedrate_mm_s = mechanics.feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
}

void Mechanics::clean_up_after_endstop_or_probe_move() {
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
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
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void Mechanics::sync_plan_position() {
  if (printer.debugFeature()) DEBUG_POS("sync_plan_position", current_position);
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
void Mechanics::sync_plan_position_e() {
  planner.set_e_position_mm(current_position[E_AXIS]);
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

/**
 * Homing bump feedrate (mm/s)
 */
float Mechanics::get_homing_bump_feedrate(const AxisEnum axis) {
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS) return MMM_TO_MMS(Z_PROBE_SPEED_SLOW);
  #endif
  static const uint8_t homing_bump_divisor[] PROGMEM = HOMING_BUMP_DIVISOR;
  uint8_t hbd = pgm_read_byte(&homing_bump_divisor[axis]);
  if (hbd < 1) {
    hbd = 10;
    SERIAL_LM(ER, "Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate_mm_s[axis] / hbd;
}

bool Mechanics::axis_unhomed_error(const bool x/*=true*/, const bool y/*=true*/, const bool z/*=true*/) {
  const bool  xx = x && !home_flag.XHomed,
              yy = y && !home_flag.YHomed,
              zz = z && !home_flag.ZHomed;

  if (xx || yy || zz) {
    SERIAL_SM(ECHO, MSG_HOME " ");
    if (xx) SERIAL_MSG(MSG_X);
    if (yy) SERIAL_MSG(MSG_Y);
    if (zz) SERIAL_MSG(MSG_Z);
    SERIAL_EM(" " MSG_FIRST);

    #if HAS_SPI_LCD
      lcdui.status_printf_P(0, PSTR(MSG_HOME " %s%s%s " MSG_FIRST), xx ? MSG_X : "", yy ? MSG_Y : "", zz ? MSG_Z : "");
    #endif

    sound.feedback(false);

    return true;
  }
  return false;
}

#if ENABLED(WORKSPACE_OFFSETS)

  void Mechanics::update_workspace_offset(const AxisEnum axis) {
    workspace_offset[axis] = mechanics.data.home_offset[axis] + position_shift[axis];
    if (printer.debugFeature()) {
      DEBUG_MT("For ", axis_codes[axis]);
      DEBUG_MV(" axis:\n home_offset = ", home_offset[axis]);
      DEBUG_EMV("\n position_shift = ", position_shift[axis]);
    }
  }

  /**
   * Change the home offset for an axis.
   * Also refreshes the workspace offset.
   */
  void Mechanics::set_home_offset(const AxisEnum axis, const float v) {
    mechanics.data.home_offset[axis] = v;
    update_workspace_offset(axis);
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

/** Protected Function */
#if ENABLED(SENSORLESS_HOMING)

  /**
   * Start sensorless homing if the axis has it, accounting for Core Kinematics.
   */
  sensorless_t Mechanics::start_sensorless_homing_per_axis(const AxisEnum axis) {

    sensorless_t stealth_states;

    switch (axis) {
      default: break;
      #if X_HAS_SENSORLESS
        case X_AXIS:
          stealth_states.x = tmc.enable_stallguard(stepperX);
          #if X2_HAS_SENSORLESS
            stealth_states.x2 = tmc.enable_stallguard(stepperX2);
          #elif CORE_IS_XY && Y_HAS_SENSORLESS
            stealth_states.y = tmc.enable_stallguard(stepperY);
          #elif CORE_IS_XZ && Z_HAS_SENSORLESS
            stealth_states.z = tmc.enable_stallguard(stepperZ);
          #endif
          break;
      #endif
      #if Y_HAS_SENSORLESS
        case Y_AXIS:
          stealth_states.y = tmc.enable_stallguard(stepperY);
          #if Y2_HAS_SENSORLESS
            stealth_states.y2 = tmc.enable_stallguard(stepperY2);
          #elif CORE_IS_XY && X_HAS_SENSORLESS
            stealth_states.x = tmc.enable_stallguard(stepperX);
          #elif CORE_IS_YZ && Z_HAS_SENSORLESS
            stealth_states.z = tmc.enable_stallguard(stepperZ);
          #endif
          break;
      #endif
      #if Z_HAS_SENSORLESS
        case Z_AXIS:
          stealth_states.z = tmc.enable_stallguard(stepperZ);
          #if Z2_HAS_SENSORLESS
            stealth_states.z2 = tmc.enable_stallguard(stepperZ2);
          #endif
          #if Z3_HAS_SENSORLESS
            stealth_states.z3 = tmc.enable_stallguard(stepperZ3);
          #endif
          #if CORE_IS_XZ && X_HAS_SENSORLESS
            stealth_states.x = tmc.enable_stallguard(stepperX);
          #elif CORE_IS_YZ && Y_HAS_SENSORLESS
            stealth_states.z = tmc.enable_stallguard(stepperZ);
          #endif
          break;
      #endif
    }

    return stealth_states;
  }

  /**
   * Stop sensorless homing if the axis has it, accounting for Core Kinematics.
   */
  void Mechanics::stop_sensorless_homing_per_axis(const AxisEnum axis, sensorless_t enable_stealth) {

    switch (axis) {
      default: break;
      #if X_HAS_SENSORLESS
        case X_AXIS:
          tmc.disable_stallguard(stepperX, enable_stealth.x);
          #if X2_HAS_SENSORLESS
            tmc.disable_stallguard(stepperX2, enable_stealth.x2);
          #elif CORE_IS_XY && Y_HAS_SENSORLESS
            tmc.disable_stallguard(stepperY, enable_stealth.y);
          #elif CORE_IS_XZ && Z_HAS_SENSORLESS
            tmc.disable_stallguard(stepperZ, enable_stealth.z);
          #endif
          break;
      #endif
      #if Y_HAS_SENSORLESS
        case Y_AXIS:
          tmc.disable_stallguard(stepperY, enable_stealth.y);
          #if Y2_HAS_SENSORLESS
            tmc.disable_stallguard(stepperY2, enable_stealth.y2);
          #elif CORE_IS_XY && X_HAS_SENSORLESS
            tmc.disable_stallguard(stepperX, enable_stealth.x);
          #elif CORE_IS_YZ && Z_HAS_SENSORLESS
            tmc.disable_stallguard(stepperZ, enable_stealth.z);
          #endif
          break;
      #endif
      #if Z_HAS_SENSORLESS
        case Z_AXIS:
          tmc.disable_stallguard(stepperZ, enable_stealth.z);
          #if Z2_HAS_SENSORLESS
            tmc.disable_stallguard(stepperZ2, enable_stealth.z2);
          #endif
          #if Z3_HAS_SENSORLESS
            tmc.disable_stallguard(stepperZ3, enable_stealth.z3);
          #endif
          #if CORE_IS_XZ && X_HAS_SENSORLESS
            tmc.disable_stallguard(stepperX, enable_stealth.x);
          #elif CORE_IS_YZ && Y_HAS_SENSORLESS
            tmc.disable_stallguard(stepperY, enable_stealth.y);
          #endif
          break;
      #endif
    }

  }

#endif // SENSORLESS_HOMING
