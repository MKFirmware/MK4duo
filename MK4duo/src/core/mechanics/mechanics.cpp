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
 * mechanics.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"
#include "mechanics.h"

/** Public Parameters */
generic_data_t    Mechanics::data;

home_flag_t       Mechanics::home_flag;

const dir_flag_t  Mechanics::home_dir(X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR);

const xyz_float_t Mechanics::homing_feedrate_mm_s = { MMM_TO_MMS(HOMING_FEEDRATE_X), MMM_TO_MMS(HOMING_FEEDRATE_Y), MMM_TO_MMS(HOMING_FEEDRATE_Z) },
                  Mechanics::home_bump_mm         = { X_HOME_BUMP_MM, Y_HOME_BUMP_MM, Z_HOME_BUMP_MM };

feedrate_t        Mechanics::feedrate_mm_s        = MMM_TO_MMS(1500.0);

xyz_float_t       Mechanics::steps_to_mm;

int16_t           Mechanics::feedrate_percentage  = 100;

xyz_ulong_t       Mechanics::max_acceleration_steps_per_s2;

xyze_pos_t        Mechanics::position,
                  Mechanics::destination,
                  Mechanics::stored_position[NUM_POSITON_SLOTS];

xyz_pos_t         Mechanics::cartesian_position{0};

static constexpr bool axis_relative_temp[XYZE] = AXIS_RELATIVE_MODES;
uint8_t Mechanics::axis_relative_modes = (
    (axis_relative_temp[X_AXIS] ? _BV(X_AXIS) : 0)
  | (axis_relative_temp[Y_AXIS] ? _BV(Y_AXIS) : 0)
  | (axis_relative_temp[Z_AXIS] ? _BV(Z_AXIS) : 0)
  | (axis_relative_temp[E_AXIS] ? _BV(E_AXIS) : 0)
);

#if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
  // The distance that XYZ has been offset by G92. Reset by G28.
  xyz_pos_t Mechanics::position_shift{ 0.0f, 0.0f, 0.0f };

  // The above two are combined to save on computes
  xyz_pos_t Mechanics::workspace_offset{ 0.0f, 0.0f, 0.0f };
#endif

#if ENABLED(CNC_WORKSPACE_PLANES)
  WorkspacePlaneEnum Mechanics::workspace_plane = PLANE_XY;
#endif

/** Private Parameters */
feedrate_t  Mechanics::saved_feedrate_mm_s        = 0.0f;
int16_t     Mechanics::saved_feedrate_percentage  = 0;

/**
 * Get homedir for axis
 */
int8_t Mechanics::get_homedir(const AxisEnum axis) {
  switch (axis) {
    case X_AXIS:  return home_dir.x; break;
    case Y_AXIS:  return home_dir.y; break;
    case Z_AXIS:  return home_dir.z; break;
    case E_AXIS:  return home_dir.e; break;
    default:      return 0;
  }
}

/**
 * Set the position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * sync_plan_position after updating axes with this.
 *
 * To keep hosts in sync, always call report_position
 * after updating the position.
 */
void Mechanics::set_position_from_steppers_for_axis(const AxisEnum axis) {

  mechanics.get_cartesian_from_steppers();
  xyze_pos_t pos = cartesian_position;
  pos.e = planner.get_axis_position_mm(E_AXIS);

  #if HAS_POSITION_MODIFIERS
    planner.unapply_modifiers(pos
      #if HAS_LEVELING
        , true
      #endif
    );
  #endif

  if (axis == ALL_AXES)
    position = pos;
  else
    position[axis] = pos[axis];
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void Mechanics::line_to_position(const feedrate_t &fr_mm_s/*=feedrate_mm_s*/) {
  planner.buffer_line(position, fr_mm_s, toolManager.extruder.active);
}

/**
 * Move the planner to the destination from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void Mechanics::line_to_destination(const feedrate_t &fr_mm_s/*=feedrate_mm_s*/) {
  planner.buffer_line(destination, fr_mm_s, toolManager.extruder.active);
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
        ubl.line_to_destination_segmented(MMS_SCALED(feedrate_mm_s))
      #else
        mechanics.prepare_move_to_destination_mech_specific()
      #endif
    ) return;
  }

  position = destination;
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
    Bezier::cubic_b_spline(position, destination, offset, MMS_SCALED(feedrate_mm_s), toolManager.extruder.active);

    // As far as the parser is concerned, the position is now == destination. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    position = destination;
  }

#endif // G5_BEZIER

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void Mechanics::sync_plan_position() {
  if (printer.debugFeature()) DEBUG_POS("sync_plan_position", position);
  planner.set_position_mm(position);
}
void Mechanics::sync_plan_position_e() {
  if (printer.debugFeature()) DEBUG_EMV("sync_plan_position_e", position.e);
  planner.set_e_position_mm(position.e);
}

/**
 * Report position to host
 */
void Mechanics::report_some_position(const xyze_pos_t &pos) {
  report_xyze(pos);
  stepper.report_positions();
}

void Mechanics::report_xyze(const xyze_pos_t &pos, const uint8_t n/*=4*/) {
  for (uint8_t i = 0; i < n; i++) {
    SERIAL_CHR(' ');
    SERIAL_CHR(axis_codes[i]);
    SERIAL_CHR(':');
    SERIAL_VAL(pos[i], 3);
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

uint8_t Mechanics::axis_need_homing(uint8_t axis_bits/*=0x07*/) {
  // Clear test bits that are homed
  if (TEST(axis_bits, X_AXIS) && home_flag.XHomed) CBI(axis_bits, X_AXIS);
  if (TEST(axis_bits, Y_AXIS) && home_flag.YHomed) CBI(axis_bits, Y_AXIS);
  if (TEST(axis_bits, Z_AXIS) && home_flag.ZHomed) CBI(axis_bits, Z_AXIS);
  return axis_bits;
}

bool Mechanics::axis_unhomed_error(uint8_t axis_bits/*=0x07*/) {
  if ((axis_bits = axis_need_homing(axis_bits))) {
    PGM_P home_first = GET_TEXT(MSG_HOME_FIRST);
    char msg[strlen_P(home_first)+1];
    sprintf_P(msg, home_first,
      TEST(axis_bits, X_AXIS) ? MSG_HOST_X : "",
      TEST(axis_bits, Y_AXIS) ? MSG_HOST_Y : "",
      TEST(axis_bits, Z_AXIS) ? MSG_HOST_Y : ""
    );
    SERIAL_STR(ECHO);
    SERIAL_STR(msg);
    SERIAL_EOL();

    #if HAS_LCD
      lcdui.set_status(msg);
    #endif

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

#endif

#if HAS_LINEAR_E_JERK

  void Mechanics::recalculate_max_e_jerk() {
    LOOP_EXTRUDER() {
      extruders[e]->data.max_jerk = SQRT(SQRT(0.5) *
        data.junction_deviation_mm *
        extruders[e]->data.max_acceleration_mm_per_s2 *
        RECIPROCAL(1.0 - SQRT(0.5))
      );
    }
  }

#endif

/** Protected Function */
#if ENABLED(SENSORLESS_HOMING)

  /**
   * Start sensorless homing if the axis has it, accounting for Core Kinematics.
   */
  sensorless_flag_t Mechanics::start_sensorless_homing_per_axis(const AxisEnum axis) {

    sensorless_flag_t stealth_states;

    switch (axis) {
      default: break;
      #if X_HAS_SENSORLESS
        case X_AXIS:
          stealth_states.x = tmcManager.enable_stallguard(driver.x);
          #if X2_HAS_SENSORLESS
            stealth_states.x2 = tmcManager.enable_stallguard(driver.x2);
          #elif CORE_IS_XY && Y_HAS_SENSORLESS
            stealth_states.y = tmcManager.enable_stallguard(driver.y);
          #elif CORE_IS_XZ && Z_HAS_SENSORLESS
            stealth_states.z = tmcManager.enable_stallguard(driver.z);
          #endif
          break;
      #endif
      #if Y_HAS_SENSORLESS
        case Y_AXIS:
          stealth_states.y = tmcManager.enable_stallguard(driver.y);
          #if Y2_HAS_SENSORLESS
            stealth_states.y2 = tmcManager.enable_stallguard(driver.y2);
          #elif CORE_IS_XY && X_HAS_SENSORLESS
            stealth_states.x = tmcManager.enable_stallguard(driver.x);
          #elif CORE_IS_YZ && Z_HAS_SENSORLESS
            stealth_states.z = tmcManager.enable_stallguard(driver.z);
          #endif
          break;
      #endif
      #if Z_HAS_SENSORLESS
        case Z_AXIS:
          stealth_states.z = tmcManager.enable_stallguard(driver.z);
          #if Z2_HAS_SENSORLESS
            stealth_states.z2 = tmcManager.enable_stallguard(driver.z2);
          #endif
          #if Z3_HAS_SENSORLESS
            stealth_states.z3 = tmcManager.enable_stallguard(driver.z3);
          #endif
          #if CORE_IS_XZ && X_HAS_SENSORLESS
            stealth_states.x = tmcManager.enable_stallguard(driver.x);
          #elif CORE_IS_YZ && Y_HAS_SENSORLESS
            stealth_states.z = tmcManager.enable_stallguard(driver.y);
          #endif
          break;
      #endif
    }

    #if ENABLED(SPI_ENDSTOPS)
      endstops.clear_state();
      switch (axis) {
        #if X_SPI_SENSORLESS
          case X_AXIS: endstops.tmc_spi_homing.x = true; break;
        #endif
        #if Y_SPI_SENSORLESS
          case Y_AXIS: endstops.tmc_spi_homing.y = true; break;
        #endif
        #if Z_SPI_SENSORLESS
          case Z_AXIS: endstops.tmc_spi_homing.z = true; break;
        #endif
        default: break;
      }
    #endif

    #if ENABLED(IMPROVE_HOMING_RELIABILITY)
      tmcManager.sg_guard_period = millis() + tmcManager.default_sg_guard_duration;
    #endif

    return stealth_states;
  }

  /**
   * Stop sensorless homing if the axis has it, accounting for Core Kinematics.
   */
  void Mechanics::stop_sensorless_homing_per_axis(const AxisEnum axis, sensorless_flag_t enable_stealth) {

    switch (axis) {
      default: break;
      #if X_HAS_SENSORLESS
        case X_AXIS:
          tmcManager.disable_stallguard(driver.x, enable_stealth.x);
          #if X2_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.x2, enable_stealth.x2);
          #elif CORE_IS_XY && Y_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.y, enable_stealth.y);
          #elif CORE_IS_XZ && Z_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.z, enable_stealth.z);
          #endif
          break;
      #endif
      #if Y_HAS_SENSORLESS
        case Y_AXIS:
          tmcManager.disable_stallguard(driver.y, enable_stealth.y);
          #if Y2_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.y2, enable_stealth.y2);
          #elif CORE_IS_XY && X_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.x, enable_stealth.x);
          #elif CORE_IS_YZ && Z_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.z, enable_stealth.z);
          #endif
          break;
      #endif
      #if Z_HAS_SENSORLESS
        case Z_AXIS:
          tmcManager.disable_stallguard(driver.z, enable_stealth.z);
          #if Z2_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.z2, enable_stealth.z2);
          #endif
          #if Z3_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.z3, enable_stealth.z3);
          #endif
          #if CORE_IS_XZ && X_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.x, enable_stealth.x);
          #elif CORE_IS_YZ && Y_HAS_SENSORLESS
            tmcManager.disable_stallguard(driver.y, enable_stealth.y);
          #endif
          break;
      #endif
    }

    #if ENABLED(SPI_ENDSTOPS)
      endstops.clear_state();
      endstops.tmc_spi_homing.any = false;
    #endif

  }

#endif // SENSORLESS_HOMING
