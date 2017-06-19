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
 * probe.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"

Probe probe;

float Probe::z_offset = Z_PROBE_OFFSET_FROM_NOZZLE;
bool  Probe::enabled = false;

// returns false for ok and true for failure
bool Probe::set_deployed(bool deploy) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      DEBUG_POS("probe.set_deployed", Mechanics.current_position);
      SERIAL_EMV("deploy: ", deploy);
    }
  #endif

  if (enabled == deploy) return false;

  // Make room for probe
  raise(_Z_PROBE_DEPLOY_HEIGHT);

  // When deploying make sure BLTOUCH is not already triggered
  #if ENABLED(BLTOUCH)
    if (deploy && TEST_BLTOUCH()) {      // If BL-Touch says it's triggered
      bltouch_command(BLTOUCH_RESET);    // try to reset it.
      bltouch_command(BLTOUCH_DEPLOY);   // Also needs to deploy and stow to
      bltouch_command(BLTOUCH_STOW);     // clear the triggered condition.
      safe_delay(1500);                  // wait for internal self test to complete
                                         //   measured completion time was 0.65 seconds
                                         //   after reset, deploy & stow sequence
      if (TEST_BLTOUCH()) {              // If it still claims to be triggered...
        SERIAL_LM(ER, MSG_STOP_BLTOUCH);
        Stop();                          // punt!
        return true;
      }
    }
  #elif ENABLED(Z_PROBE_SLED)
    if (Mechanics.axis_unhomed_error(true, false, false)) {
      SERIAL_LM(ER, MSG_STOP_UNHOMED);
      Stop();
      return true;
    }
  #elif ENABLED(Z_PROBE_ALLEN_KEY)
    if (Mechanics.axis_unhomed_error(true, true,  true )) {
      SERIAL_LM(ER, MSG_STOP_UNHOMED);
      Stop();
      return true;
    }
  #endif

  const float oldXpos = Mechanics.current_position[X_AXIS],
              oldYpos = Mechanics.current_position[Y_AXIS];

  #if ENABLED(_TRIGGERED_WHEN_STOWED_TEST)
    // If endstop is already false, the Z probe is deployed
    if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {        // closed after the probe specific actions.
                                                        // Would a goto be less ugly?
      //while (!_TRIGGERED_WHEN_STOWED_TEST) { idle();  // would offer the opportunity
                                                        // for a triggered when stowed manual probe.
      if (!deploy) set_enable(false); // Switch off triggered when stowed probes early
                                          // otherwise an Allen-Key probe can't be stowed.
  #endif

  #if ENABLED(Z_PROBE_SLED)
    dock_sled(!deploy);
  #elif ENABLED(BLTOUCH) && MECH(DELTA)
    set_bltouch_deployed(deploy);
  #elif HAS_Z_SERVO_PROBE && DISABLED(BLTOUCH)
    servo[Z_ENDSTOP_SERVO_NR].move(z_servo_angle[deploy ? 0 : 1]);
  #elif ENABLED(Z_PROBE_ALLEN_KEY)
    deploy ? run_deploy_moves_script() : run_stow_moves_script();
  #endif

  #if ENABLED(_TRIGGERED_WHEN_STOWED_TEST)
    } // opened before the probe specific actions

    if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {
      if (IsRunning()) {
        SERIAL_LM(ER, "Z-Probe failed");
        LCD_ALERTMESSAGEPGM("Err: ZPROBE");
      }
      stop();
      return true;
    }
  #endif

  Mechanics.do_blocking_move_to(oldXpos, oldYpos, Mechanics.current_position[Z_AXIS]); // return to position before deploy
  set_enable(deploy);
  return false;
}

/**
 * Raise Z to a minimum height to make room for a servo to move
 */
void Probe::raise(const float z_raise) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV("raise(", z_raise);
      SERIAL_CHR(')'); SERIAL_EOL();
    }
  #endif

  float z_dest = LOGICAL_Z_POSITION(z_raise);
  if (z_offset < 0) z_dest -= z_offset;

  if (z_dest > Mechanics.current_position[Z_AXIS])
    Mechanics.do_blocking_move_to_z(z_dest);
}

/**
 * Check Pt (ex probe_pt)
 * - Move to the given XY
 * - Deploy the probe, if not already deployed
 * - Probe the bed, get the Z position
 * - Depending on the 'stow' flag
 *   - Stow the probe, or
 *   - Raise to the BETWEEN height
 * - Return the probed Z position
 */
float Probe::check_pt(const float &x, const float &y, const bool stow/*=true*/, const int verbose_level/*=1*/) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV(">>> check_pt(", x);
      SERIAL_MV(", ", y);
      SERIAL_MV(", ", stow ? "" : "no ");
      SERIAL_EM("stow)");
      DEBUG_POS("", Mechanics.current_position);
    }
  #endif

  if (!Mechanics.position_is_reachable_by_probe_xy(x, y)) return NAN;

  const float old_feedrate_mm_s = Mechanics.feedrate_mm_s;

  #if MECH(DELTA)
    if (Mechanics.current_position[Z_AXIS] > Mechanics.clip_start_height)
      Mechanics.do_blocking_move_to_z(Mechanics.clip_start_height);
  #endif

  #if MECH(MAKERARM_SCARA)
    vector_3 point = probe_point_to_end_point(x, y);
    float dx = point.x, dy = point.y;
    if (dx == 0.0 && dy == 0.0) {
      #if HAS_BUZZER
        BUZZ(100, 220);
      #endif
      return 0.0;
    }
  #else
    const float dx = x - (X_PROBE_OFFSET_FROM_NOZZLE),
                dy = y - (Y_PROBE_OFFSET_FROM_NOZZLE);
  #endif

  // Ensure a minimum height before moving the probe
  raise(Z_PROBE_BETWEEN_HEIGHT);

  Mechanics.feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

  // Move the probe to the given XY
  Mechanics.do_blocking_move_to_xy(dx, dy);

  if (set_deployed(true)) return NAN;

  float measured_z;

  // Prevent stepper_inactive_time from running out and EXTRUDER_RUNOUT_PREVENT from extruding
  refresh_cmd_timeout();

  // If the nozzle is above the travel height then
  // move down quickly before doing the slow probe
  float z = LOGICAL_Z_POSITION(Z_PROBE_BETWEEN_HEIGHT);
  if (z_offset < 0) z -= z_offset;
  if (z < Mechanics.current_position[Z_AXIS])
    Mechanics.do_blocking_move_to_z(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST));

  for (int8_t r = 0; r < Z_PROBE_REPETITIONS; r++) {

    // move down slowly to find bed
    move_to_z(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_SLOW);

    measured_z += Mechanics.current_position[Z_AXIS] + z_offset;

    if (r + 1 < Z_PROBE_REPETITIONS) {
      // move up by the bump distance
      Mechanics.do_blocking_move_to_z(Mechanics.current_position[Z_AXIS] + Mechanics.home_bump_mm[Z_AXIS], MMM_TO_MMS(Z_PROBE_SPEED_FAST));
    }
  }

  measured_z /= (float)Z_PROBE_REPETITIONS;

  if (!stow)
    raise(Z_PROBE_BETWEEN_HEIGHT);
  else
    if (set_deployed(false)) return NAN;

  if (verbose_level > 2) {
    SERIAL_MV(MSG_BED_LEVELING_Z, FIXFLOAT(measured_z), 3);
    SERIAL_MV(MSG_BED_LEVELING_X, x, 3);
    SERIAL_MV(MSG_BED_LEVELING_Y, y, 3);
    SERIAL_EOL();
  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("<<< check_pt");
  #endif

  Mechanics.feedrate_mm_s = old_feedrate_mm_s;

  return measured_z;
}

#if ENABLED(BLTOUCH)
  void Probe::bltouch_command(int angle) {
    servo[Z_ENDSTOP_SERVO_NR].move(angle);  // Give the BL-Touch the command and wait
    safe_delay(BLTOUCH_DELAY);
  }

  void Probe::set_bltouch_deployed(const bool deploy) {
    if (deploy && TEST_BLTOUCH()) {      // If BL-Touch says it's triggered
      bltouch_command(BLTOUCH_RESET);    // try to reset it.
      bltouch_command(BLTOUCH_DEPLOY);   // Also needs to deploy and stow to
      bltouch_command(BLTOUCH_STOW);     // clear the triggered condition.
      safe_delay(1500);                  // wait for internal self test to complete
                                         //   measured completion time was 0.65 seconds
                                         //   after reset, deploy & stow sequence
      if (TEST_BLTOUCH()) {              // If it still claims to be triggered...
        SERIAL_LM(ER, MSG_STOP_BLTOUCH);
        Stop();                          // punt!
      }
    }
    bltouch_command(deploy ? BLTOUCH_DEPLOY : BLTOUCH_STOW);
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("set_bltouch_deployed(", deploy);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif
  }
#endif

void Probe::move_to_z(float z, float fr_mm_m) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS(">>> move_to_z", Mechanics.current_position);
  #endif

  // Deploy BLTouch at the start of any probe
  #if ENABLED(BLTOUCH) && NOMECH(DELTA)
    set_bltouch_deployed(true);
  #endif

  // Move down until probe triggered
  Mechanics.do_blocking_move_to_z(LOGICAL_Z_POSITION(z), MMM_TO_MMS(fr_mm_m));

  // Retract BLTouch immediately after a probe
  #if ENABLED(BLTOUCH) && NOMECH(DELTA)
    set_bltouch_deployed(false);
  #endif

  // Clear endstop flags
  endstops.hit_on_purpose();

  // Get Z where the steppers were interrupted
  Mechanics.set_current_from_steppers_for_axis(Z_AXIS);

  // Tell the planner where we actually are
  Mechanics.sync_plan_position();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("<<< move_to_z", Mechanics.current_position);
  #endif
}
