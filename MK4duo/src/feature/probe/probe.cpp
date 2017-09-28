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
 * probe.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../base.h"

Probe probe;

float Probe::offset[XYZ] = { X_PROBE_OFFSET_FROM_NOZZLE, Y_PROBE_OFFSET_FROM_NOZZLE, Z_PROBE_OFFSET_FROM_NOZZLE };
bool  Probe::enabled  = false;

#if HAS_Z_SERVO_PROBE
  const int Probe::z_servo_angle[2] = Z_ENDSTOP_SERVO_ANGLES;
#endif

// returns false for ok and true for failure
bool Probe::set_deployed(const bool deploy) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      DEBUG_POS("probe.set_deployed", mechanics.current_position);
      SERIAL_EMV("deploy: ", deploy);
    }
  #endif

  if (enabled == deploy) return false;

  // Make room for probe
  float z_dest = _Z_PROBE_DEPLOY_HEIGHT;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV("raise(", z_dest);
      SERIAL_CHR(')'); SERIAL_EOL();
    }
  #endif

  if (offset[Z_AXIS] < 0) z_dest -= offset[Z_AXIS];

  if (z_dest > mechanics.current_position[Z_AXIS])
    mechanics.do_blocking_move_to_z(z_dest);

  #if ENABLED(Z_PROBE_SLED)
    if (mechanics.axis_unhomed_error(true, false, false)) {
      SERIAL_LM(ER, MSG_STOP_UNHOMED);
      printer.Stop();
      return true;
    }
  #elif ENABLED(Z_PROBE_ALLEN_KEY)
    if (mechanics.axis_unhomed_error(true, true,  true )) {
      SERIAL_LM(ER, MSG_STOP_UNHOMED);
      printer.Stop();
      return true;
    }
  #endif

  const float oldXpos = mechanics.current_position[X_AXIS],
              oldYpos = mechanics.current_position[Y_AXIS];

  #if ENABLED(_TRIGGERED_WHEN_STOWED_TEST)
    // If endstop is already false, the Z probe is deployed
    if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {        // closed after the probe specific actions.
                                                        // Would a goto be less ugly?
      //while (!_TRIGGERED_WHEN_STOWED_TEST) { printer.idle();  // would offer the opportunity
                                                        // for a triggered when stowed manual probe.
      if (!deploy) set_enable(false); // Switch off triggered when stowed probes early
                                          // otherwise an Allen-Key probe can't be stowed.
  #endif

  #if ENABLED(Z_PROBE_SLED)
    dock_sled(!deploy);
  #elif ENABLED(BLTOUCH) && MECH(DELTA)
    if (set_bltouch_deployed(deploy)) return true;
  #elif HAS_Z_SERVO_PROBE && DISABLED(BLTOUCH)
    MOVE_SERVO(Z_ENDSTOP_SERVO_NR, z_servo_angle[deploy ? 0 : 1]);
  #elif ENABLED(Z_PROBE_ALLEN_KEY)
    deploy ? run_deploy_moves_script() : run_stow_moves_script();
  #endif

  #if ENABLED(_TRIGGERED_WHEN_STOWED_TEST)
    } // opened before the probe specific actions

    if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {
      if (printer.IsRunning()) {
        SERIAL_LM(ER, "Z-Probe failed");
        LCD_ALERTMESSAGEPGM("Err: ZPROBE");
      }
      stop();
      return true;
    }
  #endif

  mechanics.do_blocking_move_to(oldXpos, oldYpos, mechanics.current_position[Z_AXIS]); // return to position before deploy
  set_enable(deploy);
  return false;
}

/**
 * @brief Used by run_z_probe to do a single Z probe move.
 *
 * @param  z        Z destination
 * @param  fr_mm_s  Feedrate in mm/s
 * @return true to indicate an error
 */
bool Probe::move_to_z(const float z, const float fr_mm_m) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS(">>> move_to_z", mechanics.current_position);
  #endif

  // Deploy BLTouch at the start of any probe
  #if ENABLED(BLTOUCH) && NOMECH(DELTA)
     if (set_bltouch_deployed(true)) return true;
  #endif

  #if QUIET_PROBING
    probing_pause(true);
  #endif

  // Move down until probe triggered
  mechanics.do_blocking_move_to_z(z, MMM_TO_MMS(fr_mm_m));

  // Check to see if the probe was triggered
  const bool probe_triggered = TEST(endstops.endstop_hit_bits,
    #if HAS_Z_PROBE_PIN
      Z_PROBE
    #else
      Z_MIN
    #endif
  );

  #if QUIET_PROBING
    probing_pause(false);
  #endif

  // Retract BLTouch immediately after a probe if it was triggered
  #if ENABLED(BLTOUCH) && NOMECH(DELTA)
    if (probe_triggered && set_bltouch_deployed(false)) return true;
  #endif

  // Clear endstop flags
  endstops.hit_on_purpose();

  // Get Z where the steppers were interrupted
  mechanics.set_current_from_steppers_for_axis(Z_AXIS);

  // Tell the planner where we actually are
  mechanics.sync_plan_position();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("<<< move_to_z", mechanics.current_position);
  #endif

  return !probe_triggered;
}

/**
 * @details Used by check_pt to do a single Z probe.
 *          Leaves current_position[Z_AXIS] at the height where the probe triggered.
 *
 * @param  short_move Flag for a shorter probe move towards the bed
 * @return The raw Z position where the probe was triggered
 */
float Probe::run_z_probe(const bool short_move/*=true*/) {

  float probe_z = 0.0;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS(">>> run_z_probe", mechanics.current_position);
  #endif

  // Prevent stepper_inactive_time from running out and EXTRUDER_RUNOUT_PREVENT from extruding
  commands.refresh_cmd_timeout();

  // If the nozzle is above the travel height then
  // move down quickly before doing the slow probe
  float z = LOGICAL_Z_POSITION(Z_PROBE_BETWEEN_HEIGHT);
  if (offset[Z_AXIS] < 0) z -= offset[Z_AXIS];
  if (z < mechanics.current_position[Z_AXIS])
    mechanics.do_blocking_move_to_z(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST));

  for (int8_t r = 0; r < Z_PROBE_REPETITIONS; r++) {

    // move down slowly to find bed
    if (move_to_z(-10 + (short_move ? 0 : -(Z_MAX_LENGTH)), Z_PROBE_SPEED_SLOW)) return NAN;

    probe_z += RAW_CURRENT_POSITION(Z);

    if (r + 1 < Z_PROBE_REPETITIONS) {
      // move up to probe between height
      mechanics.do_blocking_move_to_z(mechanics.current_position[Z_AXIS] + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
    }
  }

  probe_z /= (float)Z_PROBE_REPETITIONS;

  return probe_z + offset[Z_AXIS];
}

/**
 * Check Pt (ex probe_pt)
 * - Move to the given XY
 * - Deploy the probe, if not already deployed
 * - Probe the bed, get the Z position
 * - Depending on the 'stow' flag
 *   - Stow the probe, or
 *   - Raise to the BETWEEN height
 * - short_move Flag for a shorter probe move towards the bed
 * - Return the probed Z position
 */
float Probe::check_pt(const float &lx, const float &ly, const bool stow/*=true*/, const int verbose_level/*=1*/, const bool printable/*=true*/) {

  #if HAS_BED_PROBE

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> check_pt(", lx);
        SERIAL_MV(", ", ly);
        SERIAL_MV(", ", stow ? "" : "no ");
        SERIAL_EM("stow)");
        DEBUG_POS("", mechanics.current_position);
      }
    #endif

    const float dx = lx - offset[X_AXIS],
                dy = ly - offset[Y_AXIS];

    if (printable) {
      if (!mechanics.position_is_reachable_by_probe_xy(lx, ly)) return NAN;
    }
    else if (!mechanics.position_is_reachable_xy(dx, dy)) return NAN;

    const float old_feedrate_mm_s = mechanics.feedrate_mm_s;

    #if MECH(DELTA)
      if (mechanics.current_position[Z_AXIS] > mechanics.delta_clip_start_height)
        mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
    #endif

    #if HAS_SOFTWARE_ENDSTOPS
      // Store the status of the soft endstops and disable if we're probing a non-printable location
      const bool old_enable_soft_endstops = endstops.soft_endstops_enabled;
      if (!printable) endstops.soft_endstops_enabled = false;
    #endif

    mechanics.feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

    // Move the probe to the given XY
    mechanics.do_blocking_move_to_xy(dx, dy);

    float measured_z = NAN;
    if (!set_deployed(true)) {
      measured_z = run_z_probe(printable);

      if (!stow)
        mechanics.do_blocking_move_to_z(mechanics.current_position[Z_AXIS] + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
      else
        if (set_deployed(false)) measured_z = NAN;
    }

    #if HAS_SOFTWARE_ENDSTOPS
      // Restore the soft endstop status
      endstops.soft_endstops_enabled = old_enable_soft_endstops;
    #endif

    if (verbose_level > 2) {
      SERIAL_MV(MSG_BED_LEVELING_Z, FIXFLOAT(measured_z), 3);
      SERIAL_MV(MSG_BED_LEVELING_X, lx, 3);
      SERIAL_MV(MSG_BED_LEVELING_Y, ly, 3);
      SERIAL_EOL();
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< check_pt");
    #endif

    mechanics.feedrate_mm_s = old_feedrate_mm_s;

    if (isnan(measured_z)) {
      LCD_MESSAGEPGM(MSG_ERR_PROBING_FAILED);
      SERIAL_LM(ER, MSG_ERR_PROBING_FAILED);
    }

    return measured_z;

  #elif HAS_RESUME_CONTINUE && ENABLED(PROBE_MANUALLY)

    UNUSED(stow);
    UNUSED(verbose_level);
    UNUSED(printable);

    // Ensure a minimum height before moving the probe
    mechanics.do_blocking_move_to_z(mechanics.current_position[Z_AXIS] + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(Z_PROBE_SPEED_FAST));

    // Move the probe to the given XY
    mechanics.manual_goto_xy(lx, ly);

    // Disable software endstops to allow manual adjustment
    #if HAS_SOFTWARE_ENDSTOPS
      const bool old_enable_soft_endstops = endstops.soft_endstops_enabled;
      endstops.soft_endstops_enabled = false;
    #endif

    KEEPALIVE_STATE(PAUSED_FOR_USER);
    printer.wait_for_user = true;

    #if ENABLED(ULTIPANEL)
      lcd_move_z_probe();
    #elif ENABLED(NEXTION)
      LcdBedLevelOn();
    #endif

    while (printer.wait_for_user) printer.idle();

    #if ENABLED(NEXTION)
      LcdBedLevelOff();
    #endif

    KEEPALIVE_STATE(IN_HANDLER);

    #if HAS_SOFTWARE_ENDSTOPS
      // Restore the soft endstop status
      endstops.soft_endstops_enabled = old_enable_soft_endstops;
    #endif

    return RAW_CURRENT_POSITION(Z);

  #endif // HAS_RESUME_CONTINUE

}

#if QUIET_PROBING
  void Probe::probing_pause(const bool p) {
    #if ENABLED(PROBING_HEATERS_OFF)
      thermalManager.pause(p);
    #endif
    #if ENABLED(PROBING_FANS_OFF)
      LOOP_FAN() fans[f].pause(p);
    #endif
    if (p) printer.safe_delay(25);
  }
#endif // QUIET_PROBING

#if ENABLED(BLTOUCH)

  void Probe::bltouch_command(int angle) {
    MOVE_SERVO(Z_ENDSTOP_SERVO_NR, angle);  // Give the BL-Touch the command and wait
    printer.safe_delay(BLTOUCH_DELAY);
  }

  bool Probe::set_bltouch_deployed(const bool deploy) {
    if (deploy && TEST_BLTOUCH()) {      // If BL-Touch says it's triggered
      bltouch_command(BLTOUCH_RESET);    // try to reset it.
      bltouch_command(BLTOUCH_DEPLOY);   // Also needs to deploy and stow to
      bltouch_command(BLTOUCH_STOW);     // clear the triggered condition.
      printer.safe_delay(1500);                  // wait for internal self test to complete
                                         //   measured completion time was 0.65 seconds
                                         //   after reset, deploy & stow sequence
      if (TEST_BLTOUCH()) {              // If it still claims to be triggered...
        SERIAL_LM(ER, MSG_STOP_BLTOUCH);
        printer.Stop();
        return true;
      }
    }

    bltouch_command(deploy ? BLTOUCH_DEPLOY : BLTOUCH_STOW);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("set_bltouch_deployed(", deploy);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    return false;
  }

#endif

void Probe::refresh_offset(const bool no_babystep/*=false*/) {

  static float last_z_offset = NAN;

  if (!isnan(last_z_offset)) {

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(BABYSTEP_ZPROBE_OFFSET) || MECH(DELTA)
      const float diff = offset[Z_AXIS] - last_z_offset;
    #endif

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      // Correct bilinear grid for new probe offset
      if (diff) {
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
          for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
            bedlevel.z_values[x][y] -= diff;
      }
      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        bedlevel.virt_interpolate();
      #endif
    #endif

    #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
      if (!no_babystep && bedlevel.leveling_is_active())
        mechanics.babystep_axis(Z_AXIS, -LROUND(diff * mechanics.axis_steps_per_mm[Z_AXIS]));
    #else
      UNUSED(no_babystep);
    #endif
  }

  last_z_offset = offset[Z_AXIS];
}

#if ENABLED(Z_PROBE_ALLEN_KEY)

  void Probe::run_deploy_moves_script() {

    const float z_probe_deploy_start_location[] = Z_PROBE_DEPLOY_START_LOCATION,
                z_probe_deploy_end_location[] = Z_PROBE_DEPLOY_END_LOCATION;

    // Move to the start position to initiate deployment
    mechanics.do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    // Move to engage deployment
    mechanics.do_blocking_move_to(z_probe_deploy_end_location, mechanics.homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move to trigger deployment
    mechanics.do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);
  }
  void run_stow_moves_script() {

    const float z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION,
                z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;

    // Move to the start position to initiate retraction
    mechanics.do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    // Move the nozzle down to push the Z probe into retracted position
    mechanics.do_blocking_move_to(z_probe_retract_end_location, mechanics.homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move up for safety
    mechanics.do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);
  }

#endif

#if HAS_Z_PROBE_SLED

  #if DISABLED(SLED_DOCKING_OFFSET)
    #define SLED_DOCKING_OFFSET 0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.
   *
   * stow[in]     If false, move to MAX_ and engage the solenoid
   *              If true, move to MAX_X and release the solenoid
   */
  void Probe::dock_sled(bool stow) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("dock_sled(", stow);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    // Dock sled a bit closer to ensure proper capturing
    mechanics.do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));
    WRITE(SLED_PIN, !stow); // switch solenoid
  }

#endif // Z_PROBE_SLED
