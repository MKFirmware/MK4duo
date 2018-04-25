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

#include "../../../MK4duo.h"

Probe probe;

float Probe::offset[XYZ] = { X_PROBE_OFFSET_FROM_NOZZLE, Y_PROBE_OFFSET_FROM_NOZZLE, Z_PROBE_OFFSET_FROM_NOZZLE };

#if HAS_Z_SERVO_PROBE
  const int Probe::z_servo_angle[2] = Z_ENDSTOP_SERVO_ANGLES;
#endif

#if ENABLED(Z_PROBE_ALLEN_KEY)

  FORCE_INLINE void do_blocking_move_to(const float raw[XYZ], const float &fr_mm_s/*=0.0*/) {
    mechanics.do_blocking_move_to(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS], fr_mm_s);
  }

  void Probe::run_deploy_moves_script() {

    const float z_probe_deploy_start_location[]  = Z_PROBE_DEPLOY_START_LOCATION,
                z_probe_deploy_end_location[]    = Z_PROBE_DEPLOY_END_LOCATION;

    // Move to the start position to initiate deployment
    do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    // Move to engage deployment
    do_blocking_move_to(z_probe_deploy_end_location, mechanics.homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move to trigger deployment
    do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);
  }
  void run_stow_moves_script() {

    const float z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION,
                z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;

    // Move to the start position to initiate retraction
    do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    // Move the nozzle down to push the Z probe into retracted position
    do_blocking_move_to(z_probe_retract_end_location, mechanics.homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move up for safety
    do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);
  }

#endif

// returns false for ok and true for failure
bool Probe::set_deployed(const bool deploy) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) {
      DEBUG_POS("probe.set_deployed", mechanics.current_position);
      SERIAL_EMV("deploy: ", deploy);
    }
  #endif

  if (endstops.isProbeEndstop() == deploy) return false;

  // Make room for probe
  float z_dest = _Z_PROBE_DEPLOY_HEIGHT;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) {
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
      if (!deploy) endstops.setProbeEndstop(false);  // Switch off triggered when stowed probes early
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
      if (printer.isRunning()) {
        SERIAL_LM(ER, "Z-Probe failed");
        LCD_ALERTMESSAGEPGM("Err: ZPROBE");
      }
      printer.Stop();
      return true;
    }
  #endif

  mechanics.do_blocking_move_to(oldXpos, oldYpos, mechanics.current_position[Z_AXIS]); // return to position before deploy
  endstops.setProbeEndstop(deploy);
  return false;
}

#if Z_PROBE_AFTER_PROBING > 0
  // After probing move to a preferred Z position
  void Probe::move_z_after_probing() {
    if (mechanics.current_position[Z_AXIS] != Z_PROBE_AFTER_PROBING) {
      mechanics.do_blocking_move_to_z(Z_PROBE_AFTER_PROBING);
      mechanics.current_position[Z_AXIS] = Z_PROBE_AFTER_PROBING;
    }
  }
#endif

/**
 * @brief Used by run_z_probe to do a single Z probe move.
 *
 * @param  z        Z destination
 * @param  fr_mm_s  Feedrate in mm/s
 * @return true to indicate an error
 */
bool Probe::move_to_z(const float z, const float fr_mm_m) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) DEBUG_POS(">>> move_to_z", mechanics.current_position);
  #endif

  // Deploy BLTouch at the start of any probe
  #if ENABLED(BLTOUCH) && NOMECH(DELTA)
     if (set_bltouch_deployed(true)) return true;
  #endif

  #if QUIET_PROBING
    probing_pause(true);
  #endif

  #if MECH(DELTA)
    const float z_start = mechanics.current_position[Z_AXIS];
    const long steps_start[ABC] = {
      stepper.position(A_AXIS),
      stepper.position(B_AXIS),
      stepper.position(C_AXIS)
    };
  #endif

  // Move down until probe triggered
  mechanics.do_blocking_move_to_z(z, MMM_TO_MMS(fr_mm_m));

  // Check to see if the probe was triggered
  const bool probe_triggered = TEST(endstops.hit_bits,
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

  if (probe_triggered) {
    // Get Z where the steppers were interrupted
    #if MECH(DELTA)
      float z_dist = 0.0;
      LOOP_ABC(i)
        z_dist += FABS(steps_start[i] - stepper.position((AxisEnum)i)) / mechanics.axis_steps_per_mm[i];

      mechanics.current_position[Z_AXIS] = z_start - (z_dist / ABC);
    #else
      mechanics.set_current_from_steppers_for_axis(Z_AXIS);
    #endif

    // Tell the planner where we actually are
    mechanics.sync_plan_position_mech_specific();
  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) DEBUG_POS("<<< move_to_z", mechanics.current_position);
  #endif

  return !probe_triggered;
}

/**
 * @details Used by check_pt to do a single Z probe at the current position.
 *          Leaves current_position[Z_AXIS] at the height where the probe triggered.
 *
 * @return The raw Z position where the probe was triggered
 */
float Probe::run_z_probe() {

  float probe_z = 0.0;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (printer.debugLeveling()) DEBUG_POS(">>> run_z_probe", mechanics.current_position);
  #endif

  // If the nozzle is above the travel height then
  // move down quickly before doing the slow probe
  float z = Z_PROBE_DEPLOY_HEIGHT;
  if (offset[Z_AXIS] < 0) z -= offset[Z_AXIS];
  if (z < mechanics.current_position[Z_AXIS]) {
    if (!move_to_z(z, Z_PROBE_SPEED_FAST))
      mechanics.do_blocking_move_to_z(z + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
  }

  for (uint8_t r = 0; r < Z_PROBE_REPETITIONS; r++) {

    // move down slowly to find bed
    if (move_to_z(-10, Z_PROBE_SPEED_SLOW)) return NAN;

    probe_z += mechanics.current_position[Z_AXIS];

    if (r + 1 < Z_PROBE_REPETITIONS) {
      // move up to probe between height
      mechanics.do_blocking_move_to_z(mechanics.current_position[Z_AXIS] + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
    }
  }

  return probe_z * (1.0 / (Z_PROBE_REPETITIONS));
}

#if HAS_BED_PROBE || ENABLED(PROBE_MANUALLY)

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

  float Probe::check_pt(const float &rx, const float &ry, const ProbePtRaise raise_after/*=PROBE_PT_NONE*/, const uint8_t verbose_level/*=0*/, const bool probe_relative/*=true*/) {

    #if HAS_BED_PROBE

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) {
          SERIAL_MV(">>> check_pt(", LOGICAL_X_POSITION(rx));
          SERIAL_MV(", ", LOGICAL_Y_POSITION(ry));
          SERIAL_MT(", ", raise_after == PROBE_PT_RAISE ? "raise" : raise_after == PROBE_PT_STOW ? "stow" : "none");
          SERIAL_MV(", ", int(verbose_level));
          SERIAL_MT(", ", probe_relative ? "probe" : "nozzle");
          SERIAL_EM("_relative)");
          DEBUG_POS("", mechanics.current_position);
        }
      #endif

      float nx = rx, ny = ry;
      if (probe_relative) {
        if (!mechanics.position_is_reachable_by_probe(rx, ry)) return NAN;
        nx -= offset[X_AXIS];
        ny -= offset[Y_AXIS];
      }
      else if (!mechanics.position_is_reachable(nx, ny)) return NAN;

      const float nz = 
        #if MECH(DELTA)
          // Move below clip height or xy move will be aborted by do_blocking_move_to
          min(mechanics.current_position[Z_AXIS], mechanics.delta_clip_start_height)
        #else
          mechanics.current_position[Z_AXIS]
        #endif
      ;

      const float old_feedrate_mm_s = mechanics.feedrate_mm_s;
      mechanics.feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

      // Move the probe to the starting XYZ
      mechanics.do_blocking_move_to(nx, ny, nz);

      float measured_z = NAN;
      if (!set_deployed(true)) {
        measured_z = run_z_probe() + offset[Z_AXIS];

        if (raise_after == PROBE_PT_RAISE)
          mechanics.do_blocking_move_to_z(mechanics.current_position[Z_AXIS] + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(Z_PROBE_SPEED_FAST));
        else if (raise_after == PROBE_PT_STOW)
          if (set_deployed(false)) measured_z = NAN;
      }

      if (verbose_level > 2) {
        SERIAL_MV(MSG_BED_LEVELING_Z, FIXFLOAT(measured_z), 3);
        SERIAL_MV(MSG_BED_LEVELING_X, LOGICAL_X_POSITION(rx), 3);
        SERIAL_MV(MSG_BED_LEVELING_Y, LOGICAL_Y_POSITION(ry), 3);
        SERIAL_EOL();
      }

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (printer.debugLeveling()) SERIAL_EM("<<< check_pt");
      #endif

      mechanics.feedrate_mm_s = old_feedrate_mm_s;

      if (isnan(measured_z)) {
        LCD_MESSAGEPGM(MSG_ERR_PROBING_FAILED);
        SERIAL_LM(ER, MSG_ERR_PROBING_FAILED);
      }

      return measured_z;

    #elif ENABLED(PROBE_MANUALLY)

      UNUSED(stow);
      UNUSED(verbose_level);
      UNUSED(probe_relative);

      float measured_z = NAN;

      // Disable software endstops to allow manual adjustment
      #if HAS_SOFTWARE_ENDSTOPS
        const bool old_enable_soft_endstops = endstops.isSoftEndstop();
        endstops.setSoftEndstop(false);
      #endif

      measured_z = lcd_probe_pt(rx, ry);

      // Restore the soft endstop status
      #if HAS_SOFTWARE_ENDSTOPS
        endstops.setSoftEndstop(old_enable_soft_endstops);
      #endif

      return measured_z;

    #endif // ENABLED(PROBE_MANUALLY)

  }

#endif // HAS_BED_PROBE || ENABLED(PROBE_MANUALLY)

#if QUIET_PROBING
  void Probe::probing_pause(const bool onoff) {
    #if ENABLED(PROBING_HEATERS_OFF)
      thermalManager.pause(onoff);
    #endif
    #if ENABLED(PROBING_FANS_OFF)
      LOOP_FAN() fans[f].setIdle(onoff);
    #endif
    if (onoff) printer.safe_delay(25);
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
      printer.safe_delay(1500);          // wait for internal self test to complete
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
      if (printer.debugLeveling()) {
        SERIAL_MV("set_bltouch_deployed(", deploy);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    return false;
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
      if (printer.debugLeveling()) {
        SERIAL_MV("dock_sled(", stow);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    // Dock sled a bit closer to ensure proper capturing
    mechanics.do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));
    WRITE(SLED_PIN, !stow); // switch solenoid
  }

#endif // Z_PROBE_SLED
