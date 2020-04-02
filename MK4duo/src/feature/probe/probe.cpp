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
 * probe.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

Probe probe;

/** Public Parameters */
probe_data_t Probe::data;

/** Public Function */
void Probe::factory_parameters() {
  data.offset.set(X_PROBE_OFFSET_FROM_NOZZLE, Y_PROBE_OFFSET_FROM_NOZZLE, Z_PROBE_OFFSET_FROM_NOZZLE);
  data.speed_fast     = Z_PROBE_SPEED_FAST;
  data.speed_slow     = Z_PROBE_SPEED_SLOW;
  data.repetitions    = Z_PROBE_REPETITIONS;
}

// returns false for ok and true for failure
bool Probe::set_deployed(const bool deploy) {

  if (printer.debugFeature()) {
    DEBUG_POS("probe.set_deployed", mechanics.position);
    DEBUG_ELOGIC("deploy", deploy);
  }

  if (endstops.isProbeEnabled() == deploy) return false;

  // Make room for probe to deploy (or stow)
  // Fix-mounted probe should only raise for deploy
  #if HAS_PROBE_FIX && DISABLED(PAUSE_BEFORE_DEPLOY_STOW)
    const bool deploy_stow_condition = deploy;
  #else
    constexpr bool deploy_stow_condition = true;
  #endif

  if (deploy_stow_condition)
    do_raise(MAX(Z_PROBE_BETWEEN_HEIGHT, Z_PROBE_DEPLOY_HEIGHT));

  #if HAS_SLED
    if (mechanics.axis_unhomed_error(HOME_X)) {
      SERIAL_LM(ER, STR_STOP_UNHOMED);
      printer.stop();
      return true;
    }
  #elif HAS_ALLEN_KEY
    if (mechanics.axis_unhomed_error()) {
      SERIAL_LM(ER, STR_STOP_UNHOMED);
      printer.stop();
      return true;
    }
  #endif

  mechanics.stored_position[0] = mechanics.position;

  #if HAS_ALLEN_KEY

    #if HAS_Z_PROBE_PIN
      #define PROBE_STOWED() (READ(Z_PROBE_PIN) != endstops.isLogic(Z_PROBE))
    #else
      #define PROBE_STOWED() (READ(Z_MIN_PIN) != endstops.isLogic(Z_MIN))
    #endif

    // Only deploy/stow if needed
    if (PROBE_STOWED() == deploy) {
      if (!deploy) endstops.setProbeEnabled(false); // Switch off triggered when stowed probes early
                                                    // otherwise an Allen-Key probe can't be stowed.
      specific_action(deploy);
    }

    if (PROBE_STOWED() == deploy) {
      if (printer.isRunning()) {
        SERIAL_LM(ER, "Z-Probe failed");
        LCD_ALERTMESSAGEPGM_P(PSTR("Err: ZPROBE"));
        sound.feedback(false);
      }
      printer.stop();
      return true;
    }

  #else

    specific_action(deploy);

  #endif

  mechanics.do_blocking_move_to(mechanics.stored_position[0].x, mechanics.stored_position[0].y, mechanics.position.z); // return to position before deploy
  endstops.setProbeEnabled(deploy);
  return false;
}

#if Z_PROBE_AFTER_PROBING > 0
  // After probing move to a preferred Z position
  void Probe::move_z_after_probing() {
    if (mechanics.position.z != Z_PROBE_AFTER_PROBING) {
      mechanics.do_blocking_move_to_z(Z_PROBE_AFTER_PROBING);
      mechanics.position.z = Z_PROBE_AFTER_PROBING;
    }
  }
#endif

#if HAS_BED_PROBE || HAS_PROBE_MANUALLY

  /**
   * check_at_point (ex probe_pt)
   * - Move to the given XY
   * - Deploy the probe, if not already deployed
   * - Probe the bed, get the Z position
   * - Depending on the 'stow' flag
   *   - Stow the probe, or
   *   - Raise to the BETWEEN height
   * - short_move Flag for a shorter probe move towards the bed
   * - Return the probed Z position
   */

  float Probe::check_at_point(const float &rx, const float &ry, const ProbePtRaiseEnum raise_after/*=PROBE_PT_NONE*/, const uint8_t verbose_level/*=0*/, const bool probe_relative/*=true*/) {

    #if HAS_BED_PROBE

      if (printer.debugFeature()) {
        DEBUG_MV(">>> check_at_point(", LOGICAL_X_POSITION(rx));
        DEBUG_MV(", ", LOGICAL_Y_POSITION(ry));
        DEBUG_MT(", ", raise_after == PROBE_PT_RAISE ? "raise" : raise_after == PROBE_PT_STOW ? "stow" : "none");
        DEBUG_MV(", ", int(verbose_level));
        DEBUG_MT(", ", probe_relative ? "probe" : "nozzle");
        DEBUG_EM("_relative)");
        DEBUG_POS("", mechanics.position);
      }

      #if HAS_BLTOUCH && ENABLED(BLTOUCH_HIGH_SPEED_MODE)
        if (bltouch.triggered()) bltouch.cmd_reset();
      #endif

      xyz_pos_t npos = { rx, ry };
      if (probe_relative) {
        if (!mechanics.position_is_reachable_by_probe(npos)) {
          print_error();
          return NAN;
        }
        npos -= data.offset;
      }
      else if (!mechanics.position_is_reachable(npos)) {
        print_error();
        return NAN;
      }

      #if MECH(DELTA)
        // Move below clip height or xy move will be aborted by do_blocking_move_to
        npos.z = MIN(mechanics.position.z, mechanics.delta_clip_start_height);
      #else
        npos.z = mechanics.position.z;
      #endif

      const float old_feedrate_mm_s = mechanics.feedrate_mm_s;
      mechanics.feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

      // Move the probe to the starting XYZ
      mechanics.do_blocking_move_to(npos);

      float measured_z = NAN;
      if (!DEPLOY_PROBE()) measured_z = run_probing() + data.offset.z;
      if (!isnan(measured_z)) {
        if (raise_after == PROBE_PT_RAISE)
          mechanics.do_blocking_move_to_z(mechanics.position.z + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(data.speed_fast));
        else if (raise_after == PROBE_PT_STOW && STOW_PROBE())
          measured_z = NAN;

        if (verbose_level > 2) {
          SERIAL_MV(STR_BED_LEVELING_Z, measured_z, 3);
          SERIAL_MV(STR_BED_LEVELING_X, LOGICAL_X_POSITION(rx), 3);
          SERIAL_MV(STR_BED_LEVELING_Y, LOGICAL_Y_POSITION(ry), 3);
          SERIAL_EOL();
        }
      }

      mechanics.feedrate_mm_s = old_feedrate_mm_s;

      if (isnan(measured_z)) {
        STOW_PROBE();
        SERIAL_LM(ER, STR_ERR_PROBING_FAILED);
        LCD_MESSAGEPGM(MSG_LCD_PROBING_FAILED);
        sound.feedback(false);
      }

      if (printer.debugFeature()) DEBUG_EM("<<< check_at_point");

      return measured_z;

    #elif HAS_PROBE_MANUALLY

      UNUSED(raise_after);
      UNUSED(verbose_level);
      UNUSED(probe_relative);

      float measured_z = NAN;

      // Disable software endstops to allow manual adjustment
      #if HAS_SOFTWARE_ENDSTOPS
        const bool old_enable_soft_endstops = endstops.isSoftEndstop();
        endstops.setSoftEndstop(false);
      #endif

      xy_pos_t pos = { rx, ry };
      measured_z = lcd_probe_pt(pos);

      // Restore the soft endstop status
      #if HAS_SOFTWARE_ENDSTOPS
        endstops.setSoftEndstop(old_enable_soft_endstops);
      #endif

      return measured_z;

    #endif // HAS_PROBE_MANUALLY

  }

#endif // HAS_BED_PROBE || HAS_PROBE_MANUALLY

#if QUIET_PROBING

  void Probe::set_paused(const bool onoff) {
    #if ENABLED(PROBING_HEATERS_OFF)
      tempManager.pause(onoff);
    #endif
    #if ENABLED(PROBING_FANS_OFF)
      LOOP_FAN() fans[f]->setIdle(onoff);
    #endif
    if (onoff) HAL::delayMilliseconds(25);
  }

#endif // QUIET_PROBING

void Probe::print_M851() {
  SERIAL_LM(CFG, "Probe Offset X Y Z, speed Fast and Slow [mm/min], Repetitions");
  SERIAL_SM(CFG, "  M851");
  SERIAL_MV(" X", LINEAR_UNIT(data.offset.x), 3);
  SERIAL_MV(" Y", LINEAR_UNIT(data.offset.y), 3);
  SERIAL_MV(" Z", LINEAR_UNIT(data.offset.z), 3);
  SERIAL_MV(" F", data.speed_fast);
  SERIAL_MV(" S", data.speed_slow);
  SERIAL_MV(" R", data.repetitions);
  SERIAL_EOL();
}

void Probe::servo_test() {

  #if !(HAS_SERVO_0)

    SERIAL_LM(ER, "SERVO not setup");

  #elif !HAS_Z_SERVO_PROBE

    SERIAL_LM(ER, "PROBE_SERVO_NR not setup");

  #elif DISABLED(BLTOUCH)// HAS_Z_SERVO_PROBE

    const uint8_t probe_index = parser.seen('P') ? parser.value_byte() : PROBE_SERVO_NR;

    SERIAL_EM("Servo probe test.");
    SERIAL_EMV(".  Using index:  ", probe_index);
    SERIAL_EMV(".  Deploy angle: ", servo[probe_index].angle[0]);
    SERIAL_EMV(".  Stow angle:   ", servo[probe_index].angle[1]);

    bool probe_logic, deploy_state, stow_state;

    #if HAS_Z_PROBE_PIN

      #define PROBE_TEST_PIN Z_PROBE_PIN
      probe_logic = endstops.isLogic(Z_PROBE);
      SERIAL_EMV(". Probe uses Z_MIN_PROBE_PIN: ", PROBE_TEST_PIN);
      SERIAL_EM(". Uses Z_PROBE_ENDSTOP_LOGIC (ignores Z_MIN_ENDSTOP_LOGIC)");
      SERIAL_ELOGIC(". Z_MIN_ENDSTOP_LOGIC", probe_logic);

    #elif HAS_Z_MIN

      #define PROBE_TEST_PIN Z_MIN_PIN
      probe_logic = endstops.isLogic(Z_MIN);
      SERIAL_EMV(". Probe uses Z_MIN pin: ", PROBE_TEST_PIN);
      SERIAL_EM(". Uses Z_MIN_ENDSTOP_LOGIC (ignores Z_PROBE_ENDSTOP_LOGIC)");
      SERIAL_ELOGIC(". Z_MIN_ENDSTOP_LOGIC", probe_logic);

    #endif

    // DEPLOY and STOW 4 times and see if the signal follows
    // Then it is a mechanical switch
    uint8_t i = 0;
    SERIAL_EM(". Deploy & stow 4 times");
    do {
      MOVE_SERVO(probe_index, servo[probe_index].angle[0]); // Deploy
      HAL::delayMilliseconds(500);
      deploy_state = HAL::digitalRead(PROBE_TEST_PIN);
      MOVE_SERVO(probe_index, servo[probe_index].angle[1]); // Stow
      HAL::delayMilliseconds(500);
      stow_state = HAL::digitalRead(PROBE_TEST_PIN);
    } while (++i < 4);

    if (probe_logic != deploy_state) SERIAL_EM("WARNING: INVERTING setting probably backwards.");

    if (deploy_state != stow_state) {
      if (deploy_state) {
        SERIAL_EM(".  DEPLOYED state: HIGH (logic 1)");
        SERIAL_EM(".  STOWED (triggered) state: LOW (logic 0)");
      }
      else {
        SERIAL_EM(".  DEPLOYED state: LOW (logic 0)");
        SERIAL_EM(".  STOWED (triggered) state: HIGH (logic 1)");
      }
      return;
    }

    // Ask the user for a trigger event and measure the pulse width.
    // Since it could be a real servo or a BLTouch (any kind) or clone
    // use only "common" functions - i.e. SERVO_MOVE. No bltouch.xxxx stuff.
    MOVE_SERVO(probe_index, servo[probe_index].angle[0]); // Deploy
    HAL::delayMilliseconds(500);
    SERIAL_EM("** Please trigger probe within 30 sec **");
    uint16_t probe_counter = 0;

    // Wait 30 seconds for user to trigger probe
    for (uint16_t j = 0; j < 500 * 30 && probe_counter == 0 ; j++) {

      HAL::delayMilliseconds(2);

      if (0 == j % (500 * 1)) printer.reset_move_timer();       // Keep steppers powered

      if (deploy_state != HAL::digitalRead(PROBE_TEST_PIN)) {   // probe triggered

        for (probe_counter = 1; probe_counter < 15 && deploy_state != HAL::digitalRead(PROBE_TEST_PIN); ++probe_counter)
          HAL::delayMilliseconds(2);

        SERIAL_EMV(". Pulse width (+/- 4mS): ", probe_counter * 2);

        if (probe_counter >= 4) SERIAL_MSG("= Z Servo Probe detected.");
        else SERIAL_EM("FAIL: Noise detected - please re-run test");

        MOVE_SERVO(probe_index, servo[probe_index].angle[1]); // Stow

        return;

      }  // pulse detected

    } // for loop waiting for trigger

    if (probe_counter == 0) SERIAL_LM(ER, " Trigger not detected");

  #endif // HAS_Z_SERVO_PROBE

} // servo_probe_test

/** Private Function */

/**
 * returns false for ok and true for failure
 */
void Probe::specific_action(const bool deploy) {

  #if ENABLED(PAUSE_BEFORE_DEPLOY_STOW)

    sound.feedback();

    PGM_P const ds_str = deploy ? GET_TEXT(MSG_MANUAL_DEPLOY) : GET_TEXT(MSG_MANUAL_STOW);
    lcdui.return_to_status();
    lcdui.set_status_P(ds_str);
    SERIAL_STR(ds_str);
    SERIAL_EOL();
    printer.wait_for_user_response();
    lcdui.reset_status();

  #endif // PAUSE_BEFORE_DEPLOY_STOW

  #if HAS_SLED
    dock_sled(!deploy);
  #elif HAS_BLTOUCH && ENABLED(BLTOUCH_HIGH_SPEED_MODE)
    deploy ? bltouch.cmd_deploy() : bltouch.cmd_stow();
  #elif HAS_Z_SERVO_PROBE && DISABLED(BLTOUCH)
    MOVE_SERVO(PROBE_SERVO_NR, servo[PROBE_SERVO_NR].angle[(deploy ? 0 : 1)]);
  #elif HAS_ALLEN_KEY
    deploy ? run_deploy_moves_script() : run_stow_moves_script();
  #elif DISABLED(PAUSE_BEFORE_DEPLOY_STOW)
    UNUSED(deploy);
  #endif
}

/**
 * Used by run_probing to do a single Z probe move.
 *
 * z        Z destination
 * fr_mm_s  Feedrate in mm/s
 *
 * return true if the probe failed to trigger
 */
bool Probe::down_to_z(const float z, const feedrate_t fr_mm_s) {

  if (printer.debugFeature()) DEBUG_POS(">>> probe.down_to_z", mechanics.position);

  // Deploy BLTouch at the start of any probe
  #if HAS_BLTOUCH && DISABLED(BLTOUCH_HIGH_SPEED_MODE)
     if (bltouch.deploy()) return true;
  #endif

  // Disable stealthChop if used. Enable diag1 pin on driver.
  #if ENABLED(PROBE_SENSORLESS)
    sensorless_flag_t stealth_states;
    #if MECH(DELTA)
      stealth_states.x = tmcManager.enable_stallguard(driver.x);
      stealth_states.y = tmcManager.enable_stallguard(driver.y);
    #endif
    stealth_states.z = tmcManager.enable_stallguard(driver.z);
    endstops.setEnabled(true);
  #endif

  #if QUIET_PROBING
    set_paused(true);
  #endif

  // Move down until probe triggered
  mechanics.do_blocking_move_to_z(z, fr_mm_s);

  // Check to see if the probe was triggered
  const bool probe_triggered =
    #if MECH(DELTA) && ENABLED(PROBE_SENSORLESS)
      endstops.trigger_state() & (_BV(X_MAX) | _BV(Y_MAX) | _BV(Z_MAX))
    #else
      TEST(endstops.trigger_state(),
        #if HAS_Z_PROBE_PIN
          Z_PROBE
        #else
          Z_MIN
        #endif
      )
    #endif
  ;

  #if QUIET_PROBING
    set_paused(false);
  #endif

  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(PROBE_SENSORLESS)
    #if MECH(DELTA)
      tmcManager.disable_stallguard(driver.x, stealth_states.x);
      tmcManager.disable_stallguard(driver.y, stealth_states.y);
    #endif
    tmcManager.disable_stallguard(driver.z, stealth_states.z);
  #endif

  // Retract BLTouch immediately after a probe if it was triggered
  #if HAS_BLTOUCH && DISABLED(BLTOUCH_HIGH_SPEED_MODE)
    if (probe_triggered && bltouch.stow()) return true;
  #endif

  // Clear endstop flags
  endstops.hit_on_purpose();

  // Get Z where the steppers were interrupted
  mechanics.set_position_from_steppers_for_axis(Z_AXIS);

  // Tell the planner where we actually are
  mechanics.sync_plan_position();

  if (printer.debugFeature()) {
    DEBUG_ELOGIC(" Probe triggered", probe_triggered);
    DEBUG_POS("<<< probe.down_to_z", mechanics.position);
  }

  return !probe_triggered;
}

/**
 * Raise Z to a minimum height to make room for a probe to move
 */
void Probe::do_raise(const float z_raise) {

  if (printer.debugFeature()) {
    DEBUG_MV("probe.do_raise(", z_raise);
    DEBUG_CHR(')');
    DEBUG_EOL();
  }

  float z_dest = z_raise;
  if (data.offset.z < 0) z_dest -= data.offset.z;

  NOMORE(z_dest, Z_MAX_BED);

  if (z_dest > mechanics.position.z)
    mechanics.do_blocking_move_to_z(z_dest);
}

/**
 * Used by check_at_point to do a single Z probe at the current position.
 * Leaves position.z at the height where the probe triggered.
 *
 * return The raw Z position where the probe was triggered
 */
float Probe::run_probing() {

  float probe_z = 0.0f;

  if (printer.debugFeature()) DEBUG_POS(">>> probe.run_probing", mechanics.position);

  // Stop the probe before it goes too low to prevent damage.
  // If Z isn't known then probe to -10mm.
  const float z_probe_low_point = mechanics.isAxisHomed(Z_AXIS) ? Z_PROBE_LOW_POINT - data.offset.z : -10.0;

  // If the nozzle is well over the travel height then
  // move down quickly before doing the slow probe
  float z = Z_PROBE_DEPLOY_HEIGHT + 5.0 + (data.offset.z < 0 ? -data.offset.z : 0);
  if (mechanics.position.z > z) {
    if (!down_to_z(z, MMM_TO_MMS(data.speed_fast)))
      mechanics.do_blocking_move_to_z(mechanics.position.z + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(data.speed_fast));
  }

  for (uint8_t r = data.repetitions + 1; --r;) {

    // move down slowly to find bed
    if (down_to_z(z_probe_low_point, MMM_TO_MMS(data.speed_slow))) {
      if (printer.debugFeature()) {
        DEBUG_EM("SLOW Probe fail!");
        DEBUG_POS("<<< probe.run_probing", mechanics.position);
      }
      return NAN;
    }

    probe_z += mechanics.position.z;
    if (r > 1) mechanics.do_blocking_move_to_z(mechanics.position.z + Z_PROBE_BETWEEN_HEIGHT, MMM_TO_MMS(data.speed_fast));

  }

  return probe_z / (float)data.repetitions;
}

void Probe::print_error() {
  #if MECH(DELTA)
    SERIAL_LM(ER, "Delta probeable radius not reachable!");
  #else
    SERIAL_LM(ER, "Probe range not reachable!");
  #endif
}

#if HAS_ALLEN_KEY

  void Probe::run_deploy_moves_script() {

    constexpr xyz_pos_t z_probe_deploy_start_location = Z_PROBE_DEPLOY_START_LOCATION,
                        z_probe_deploy_end_location   = Z_PROBE_DEPLOY_END_LOCATION;

    // Move to the start position to initiate deployment
    mechanics.do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s.z);

    // Move to engage deployment
    mechanics.do_blocking_move_to(z_probe_deploy_end_location, mechanics.homing_feedrate_mm_s.z / 10);

    // Move to trigger deployment
    mechanics.do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s.z);

  }

  void Probe::run_stow_moves_script() {

    constexpr xyz_pos_t z_probe_retract_start_location  = Z_PROBE_RETRACT_START_LOCATION,
                        z_probe_retract_end_location    = Z_PROBE_RETRACT_END_LOCATION;

    // Move to the start position to initiate retraction
    mechanics.do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s.z);

    // Move the nozzle down to push the Z probe into retracted position
    mechanics.do_blocking_move_to(z_probe_retract_end_location, mechanics.homing_feedrate_mm_s.z / 10);

    // Move up for safety
    mechanics.do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s.z);

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
    if (printer.debugFeature()) {
      DEBUG_MV("dock_sled(", stow);
      DEBUG_CHR(')'); DEBUG_EOL();
    }

    // Dock sled a bit closer to ensure proper capturing
    mechanics.do_blocking_move_to_x(X_MAX_BED + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));
    WRITE(SLED_PIN, !stow); // switch solenoid
  }

#endif // PROBE_SLED
