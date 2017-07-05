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
#include "probe.h"

#if HAS_BED_PROBE

  Probe probe;

  float Probe::z_offset = Z_PROBE_OFFSET_FROM_NOZZLE;
  bool  Probe::enabled  = false;

  #if HAS_Z_SERVO_PROBE
    const int Probe::z_servo_angle[2] = Z_ENDSTOP_SERVO_ANGLES;
  #endif

  // returns false for ok and true for failure
  bool Probe::set_deployed(bool deploy) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS("probe.set_deployed", mechanics.current_position);
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
      if (mechanics.axis_unhomed_error(true, false, false)) {
        SERIAL_LM(ER, MSG_STOP_UNHOMED);
        Stop();
        return true;
      }
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      if (mechanics.axis_unhomed_error(true, true,  true )) {
        SERIAL_LM(ER, MSG_STOP_UNHOMED);
        Stop();
        return true;
      }
    #endif

    const float oldXpos = mechanics.current_position[X_AXIS],
                oldYpos = mechanics.current_position[Y_AXIS];

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

    mechanics.do_blocking_move_to(oldXpos, oldYpos, mechanics.current_position[Z_AXIS]); // return to position before deploy
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

    if (z_dest > mechanics.current_position[Z_AXIS])
      mechanics.do_blocking_move_to_z(z_dest);
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
  float Probe::check_pt(const float &lx, const float &ly, const bool stow/*=true*/, const int verbose_level/*=1*/, const bool printable/*=true*/) {

    #if ENABLED(PROBE_MANUALLY)

      UNUSED(stow);
      UNUSED(verbose_level);
      UNUSED(printable);

      // Ensure a minimum height before moving the probe
      raise(Z_PROBE_BETWEEN_HEIGHT);

      // Move the probe to the given XY
      mechanics.manual_goto_xy(lx, ly);

      // Disable software endstops to allow manual adjustment
      #if HAS_SOFTWARE_ENDSTOPS
        const bool old_enable_soft_endstops = endstops.soft_endstops_enabled;
        endstops.soft_endstops_enabled = false;
      #endif

      KEEPALIVE_STATE(PAUSED_FOR_USER);
      wait_for_user = true;

      #if ENABLED(ULTIPANEL)
        lcd_move_menu_01mm();
      #elif ENABLED(NEXTION)
        LcdBedLevelOn();
      #endif

      while (wait_for_user) idle();

      #if ENABLED(NEXTION)
        LcdBedLevelOff();
      #endif

      KEEPALIVE_STATE(IN_HANDLER);

      // Re-enable software endstops, if needed
      #if HAS_SOFTWARE_ENDSTOPS
        endstops.soft_endstops_enabled = old_enable_soft_endstops;
      #endif

      return RAW_CURRENT_POSITION(Z);

    #else // !ENABLED(PROBE_MANUALLY)
  
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_MV(">>> check_pt(", lx);
          SERIAL_MV(", ", ly);
          SERIAL_MV(", ", stow ? "" : "no ");
          SERIAL_EM("stow)");
          DEBUG_POS("", mechanics.current_position);
        }
      #endif

      const float dx = lx - (X_PROBE_OFFSET_FROM_NOZZLE),
                  dy = ly - (Y_PROBE_OFFSET_FROM_NOZZLE);

      if (printable)
        if (!mechanics.position_is_reachable_by_probe_xy(lx, ly)) return NAN;
      else
        if (!mechanics.position_is_reachable_xy(dx, dy)) return NAN;

      const float old_feedrate_mm_s = mechanics.feedrate_mm_s;

      #if MECH(DELTA)
        if (mechanics.current_position[Z_AXIS] > mechanics.delta_clip_start_height)
          mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
      #endif

      // Ensure a minimum height before moving the probe
      raise(Z_PROBE_BETWEEN_HEIGHT);

      mechanics.feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

      // Move the probe to the given XY
      mechanics.do_blocking_move_to_xy(dx, dy);

      if (set_deployed(true)) return NAN;

      float measured_z = 0.0;

      // Prevent stepper_inactive_time from running out and EXTRUDER_RUNOUT_PREVENT from extruding
      refresh_cmd_timeout();

      // If the nozzle is above the travel height then
      // move down quickly before doing the slow probe
      float z = LOGICAL_Z_POSITION(Z_PROBE_BETWEEN_HEIGHT);
      if (z_offset < 0) z -= z_offset;
      if (z < mechanics.current_position[Z_AXIS])
        mechanics.do_blocking_move_to_z(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST));

      for (int8_t r = 0; r < Z_PROBE_REPETITIONS; r++) {

        // move down slowly to find bed
        move_to_z(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_SLOW);

        measured_z += RAW_CURRENT_POSITION(Z) + z_offset;

        if (r + 1 < Z_PROBE_REPETITIONS) {
          // move up by the bump distance
          mechanics.do_blocking_move_to_z(mechanics.current_position[Z_AXIS] + mechanics.home_bump_mm[Z_AXIS], MMM_TO_MMS(Z_PROBE_SPEED_FAST));
        }
      }

      measured_z /= (float)Z_PROBE_REPETITIONS;

      if (!stow)
        raise(Z_PROBE_BETWEEN_HEIGHT);
      else
        if (set_deployed(false)) return NAN;

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

      return measured_z;

    #endif // !ENABLED(PROBE_MANUALLY)

  }

  /**
   * Do a single Z probe
   * Usage:
   *    G30 <X#> <Y#> <S#> <Z#> <P#>
   *      X = Probe X position (default=current probe position)
   *      Y = Probe Y position (default=current probe position)
   *      S = <bool> Stows the probe if 1 (default=1)
   *      Z = <bool> with a non-zero value will apply the result to current delta_height (ONLY DELTA)
   *      P = <bool> with a non-zero value will apply the result to current probe.z_offset (ONLY DELTA)
   */
  void Probe::single_probe() {

    const float xpos = parser.seen('X') ? parser.value_linear_units() : mechanics.current_position[X_AXIS] + X_PROBE_OFFSET_FROM_NOZZLE,
                ypos = parser.seen('Y') ? parser.value_linear_units() : mechanics.current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_NOZZLE;

    // Don't allow G30 without homing first
    if (mechanics.axis_unhomed_error()) return;

    if (!mechanics.position_is_reachable_by_probe_xy(xpos, ypos)) return;

    // Disable leveling so the planner won't mess with us
    #if HAS_LEVELING
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    setup_for_endstop_or_probe_move();

    const float measured_z = check_pt(xpos, ypos, !parser.seen('S') || parser.value_bool(), 1);

    if (!isnan(measured_z)) {
      SERIAL_MV(MSG_BED_LEVELING_Z, FIXFLOAT(measured_z), 3);
      SERIAL_MV(MSG_BED_LEVELING_X, FIXFLOAT(xpos), 3);
      SERIAL_MV(MSG_BED_LEVELING_Y, FIXFLOAT(ypos), 3);
    }

    #if IS_DELTA
      if (parser.boolval('Z')) {
        mechanics.delta_height -= measured_z;
        mechanics.recalc_delta_settings();
        SERIAL_MV("  New delta height = ", mechanics.delta_height, 3);
      }
      else if (parser.boolval('P')) {
        probe.z_offset += endstops.soft_endstop_min[Z_AXIS] - measured_z;
        SERIAL_MV("  New Z probe offset = ", z_offset, 3);
      }
    #endif

    SERIAL_EOL();

    clean_up_after_endstop_or_probe_move();

    mechanics.report_current_position();
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
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> move_to_z", mechanics.current_position);
    #endif

    // Deploy BLTouch at the start of any probe
    #if ENABLED(BLTOUCH) && NOMECH(DELTA)
      set_bltouch_deployed(true);
    #endif

    // Move down until probe triggered
    mechanics.do_blocking_move_to_z(LOGICAL_Z_POSITION(z), MMM_TO_MMS(fr_mm_m));

    // Retract BLTouch immediately after a probe
    #if ENABLED(BLTOUCH) && NOMECH(DELTA)
      set_bltouch_deployed(false);
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
  }

  void Probe::refresh_zprobe_zoffset() {

    static float last_z_offset = NAN;

    if (!isnan(last_z_offset)) {

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        const float diff = z_offset - last_z_offset;

        // Correct bilinear grid for new probe offset
        if (diff) {
          for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
            for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
              bedlevel.z_values[x][y] -= diff;
        }
        #if ENABLED(ABL_BILINEAR_SUBDIVISION)
          bedlevel.bed_level_virt_interpolate();
        #endif
      #endif

    }

    last_z_offset = z_offset;
  }

#endif // HAS_BED_PROBE
