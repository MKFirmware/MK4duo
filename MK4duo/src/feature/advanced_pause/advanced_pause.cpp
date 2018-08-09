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
 * advanced_pause.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  // private:

  static float resume_position[XYZE];

  AdvancedPauseMenuResponse advanced_pause_menu_response;

  float filament_change_unload_length[EXTRUDERS],
        filament_change_load_length[EXTRUDERS];

  #if HAS_BUZZER

    static void filament_change_beep(const int8_t max_beep_count, const bool init=false) {
      static millis_t next_buzz = 0;
      static int8_t runout_beep = 0;

      if (init) next_buzz = runout_beep = 0;

      const millis_t ms = millis();
      if (ELAPSED(ms, next_buzz)) {
        if (max_beep_count < 0 || runout_beep < max_beep_count + 5) { // Only beep as long as we're supposed to
          next_buzz = ms + ((max_beep_count < 0 || runout_beep < max_beep_count) ? 1000 : 500);
          BUZZ(50, 880 - (runout_beep & 1) * 220);
          runout_beep++;
        }
      }
    }

  #endif

  /**
   * Ensure a safe temperature for extrusion
   *
   * - Fail if the TARGET temperature is too low
   * - Display LCD placard with temperature status
   * - Return when heating is done or aborted
   *
   * Returns 'true' if heating was completed, 'false' for abort
   */
  static bool ensure_safe_temperature(const AdvancedPauseMode mode=ADVANCED_PAUSE_MODE_PAUSE_PRINT) {

    if (printer.debugDryrun()) return true;

    #if HAS_LCD
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT, mode);
    #else
      UNUSED(mode);
    #endif

    printer.setWaitForHeatUp(true);
    while (printer.isWaitForHeatUp() && heaters[TARGET_HOTEND].wait_for_heating()) printer.idle();
    const bool status = printer.isWaitForHeatUp();
    printer.setWaitForHeatUp(false);

    return status;
  }

  void do_pause_e_move(const float &length, const float fr) {
    mechanics.set_destination_to_current();
    mechanics.destination[E_AXIS] += length / tools.e_factor[tools.active_extruder];
    planner.buffer_line_kinematic(mechanics.destination, fr, tools.active_extruder);
    mechanics.set_current_to_destination();
    planner.synchronize();
  }

  /**
   * Load filament into the hotend
   *
   * - Fail if the a safe temperature was not reached
   * - If pausing for confirmation, wait for a click or M108
   * - Show "wait for load" placard
   * - Load and purge filament
   * - Show "Purge more" / "Continue" menu
   * - Return when "Continue" is selected
   *
   * Returns 'true' if load was completed, 'false' for abort
   */
  bool load_filament(const float &slow_load_length/*=0*/, const float &fast_load_length/*=0*/, const float &purge_length/*=0*/, const int8_t max_beep_count/*=0*/,
                     const bool show_lcd/*=false*/, const bool pause_for_user/*=false*/,
                     const AdvancedPauseMode mode/*=ADVANCED_PAUSE_MODE_PAUSE_PRINT*/
  ) {
    #if !HAS_LCD
      UNUSED(show_lcd);
    #endif

    if (!ensure_safe_temperature(mode)) {
      #if HAS_LCD
        if (show_lcd) // Show status screen
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
      #endif

      return false;
    }

    if (pause_for_user) {
      #if HAS_LCD
        if (show_lcd) // Show "insert filament"
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT, mode);
      #endif
      SERIAL_LM(ECHO, MSG_FILAMENT_CHANGE_INSERT);

      #if HAS_BUZZER
        filament_change_beep(max_beep_count, true);
      #else
        UNUSED(max_beep_count);
      #endif

      printer.keepalive(PausedforUser);
      printer.setWaitForUser(true);    // LCD click or M108 will clear this
      while (printer.isWaitForUser()) {
        #if HAS_BUZZER
          filament_change_beep(max_beep_count);
        #endif
        printer.idle(true);
      }
      printer.keepalive(InHandler);
    }

    #if HAS_LCD
      if (show_lcd) // Show "wait for load" message
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_LOAD, mode);
    #endif

    // Slow Load filament
    if (slow_load_length) do_pause_e_move(slow_load_length, PAUSE_PARK_SLOW_LOAD_FEEDRATE);

    // Fast Load Filament
    if (fast_load_length) do_pause_e_move(fast_load_length, PAUSE_PARK_FAST_LOAD_FEEDRATE);

    do {
      if (purge_length > 0) {
        // "Wait for filament purge"
        #if HAS_LCD
          if (show_lcd)
            lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_PURGE, mode);
        #endif

        // Extrude filament to get into hotend
        do_pause_e_move(purge_length, PAUSE_PARK_EXTRUDE_FEEDRATE);
      }

      // Show "Purge More" / "Resume" menu and wait for reply
      #if HAS_LCD
        if (show_lcd) {
          printer.keepalive(PausedforUser);
          printer.setWaitForUser(false);
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_OPTION, mode);
          while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_WAIT_FOR) printer.idle(true);
          printer.keepalive(InHandler);
        }
      #endif

      // Keep looping if "Purge More" was selected
    } while (
      #if HAS_LCD
        show_lcd && advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE
      #else
        0
      #endif
    );

    return true;
  }

  /**
   * Unload filament from the hotend
   *
   * - Fail if the a safe temperature was not reached
   * - Show "wait for unload" placard
   * - Retract, pause, then unload filament
   * - Disable E stepper (on most machines)
   *
   * Returns 'true' if unload was completed, 'false' for abort
   */
  bool unload_filament(const float &unload_length, const bool show_lcd/*=false*/,
                       const AdvancedPauseMode mode/*=ADVANCED_PAUSE_MODE_PAUSE_PRINT*/
  ) {
    if (!ensure_safe_temperature(mode)) {
      #if HAS_LCD
        if (show_lcd) // Show status screen
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
      #endif

      return false;
    }

    #if DISABLED(ULTIPANEL)
      UNUSED(show_lcd);
    #else
      if (show_lcd)
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_UNLOAD, mode);
    #endif

    // Retract filament
    do_pause_e_move(-FILAMENT_UNLOAD_RETRACT_LENGTH, PAUSE_PARK_RETRACT_FEEDRATE);

    // Wait for filament to cool
    printer.safe_delay(FILAMENT_UNLOAD_DELAY);

    // Quickly purge
    do_pause_e_move(FILAMENT_UNLOAD_RETRACT_LENGTH + FILAMENT_UNLOAD_PURGE_LENGTH, mechanics.max_feedrate_mm_s[E_AXIS]);

    // Unload filament
    do_pause_e_move(-unload_length, PAUSE_PARK_UNLOAD_FEEDRATE);

    // Disable extruders steppers for manual filament changing
    #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN
      stepper.disable_E();
      printer.safe_delay(100);
    #endif

    return true;
  }

  // public function

  /**
   * Pause procedure
   *
   * - Abort if already paused
   * - Send host action for pause, if configured
   * - Abort if TARGET temperature is too low
   * - Display "wait for start of filament change" (if a length was specified)
   * - Initial retract, if current temperature is hot enough
   * - Park the nozzle at the given position
   * - Call unload_filament (if a length was specified)
   *
   * Returns 'true' if pause was completed, 'false' for abort
   */
  uint8_t did_pause_print = 0;

  bool pause_print(const float &retract, const point_t &park_point, const float &unload_length/*=0*/, const bool show_lcd/*=false*/) {

    if (did_pause_print) return false; // already paused

    SERIAL_STR(PAUSE);
    SERIAL_EOL();

    #if HAS_LCD
      if (show_lcd) // Show initial message
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT);
    #endif

    if (!printer.debugDryrun() && unload_length && thermalManager.tooColdToExtrude(ACTIVE_HOTEND)) {
      SERIAL_LM(ER, MSG_HOTEND_TOO_COLD);

      #if HAS_LCD
        if (show_lcd) { // Show status screen
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
          LCD_MESSAGEPGM(MSG_HOTEND_TOO_COLD);
        }
      #endif

      return false; // unable to reach safe temperature
    }

    // Indicate that the printer is paused
    ++did_pause_print;

    // Pause the print job and timer
    #if HAS_SDSUPPORT
      if (IS_SD_PRINTING) {
        card.pauseSDPrint();
        ++did_pause_print;
      }
    #endif
    print_job_counter.pause();

    // Save current position
    COPY_ARRAY(resume_position, mechanics.current_position);

    // Wait for synchronize steppers
    planner.synchronize();

    // Initial retract before move to filament change position
    if (retract && !thermalManager.tooColdToExtrude(ACTIVE_HOTEND))
      do_pause_e_move(retract, PAUSE_PARK_RETRACT_FEEDRATE);

    // Park the nozzle by moving up by z_lift and then moving to (x_pos, y_pos)
    Nozzle::park(2, park_point);

    // Unload the filament
    if (unload_length)
      unload_filament(unload_length, show_lcd);

    return true;
  }

  /**
   * - Show "Insert filament and press button to continue"
   * - Wait for a click before returning
   * - Heaters can time out, reheated before accepting a click
   *
   * Used by M125 and M600
   */
  void wait_for_filament_reload(const int8_t max_beep_count/*=0*/) {
    bool nozzle_timed_out = false,
         bed_timed_out = false;

    #if HAS_LCD
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
    #endif
    SERIAL_LM(ECHO, MSG_FILAMENT_CHANGE_INSERT);

    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #else
      UNUSED(max_beep_count);
    #endif

    // Start the heater idle timers
    const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
    const millis_t bed_timeout    = (millis_t)(PAUSE_PARK_PRINTER_OFF) * 60000UL;

    LOOP_HOTEND()
      heaters[h].start_idle_timer(nozzle_timeout);

    #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
      heaters[BED_INDEX].start_idle_timer(bed_timeout);
    #endif

    // Wait for filament insert by user and press button
    printer.keepalive(PausedforUser);
    printer.setWaitForUser(true); // LCD click or M108 will clear this
    while (printer.isWaitForUser()) {
      #if HAS_BUZZER
        filament_change_beep(max_beep_count);
      #endif

      // If the nozzle has timed out, wait for the user to press the button to re-heat the nozzle, then
      // re-heat the nozzle, re-show the insert screen, restart the idle timers, and start over
      if (!nozzle_timed_out)
        LOOP_HOTEND()
          nozzle_timed_out |= heaters[h].isIdle();

      if (nozzle_timed_out) {
        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE);
        #endif
        #if HAS_LCD && ENABLED(EMERGENCY_PARSER)
          SERIAL_LM(ER, MSG_FILAMENT_CHANGE_HEAT);
        #elif ENABLED(EMERGENCY_PARSER)
          SERIAL_LM(ER, MSG_FILAMENT_CHANGE_HEAT_M108);
        #else
          SERIAL_LM(ER, MSG_FILAMENT_CHANGE_HEAT_LCD);
        #endif
        
        // Wait for LCD click or M108
        while (printer.isWaitForUser()) {

          if (!bed_timed_out) {
            #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
              bed_timed_out = heaters[BED_INDEX].isIdle();
            #endif
          }
          else {
            #if HAS_LCD
              lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_PRINTER_OFF);
            #endif
          }

          printer.idle(true);
        }
        
        // Re-enable the bed if they timed out
        #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
          if (bed_timed_out) {
            heaters[BED_INDEX].reset_idle_timer();
            #if HAS_LCD
              lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT);
            #endif
            thermalManager.wait_heater(&heaters[BED_INDEX]);
          }
        #endif

        // Re-enable the heaters if they timed out
        LOOP_HOTEND() heaters[h].reset_idle_timer();

        // Wait for the heaters to reach the target temperatures
        ensure_safe_temperature();

        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
        #endif
        #if HAS_LCD && ENABLED(EMERGENCY_PARSER)
          SERIAL_LM(ER, MSG_FILAMENT_CHANGE_INSERT);
        #elif ENABLED(EMERGENCY_PARSER)
          SERIAL_LM(ER, MSG_FILAMENT_CHANGE_INSERT_M108);
        #else
          SERIAL_LM(ER, MSG_FILAMENT_CHANGE_INSERT_LCD);
        #endif

        LOOP_HOTEND()
          heaters[h].start_idle_timer(nozzle_timeout);

        #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
          heaters[BED_INDEX].start_idle_timer(bed_timeout);
        #endif

        printer.setWaitForUser(true); /* Wait for user to load filament */
        nozzle_timed_out = false;
        bed_timed_out = false;

        #if HAS_BUZZER
          filament_change_beep(max_beep_count, true);
        #endif
      }

      printer.idle(true);
    }
    printer.keepalive(InHandler);
  }

  /**
   * Resume or Start print procedure
   *
   * - Abort if not paused
   * - Reset heater idle timers
   * - Load filament if specified, but only if:
   *   - a nozzle timed out, or
   *   - the nozzle is already heated.
   * - Display "wait for print to resume"
   * - Re-prime the nozzle...
   *   -  FWRETRACT: Recover/prime from the prior G10.
   *   - !FWRETRACT: Retract by resume_position[E], if negative.
   *                 Not sure how this logic comes into use.
   * - Move the nozzle back to resume_position
   * - Sync the planner E to resume_position[E]
   * - Send host action for resume, if configured
   * - Resume the current SD print job, if any
   */
  void resume_print(const float &slow_load_length/*=0*/, const float &fast_load_length/*=0*/, const float &purge_length/*=PAUSE_PARK_EXTRUDE_LENGTH*/, const int8_t max_beep_count/*=0*/) {
    if (!did_pause_print) return;

    // Re-enable the heaters if they timed out
    bool  nozzle_timed_out  = false,
          bed_timed_out     = false;

    #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
      bed_timed_out |= heaters[BED_INDEX].isIdle();
      heaters[BED_INDEX].reset_idle_timer();
    #endif

    LOOP_HOTEND() {
      nozzle_timed_out |= heaters[h].isIdle();
      heaters[h].reset_idle_timer();
    }

    if (nozzle_timed_out || thermalManager.hotEnoughToExtrude(TARGET_EXTRUDER)) {
      // Load the new filament
      load_filament(slow_load_length, fast_load_length, purge_length, max_beep_count, true, nozzle_timed_out);
    }

    #if HAS_LCD
      // "Wait for print to resume"
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_RESUME);
    #endif

    // Intelligent resuming
    #if ENABLED(FWRETRACT)
      // If retracted before goto pause
      if (fwretract.retracted[tools.active_extruder])
        do_pause_e_move(-fwretract.retract_length, fwretract.retract_feedrate_mm_s);
    #endif
    // If resume_position is negative
    if (resume_position[E_AXIS] < 0) do_pause_e_move(resume_position[E_AXIS], PAUSE_PARK_RETRACT_FEEDRATE);

    // Move XY to starting position, then Z
    mechanics.do_blocking_move_to_xy(resume_position[X_AXIS], resume_position[Y_AXIS], NOZZLE_PARK_XY_FEEDRATE);

    // Set Z_AXIS to saved position
    mechanics.do_blocking_move_to_z(resume_position[Z_AXIS], NOZZLE_PARK_Z_FEEDRATE);

    // Now all extrusion positions are resumed and ready to be confirmed
    // Set extruder to saved position
    planner.set_e_position_mm(mechanics.destination[E_AXIS] = mechanics.current_position[E_AXIS] = resume_position[E_AXIS]);

    printer.setFilamentOut(false);

    #if HAS_LCD
      // Show status screen
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
    #endif

    #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
      mechanics.Nextion_gfx_clear();
    #endif

    SERIAL_STR(RESUME);
    SERIAL_EOL();

    --did_pause_print;

    #if HAS_SDSUPPORT
      if (did_pause_print) {
        card.startFileprint();
        --did_pause_print;
      }
    #endif

  }

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)
