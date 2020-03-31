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
 * advanced_pause.cpp - Advance Pause feature
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if ENABLED(ADVANCED_PAUSE_FEATURE)

#if HAS_LCD && ENABLED(EMERGENCY_PARSER)
  #define _PMSG(L) L
#elif ENABLED(EMERGENCY_PARSER)
  #define _PMSG(L) L##_M108
#else
  #define _PMSG(L) L##_LCD
#endif

AdvancedPause advancedpause;

/** Public Parameters */
PauseModeEnum AdvancedPause::mode = PAUSE_MODE_PAUSE_PRINT;

PauseMenuResponseEnum AdvancedPause::menu_response;

uint8_t AdvancedPause::did_pause_print = 0;

/** Public Function */

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
 * Return 'true' if pause was completed, 'false' for abort
 */
bool AdvancedPause::pause_print(const float &retract, const xyz_pos_t &park_point, const float &unload_length/*=0*/, const bool show_lcd/*=false*/ DXC_ARGS) {

  #if !HAS_LCD_MENU
    UNUSED(show_lcd);
  #endif

  if (did_pause_print) return false; // already paused

  host_action.paused();
  host_action.prompt_open(PROMPT_INFO, PSTR("Pause"), DISMISS_BTN);

  if (!printer.debugDryrun() && unload_length && tempManager.tooColdToExtrude(toolManager.active_hotend())) {
    SERIAL_LM(ER, STR_HOTEND_TOO_COLD);

    #if HAS_LCD_MENU
      if (show_lcd) { // Show status screen
        lcd_pause_show_message(PAUSE_MESSAGE_STATUS);
        LCD_MESSAGEPGM(MSG_HOTEND_TOO_COLD);
      }
    #endif

    return false; // unable to reach safe temperature
  }

  // Indicate that the printer is paused
  ++did_pause_print;

  // Pause the print job and timer
  #if HAS_SD_SUPPORT
    if (IS_SD_PRINTING()) {
      card.pauseSDPrint();
      ++did_pause_print;
    }
  #endif

  print_job_counter.pause();

  // Save current position
  mechanics.stored_position[0] = mechanics.position;

  // Wait for synchronize steppers
  planner.synchronize();

  // Initial retract before move to filament change position
  if (retract && !tempManager.tooColdToExtrude(toolManager.active_hotend()))
    mechanics.unscaled_e_move(retract, feedrate_t(PAUSE_PARK_RETRACT_FEEDRATE));

  // Park the nozzle by moving up by z_lift and then moving to (x_pos, y_pos)
  nozzle.park(2, park_point);

  #if ENABLED(DUAL_X_CARRIAGE)
    const int8_t saved_ext        = toolManager.extruder.active;
    const bool saved_ext_dup_mode = mechanics.extruder_duplication_enabled;
    toolManager.extruder.active   = DXC_ext;
    mechanics.extruder_duplication_enabled = false;
  #endif

  // Unload the filament
  if (unload_length)
    unload_filament(unload_length, show_lcd);

  #if ENABLED(DUAL_X_CARRIAGE)
    toolManager.extruder.active = saved_ext;
    mechanics.extruder_duplication_enabled = saved_ext_dup_mode;
    stepper.set_directions();
  #endif

  return true;
}

/**
 * - Show "Insert filament and press button to continue"
 * - Wait for a click before returning
 * - Heaters can time out, reheated before accepting a click
 *
 * Used by M125 and M600
 */
void AdvancedPause::wait_for_confirmation(const bool is_reload/*=false*/, const int8_t max_beep_count/*=0*/ DXC_ARGS) {
  bool  nozzle_timed_out = false,
        bed_timed_out = false;

  show_continue_prompt(is_reload);

  #if HAS_BUZZER
    filament_change_beep(max_beep_count, true);
  #else
    UNUSED(max_beep_count);
  #endif

  // Start the heater idle timers
  constexpr millis_l  nozzle_timeout  = (PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL,
                      bed_timeout     = (PAUSE_PARK_PRINTER_OFF)    * 60000UL;

  LOOP_HOTEND() hotends[h]->start_idle_timer(nozzle_timeout);

  #if HAS_BEDS && PAUSE_PARK_PRINTER_OFF > 0
    LOOP_BED() beds[h]->start_idle_timer(bed_timeout);
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    const int8_t saved_ext        = toolManager.extruder.active;
    const bool saved_ext_dup_mode = mechanics.extruder_duplication_enabled;
    toolManager.extruder.active    = DXC_ext;
    mechanics.extruder_duplication_enabled = false;
  #endif

  // Wait for filament insert by user and press button
  PRINTER_KEEPALIVE(PausedforUser);
  host_action.prompt_do(PROMPT_USER_CONTINUE, PSTR("Nozzle Parked"), CONTINUE_BTN);
  printer.setWaitForUser(true); // LCD click or M108 will clear this

  while (printer.isWaitForUser()) {
    #if HAS_BUZZER
      filament_change_beep(max_beep_count);
    #endif

    // If the nozzle has timed out, wait for the user to press the button to re-heat the nozzle, then
    // re-heat the nozzle, re-show the insert screen, restart the idle timers, and start over
    if (!nozzle_timed_out)
      LOOP_HOTEND() nozzle_timed_out |= hotends[h]->isIdle();

    if (nozzle_timed_out) {
      #if HAS_LCD_MENU
        lcd_pause_show_message(PAUSE_MESSAGE_HEAT);
      #endif
      SERIAL_STR(ECHO);
      SERIAL_EM(_PMSG(STR_FILAMENT_CHANGE_HEAT));

      host_action.prompt_do(PROMPT_USER_CONTINUE, PSTR("HeaterTimeout"), PSTR("Reheat"));

      // Wait for LCD click or M108
      while (printer.isWaitForUser()) {

        if (!bed_timed_out) {
          #if HAS_BEDS && PAUSE_PARK_PRINTER_OFF > 0
            LOOP_BED() bed_timed_out |= beds[h]->isIdle();
          #endif
        }
        else {
          #if HAS_LCD_MENU
            lcd_pause_show_message(PAUSE_MESSAGE_PRINTER_OFF);
          #endif
        }

        printer.idle_no_sleep();
      }

      host_action.prompt_do(PROMPT_INFO, PSTR("Reheating"));

      // Re-enable the bed if they timed out
      #if HAS_BEDS && PAUSE_PARK_PRINTER_OFF > 0
        if (bed_timed_out) LOOP_BED() beds[h]->reset_idle_timer();
      #endif

      // Re-enable the hotends if they timed out
      LOOP_HOTEND() hotends[h]->reset_idle_timer();

      // Wait for the heaters to reach the target temperatures
      ensure_safe_temperature();

      // Show the prompt to continue
      show_continue_prompt(is_reload);

      LOOP_HOTEND() hotends[h]->start_idle_timer(nozzle_timeout);

      #if HAS_BEDS && PAUSE_PARK_PRINTER_OFF > 0
        LOOP_BED() beds[h]->start_idle_timer(bed_timeout);
      #endif

      host_action.prompt_do(PROMPT_USER_CONTINUE, PSTR("Reheat Done"), CONTINUE_BTN);

      printer.setWaitForUser(true);
      nozzle_timed_out = false;
      bed_timed_out = false;

      #if HAS_BUZZER
        filament_change_beep(max_beep_count, true);
      #endif
    }

    printer.idle_no_sleep();
  }

  #if ENABLED(DUAL_X_CARRIAGE)
    toolManager.extruder.active = saved_ext;
    mechanics.extruder_duplication_enabled = saved_ext_dup_mode;
    stepper.set_directions();
  #endif

}

/**
 * Resume or Start print procedure
 *
 * - If not paused, do nothing and return
 * - Reset heater idle timers
 * - Load filament if specified, but only if:
 *   - a nozzle timed out, or
 *   - the nozzle is already heated.
 * - Display "wait for print to resume"
 * - Re-prime the nozzle...
 *   -  FWRETRACT: Recover/prime from the prior G10.
 *   - !FWRETRACT: Retract by stored_position[0][E], if negative.
 *                 Not sure how this logic comes into use.
 * - Move the nozzle back to stored_position[0]
 * - Sync the planner E to stored_position[0][E]
 * - Send host action for resume, if configured
 * - Resume the current SD print job, if any
 */
void AdvancedPause::resume_print(const float &slow_load_length/*=0*/, const float &fast_load_length/*=0*/, const float &purge_length/*=PAUSE_PARK_PURGE_LENGTH*/, const int8_t max_beep_count/*=0*/ DXC_ARGS) {

  if (!did_pause_print) return;

  // Re-enable the heaters if they timed out
  bool  nozzle_timed_out  = false,
        bed_timed_out     = false;

  #if HAS_BEDS && PAUSE_PARK_PRINTER_OFF > 0
    LOOP_BED() {
      bed_timed_out |= beds[h]->isIdle();
      beds[h]->reset_idle_timer();
    }
  #endif

  LOOP_HOTEND() {
    nozzle_timed_out |= hotends[h]->isIdle();
    hotends[h]->reset_idle_timer();
  }

  if (nozzle_timed_out || tempManager.hotEnoughToExtrude(toolManager.extruder.target)) {
    // Load the new filament
    load_filament(slow_load_length, fast_load_length, purge_length, max_beep_count, true, nozzle_timed_out, PAUSE_MODE_PAUSE_PRINT DXC_PASS);
  }

  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_RESUME); // "Wait for print to resume"
  #endif

  // Intelligent resuming
  #if ENABLED(FWRETRACT)
    // If retracted before goto pause
    if (fwretract.retracted[toolManager.extruder.active])
      mechanics.unscaled_e_move(-fwretract.data.retract_length, fwretract.data.retract_feedrate_mm_s);
  #endif

  // If stored_position[0] is negative
  if (mechanics.stored_position[0][E_AXIS] < 0) mechanics.unscaled_e_move(mechanics.stored_position[0][E_AXIS], feedrate_t(PAUSE_PARK_RETRACT_FEEDRATE));

  // Move XY to starting position, then Z
  mechanics.do_blocking_move_to_xy(mechanics.stored_position[0][X_AXIS], mechanics.stored_position[0][Y_AXIS], feedrate_t(NOZZLE_PARK_XY_FEEDRATE));

  // Set Z_AXIS to saved position
  mechanics.do_blocking_move_to_z(mechanics.stored_position[0][Z_AXIS], feedrate_t(NOZZLE_PARK_Z_FEEDRATE));

  // Now all extrusion positions are resumed and ready to be confirmed
  // Set extruder to saved position
  planner.set_e_position_mm(mechanics.destination.e = mechanics.position.e = mechanics.stored_position[0].e);

  #if HAS_FILAMENT_SENSOR
    filamentrunout.reset();
  #endif

  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_STATUS);
  #endif

  #if HAS_NEXTION_LCD && ENABLED(NEXTION_GFX)
    mechanics.nextion_gfx_clear();
  #endif

  host_action.resumed();

  --did_pause_print;

  host_action.prompt_open(PROMPT_INFO, PSTR("Resuming"), DISMISS_BTN);

  #if HAS_SD_SUPPORT
    if (did_pause_print) {
      card.startFilePrint();
      --did_pause_print;
    }
  #endif

  // Resume the print job timer if it was running
  if (print_job_counter.isPaused()) print_job_counter.start();

  #if HAS_LCD
    lcdui.reset_status();
    #if HAS_LCD_MENU
      lcdui.return_to_status();
    #endif
  #endif

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
bool AdvancedPause::load_filament(const float &slow_load_length/*=0*/, const float &fast_load_length/*=0*/, const float &purge_length/*=0*/, const int8_t max_beep_count/*=0*/,
                   const bool show_lcd/*=false*/, const bool pause_for_user/*=false*/,
                   const PauseModeEnum tmode/*=PAUSE_MODE_PAUSE_PRINT*/
                   DXC_ARGS
) {

  #if !HAS_LCD_MENU
    UNUSED(show_lcd);
  #endif

  if (!ensure_safe_temperature(tmode)) {
    #if HAS_LCD_MENU
      if (show_lcd) lcd_pause_show_message(PAUSE_MESSAGE_STATUS, tmode);
    #endif
    return false;
  }

  if (pause_for_user) {
    #if HAS_LCD_MENU
      if (show_lcd) lcd_pause_show_message(PAUSE_MESSAGE_INSERT, tmode);
    #endif
    SERIAL_LM(ECHO, STR_FILAMENT_CHANGE_INSERT);

    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #else
      UNUSED(max_beep_count);
    #endif

    PRINTER_KEEPALIVE(PausedforUser);
    printer.setWaitForUser(true);

    const char tool = DIGIT(toolManager.extruder.active);

    host_action.prompt_begin(PROMPT_USER_CONTINUE, PSTR("Load Filament T"), tool);
    host_action.prompt_button(CONTINUE_BTN);
    host_action.prompt_show();

    while (printer.isWaitForUser()) {
      #if HAS_BUZZER
        filament_change_beep(max_beep_count);
      #endif
      printer.idle_no_sleep();
    }
  }

  #if HAS_LCD_MENU
    if (show_lcd) lcd_pause_show_message(PAUSE_MESSAGE_LOAD, tmode);
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    const int8_t saved_ext        = toolManager.extruder.active;
    const bool saved_ext_dup_mode = mechanics.extruder_duplication_enabled;
    toolManager.extruder.active = DXC_ext;
    mechanics.extruder_duplication_enabled = false;
  #endif

  // Slow Load filament
  if (slow_load_length) mechanics.unscaled_e_move(slow_load_length, feedrate_t(PAUSE_PARK_SLOW_LOAD_FEEDRATE));

  // Fast Load Filament
  if (fast_load_length) mechanics.unscaled_e_move(fast_load_length, feedrate_t(PAUSE_PARK_FAST_LOAD_FEEDRATE));

  #if ENABLED(DUAL_X_CARRIAGE)
    toolManager.extruder.active = saved_ext;
    mechanics.extruder_duplication_enabled = saved_ext_dup_mode;
    stepper.set_directions();
  #endif

  do {
    if (purge_length > 0) {
      // "Wait for filament purge"
      #if HAS_LCD_MENU
        if (show_lcd) lcd_pause_show_message(PAUSE_MESSAGE_PURGE);
      #endif

      // Extrude filament to get into hotend
      mechanics.unscaled_e_move(purge_length, feedrate_t(PAUSE_PARK_PURGE_FEEDRATE));
    }

    // Show "Purge More" / "Resume" menu and wait for reply
    host_action.prompt_begin(PROMPT_FILAMENT_RUNOUT, PSTR("Paused"));
    host_action.prompt_button(PSTR("PurgeMore"));
    if (false
      #if HAS_FILAMENT_SENSOR
        || filamentrunout.sensor.isFilamentOut()
      #endif
    )
      host_action.prompt_button(PSTR("DisableRunout"));
    else {
      host_action.prompt_button(CONTINUE_BTN);
    }
    host_action.prompt_show();

    #if HAS_LCD_MENU
      if (show_lcd) {
        PRINTER_KEEPALIVE(PausedforUser);
        printer.setWaitForUser(false);
        lcd_pause_show_message(PAUSE_MESSAGE_OPTION);
        while (menu_response == PAUSE_RESPONSE_WAIT_FOR) printer.idle_no_sleep();
      }
    #endif

    // Keep looping if "Purge More" was selected
  } while (false
    #if HAS_LCD_MENU
      || (show_lcd && menu_response == PAUSE_RESPONSE_EXTRUDE_MORE)
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
bool AdvancedPause::unload_filament(const float &unload_length, const bool show_lcd/*=false*/,
                                    const PauseModeEnum tmode/*=PAUSE_MODE_PAUSE_PRINT*/
) {

  #if !HAS_LCD_MENU
    UNUSED(show_lcd);
  #endif

  if (!ensure_safe_temperature(tmode)) {
    #if HAS_LCD_MENU
      if (show_lcd) lcd_pause_show_message(PAUSE_MESSAGE_STATUS);
    #endif

    return false;
  }

  #if HAS_LCD_MENU
    if (show_lcd) lcd_pause_show_message(PAUSE_MESSAGE_UNLOAD, tmode);
  #endif

  // Retract filament
  mechanics.unscaled_e_move(-FILAMENT_UNLOAD_RETRACT_LENGTH, feedrate_t(PAUSE_PARK_RETRACT_FEEDRATE));

  // Wait for filament to cool
  HAL::delayMilliseconds(FILAMENT_UNLOAD_DELAY);

  // Quickly purge
  mechanics.unscaled_e_move(FILAMENT_UNLOAD_RETRACT_LENGTH + FILAMENT_UNLOAD_PURGE_LENGTH, extruders[toolManager.extruder.active]->data.max_feedrate_mm_s);

  // Unload filament
  mechanics.unscaled_e_move(unload_length, feedrate_t(PAUSE_PARK_UNLOAD_FEEDRATE));

  // Disable extruders steppers for manual filament changing
  #if HAS_E_STEPPER_ENABLE
    stepper.disable_E(toolManager.extruder.active);
    HAL::delayMilliseconds(100);
  #endif

  return true;
}

/** Private Function */
void AdvancedPause::show_continue_prompt(const bool is_reload) {
  #if HAS_LCD_MENU
    lcd_pause_show_message(is_reload ? PAUSE_MESSAGE_INSERT : PAUSE_MESSAGE_WAITING);
  #endif
  SERIAL_STR(ECHO);
  SERIAL_STR(is_reload ? PSTR(_PMSG(STR_FILAMENT_CHANGE_INSERT) "\n") : PSTR(_PMSG(STR_FILAMENT_CHANGE_WAIT) "\n"));
}

/**
 * Ensure a safe temperature for extrusion
 *
 * - Fail if the TARGET temperature is too low
 * - Display LCD placard with temperature status
 * - Return when heating is done or aborted
 *
 * Returns 'true' if heating was completed, 'false' for abort
 */
bool AdvancedPause::ensure_safe_temperature(const PauseModeEnum tmode/*=PAUSE_MODE_SAME*/) {

  if (printer.debugDryrun()) return true;

  #if HAS_LCD_MENU
    lcd_pause_show_message(PAUSE_MESSAGE_HEATING, tmode);
  #else
    UNUSED(tmode);
  #endif

  #if PAUSE_PARK_PRINTER_OFF > 0
    LOOP_BED() beds[h]->wait_for_target();
  #endif

  hotends[toolManager.active_hotend()]->wait_for_target();

  return !printer.isWaitForHeatUp();
}

#if HAS_BUZZER

  void AdvancedPause::filament_change_beep(const int8_t max_beep_count, const bool init/*=false*/) {
    static millis_l next_buzz_ms  = 0;
    static int8_t   runout_beep   = 0;

    if (init) next_buzz_ms = runout_beep = 0;

    const millis_l now = millis();
    if (ELAPSED(now, next_buzz_ms)) {
      if (max_beep_count < 0 || runout_beep < max_beep_count + 5) { // Only beep as long as we're supposed to
        next_buzz_ms = now + ((max_beep_count < 0 || runout_beep < max_beep_count) ? 1000UL : 500UL);
        sound.playtone(50, NOTE_A5 - (runout_beep & 1) * NOTE_A3);
        runout_beep++;
      }
    }
  }

#endif

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)
