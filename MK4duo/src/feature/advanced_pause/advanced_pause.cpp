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

#include "../../../base.h"

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  static float  resume_position[XYZE];

  #if HAS_SDSUPPORT
    static bool sd_print_paused = false;
  #endif

  #if HAS_BUZZER

    static void filament_change_beep(const int8_t max_beep_count, const bool init=false) {
      static millis_t next_buzz = 0;
      static int8_t runout_beep = 0;

      if (init) next_buzz = runout_beep = 0;

      const millis_t ms = millis();
      if (ELAPSED(ms, next_buzz)) {
        if (max_beep_count < 0 || runout_beep < max_beep_count + 5) { // Only beep as long as we're supposed to
          next_buzz = ms + ((max_beep_count < 0 || runout_beep < max_beep_count) ? 2500 : 400);
          BUZZ(300, 2000);
          runout_beep++;
        }
      }
    }

  #endif

  static void ensure_safe_temperature() {
    bool heaters_heating = true;

    thermalManager.wait_for_heatup = true;
    while (thermalManager.wait_for_heatup && heaters_heating) {
      printer.idle();
      heaters_heating = false;
      LOOP_HOTEND() {
        if (heaters[h].target_temperature && abs(heaters[h].current_temperature - heaters[h].target_temperature) > TEMP_HYSTERESIS) {
          heaters_heating = true;
          #if HAS_LCD
            lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT);
          #endif
          break;
        }
      }
    }
  }

  // public function

  bool move_away_flag = false;

  AdvancedPauseMenuResponse advanced_pause_menu_response;

  // Define runplan for move axes
  #if IS_KINEMATIC
    #define RUNPLAN(RATE_MM_S) planner.buffer_line_kinematic(mechanics.destination, RATE_MM_S, tools.active_extruder)
  #else
    #define RUNPLAN(RATE_MM_S) mechanics.line_to_destination(RATE_MM_S)
  #endif

  bool pause_print(const float &retract, const float &retract2, const float &z_lift, const float &x_pos, const float &y_pos,
                   const float &unload_length/*=0*/, const int16_t new_temp/*=0*/, const int8_t max_beep_count/*=0*/, const bool show_lcd/*=false*/
  ) {

    if (move_away_flag) return false; // already paused

    if (!DEBUGGING(DRYRUN) && (unload_length != 0 || retract != 0 || retract2 != 0)) {
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (thermalManager.tooColdToExtrude(tools.active_extruder)) {
          SERIAL_LM(ER, MSG_TOO_COLD_FOR_M600);
          return false;
        }
      #endif
      ensure_safe_temperature(); // wait for hotend to heat up before unloading
    }

    // Indicate that the printer is paused
    move_away_flag = true;

    // Pause the print job and timer
    #if HAS_SDSUPPORT
      if (card.sdprinting) {
        card.pauseSDPrint();
        sd_print_paused = true;
      }
    #endif
    print_job_counter.pause();

    // Show initial message and wait for synchronize steppers
    if (show_lcd) {
      #if HAS_LCD
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT);
      #endif
    }

    // Save current position
    stepper.synchronize();
    COPY_ARRAY(resume_position, mechanics.current_position);

    if (retract) {
      // Initial retract before move to filament change position
      mechanics.set_destination_to_current();
      mechanics.destination[E_AXIS] += retract;
      RUNPLAN(PAUSE_PARK_RETRACT_FEEDRATE);
      stepper.synchronize();
    }

    // Lift Z axis
    if (z_lift > 0)
      mechanics.do_blocking_move_to_z(mechanics.current_position[Z_AXIS] + z_lift, PAUSE_PARK_Z_FEEDRATE);

    // Move XY axes to filament exchange position
    mechanics.do_blocking_move_to_xy(x_pos, y_pos, PAUSE_PARK_XY_FEEDRATE);

    // Store in old temperature the target temperature for hotend and bed
    int16_t old_target_temperature[HOTENDS];
    LOOP_HOTEND()
      old_target_temperature[h] = (new_temp != 0) ? new_temp : heaters[h].target_temperature; // Save nozzle temps

    // Second retract filament with Cool Down
    #if ENABLED(PAUSE_PARK_COOLDOWN_TEMP) && PAUSE_PARK_COOLDOWN_TEMP > 0
      if (retract2) {
        // Cool Down hotend
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_COOLDOWN);
        heaters[tools.active_extruder].setTarget(PAUSE_PARK_COOLDOWN_TEMP);
        thermalManager.wait_heater(false);

        // Second retract filament
        mechanics.set_destination_to_current();
        mechanics.destination[E_AXIS] += retract2;
        RUNPLAN(PAUSE_PARK_RETRACT_2_FEEDRATE);
        stepper.synchronize();
      }
    #endif

    if (unload_length != 0) {
      if (show_lcd) {
        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_UNLOAD);
          printer.idle();
        #endif
      }

      // Unload filament
      mechanics.set_destination_to_current();
      mechanics.destination[E_AXIS] += unload_length;
      RUNPLAN(PAUSE_PARK_UNLOAD_FEEDRATE);
      stepper.synchronize();
    }

    if (show_lcd) {
      #if HAS_LCD
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
      #endif
    }

    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #else
      UNUSED(max_beep_count);
    #endif

    printer.idle();

    stepper.disable_e_steppers();
    printer.safe_delay(100);

    // Start the heater idle timers
    const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
    const millis_t bed_timeout    = (millis_t)(PAUSE_PARK_PRINTER_OFF) * 60000UL;

    LOOP_HOTEND() {
      thermalManager.start_heater_idle_timer(h, nozzle_timeout);
      heaters[h].setTarget(old_target_temperature[h]);
    }

    #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
      thermalManager.start_heater_idle_timer(BED_INDEX, bed_timeout);
    #endif

    return true;
  }

  void wait_for_filament_reload(const int8_t max_beep_count/*=0*/) {
    bool nozzle_timed_out = false,
         bed_timed_out = false;

    // Wait for filament insert by user and press button
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    printer.wait_for_user = true;    // LCD click or M108 will clear this
    while (printer.wait_for_user) {

      #if HAS_BUZZER
        filament_change_beep(max_beep_count);
      #else
        UNUSED(max_beep_count);
      #endif

      if (!nozzle_timed_out)
        LOOP_HOTEND()
          nozzle_timed_out |= thermalManager.is_heater_idle(h);

      if (nozzle_timed_out) {

        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE);
        #endif

        // Wait for LCD click or M108
        while (printer.wait_for_user) {

          if (!bed_timed_out) {
            #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
              bed_timed_out = thermalManager.is_heater_idle(BED_INDEX);
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
            thermalManager.reset_heater_idle_timer(BED_INDEX);
            #if HAS_LCD
              lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT);
            #endif
            thermalManager.wait_heater(BED_INDEX);
          }
        #endif

        // Re-enable the heaters if they timed out
        LOOP_HOTEND() thermalManager.reset_heater_idle_timer(h);

        // Wait for the heaters to reach the target temperatures
        ensure_safe_temperature();

        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
        #endif

        // Start the heater idle timers
        const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
        const millis_t bed_timeout    = (millis_t)(PAUSE_PARK_PRINTER_OFF) * 60000UL;

        LOOP_HOTEND()
          thermalManager.start_heater_idle_timer(h, nozzle_timeout);

        #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
          thermalManager.start_heater_idle_timer(BED_INDEX, bed_timeout);
        #endif

        printer.wait_for_user = true; /* Wait for user to load filament */
        nozzle_timed_out = false;
        bed_timed_out = false;

        #if HAS_BUZZER
          filament_change_beep(max_beep_count, true);
        #endif
      }

      printer.idle(true);
    }

    KEEPALIVE_STATE(IN_HANDLER);
  }

  void resume_print(const float &load_length/*=0*/, const float &initial_extrude_length/*=0*/, const int8_t max_beep_count/*=0*/) {
    bool  nozzle_timed_out  = false,
          bed_timed_out     = false;

    if (!move_away_flag) return;

    // Re-enable the heaters if they timed out
    #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
      bed_timed_out = thermalManager.is_heater_idle(BED_INDEX);
      thermalManager.reset_heater_idle_timer(BED_INDEX);
      if (bed_timed_out) thermalManager.wait_heater(BED_INDEX);
    #endif

    LOOP_HOTEND() {
      nozzle_timed_out |= thermalManager.is_heater_idle(h);
      thermalManager.reset_heater_idle_timer(h);
    }

    if (nozzle_timed_out) ensure_safe_temperature();

    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #else
      UNUSED(max_beep_count);
    #endif

    mechanics.set_destination_to_current();

    if (load_length != 0) {
      #if HAS_LCD
        // Show "insert filament"
        if (nozzle_timed_out)
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
      #endif

      KEEPALIVE_STATE(PAUSED_FOR_USER);
      printer.wait_for_user = true;    // LCD click or M108 will clear this
      while (printer.wait_for_user && nozzle_timed_out) {
        #if HAS_BUZZER
          filament_change_beep(max_beep_count);
        #endif
        printer.idle(true);
      }
      KEEPALIVE_STATE(IN_HANDLER);

      #if HAS_LCD
        // Show "load" message
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_LOAD);
      #endif

      // Load filament
      mechanics.destination[E_AXIS] += load_length;
      RUNPLAN(PAUSE_PARK_LOAD_FEEDRATE);
      stepper.synchronize();
    }

    #if HAS_LCD && ENABLED(PAUSE_PARK_EXTRUDE_LENGTH) && PAUSE_PARK_EXTRUDE_LENGTH > 0

      float extrude_length = initial_extrude_length;

      do {
        if (extrude_length > 0) {
          // "Wait for filament extrude"
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_EXTRUDE);

          // Extrude filament to get into hotend
          mechanics.destination[E_AXIS] += extrude_length;
          RUNPLAN(PAUSE_PARK_EXTRUDE_FEEDRATE);
          stepper.synchronize();
        }

        // Show "Extrude More" / "Resume" menu and wait for reply
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        printer.wait_for_user = false;
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_OPTION);
        while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_WAIT_FOR) printer.idle(true);
        KEEPALIVE_STATE(IN_HANDLER);

        extrude_length = PAUSE_PARK_EXTRUDE_LENGTH;

        // Keep looping if "Extrude More" was selected
      } while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE);

    #endif

    #if HAS_LCD
      // "Wait for print to resume"
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_RESUME);
    #endif

    // Set extruder to saved position
    mechanics.destination[E_AXIS] = mechanics.current_position[E_AXIS] = resume_position[E_AXIS];
    mechanics.set_e_position_mm(mechanics.current_position[E_AXIS]);

    // Move XY to starting position, then Z
    mechanics.do_blocking_move_to_xy(resume_position[X_AXIS], resume_position[Y_AXIS], PAUSE_PARK_XY_FEEDRATE);
    mechanics.do_blocking_move_to_z(resume_position[Z_AXIS], PAUSE_PARK_Z_FEEDRATE);

    #if HAS_FIL_RUNOUT || HAS_EXT_ENCODER
      printer.filament_ran_out = false;
    #endif

    #if HAS_LCD
      // Show status screen
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
    #endif

    #if HAS_SDSUPPORT
      if (sd_print_paused) {
        card.startFileprint();
        sd_print_paused = false;
      }
    #endif

    move_away_flag = false;
  }

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)