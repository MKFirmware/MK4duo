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
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if HAS_SD_SUPPORT

#define CODE_M1001

/**
 * M1001: Execute actions for SD print completion
 */
inline void gcode_M1001() {

  // Report total print time
  const bool long_print = print_job_counter.duration() > 60;
  if (long_print) commands.process_now_P(PSTR("M31"));

  // Stop the print job timer
  commands.process_now_P(PSTR("M77"));

  // Purge the restart file
  #if HAS_SD_RESTART
    restart.purge_job();
  #endif

  // Announce SD file completion
  SERIAL_EM(STR_FILE_PRINTED);

  #if HAS_LEDS_OFF_FLAG
    if (long_print) {
      ledevents.onPrintCompleted();
      host_action.prompt_do(PROMPT_USER_CONTINUE, GET_TEXT(MSG_PRINT_DONE), CONTINUE_BTN);
      printer.wait_for_user_response(
        #if HAS_LCD_MENU
          180000UL        // ...for 30 minutes with LCD
        #else
          60000UL         // ...for 1 minute with no LCD
        #endif
      );
      ledevents.onResumeAfterWait();
    }
  #endif

  #if SD_FINISHED_STEPPERRELEASE && ENABLED(SD_FINISHED_RELEASECOMMAND)
     planner.finish_and_disable();
  #endif

  #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
    lcdui.reselect_last_file();
  #endif

}

#endif // HAS_SD_SUPPORT
