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

#define CODE_M24
#define CODE_M25

/**
 * M24: Start or Resume SD Print
 */
inline void gcode_M24() {

  if (parser.seenval('S')) card.setIndex(parser.value_long());
  if (parser.seenval('T')) print_job_counter.resume(parser.value_long());

  #if ENABLED(PARK_HEAD_ON_PAUSE)
    if (advancedpause.did_pause_print) {
      advancedpause.resume_print();
      return;
    }
  #endif

  if (card.isFileOpen()) {
    card.startFilePrint();
    print_job_counter.start();
    #if HAS_SD_RESTART
      restart.start_job();
    #endif
  }

  host_action.resume();
  host_action.prompt_open(PROMPT_INFO, PSTR("Resuming SD"), DISMISS_BTN);

  lcdui.reset_status();

}

/**
 * M25: Pause SD Print
 */
void gcode_M25() {

  // Set initial pause flag to prevent more commands from landing in the queue while we try to pause
  #if HAS_SD_SUPPORT
    if (IS_SD_PRINTING()) card.pauseSDPrint();
  #endif

  #if ENABLED(PARK_HEAD_ON_PAUSE)
    gcode_M125();
  #else
    #if HAS_SD_RESTART
      if (restart.enabled) restart.save_job();
    #endif
    print_job_counter.pause();
    lcdui.reset_status();
    host_action.prompt_open(PROMPT_PAUSE_RESUME, PSTR("Pause SD"), PSTR("Resume"));
    host_action.pause();
  #endif
}

#endif // HAS_SD_SUPPORT
