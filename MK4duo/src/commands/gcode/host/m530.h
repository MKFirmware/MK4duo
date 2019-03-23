/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#define CODE_M530

/**
 * M530: S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
 */
inline void gcode_M530(void) {

  if (parser.seen('L')) printer.maxLayer = parser.value_long();

  if (parser.seen('S')) {
    if (parser.value_bool()) {
      print_job_counter.start(); // Start the timer job

      SERIAL_MSG("Start Printing");
      if (printer.maxLayer > 0) SERIAL_MV(" - MaxLayer:", printer.maxLayer);
      SERIAL_EOL();

      #if ENABLED(START_GCODE)
        commands.enqueue_and_echo_P(PSTR(START_PRINTING_SCRIPT));
      #endif

      #if HAS_FILAMENT_SENSOR
        if (filamentrunout.isEnabled())
          SERIAL_EM("Filament runout activated.");
      #endif
    }
    else {
      print_job_counter.stop();   // Stop the timer job
      SERIAL_EM("Stop Printing");

      #if ENABLED(STOP_GCODE)
        commands.enqueue_and_echo_P(PSTR(STOP_PRINTING_SCRIPT));
      #endif
    }

    #if HAS_FILAMENT_SENSOR
      filamentrunout.setFilamentOut(false);
    #endif

  }

}
