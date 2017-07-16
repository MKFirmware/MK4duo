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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define CODE_M530

/**
 * M530: S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
 */
inline void gcode_M530(void) {

  if (parser.seen('L')) printer.maxLayer = parser.value_long();

  if (parser.seen('S') && parser.value_bool()) {
    printer.print_job_counter.start();

    SERIAL_MSG("Start Printing");
    if (printer.maxLayer > 0) SERIAL_EMV(" - MaxLayer:", printer.maxLayer);
    else SERIAL_EOL();

    #if ENABLED(START_GCODE)
      commands.enqueue_and_echo_commands_P(PSTR(START_PRINTING_SCRIPT));
    #endif
    #if HAS_FIL_RUNOUT
      printer.filament_ran_out = false;
      SERIAL_EM("Filament runout activated.");
      SERIAL_STR(RESUME);
      SERIAL_EOL();
    #endif
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }
  else {
    printer.print_job_counter.stop();
    SERIAL_EM("Stop Printing");
    #if ENABLED(STOP_GCODE)
      commands.enqueue_and_echo_commands_P(PSTR(STOP_PRINTING_SCRIPT));
    #endif
    #if HAS_FIL_RUNOUT
      printer.filament_ran_out = false;
      SERIAL_EM("Filament runout deactivated.");
    #endif
  }
}
