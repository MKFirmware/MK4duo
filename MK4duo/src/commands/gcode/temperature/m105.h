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

#define CODE_M105

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105(void) {

  const bool showRaw = parser.boolval('X');

  #if HEATER_COUNT > 0
    SERIAL_STR(OK);
    thermalManager.report_temperatures(showRaw);
    #if ENABLED(FLOWMETER_SENSOR)
      flowmeter.print_flow_rate_state();
    #endif
    #if ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER)
      cnc.print_Speed();
      SERIAL_MV(" fr:", MMS_TO_MMM(mechanics.feedrate_mm_s));
    #endif
  #else
    SERIAL_LM(ER, MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_EOL();
}
