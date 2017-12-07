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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)

  #define CODE_M155

  /**
   * M155: Set temperature auto-report interval. M155 S<seconds>
   */
  inline void gcode_M155(void) {
    if (parser.seenval('S')) {
      thermalManager.auto_report_temp_interval = parser.value_byte();
      NOMORE(thermalManager.auto_report_temp_interval, 60);
      thermalManager.next_temp_report_ms = millis() + 1000UL * thermalManager.auto_report_temp_interval;
    }
  }

#endif // AUTO_REPORT_TEMPERATURES
