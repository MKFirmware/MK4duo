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

#if HAS_AD8495 || HAS_AD595

#define CODE_M595

/**
 * M595 - set AD595 or AD8495 offset & Gain
 *
 *   H[heaters] H = 0-5 Hotend, H = -1 BED, H = -2 CHAMBER
 *
 *    T[int]      0-3 For Select Beds or Chambers
 *
 *    O<offset> P<gain>
 */
inline void gcode_M595() {

  Heater * const act = commands.get_target_heater();

  if (!act) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M595 report.
    if (!parser.seen("OP")) {
      act->print_M595();
      return;
    }
  #endif

  act->data.sensor.ad595_offset = parser.floatval('O');
  act->data.sensor.ad595_gain   = parser.floatval('P', 1);
  if (act->data.sensor.ad595_gain == 0) act->data.sensor.ad595_gain = 1.0;

}

#endif // HAS_AD8495 || HAS_AD595
