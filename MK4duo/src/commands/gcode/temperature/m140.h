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

#if HAS_BEDS

#define CODE_M140

/**
 * M140: Set Bed temperature
 */
inline void gcode_M140() {

  if (printer.debugDryrun() || printer.debugSimulation()) return;

  const uint8_t b = parser.byteval('T');
  if (WITHIN(b, 0 , tempManager.heater.beds - 1) && beds[b]) {
    if (parser.seenval('S')) beds[b]->set_target_temp(parser.value_celsius());
    if (parser.seenval('R')) beds[b]->set_idle_temp(parser.value_celsius());
  }

}

#endif
