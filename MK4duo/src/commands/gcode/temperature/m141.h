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

#if HAS_CHAMBERS

#define CODE_M141

/**
 * M141: Set Chamber temperature
 */
inline void gcode_M141() {

  if (printer.debugDryrun() || printer.debugSimulation()) return;

  const uint8_t c = parser.byteval('T');
  if (WITHIN(c, 0 , MAX_CHAMBER - 1) && chambers[c]) {
    if (parser.seenval('S')) chambers[c]->set_target_temp(parser.value_celsius());
    if (parser.seenval('R')) chambers[c]->set_idle_temp(parser.value_celsius());
  }

}

#endif
