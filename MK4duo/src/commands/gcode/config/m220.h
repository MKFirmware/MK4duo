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

#define CODE_M220

/**
 * M220 Sxxx: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 * M220 B: backup current speed override
 * M220 R: restore previously saved speed override
 */
inline void gcode_M220() {
  static int16_t backup_feedrate_percentage = 100;

  if (parser.seenval('S'))
    mechanics.feedrate_percentage = parser.value_int();

  if (parser.seen('B'))
    backup_feedrate_percentage = mechanics.feedrate_percentage;

  if (parser.seen('R'))
    mechanics.feedrate_percentage = backup_feedrate_percentage;

}
