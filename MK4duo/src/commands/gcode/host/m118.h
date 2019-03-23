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

#define CODE_M118

/**
 * M118: Display a message in the host console.
 *
 *  A1  Append '// ' for an action command, as in OctoPrint
 *  E1  Have the host 'echo:' the text
 */
inline void gcode_M118(void) {
  if (parser.seenval('E') && parser.value_bool()) SERIAL_STR(ECHO);
  if (parser.seenval('A') && parser.value_bool()) SERIAL_MSG("// ");
  SERIAL_ET(parser.string_arg);
}
