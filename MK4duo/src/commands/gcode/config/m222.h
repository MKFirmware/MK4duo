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

#define CODE_M222

/**
 * M222: Set density extrusion percentage (M222 T0 S95)
 */
inline void gcode_M222() {

  if (commands.get_target_tool(222)) return;

  if (parser.seenval('S')) {
    extruders[toolManager.extruder.target]->density_percentage = parser.value_int();
    #if ENABLED(RFID_MODULE)
      rfid522.data[toolManager.extruder.target].data.density = extruders[toolManager.extruder.target]->density_percentage;
    #endif
  }
}
