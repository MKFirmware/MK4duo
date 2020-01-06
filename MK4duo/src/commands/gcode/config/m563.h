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

#define CODE_M563

/**
 * M563: Set Tools heater assignment
 *
 *  T[tools]  - Set Tool
 *  D[int]    - Set Driver for tool
 *  H[bool]   - Set Hotend for tool
 *
 */
inline void gcode_M563() {

  if (commands.get_target_tool(563)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M563 report.
    if (!parser.seen("DH")) {
      toolManager.print_M563();
      return;
    }
  #endif

  if (parser.seen('D')) {
    const uint8_t d = parser.value_byte();
    if (WITHIN(d, 0, stepper.data.drivers_e - 1))
      extruders[toolManager.extruder.target]->data.driver = d;
    else
      SERIAL_LM(ECHO, "Driver is invalid");
  }

  if (parser.seen('H')) {
    const uint8_t h = parser.value_byte();
    if (WITHIN(h, 0, tempManager.heater.hotends - 1))
      extruders[toolManager.extruder.target]->data.hotend = h;
    else
      SERIAL_LM(ECHO, "Hotend is invalid");
  }

}
