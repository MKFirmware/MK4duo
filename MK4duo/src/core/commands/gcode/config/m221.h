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

#if EXTRUDERS > 0

  #define CODE_M221

  /**
   * M221: Set extrusion percentage (M221 T0 S95)
   */
  inline void gcode_M221(void) {

    if (commands.get_target_tool(221)) return;

    if (parser.seenval('S')) {
      tools.flow_percentage[TARGET_EXTRUDER] = parser.value_int();
      tools.refresh_e_factor(TARGET_EXTRUDER);
    }
    else {
      SERIAL_SMV(ECHO, "E", TARGET_EXTRUDER);
      SERIAL_MV(" Flow: ", tools.flow_percentage[TARGET_EXTRUDER]);
      SERIAL_CHR('%');
      SERIAL_EOL();
    }
  }

#endif // EXTRUDERS > 0
