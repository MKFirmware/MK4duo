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

#if ENABLED(LIN_ADVANCE)

#define CODE_M900

/**
 * M900: Set Linear Advance K-factor
 *
 *  T<tools>    Set extruder
 *  K<factor>   Set advance K factor
 *  S<bool>     Set Test Linear Advance
 */
inline void gcode_M900() {

  if (commands.get_target_tool(900)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M218 report.
    if (parser.seen_any()) {
      toolManager.print_M900();
      return;
    }
  #endif

  if (parser.seenval('K')) {
    const float newK = parser.value_float();
    if (WITHIN(newK, 0, 10)) {
      planner.synchronize();
      extruders[toolManager.extruder.target]->data.advance_K = newK;
    }
    else
      SERIAL_EM("?K value out of range (0-10).");
  }

  if (parser.seenval('S')) {
    toolManager.setTestLinAdvance(parser.value_bool());
    toolManager.setup_test_linadvance();
  }

}

#endif // ENABLED(LIN_ADVANCE)
