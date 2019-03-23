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

#define CODE_M204

/**
 * M204: Set planner.accelerations in units/sec^2 (M204 P1200 T0 R3000 V3000)
 *
 *    P     = Printing moves
 *    T* R  = Retract only (no X, Y, Z) moves
 *    V     = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum mechanics.feedrate_mm_s
 */
inline void gcode_M204(void) {

  if (commands.get_target_tool(204)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M204 report.
    if (!parser.seen("SPRVE")) {
      mechanics.print_M204();
      return;
    }
  #endif

  if (parser.seen('S')) // Kept for legacy compatibility. Should NOT BE USED for new developments.
    mechanics.data.travel_acceleration = mechanics.data.acceleration = parser.value_linear_units();

  if (parser.seen('P'))
    mechanics.data.acceleration = parser.value_linear_units();

  if (parser.seen('R'))
    mechanics.data.retract_acceleration[TARGET_EXTRUDER] = parser.value_linear_units();

  if (parser.seen('V'))
    mechanics.data.travel_acceleration = parser.value_linear_units();

}
