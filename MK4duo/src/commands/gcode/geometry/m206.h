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

#if ENABLED(WORKSPACE_OFFSETS)

#define CODE_M206

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M206 report.
    if (parser.seen_any()) {
      mechanics.print_M206();
      return;
    }
  #endif

  LOOP_XYZ(i) {
    if (parser.seen(axis_codes[i]))
      mechanics.set_home_offset((AxisEnum)i, parser.value_linear_units());
  }

  #if MECH(MORGAN_SCARA)
    if (parser.seen('T')) mechanics.set_home_offset(A_AXIS, parser.value_float()); // Theta
    if (parser.seen('P')) mechanics.set_home_offset(Y_AXIS, parser.value_float()); // Psi
  #endif

  mechanics.report_position();
}

#endif // ENABLED(WORKSPACE_OFFSETS)
