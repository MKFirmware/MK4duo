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

#define CODE_M205

/**
 * M205: Set Advanced Settings
 *
 *    B = Min Segment Time (µs)
 *    S = Min Feed Rate (units/s)
 *    V = Min Travel Feed Rate (units/s)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 *    J = Junction Deviation mm
 */
inline void gcode_M205(void) {

  if (commands.get_target_tool(205)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M205 report.
    if (!parser.seen("XYZEBSVJ")) {
      mechanics.print_M205();
      return;
    }
  #endif

  if (parser.seen('B')) mechanics.data.min_segment_time_us = parser.value_ulong();
  if (parser.seen('S')) mechanics.data.min_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('V')) mechanics.data.min_travel_feedrate_mm_s = parser.value_linear_units();

  #if ENABLED(JUNCTION_DEVIATION)
    if (parser.seen('J')) {
      const float junc_dev = parser.value_linear_units();
      if (WITHIN(junc_dev, 0.01f, 0.3f)) {
        mechanics.data.junction_deviation_mm = junc_dev;
        #if ENABLED(LIN_ADVANCE)
          mechanics.recalculate_max_e_jerk();
        #endif
      }
      else
        SERIAL_LM(ER, "?J out of range (0.01 to 0.3)");
    }
  #endif

  #if HAS_CLASSIC_JERK
    #if MECH(DELTA)
      if (parser.seen('X') || parser.seen('Y') || parser.seen('Z')) {
        const float value = parser.value_linear_units();
        LOOP_XYZ(axis) mechanics.data.max_jerk[axis] = value;
      }
    #else
      if (parser.seen('X')) mechanics.data.max_jerk[X_AXIS] = parser.value_linear_units();
      if (parser.seen('Y')) mechanics.data.max_jerk[Y_AXIS] = parser.value_linear_units();
      if (parser.seen('Z')) mechanics.data.max_jerk[Z_AXIS] = parser.value_linear_units();
    #endif
    #if DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE)
      if (parser.seen('E')) mechanics.data.max_jerk[E_AXIS] = parser.value_linear_units();
    #endif
  #endif

}
