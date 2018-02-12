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

#define CODE_M205

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    V = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (µs)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205(void) {

  GET_TARGET_EXTRUDER(205);

  if (parser.seen('S')) mechanics.min_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('V')) mechanics.min_travel_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('B')) mechanics.min_segment_time_us = parser.value_ulong();

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      #if MECH(DELTA)
        const float value = parser.value_per_axis_unit((AxisEnum)a);
        if (i == E_AXIS)
          mechanics.max_jerk[a] = value;
        else
          LOOP_XYZ(axis) mechanics.max_jerk[axis] = value;
      #else
        mechanics.max_jerk[a] = parser.value_axis_units((AxisEnum)a);
      #endif
    }
  }
}
