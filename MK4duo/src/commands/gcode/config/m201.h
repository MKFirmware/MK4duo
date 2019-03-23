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

#define CODE_M201

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M201(void) {

  if (commands.get_target_tool(201)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M201 report.
    if (!parser.seen("XYZE")) {
      mechanics.print_M201();
      return;
    }
  #endif

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = (i == E_AXIS ? E_INDEX : i);
      #if MECH(DELTA)
        const float value = parser.value_per_axis_unit((AxisEnum)a);
        if (i == E_AXIS)
          mechanics.data.max_acceleration_mm_per_s2[a] = value;
        else
          LOOP_XYZ(axis) mechanics.data.max_acceleration_mm_per_s2[axis] = value;
      #else
        mechanics.data.max_acceleration_mm_per_s2[a] = parser.value_axis_units((AxisEnum)a);
      #endif
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  planner.reset_acceleration_rates();
}
