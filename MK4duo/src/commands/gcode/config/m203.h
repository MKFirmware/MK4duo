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

#define CODE_M203

/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {

  if (commands.get_target_tool(203)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M203 report.
    if (parser.seen_any()) {
      mechanics.print_M203();
      return;
    }
  #endif

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      #if MECH(DELTA)
        const float value = parser.value_per_axis_unit((AxisEnum)i);
        if (i == E_AXIS)
          extruders[toolManager.extruder.target]->data.max_feedrate_mm_s = value;
        else
          LOOP_XYZ(axis) mechanics.data.max_feedrate_mm_s[axis] = value;
      #else
        const float value = parser.value_per_axis_unit((AxisEnum)i);
        if (i == E_AXIS)
          extruders[toolManager.extruder.target]->data.max_feedrate_mm_s = value;
        else
        mechanics.data.max_feedrate_mm_s[i] = parser.value_axis_units((AxisEnum)i);
      #endif
    }
  }
}
