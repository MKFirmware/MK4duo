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

#define CODE_M92

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *      (Follows the same syntax as G92)
 *
 *      With multiple extruders use T to specify which one.
 */
inline void gcode_M92(void) {

  GET_TARGET_EXTRUDER(92);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      const float value = parser.value_per_axis_unit((AxisEnum)a);
      if (i == E_AXIS) {
        if (value < 20.0) {
          float factor = mechanics.axis_steps_per_mm[a] / value; // increase e constants if M92 E14 is given for netfab.
          mechanics.max_jerk[a] *= factor;
          mechanics.max_feedrate_mm_s[a] *= factor;
          mechanics.max_acceleration_steps_per_s2[a] *= factor;
        }
        mechanics.axis_steps_per_mm[a] = value;
      }
      else {
        #if MECH(DELTA)
          LOOP_XYZ(axis)
            mechanics.axis_steps_per_mm[axis] = value;
        #else
          mechanics.axis_steps_per_mm[a] = value;
        #endif
      }
    }
  }
  mechanics.refresh_positioning();
}
