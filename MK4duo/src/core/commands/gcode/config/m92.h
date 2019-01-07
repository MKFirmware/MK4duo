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
 *
 *      If no argument is given print the current values.
 *
 *    With MAGIC_NUMBERS_GCODE:
 *      Use 'H' and/or 'L' to get ideal layer-height information.
 *      'H' specifies micro-steps to use. We guess if it's not supplied.
 *      'L' specifies a desired layer height. Nearest good heights are shown.
 */
inline void gcode_M92(void) {

  if (commands.get_target_tool(92)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M92 report.
    if (!parser.seen("XYZEHL")) {
      mechanics.print_M92();
      return;
    }
  #endif

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      const float value = parser.value_per_axis_unit((AxisEnum)a);
      if (i == E_AXIS) {
        if (value < 20.0) {
          float factor = mechanics.data.axis_steps_per_mm[a] / value; // increase e constants if M92 E14 is given for netfab.
          #if HAS_CLASSIC_JERK && (DISABLED(JUNCTION_DEVIATION) || DISABLED(LIN_ADVANCE))
            mechanics.data.max_jerk[a] *= factor;
          #endif
          mechanics.data.max_feedrate_mm_s[a] *= factor;
          mechanics.max_acceleration_steps_per_s2[a] *= factor;
        }
        mechanics.data.axis_steps_per_mm[a] = value;
      }
      else {
        #if MECH(DELTA)
          LOOP_XYZ(axis)
            mechanics.data.axis_steps_per_mm[axis] = value;
        #else
          mechanics.data.axis_steps_per_mm[a] = value;
        #endif
      }
    }
  }

  planner.refresh_positioning();

  const float layer_wanted = parser.floatval('L');
  if (parser.seen('H') || layer_wanted) {
    const uint16_t argH = parser.ushortval('H'),
                   micro_steps = argH ? argH : 1;
    const float minimum_layer_height = micro_steps * mechanics.steps_to_mm[Z_AXIS];
    SERIAL_SMV(ECHO, "{ micro steps:", micro_steps);
    SERIAL_MV(", minimum layer height:", minimum_layer_height, 3);
    if (layer_wanted) {
      const float layer = uint16_t(layer_wanted / minimum_layer_height) * minimum_layer_height;
      SERIAL_MV(", layer:[", layer);
      if (layer != layer_wanted)
        SERIAL_MV(",", layer + minimum_layer_height, 3);
      SERIAL_CHR(']');
    }
    SERIAL_EM(" }");
  }

}
