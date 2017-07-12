/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define G92

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92(void) {
  bool didXYZ = false,
       didE = parser.seenval('E');

  if (!didE) stepper.synchronize();

  LOOP_XYZE(i) {
    if (parser.seenval(axis_codes[i])) {
      #if IS_SCARA
        mechanics.current_position[i] = parser.value_axis_units((AxisEnum)i);
        if (i != E_AXIS) didXYZ = true;
      #else
        #if ENABLED(WORKSPACE_OFFSETS)
          const float p = mechanics.current_position[i];
        #endif
        float v = parser.value_axis_units((AxisEnum)i);

        mechanics.current_position[i] = v;

        if (i != E_AXIS) {
          didXYZ = true;
          #if ENABLED(WORKSPACE_OFFSETS)
            mechanics.position_shift[i] += v - p; // Offset the coordinate space
            endstops.update_software_endstops((AxisEnum)i);
          #endif
        }
      #endif
    }
  }
  if (didXYZ)
    mechanics.sync_plan_position();
  else if (didE)
    mechanics.sync_plan_position_e();

  mechanics.report_current_position();
}
