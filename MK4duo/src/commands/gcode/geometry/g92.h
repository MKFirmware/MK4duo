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
 * gcode.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#define CODE_G92

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {

  xyze_bool_t   codes;
  xyze_float_t  values;

  #if USE_GCODE_SUBCODES
    const uint8_t subcode_G92 = parser.subcode;
  #else
    constexpr uint8_t subcode_G92 = 0;
  #endif

  LOOP_XYZE(i) {
    if ((codes[i]   = parser.seenval(axis_codes[i]))) {
      const float v = parser.value_axis_units((AxisEnum)i);
      values[i]     = (i == E_AXIS) ? v : LOGICAL_TO_NATIVE(v, (AxisEnum)i);
    }
  }

  switch (subcode_G92) {

    case 0: {
      if ((codes[E_AXIS] && values[E_AXIS] == 0) && (!codes[X_AXIS] && !codes[Y_AXIS] && !codes[Z_AXIS])) {
        mechanics.position.e = 0;
        mechanics.sync_plan_position_e();
      }
      else {
        planner.synchronize();
        LOOP_XYZE(i) {
          if (codes[i]) {
            const float d = values[i] - mechanics.position[i];
            if (!NEAR_ZERO(d)) {
              #if IS_SCARA || DISABLED(WORKSPACE_OFFSETS)
                mechanics.position[i] = values[i];
              #elif ENABLED(WORKSPACE_OFFSETS)
                if (i == E_AXIS) {
                  mechanics.position.e = values[i];        // When using coordinate spaces, only E is set directly
                }
                else {
                  mechanics.position_shift[i] += d;       // Other axes simply offset the coordinate space
                  endstops.update_software_endstops((AxisEnum)i);
                }
              #endif
            }
          }
        }
        mechanics.sync_plan_position();
      }
    } break;

    #if USE_GCODE_SUBCODES
      case 9: {
        planner.synchronize();
        LOOP_XYZE(i) {
          if (parser.seenval(axis_codes[i])) {
            mechanics.position[i] = parser.value_axis_units((AxisEnum)i);
            if (i == E_AXIS) mechanics.sync_plan_position_e();
            else mechanics.sync_plan_position();
          }
        }
      } break;
    #endif

    default: break;

  } // switch

  mechanics.report_position();
}
