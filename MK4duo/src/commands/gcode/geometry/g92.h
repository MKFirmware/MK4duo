/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#define CODE_G92

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {

  bool  sync_E    = false,
        sync_XYZ  = false;

  #if USE_GCODE_SUBCODES
    const uint8_t subcode_G92 = parser.subcode;
  #else
    constexpr uint8_t subcode_G92 = 0;
  #endif

  planner.synchronize();

  switch (subcode_G92) {

    case 0: {
      LOOP_XYZE(i) {
        if (parser.seenval(axis_codes[i])) {
          const float l = parser.value_axis_units((AxisEnum)i),
                      v = (i == E_AXIS) ? l : LOGICAL_TO_NATIVE(l, (AxisEnum)i),
                      d = v - mechanics.current_position[i];

          if (!NEAR_ZERO(d)) {
            #if IS_SCARA || DISABLED(WORKSPACE_OFFSETS)
              if (i == E_AXIS) sync_E = true;
              else sync_XYZ = true;
              mechanics.current_position[i] = v;        // For SCARA just set the position directly
            #elif ENABLED(WORKSPACE_OFFSETS)
              if (i == E_AXIS) {
                sync_E = true;
                mechanics.current_position.e = v; // When using coordinate spaces, only E is set directly
              }
              else {
                mechanics.position_shift[i] += d;       // Other axes simply offset the coordinate space
                endstops.update_software_endstops((AxisEnum)i);
              }
            #endif
          }
        }
      }
    } break;

    case 9: {
      LOOP_XYZE(i) {
        if (parser.seenval(axis_codes[i])) {
          mechanics.current_position[i] = parser.value_axis_units((AxisEnum)i);
          if (i == E_AXIS) sync_E = true;
          else sync_XYZ = true;
        }
      }
    } break;

    default: break;

  } // switch

  if    (sync_XYZ)  mechanics.sync_plan_position();
  else if (sync_E)  mechanics.sync_plan_position_e();

  mechanics.report_current_position();
}
