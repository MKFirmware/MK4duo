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

#if ENABLED(WORKSPACE_OFFSETS)

  #define CODE_M206

  /**
   * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
   */
  inline void gcode_M206(void) {
    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i]))
        mechanics.home_offset[(AxisEnum)i] = parser.value_linear_units();
    }
    #if MECH(MORGAN_SCARA)
      if (parser.seen('T'))
        mechanics.home_offset[X_AXIS] = parser.value_linear_units(); // Theta
      if (parser.seen('P'))
        mechanics.home_offset[Y_AXIS] = parser.value_linear_units(); // Psi
    #endif

    #if IS_SCARA
      mechanics.sync_plan_position_kinematic();
    #else
      mechanics.sync_plan_position();
    #endif

    mechanics.report_current_position();
  }

#endif // ENABLED(WORKSPACE_OFFSETS)
