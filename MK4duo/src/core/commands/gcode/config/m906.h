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

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

  #define CODE_M906

  /**
   * M906: Set motor currents
   */
  inline void gcode_M906(void) {

    GET_TARGET_EXTRUDER(906);

    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i])) {
        const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
        externaldac.motor_current[a] = parser.value_float();
      }
    }
    externaldac.set_driver_current();
  }

#elif HAS_TRINAMIC

  #define CODE_M906
  
  /**
   * M906: Set motor current in milliamps using axis codes X, Y, Z, E
   * Report driver currents when no axis specified
   */
  inline void gcode_M906(void) {
    uint16_t values[XYZE];
    LOOP_XYZE(i) values[i] = parser.intval(axis_codes[i]);

    #define TMC_SET_GET_CURRENT(P,Q) do { \
      if (values[P##_AXIS]) tmc_set_current(stepper##Q, TMC_##Q, values[P##_AXIS]); \
      else tmc_get_current(stepper##Q, TMC_##Q); } while(0)

    #if X_IS_TRINAMIC
      TMC_SET_GET_CURRENT(X,X);
    #endif
    #if X2_IS_TRINAMIC
      TMC_SET_GET_CURRENT(X,X2);
    #endif
    #if Y_IS_TRINAMIC
      TMC_SET_GET_CURRENT(Y,Y);
    #endif
    #if Y2_IS_TRINAMIC
      TMC_SET_GET_CURRENT(Y,Y2);
    #endif
    #if Z_IS_TRINAMIC
      TMC_SET_GET_CURRENT(Z,Z);
    #endif
    #if Z2_IS_TRINAMIC
      TMC_SET_GET_CURRENT(Z,Z2);
    #endif
    #if E0_IS_TRINAMIC
      TMC_SET_GET_CURRENT(E,E0);
    #endif
    #if E1_IS_TRINAMIC
      TMC_SET_GET_CURRENT(E,E1);
    #endif
    #if E2_IS_TRINAMIC
      TMC_SET_GET_CURRENT(E,E2);
    #endif
    #if E3_IS_TRINAMIC
      TMC_SET_GET_CURRENT(E,E3);
    #endif
    #if E4_IS_TRINAMIC
      TMC_SET_GET_CURRENT(E,E4);
    #endif
    #if E5_IS_TRINAMIC
      TMC_SET_GET_CURRENT(E,E5);
    #endif
  }

#endif // HAS_TRINAMIC
