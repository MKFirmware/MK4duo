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

    if (commands.get_target_tool(906)) return;

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

    if (commands.get_target_tool(906)) return;

    #define TMC_SAY_CURRENT(Q) tmc_get_current(stepper##Q, TMC_##Q)
    #define TMC_SET_CURRENT(Q) tmc_set_current(stepper##Q, value)

    const uint8_t index = parser.byteval('I');
    LOOP_XYZE(i) if (uint16_t value = parser.intval(axis_codes[i])) {
      switch (i) {
        case X_AXIS:
          #if X_IS_TRINAMIC
            if (index == 0) TMC_SET_CURRENT(X);
          #endif
          #if X2_IS_TRINAMIC
            if (index == 1) TMC_SET_CURRENT(X2);
          #endif
          break;
        case Y_AXIS:
          #if Y_IS_TRINAMIC
            if (index == 0) TMC_SET_CURRENT(Y);
          #endif
          #if Y2_IS_TRINAMIC
            if (index == 1) TMC_SET_CURRENT(Y2);
          #endif
          break;
        case Z_AXIS:
          #if Z_IS_TRINAMIC
            if (index == 0) TMC_SET_CURRENT(Z);
          #endif
          #if Z2_IS_TRINAMIC
            if (index == 1) TMC_SET_CURRENT(Z2);
          #endif
          break;
        case E_AXIS: {
          switch (TARGET_EXTRUDER) {
            #if E0_IS_TRINAMIC
              case 0: TMC_SET_CURRENT(E0); break;
            #endif
            #if E1_IS_TRINAMIC
              case 1: TMC_SET_CURRENT(E1); break;
            #endif
            #if E2_IS_TRINAMIC
              case 2: TMC_SET_CURRENT(E2); break;
            #endif
            #if E3_IS_TRINAMIC
              case 3: TMC_SET_CURRENT(E3); break;
            #endif
            #if E4_IS_TRINAMIC
              case 4: TMC_SET_CURRENT(E4); break;
            #endif
            #if E5_IS_TRINAMIC
              case 5: TMC_SET_CURRENT(E5); break;
            #endif
          }
        } break;
      }
    }

    LOOP_XYZE(i) {
      switch (i) {
        case X_AXIS:
          #if X_IS_TRINAMIC
            TMC_SAY_CURRENT(X);
          #endif
          #if X2_IS_TRINAMIC
            TMC_SAY_CURRENT(X2);
          #endif
          break;
        case Y_AXIS:
          #if Y_IS_TRINAMIC
            TMC_SAY_CURRENT(Y);
          #endif
          #if Y2_IS_TRINAMIC
            TMC_SAY_CURRENT(Y2);
          #endif
          break;
        case Z_AXIS:
          #if Z_IS_TRINAMIC
            TMC_SAY_CURRENT(Z);
          #endif
          #if Z2_IS_TRINAMIC
            TMC_SAY_CURRENT(Z2);
          #endif
          break;
        case E_AXIS:
          #if E0_IS_TRINAMIC
            TMC_SAY_CURRENT(E0);
          #endif
          #if E1_IS_TRINAMIC
            TMC_SAY_CURRENT(E1);
          #endif
          #if E2_IS_TRINAMIC
            TMC_SAY_CURRENT(E2);
          #endif
          #if E3_IS_TRINAMIC
            TMC_SAY_CURRENT(E3);
          #endif
          #if E4_IS_TRINAMIC
            TMC_SAY_CURRENT(E4);
          #endif
          #if E5_IS_TRINAMIC
            TMC_SAY_CURRENT(E5);
          #endif
          break;
      }
    }
  }

#endif // HAS_TRINAMIC
