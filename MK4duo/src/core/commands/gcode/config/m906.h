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

#if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)

  #define CODE_M906

  /**
   * M906: Set motor currents
   */
  inline void gcode_M906(void) {

    if (commands.get_target_tool(906)) return;

    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i])) {
        const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
        externaldac.motor_current[a] = parser.value_ushort();
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

    #define TMC_SAY_CURRENT(Q) tmc.get_current(stepper##Q)
    #define TMC_SET_CURRENT(Q) tmc.set_current(stepper##Q, value)

    const uint8_t index = parser.byteval('I');
    LOOP_XYZE(i) if (uint16_t value = parser.intval(axis_codes[i])) {
      switch (i) {
        case X_AXIS:
          #if AXIS_HAS_TMC(X)
            if (index < 2) TMC_SET_CURRENT(X);
          #endif
          #if AXIS_HAS_TMC(X2)
            if (index == 2) TMC_SET_CURRENT(X2);
          #endif
          break;
        case Y_AXIS:
          #if AXIS_HAS_TMC(Y)
            if (index < 2) TMC_SET_CURRENT(Y);
          #endif
          #if AXIS_HAS_TMC(Y2)
            if (index == 2) TMC_SET_CURRENT(Y2);
          #endif
          break;
        case Z_AXIS:
          #if AXIS_HAS_TMC(Z)
            if (index < 2) TMC_SET_CURRENT(Z);
          #endif
          #if AXIS_HAS_TMC(Z2)
            if (index == 2) TMC_SET_CURRENT(Z2);
          #endif
          #if AXIS_HAS_TMC(Z3)
            if (index == 3) TMC_SET_CURRENT(Z3);
          #endif
          break;
        case E_AXIS: {
          switch (TARGET_EXTRUDER) {
            #if AXIS_HAS_TMC(E0)
              case 0: TMC_SET_CURRENT(E0); break;
            #endif
            #if AXIS_HAS_TMC(E1)
              case 1: TMC_SET_CURRENT(E1); break;
            #endif
            #if AXIS_HAS_TMC(E2)
              case 2: TMC_SET_CURRENT(E2); break;
            #endif
            #if AXIS_HAS_TMC(E3)
              case 3: TMC_SET_CURRENT(E3); break;
            #endif
            #if AXIS_HAS_TMC(E4)
              case 4: TMC_SET_CURRENT(E4); break;
            #endif
            #if AXIS_HAS_TMC(E5)
              case 5: TMC_SET_CURRENT(E5); break;
            #endif
          }
        } break;
      }
    }

    #if AXIS_HAS_TMC(X)
      TMC_SAY_CURRENT(X);
    #endif
    #if AXIS_HAS_TMC(X2)
      TMC_SAY_CURRENT(X2);
    #endif
    #if AXIS_HAS_TMC(Y)
      TMC_SAY_CURRENT(Y);
    #endif
    #if AXIS_HAS_TMC(Y2)
      TMC_SAY_CURRENT(Y2);
    #endif
    #if AXIS_HAS_TMC(Z)
      TMC_SAY_CURRENT(Z);
    #endif
    #if AXIS_HAS_TMC(Z2)
      TMC_SAY_CURRENT(Z2);
    #endif
    #if AXIS_HAS_TMC(Z3)
      TMC_SAY_CURRENT(Z3);
    #endif
    #if AXIS_HAS_TMC(E0)
      TMC_SAY_CURRENT(E0);
    #endif
    #if AXIS_HAS_TMC(E1)
      TMC_SAY_CURRENT(E1);
    #endif
    #if AXIS_HAS_TMC(E2)
      TMC_SAY_CURRENT(E2);
    #endif
    #if AXIS_HAS_TMC(E3)
      TMC_SAY_CURRENT(E3);
    #endif
    #if AXIS_HAS_TMC(E4)
      TMC_SAY_CURRENT(E4);
    #endif
    #if AXIS_HAS_TMC(E5)
      TMC_SAY_CURRENT(E5);
    #endif

  }

#endif // HAS_TRINAMIC
