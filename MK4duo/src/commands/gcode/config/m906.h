/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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

    #if DISABLED(DISABLE_M503)
      // No arguments? Show M906 report.
      if (!parser.seen("XYZE")) {
        tmc.print_M906();
        return;
      }
    #endif

    LOOP_XYZE(i) if (uint16_t value = parser.intval(axis_codes[i])) {
      switch (i) {
        case X_AXIS:
          #if AXIS_HAS_TMC(X)
            stepperX->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(X2)
            stepperX2->rms_current(value);
          #endif
          break;
        case Y_AXIS:
          #if AXIS_HAS_TMC(Y)
            stepperY->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(Y2)
            stepperY2->rms_current(value);
          #endif
          break;
        case Z_AXIS:
          #if AXIS_HAS_TMC(Z)
            stepperZ->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(Z2)
            stepperZ2->rms_current(value);
          #endif
          #if AXIS_HAS_TMC(Z3)
            stepperZ3->rms_current(value);
          #endif
          break;
        case E_AXIS: {
          switch (TARGET_EXTRUDER) {
            #if AXIS_HAS_TMC(E0)
              case 0: stepperE0->rms_current(value); break;
            #endif
            #if AXIS_HAS_TMC(E1)
              case 1: stepperE1->rms_current(value); break;
            #endif
            #if AXIS_HAS_TMC(E2)
              case 2: stepperE2->rms_current(value); break;
            #endif
            #if AXIS_HAS_TMC(E3)
              case 3: stepperE3->rms_current(value); break;
            #endif
            #if AXIS_HAS_TMC(E4)
              case 4: stepperE4->rms_current(value); break;
            #endif
            #if AXIS_HAS_TMC(E5)
              case 5: stepperE5->rms_current(value); break;
            #endif
          }
        } break;
      }
    }

  }

#endif // HAS_TRINAMIC
