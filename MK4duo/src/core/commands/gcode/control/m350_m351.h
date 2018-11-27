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

#if HAS_MICROSTEPS

  #define CODE_M350
  #define CODE_M351

  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350(void) {
    if (parser.seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, parser.value_byte());
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_mode(i, parser.value_byte());
    if (parser.seen('B')) stepper.microstep_mode(4, parser.value_byte());
    stepper.microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351(void) {
    if (parser.seen('S')) switch(parser.value_byte()) {
      case 1:
        LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_ms(i, parser.value_byte(), -1);
        if (parser.seen('B')) stepper.microstep_ms(4, parser.value_byte(), -1);
        break;
      case 2:
        LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_ms(i, -1, parser.value_byte());
        if (parser.seen('B')) stepper.microstep_ms(4, -1, parser.value_byte());
        break;
    }
    stepper.microstep_readings();
  }

#elif HAS_TRINAMIC

  #define CODE_M350

  inline void gcode_M350(void) {

    if (commands.get_target_tool(350)) return;

    #define TMC_SAY_MICROSTEP(Q)    tmc.get_microstep(stepper##Q)
    #define TMC_SET_MICROSTEP(Q)    tmc.set_microstep(stepper##Q, value)
    #define TMC_SAY_MICROSTEP_E(E)  tmc.get_microstep(stepperE##E);
    #define TMC_SET_MICROSTEP_E(E)  tmc.set_microstep(stepperE##E, value);

    const uint8_t index = parser.byteval('I');
    LOOP_XYZE(i) {
      if (uint16_t value = parser.ushortval(axis_codes[i])) {
        switch (i) {
          case X_AXIS:
            #if AXIS_HAS_TMC(X)
              if (index < 2) TMC_SET_MICROSTEP(X);
            #endif
            #if AXIS_HAS_TMC(X2)
              if (!(index & 1)) TMC_SET_MICROSTEP(X2);
            #endif
            break;
          case Y_AXIS:
            #if AXIS_HAS_TMC(Y)
              if (index < 2) TMC_SET_MICROSTEP(Y);
            #endif
            #if AXIS_HAS_TMC(Y2)
              if (!(index & 1)) TMC_SET_MICROSTEP(Y2);
            #endif
            break;
          case Z_AXIS:
            #if AXIS_HAS_TMC(Z)
              if (index < 2) TMC_SET_MICROSTEP(Z);
            #endif
            #if AXIS_HAS_TMC(Z2)
              if (index == 0 || index == 2) TMC_SET_MICROSTEP(Z2);
            #endif
            #if AXIS_HAS_TMC(Z3)
              if (index == 0 || index == 3) TMC_SET_MICROSTEP(Z3);
            #endif
            break;
          case E_AXIS: {
            switch (TARGET_EXTRUDER) {
              #if AXIS_HAS_TMC(E0)
                case 0: TMC_SET_MICROSTEP_E(0); break;
              #endif
              #if AXIS_HAS_TMC(E1)
                case 1: TMC_SET_MICROSTEP_E(1); break;
              #endif
              #if AXIS_HAS_TMC(E2)
                case 2: TMC_SET_MICROSTEP_E(2); break;
              #endif
              #if AXIS_HAS_TMC(E3)
                case 3: TMC_SET_MICROSTEP_E(3); break;
              #endif
              #if AXIS_HAS_TMC(E4)
                case 4: TMC_SET_MICROSTEP_E(4); break;
              #endif
              #if AXIS_HAS_TMC(E5)
                case 5: TMC_SET_MICROSTEP_E(5); break;
              #endif
            }
          } break;
        }
      }
    }

    #if AXIS_HAS_TMC(X)
      TMC_SAY_MICROSTEP(X);
    #endif
    #if AXIS_HAS_TMC(X2)
      TMC_SAY_MICROSTEP(X2);
    #endif
    #if AXIS_HAS_TMC(Y)
      TMC_SAY_MICROSTEP(Y);
    #endif
    #if AXIS_HAS_TMC(Y2)
      TMC_SAY_MICROSTEP(Y2);
    #endif
    #if AXIS_HAS_TMC(Z)
      TMC_SAY_MICROSTEP(Z);
    #endif
    #if AXIS_HAS_TMC(Z2)
      TMC_SAY_MICROSTEP(Z2);
    #endif
    #if AXIS_HAS_TMC(Z3)
      TMC_SAY_MICROSTEP(Z3);
    #endif
    #if AXIS_HAS_TMC(E0)
      TMC_SAY_MICROSTEP_E(0);
    #endif
    #if AXIS_HAS_TMC(E1)
      TMC_SAY_MICROSTEP_E(1);
    #endif
    #if AXIS_HAS_TMC(E2)
      TMC_SAY_MICROSTEP_E(2);
    #endif
    #if AXIS_HAS_TMC(E3)
      TMC_SAY_MICROSTEP_E(3);
    #endif
    #if AXIS_HAS_TMC(E4)
      TMC_SAY_MICROSTEP_E(4);
    #endif
    #if AXIS_HAS_TMC(E5)
      TMC_SAY_MICROSTEP_E(5);
    #endif

  }

#endif // HAS_TRINAMIC
