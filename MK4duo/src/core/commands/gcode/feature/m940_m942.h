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

#if HAVE_DRV(TMC2130)

#define CODE_M940
#define CODE_M941
#define CODE_M942

inline void tmc_set_stealthChop(MKTMC* st, const bool onoff) {
  st->stealthChop_enabled = onoff;
  st->refresh_stepping_mode();
}

/**
 * M940: TMC switch StealthChop.
 */
inline void gcode_M940(void) {
  
  #if DISABLED(DISABLE_M503)
    // No arguments? Show M940 report.
    if (!parser.seen("XYZE")) {
      tmc.print_M940();
      return;
    }
  #endif

  LOOP_XYZE(i) {
    if (const bool value = parser.boolval(axis_codes[i])) {
      switch (i) {
        case X_AXIS:
          #if AXIS_HAS_STEALTHCHOP(X)
            tmc_set_stealthChop(stepperX, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(X2)
            tmc_set_stealthChop(stepperX2, value);
          #endif
          break;
        case Y_AXIS:
          #if AXIS_HAS_STEALTHCHOP(Y)
            tmc_set_stealthChop(stepperY, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(Y2)
            tmc_set_stealthChop(stepperY2, value);
          #endif
          break;
        case Z_AXIS:
          #if AXIS_HAS_STEALTHCHOP(Z)
            tmc_set_stealthChop(stepperZ, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(Z2)
            tmc_set_stealthChop(stepperZ2, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(Z3)
            tmc_set_stealthChop(stepperZ3, value);
          #endif
          break;
        case E_AXIS:
          #if AXIS_HAS_STEALTHCHOP(E0)
            tmc_set_stealthChop(stepperE0, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(E1)
            tmc_set_stealthChop(stepperE1, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(E2)
            tmc_set_stealthChop(stepperE2, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(E3)
            tmc_set_stealthChop(stepperE3, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(E4)
            tmc_set_stealthChop(stepperE4, value);
          #endif
          #if AXIS_HAS_STEALTHCHOP(E5)
            tmc_set_stealthChop(stepperE5, value);
          #endif
        break;
      }
    }
  }
}

/**
 * M941: TMC switch ChopperMode.
 */
inline void gcode_M941(void) {
  if (parser.seenval('X')) {
    #if AXIS_HAS_TMC(X)
      stepperX->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(X2)
      stepperX2->chm(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if AXIS_HAS_TMC(Y)
      stepperY->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Y2)
      stepperY2->chm(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if AXIS_HAS_TMC(Z)
      stepperZ->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Z2)
      stepperZ2->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Z3)
      stepperZ3->chm(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if AXIS_HAS_TMC(E0)
      stepperE0->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E1)
      stepperE1->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E2)
      stepperE2->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E3)
      stepperE3->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E4)
      stepperE4->chm(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E5)
      stepperE5->chm(parser.value_bool());
    #endif
  }
}

/**
 * M942: TMC switch interpolation.
 */
inline void gcode_M942(void) {
  if (parser.seenval('X')) {
    #if AXIS_HAS_TMC(X)
      stepperX->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(X2)
      stepperX2->intpol(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if AXIS_HAS_TMC(Y)
      stepperY->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Y2)
      stepperY2->intpol(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if AXIS_HAS_TMC(Z)
      stepperZ->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Z2)
      stepperZ2->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Z3)
      stepperZ3->intpol(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if AXIS_HAS_TMC(E0)
      stepperE0->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E1)
      stepperE1->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E2)
      stepperE2->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E3)
      stepperE3->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E4)
      stepperE4->intpol(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E5)
      stepperE5->intpol(parser.value_bool());
    #endif
  }
}

#endif // HAVE_DRV(TMC2130)
