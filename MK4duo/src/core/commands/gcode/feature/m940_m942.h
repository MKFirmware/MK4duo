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

#define CODE_M940
#define CODE_M941
#define CODE_M942

/**
 * M940: TMC switch StealthChop.
 */
inline void gcode_M940(void) {
  if (parser.seenval('X')) {
    #if AXIS_HAS_TMC(X)
      stepperX->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(X2)
      stepperX2->en_pwm_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if AXIS_HAS_TMC(Y)
      stepperY->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Y2)
      stepperY2->en_pwm_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if AXIS_HAS_TMC(Z)
      stepperZ->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Z2)
      stepperZ2->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(Z3)
      stepperZ3->en_pwm_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if AXIS_HAS_TMC(E0)
      stepperE0->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E1)
      stepperE1->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E2)
      stepperE2->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E3)
      stepperE3->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E4)
      stepperE4->en_pwm_mode(parser.value_bool());
    #endif
    #if AXIS_HAS_TMC(E5)
      stepperE5->en_pwm_mode(parser.value_bool());
    #endif
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
