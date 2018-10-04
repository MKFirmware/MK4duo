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

#if ENABLED(MSLUT_CALIBRATION)

  #define CODE_M941
  #define CODE_M942
  #define CODE_M943

  #define TMC2130_LINEARITY_CORRECTION_AMPL 247

  /**
   * M941: TMC set MSLUT.
   */
  inline void gcode_M941(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        tmc.set_wave(stepperX, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if X2_IS_TRINAMIC
        tmc.set_wave(stepperX2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        tmc.set_wave(stepperY, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if Y2_IS_TRINAMIC
        tmc.set_wave(stepperY2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        tmc.set_wave(stepperZ, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if Z2_IS_TRINAMIC
        tmc.set_wave(stepperZ2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        tmc.set_wave(stepperE0, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
  }

  /**
   * M942: TMC reset MSLUT.
   */
  inline void gcode_M942(void) {
    if (parser.seen('X')) {
      #if X_IS_TRINAMIC
        tmc.reset_wave(stepperX);
      #endif
      #if X2_IS_TRINAMIC
        tmc.reset_wave(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if Y_IS_TRINAMIC
        tmc.reset_wave(stepperY);
      #endif
      #if Y2_IS_TRINAMIC
        tmc.reset_wave(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if Z_IS_TRINAMIC
        tmc.reset_wave(stepperZ);
      #endif
      #if Z2_IS_TRINAMIC
        tmc.reset_wave(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if E0_IS_TRINAMIC
        tmc.reset_wave(stepperE0);
      #endif
    }
  }

  /**
   * M943: TMC set preset MSLUT.
   */
  inline void gcode_M943(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        tmc.set_fixed_wave(stepperX, (uint8_t)parser.value_int());
      #endif
      #if X2_IS_TRINAMIC
        tmc.set_fixed_wave(stepperX2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        tmc.set_fixed_wave(stepperY, (uint8_t)parser.value_int());
      #endif
      #if Y2_IS_TRINAMIC
        tmc.set_fixed_wave(stepperY2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        tmc.set_fixed_wave(stepperZ, (uint8_t)parser.value_int());
      #endif
      #if Z2_IS_TRINAMIC
        tmc.set_fixed_wave(stepperZ2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        tmc.set_fixed_wave(stepperE0, (uint8_t)parser.value_int());
      #endif
    }
  }

#endif // MSLUT_CALIBRATION

#define CODE_M945
#define CODE_M946
#define CODE_M947

/**
 * M945: TMC switch StealthChop.
 */
inline void gcode_M945(void) {
  if (parser.seenval('X')) {
    #if X_IS_TRINAMIC
      stepperX.en_pwm_mode(parser.value_bool());
    #endif
    #if X2_IS_TRINAMIC
      stepperX2->en_pwm_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if Y_IS_TRINAMIC
      stepperY.en_pwm_mode(parser.value_bool());
    #endif
    #if Y2_IS_TRINAMIC
      stepperY2->en_pwm_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if Z_IS_TRINAMIC
      stepperZ.en_pwm_mode(parser.value_bool());
    #endif
    #if Z2_IS_TRINAMIC
      stepperZ2->en_pwm_mode(parser.value_bool());
    #endif
    #if Z3_IS_TRINAMIC
      stepperZ3->en_pwm_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if E0_IS_TRINAMIC
      stepperE0->en_pwm_mode(parser.value_bool());
    #endif
    #if E1_IS_TRINAMIC
      stepperE1->en_pwm_mode(parser.value_bool());
    #endif
    #if E2_IS_TRINAMIC
      stepperE2->en_pwm_mode(parser.value_bool());
    #endif
    #if E3_IS_TRINAMIC
      stepperE3->en_pwm_mode(parser.value_bool());
    #endif
    #if E4_IS_TRINAMIC
      stepperE4->en_pwm_mode(parser.value_bool());
    #endif
    #if E5_IS_TRINAMIC
      stepperE5->en_pwm_mode(parser.value_bool());
    #endif
  }
}

/**
 * M946: TMC switch ChopperMode.
 */
inline void gcode_M946(void) {
  if (parser.seenval('X')) {
    #if X_IS_TRINAMIC
      stepperX.chm(parser.value_bool());
    #endif
    #if X2_IS_TRINAMIC
      stepperX2->chm(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if Y_IS_TRINAMIC
      stepperY.chm(parser.value_bool());
    #endif
    #if Y2_IS_TRINAMIC
      stepperY2->chm(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if Z_IS_TRINAMIC
      stepperZ.chm(parser.value_bool());
    #endif
    #if Z2_IS_TRINAMIC
      stepperZ2->chm(parser.value_bool());
    #endif
    #if Z3_IS_TRINAMIC
      stepperZ3->chm(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if E0_IS_TRINAMIC
      stepperE0->chm(parser.value_bool());
    #endif
    #if E1_IS_TRINAMIC
      stepperE1->chm(parser.value_bool());
    #endif
    #if E2_IS_TRINAMIC
      stepperE2->chm(parser.value_bool());
    #endif
    #if E3_IS_TRINAMIC
      stepperE3->chm(parser.value_bool());
    #endif
    #if E4_IS_TRINAMIC
      stepperE4->chm(parser.value_bool());
    #endif
    #if E5_IS_TRINAMIC
      stepperE5->chm(parser.value_bool());
    #endif
  }
}

/**
 * M947: TMC switch interpolation.
 */
inline void gcode_M947(void) {
  if (parser.seenval('X')) {
    #if X_IS_TRINAMIC
      stepperX.intpol(parser.value_bool());
    #endif
    #if X2_IS_TRINAMIC
      stepperX2->intpol(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if Y_IS_TRINAMIC
      stepperY.intpol(parser.value_bool());
    #endif
    #if Y2_IS_TRINAMIC
      stepperY2->intpol(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if Z_IS_TRINAMIC
      stepperZ.intpol(parser.value_bool());
    #endif
    #if Z2_IS_TRINAMIC
      stepperZ2->intpol(parser.value_bool());
    #endif
    #if Z3_IS_TRINAMIC
      stepperZ3->intpol(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if E0_IS_TRINAMIC
      stepperE0->intpol(parser.value_bool());
    #endif
    #if E1_IS_TRINAMIC
      stepperE1->intpol(parser.value_bool());
    #endif
    #if E2_IS_TRINAMIC
      stepperE2->intpol(parser.value_bool());
    #endif
    #if E3_IS_TRINAMIC
      stepperE3->intpol(parser.value_bool());
    #endif
    #if E4_IS_TRINAMIC
      stepperE4->intpol(parser.value_bool());
    #endif
    #if E5_IS_TRINAMIC
      stepperE5->intpol(parser.value_bool());
    #endif
  }
}
