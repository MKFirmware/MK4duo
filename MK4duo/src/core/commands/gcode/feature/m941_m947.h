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

  /**
   * M941: TMC set MSLUT.
   */
  inline void gcode_M941(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        tmc.set_wave(tmc.stepperX, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if X2_IS_TRINAMIC
        tmc.set_wave(tmc.stepperX2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        tmc.set_wave(tmc.stepperY, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if Y2_IS_TRINAMIC
        tmc.set_wave(tmc.stepperY2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        tmc.set_wave(tmc.stepperZ, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if Z2_IS_TRINAMIC
        tmc.set_wave(tmc.stepperZ2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        tmc.set_wave(tmc.stepperE0, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
  }

  /**
   * M942: TMC reset MSLUT.
   */
  inline void gcode_M942(void) {
    if (parser.seen('X')) {
      #if X_IS_TRINAMIC
        tmc.reset_wave(tmc.stepperX);
      #endif
      #if X2_IS_TRINAMIC
        tmc.reset_wave(tmc.stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if Y_IS_TRINAMIC
        tmc.reset_wave(tmc.stepperY);
      #endif
      #if Y2_IS_TRINAMIC
        tmc.reset_wave(tmc.stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if Z_IS_TRINAMIC
        tmc.reset_wave(tmc.stepperZ);
      #endif
      #if Z2_IS_TRINAMIC
        tmc.reset_wave(tmc.stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if E0_IS_TRINAMIC
        tmc.reset_wave(tmc.stepperE0);
      #endif
    }
  }

  /**
   * M943: TMC set preset MSLUT.
   */
  inline void gcode_M943(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        tmc.set_fixed_wave(tmc.stepperX, (uint8_t)parser.value_int());
      #endif
      #if X2_IS_TRINAMIC
        tmc.set_fixed_wave(tmc.stepperX2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        tmc.set_fixed_wave(tmc.stepperY, (uint8_t)parser.value_int());
      #endif
      #if Y2_IS_TRINAMIC
        tmc.set_fixed_wave(tmc.stepperY2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        tmc.set_fixed_wave(tmc.stepperZ, (uint8_t)parser.value_int());
      #endif
      #if Z2_IS_TRINAMIC
        tmc.set_fixed_wave(tmc.stepperZ2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        tmc.set_fixed_wave(tmc.stepperE0, (uint8_t)parser.value_int());
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
      tmc.stepperX->stealthChop(parser.value_bool());
    #endif
    #if X2_IS_TRINAMIC
      tmc.stepperX2->stealthChop(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if Y_IS_TRINAMIC
      tmc.stepperY->stealthChop(parser.value_bool());
    #endif
    #if Y2_IS_TRINAMIC
      tmc.stepperY2->stealthChop(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if Z_IS_TRINAMIC
      tmc.stepperZ->stealthChop(parser.value_bool());
    #endif
    #if Z2_IS_TRINAMIC
      tmc.stepperZ2->stealthChop(parser.value_bool());
    #endif
    #if Z3_IS_TRINAMIC
      tmc.stepperZ3->stealthChop(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if E0_IS_TRINAMIC
      tmc.stepperE0->stealthChop(parser.value_bool());
    #endif
    #if E1_IS_TRINAMIC
      tmc.stepperE1->stealthChop(parser.value_bool());
    #endif
    #if E2_IS_TRINAMIC
      tmc.stepperE2->stealthChop(parser.value_bool());
    #endif
    #if E3_IS_TRINAMIC
      tmc.stepperE3->stealthChop(parser.value_bool());
    #endif
    #if E4_IS_TRINAMIC
      tmc.stepperE4->stealthChop(parser.value_bool());
    #endif
    #if E5_IS_TRINAMIC
      tmc.stepperE5->stealthChop(parser.value_bool());
    #endif
  }
}

/**
 * M946: TMC switch ChopperMode.
 */
inline void gcode_M946(void) {
  if (parser.seenval('X')) {
    #if X_IS_TRINAMIC
      tmc.stepperX->chopper_mode(parser.value_bool());
    #endif
    #if X2_IS_TRINAMIC
      tmc.stepperX2->chopper_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if Y_IS_TRINAMIC
      tmc.stepperY->chopper_mode(parser.value_bool());
    #endif
    #if Y2_IS_TRINAMIC
      tmc.stepperY2->chopper_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if Z_IS_TRINAMIC
      tmc.stepperZ->chopper_mode(parser.value_bool());
    #endif
    #if Z2_IS_TRINAMIC
      tmc.stepperZ2->chopper_mode(parser.value_bool());
    #endif
    #if Z3_IS_TRINAMIC
      tmc.stepperZ3->chopper_mode(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if E0_IS_TRINAMIC
      tmc.stepperE0->chopper_mode(parser.value_bool());
    #endif
    #if E1_IS_TRINAMIC
      tmc.stepperE1->chopper_mode(parser.value_bool());
    #endif
    #if E2_IS_TRINAMIC
      tmc.stepperE2->chopper_mode(parser.value_bool());
    #endif
    #if E3_IS_TRINAMIC
      tmc.stepperE3->chopper_mode(parser.value_bool());
    #endif
    #if E4_IS_TRINAMIC
      tmc.stepperE4->chopper_mode(parser.value_bool());
    #endif
    #if E5_IS_TRINAMIC
      tmc.stepperE5->chopper_mode(parser.value_bool());
    #endif
  }
}

/**
 * M947: TMC switch interpolation.
 */
inline void gcode_M947(void) {
  if (parser.seenval('X')) {
    #if X_IS_TRINAMIC
      tmc.stepperX->interpolate(parser.value_bool());
    #endif
    #if X2_IS_TRINAMIC
      tmc.stepperX2->interpolate(parser.value_bool());
    #endif
  }
  if (parser.seenval('Y')) {
    #if Y_IS_TRINAMIC
      tmc.stepperY->interpolate(parser.value_bool());
    #endif
    #if Y2_IS_TRINAMIC
      tmc.stepperY2->interpolate(parser.value_bool());
    #endif
  }
  if (parser.seenval('Z')) {
    #if Z_IS_TRINAMIC
      tmc.stepperZ->interpolate(parser.value_bool());
    #endif
    #if Z2_IS_TRINAMIC
      tmc.stepperZ2->interpolate(parser.value_bool());
    #endif
    #if Z3_IS_TRINAMIC
      tmc.stepperZ3->interpolate(parser.value_bool());
    #endif
  }
  if (parser.seenval('E')) {
    #if E0_IS_TRINAMIC
      tmc.stepperE0->interpolate(parser.value_bool());
    #endif
    #if E1_IS_TRINAMIC
      tmc.stepperE1->interpolate(parser.value_bool());
    #endif
    #if E2_IS_TRINAMIC
      tmc.stepperE2->interpolate(parser.value_bool());
    #endif
    #if E3_IS_TRINAMIC
      tmc.stepperE3->interpolate(parser.value_bool());
    #endif
    #if E4_IS_TRINAMIC
      tmc.stepperE4->interpolate(parser.value_bool());
    #endif
    #if E5_IS_TRINAMIC
      tmc.stepperE5->interpolate(parser.value_bool());
    #endif
  }
}
