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
        tmc2130_set_wave(stepperX, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if X2_IS_TRINAMIC
        tmc2130_set_wave(stepperX2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        tmc2130_set_wave(stepperY, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if Y2_IS_TRINAMIC
        tmc2130_set_wave(stepperY2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        tmc2130_set_wave(stepperZ, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
      #if Z2_IS_TRINAMIC
        tmc2130_set_wave(stepperZ2, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        tmc2130_set_wave(stepperE0, TMC2130_LINEARITY_CORRECTION_AMPL, (int16_t)parser.value_int());
      #endif
    }
  }

  /**
   * M942: TMC reset MSLUT.
   */
  inline void gcode_M942(void) {
    if (parser.seen('X')) {
      #if X_IS_TRINAMIC
        tmc2130_reset_wave(stepperX);
      #endif
      #if X2_IS_TRINAMIC
        tmc2130_reset_wave(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if Y_IS_TRINAMIC
        tmc2130_reset_wave(stepperY);
      #endif
      #if Y2_IS_TRINAMIC
        tmc2130_reset_wave(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if Z_IS_TRINAMIC
        tmc2130_reset_wave(stepperZ);
      #endif
      #if Z2_IS_TRINAMIC
        tmc2130_reset_wave(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if E0_IS_TRINAMIC
        tmc2130_reset_wave(stepperE0);
      #endif
    }
  }

  /**
   * M943: TMC set preset MSLUT.
   */
  inline void gcode_M943(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        tmc2130_set_fixed_wave(stepperX, (uint8_t)parser.value_int());
      #endif
      #if X2_IS_TRINAMIC
        tmc2130_set_fixed_wave(stepperX2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        tmc2130_set_fixed_wave(stepperY, (uint8_t)parser.value_int());
      #endif
      #if Y2_IS_TRINAMIC
        tmc2130_set_fixed_wave(stepperY2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        tmc2130_set_fixed_wave(stepperZ, (uint8_t)parser.value_int());
      #endif
      #if Z2_IS_TRINAMIC
        tmc2130_set_fixed_wave(stepperZ2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        tmc2130_set_fixed_wave(stepperE0, (uint8_t)parser.value_int());
      #endif
    }
  }

#endif // MSLUT_CALIBRATION

#if ENABLED(STEALTHCHOP)

  #define CODE_M945
  #define CODE_M946
  #define CODE_M947

  /**
   * M945: TMC switch StealthChop.
   */
  inline void gcode_M945(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        stepperX.stealthChop(parser.value_bool());
      #endif
      #if X2_IS_TRINAMIC
        stepperX2.stealthChop(parser.value_bool());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        stepperY.stealthChop(parser.value_bool());
      #endif
      #if Y2_IS_TRINAMIC
        stepperY2.stealthChop(parser.value_bool());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        stepperZ.stealthChop(parser.value_bool());
      #endif
      #if Z2_IS_TRINAMIC
        stepperZ2.stealthChop(parser.value_bool());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        stepperE0.stealthChop(parser.value_bool());
      #endif
    }
  }

  /**
   * M946: TMC switch ChopperMode.
   */
  inline void gcode_M946(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        stepperX.chopper_mode(parser.value_bool());
      #endif
      #if X2_IS_TRINAMIC
        stepperX2.chopper_mode(parser.value_bool());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        stepperY.chopper_mode(parser.value_bool());
      #endif
      #if Y2_IS_TRINAMIC
        stepperY2.chopper_mode(parser.value_bool());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        stepperZ.chopper_mode(parser.value_bool());
      #endif
      #if Z2_IS_TRINAMIC
        stepperZ2.chopper_mode(parser.value_bool());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        stepperE0.chopper_mode(parser.value_bool());
      #endif
    }
  }

  /**
   * M947: TMC switch interpolation.
   */
  inline void gcode_M947(void) {
    if (parser.seenval('X')) {
      #if X_IS_TRINAMIC
        stepperX.interpolate(parser.value_bool());
      #endif
      #if X2_IS_TRINAMIC
        stepperX2.interpolate(parser.value_bool());
      #endif
    }
    if (parser.seenval('Y')) {
      #if Y_IS_TRINAMIC
        stepperY.interpolate(parser.value_bool());
      #endif
      #if Y2_IS_TRINAMIC
        stepperY2.interpolate(parser.value_bool());
      #endif
    }
    if (parser.seenval('Z')) {
      #if Z_IS_TRINAMIC
        stepperZ.interpolate(parser.value_bool());
      #endif
      #if Z2_IS_TRINAMIC
        stepperZ2.interpolate(parser.value_bool());
      #endif
    }
    if (parser.seenval('E')) {
      #if E0_IS_TRINAMIC
        stepperE0.interpolate(parser.value_bool());
      #endif
    }
  }

#endif // STEALTHCHOP
