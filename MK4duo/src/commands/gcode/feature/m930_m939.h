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

#if HAVE_DRV(TMC2130)

  #define CODE_M930
  #define CODE_M931
  #define CODE_M932
  #define CODE_M933
  //#define CODE_M934
  #define CODE_M935
  #define CODE_M936
  #define CODE_M937
  #define CODE_M938
  #define CODE_M939

  /**
   * M930: TMC set blank_time.
   */
  inline void gcode_M930(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_blank_time(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_blank_time(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_blank_time(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_blank_time(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_blank_time(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_blank_time(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_blank_time(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_blank_time(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_blank_time(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_blank_time(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_blank_time(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_blank_time(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_blank_time(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_blank_time(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_blank_time(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_blank_time(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_blank_time(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_blank_time(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_blank_time(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_blank_time(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_blank_time(stepperE0);
      #endif
    }
  }

  /**
   * M931: TMC set off_time.
   */
  inline void gcode_M931(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_off_time(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_off_time(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_off_time(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_off_time(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_off_time(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_off_time(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_off_time(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_off_time(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_off_time(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_off_time(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_off_time(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_off_time(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_off_time(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_off_time(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_off_time(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_off_time(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_off_time(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_off_time(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_off_time(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_off_time(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_off_time(stepperE0);
      #endif
    }
  }

  /**
   * M932: TMC set hysteresis_start.
   */
  inline void gcode_M932(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_hysteresis_start(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_hysteresis_start(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_hysteresis_start(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_hysteresis_start(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_hysteresis_start(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_hysteresis_start(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_hysteresis_start(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_start(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_start(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_start(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_start(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_start(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_start(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_start(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_start(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_start(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_start(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_start(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_start(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_start(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_start(stepperE0);
      #endif
    }
  }

  /**
   * M933: TMC set hysteresis_end.
   */
  inline void gcode_M933(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_hysteresis_end(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_hysteresis_end(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_hysteresis_end(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_hysteresis_end(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_hysteresis_end(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_hysteresis_end(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_hysteresis_end(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_end(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_end(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_end(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_end(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_end(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_end(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_end(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_end(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_end(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_end(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_end(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_end(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_end(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_end(stepperE0);
      #endif
    }
  }

  /**
   * M934: TMC set fast_decay_time (chm = 1).
    not user for now
  inline void gcode_M934(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_fast_decay_time(stepperX, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_fast_decay_time(stepperX2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_fast_decay_time(stepperY, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_fast_decay_time(stepperY2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_fast_decay_time(stepperZ, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_fast_decay_time(stepperZ2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_fast_decay_time(stepperE0, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_fast_decay_time(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_fast_decay_time(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_fast_decay_time(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_fast_decay_time(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_fast_decay_time(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_fast_decay_time(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_fast_decay_time(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_fast_decay_time(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_fast_decay_time(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_fast_decay_time(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_fast_decay_time(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_fast_decay_time(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_fast_decay_time(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_fast_decay_time(stepperE0);
      #endif
    }
  }
  */

  /**
   * M935: TMC set disable_I_comparator (chm = 1).
   */
  inline void gcode_M935(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_disable_I_comparator(stepperX, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_disable_I_comparator(stepperX2, parser.value_bool());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_disable_I_comparator(stepperY, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_disable_I_comparator(stepperY2, parser.value_bool());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_disable_I_comparator(stepperZ, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_disable_I_comparator(stepperZ2, parser.value_bool());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_disable_I_comparator(stepperE0, parser.value_bool());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_disable_I_comparator(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_disable_I_comparator(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_disable_I_comparator(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_disable_I_comparator(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_disable_I_comparator(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_disable_I_comparator(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_disable_I_comparator(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_disable_I_comparator(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_disable_I_comparator(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_disable_I_comparator(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_disable_I_comparator(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_disable_I_comparator(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_disable_I_comparator(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_disable_I_comparator(stepperE0);
      #endif
    }
  }

  /**
   * M936: TMC set stealth_gradient.
   */
  inline void gcode_M936(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_gradient(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_gradient(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_gradient(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_gradient(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_gradient(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_gradient(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_gradient(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_gradient(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_gradient(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_gradient(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_gradient(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_gradient(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_gradient(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_gradient(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_gradient(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_gradient(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_gradient(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_gradient(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_gradient(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_gradient(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_gradient(stepperE0);
      #endif
    }
  }

  /**
   * M937: TMC set stealth_amplitude.
   */
  inline void gcode_M937(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_amplitude(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_amplitude(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_amplitude(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_amplitude(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_amplitude(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_amplitude(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_amplitude(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_amplitude(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_amplitude(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_amplitude(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_amplitude(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_amplitude(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_amplitude(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_amplitude(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_amplitude(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_amplitude(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_amplitude(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_amplitude(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_amplitude(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_amplitude(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_amplitude(stepperE0);
      #endif
    }
  }

  /**
   * M938: TMC set stealth_freq.
   */
  inline void gcode_M938(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_freq(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_freq(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_freq(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_freq(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_freq(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_freq(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_freq(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_freq(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_freq(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_freq(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_freq(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_freq(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_freq(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_freq(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_freq(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_freq(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_freq(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_freq(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_freq(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_freq(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_freq(stepperE0);
      #endif
    }
  }

  /**
   * M939: TMC switch stealth_autoscale.
   */
  inline void gcode_M939(void) {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_autoscale(stepperX, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_autoscale(stepperX2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_autoscale(stepperY, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_autoscale(stepperY2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_autoscale(stepperZ, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_autoscale(stepperZ2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_autoscale(stepperE0, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_autoscale(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_autoscale(stepperX2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_autoscale(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_autoscale(stepperY2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_autoscale(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_autoscale(stepperZ2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_autoscale(stepperE0);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_autoscale(stepperX);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_autoscale(stepperY);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_autoscale(stepperZ);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_autoscale(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_autoscale(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_autoscale(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_autoscale(stepperE0);
      #endif
    }
  }

#endif // HAVE_DRV(TMC2130)
