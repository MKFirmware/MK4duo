/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
  inline void gcode_M930() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_blank_time(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_blank_time(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_blank_time(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_blank_time(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_blank_time(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_blank_time(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_blank_time(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_blank_time(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_blank_time(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_blank_time(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_blank_time(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_blank_time(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_blank_time(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_blank_time(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_blank_time(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_blank_time(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_blank_time(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_blank_time(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_blank_time(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_blank_time(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_blank_time(E0_DRV);
      #endif
    }
  }

  /**
   * M931: TMC set off_time.
   */
  inline void gcode_M931() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_off_time(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_off_time(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_off_time(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_off_time(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_off_time(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_off_time(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_off_time(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_off_time(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_off_time(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_off_time(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_off_time(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_off_time(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_off_time(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_off_time(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_off_time(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_off_time(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_off_time(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_off_time(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_off_time(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_off_time(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_off_time(E0_DRV);
      #endif
    }
  }

  /**
   * M932: TMC set hysteresis_start.
   */
  inline void gcode_M932() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_hysteresis_start(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_hysteresis_start(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_hysteresis_start(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_hysteresis_start(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_hysteresis_start(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_hysteresis_start(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_hysteresis_start(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_start(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_start(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_start(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_start(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_start(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_start(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_start(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_start(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_start(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_start(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_start(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_start(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_start(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_start(E0_DRV);
      #endif
    }
  }

  /**
   * M933: TMC set hysteresis_end.
   */
  inline void gcode_M933() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_hysteresis_end(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_hysteresis_end(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_hysteresis_end(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_hysteresis_end(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_hysteresis_end(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_hysteresis_end(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_hysteresis_end(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_end(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_end(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_end(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_end(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_end(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_end(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_end(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_hysteresis_end(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_hysteresis_end(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_hysteresis_end(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_hysteresis_end(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_hysteresis_end(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_hysteresis_end(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_hysteresis_end(E0_DRV);
      #endif
    }
  }

  /**
   * M934: TMC set fast_decay_time (chm = 1).
    not user for now
  inline void gcode_M934() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_fast_decay_time(X_DRV, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_fast_decay_time(X2_DRV, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_fast_decay_time(Y_DRV, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_fast_decay_time(Y2_DRV, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_fast_decay_time(Z_DRV, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_fast_decay_time(Z2_DRV, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_fast_decay_time(E0_DRV, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_fast_decay_time(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_fast_decay_time(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_fast_decay_time(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_fast_decay_time(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_fast_decay_time(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_fast_decay_time(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_fast_decay_time(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_fast_decay_time(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_fast_decay_time(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_fast_decay_time(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_fast_decay_time(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_fast_decay_time(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_fast_decay_time(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_fast_decay_time(E0_DRV);
      #endif
    }
  }
  */

  /**
   * M935: TMC set disable_I_comparator (chm = 1).
   */
  inline void gcode_M935() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_disable_I_comparator(X_DRV, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_disable_I_comparator(X2_DRV, parser.value_bool());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_disable_I_comparator(Y_DRV, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_disable_I_comparator(Y2_DRV, parser.value_bool());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_disable_I_comparator(Z_DRV, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_disable_I_comparator(Z2_DRV, parser.value_bool());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_disable_I_comparator(E0_DRV, parser.value_bool());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_disable_I_comparator(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_disable_I_comparator(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_disable_I_comparator(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_disable_I_comparator(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_disable_I_comparator(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_disable_I_comparator(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_disable_I_comparator(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_disable_I_comparator(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_disable_I_comparator(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_disable_I_comparator(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_disable_I_comparator(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_disable_I_comparator(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_disable_I_comparator(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_disable_I_comparator(E0_DRV);
      #endif
    }
  }

  /**
   * M936: TMC set stealth_gradient.
   */
  inline void gcode_M936() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_gradient(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_gradient(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_gradient(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_gradient(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_gradient(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_gradient(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_gradient(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_gradient(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_gradient(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_gradient(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_gradient(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_gradient(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_gradient(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_gradient(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_gradient(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_gradient(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_gradient(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_gradient(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_gradient(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_gradient(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_gradient(E0_DRV);
      #endif
    }
  }

  /**
   * M937: TMC set stealth_amplitude.
   */
  inline void gcode_M937() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_amplitude(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_amplitude(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_amplitude(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_amplitude(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_amplitude(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_amplitude(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_amplitude(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_amplitude(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_amplitude(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_amplitude(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_amplitude(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_amplitude(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_amplitude(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_amplitude(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_amplitude(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_amplitude(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_amplitude(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_amplitude(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_amplitude(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_amplitude(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_amplitude(E0_DRV);
      #endif
    }
  }

  /**
   * M938: TMC set stealth_freq.
   */
  inline void gcode_M938() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_freq(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_freq(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_freq(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_freq(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_freq(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_freq(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_freq(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_freq(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_freq(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_freq(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_freq(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_freq(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_freq(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_freq(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_freq(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_freq(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_freq(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_freq(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_freq(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_freq(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_freq(E0_DRV);
      #endif
    }
  }

  /**
   * M939: TMC switch stealth_autoscale.
   */
  inline void gcode_M939() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.set_stealth_autoscale(X_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.set_stealth_autoscale(X2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.set_stealth_autoscale(Y_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.set_stealth_autoscale(Y2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.set_stealth_autoscale(Z_DRV, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.set_stealth_autoscale(Z2_DRV, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.set_stealth_autoscale(E0_DRV, parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_autoscale(X_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_autoscale(X2_DRV);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_autoscale(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_autoscale(Y2_DRV);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_autoscale(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_autoscale(Z2_DRV);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_autoscale(E0_DRV);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmc.get_stealth_autoscale(X_DRV);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.get_stealth_autoscale(Y_DRV);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.get_stealth_autoscale(Z_DRV);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.get_stealth_autoscale(X2_DRV);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.get_stealth_autoscale(Y2_DRV);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.get_stealth_autoscale(Z2_DRV);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.get_stealth_autoscale(E0_DRV);
      #endif
    }
  }

#endif // HAVE_DRV(TMC2130)
