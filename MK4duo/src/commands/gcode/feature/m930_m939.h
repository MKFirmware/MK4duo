/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
        tmcManager.set_blank_time(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_blank_time(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_blank_time(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_blank_time(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_blank_time(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_blank_time(driver.z2, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.set_blank_time(driver.z3, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_blank_time(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_blank_time(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_blank_time(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_blank_time(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_blank_time(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_blank_time(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_blank_time(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.get_blank_time(driver.z3);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_blank_time(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_blank_time(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_blank_time(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_blank_time(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_blank_time(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_blank_time(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_blank_time(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.get_blank_time(driver.z3);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_blank_time(driver.e[0]);
      #endif
    }
  }

  /**
   * M931: TMC set off_time.
   */
  inline void gcode_M931() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_off_time(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_off_time(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_off_time(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_off_time(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_off_time(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_off_time(driver.z2, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.set_off_time(driver.z3, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_off_time(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_off_time(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_off_time(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_off_time(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_off_time(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_off_time(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_off_time(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.get_off_time(driver.z3);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_off_time(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_off_time(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_off_time(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_off_time(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_off_time(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_off_time(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_off_time(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.get_off_time(driver.z3);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_off_time(driver.e[0]);
      #endif
    }
  }

  /**
   * M932: TMC set hysteresis_start.
   */
  inline void gcode_M932() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_hysteresis_start(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_hysteresis_start(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_hysteresis_start(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_hysteresis_start(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_hysteresis_start(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_hysteresis_start(driver.z2, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.set_hysteresis_start(driver.z3, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_hysteresis_start(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_hysteresis_start(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_hysteresis_start(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_hysteresis_start(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_hysteresis_start(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_hysteresis_start(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_hysteresis_start(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.get_hysteresis_start(driver.z3);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_hysteresis_start(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_hysteresis_start(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_hysteresis_start(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_hysteresis_start(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_hysteresis_start(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_hysteresis_start(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_hysteresis_start(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.get_hysteresis_start(driver.z3);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_hysteresis_start(driver.e[0]);
      #endif
    }
  }

  /**
   * M933: TMC set hysteresis_end.
   */
  inline void gcode_M933() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_hysteresis_end(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_hysteresis_end(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_hysteresis_end(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_hysteresis_end(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_hysteresis_end(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_hysteresis_end(driver.z2, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.set_hysteresis_end(driver.z3, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_hysteresis_end(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_hysteresis_end(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_hysteresis_end(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_hysteresis_end(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_hysteresis_end(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_hysteresis_end(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_hysteresis_end(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmcManager.get_hysteresis_end(driver.z3);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_hysteresis_end(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_hysteresis_end(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_hysteresis_end(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_hysteresis_end(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_hysteresis_end(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_hysteresis_end(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_hysteresis_end(driver.z2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_hysteresis_end(driver.e[0]);
      #endif
    }
  }

  /**
   * M934: TMC set fast_decay_time (chm = 1).
    not user for now
  inline void gcode_M934() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_fast_decay_time(driver.x, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_fast_decay_time(driver.x2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_fast_decay_time(driver.y, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_fast_decay_time(driver.y2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_fast_decay_time(driver.z, (uint8_t)parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_fast_decay_time(driver.z2, (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_fast_decay_time(driver.e[0], (uint8_t)parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_fast_decay_time(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_fast_decay_time(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_fast_decay_time(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_fast_decay_time(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_fast_decay_time(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_fast_decay_time(driver.z2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_fast_decay_time(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_fast_decay_time(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_fast_decay_time(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_fast_decay_time(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_fast_decay_time(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_fast_decay_time(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_fast_decay_time(driver.z2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_fast_decay_time(driver.e[0]);
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
        tmcManager.set_disable_I_comparator(driver.x, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_disable_I_comparator(driver.x2, parser.value_bool());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_disable_I_comparator(driver.y, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_disable_I_comparator(driver.y2, parser.value_bool());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_disable_I_comparator(driver.z, parser.value_bool());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_disable_I_comparator(driver.z2, parser.value_bool());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_disable_I_comparator(driver.e[0], parser.value_bool());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_disable_I_comparator(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_disable_I_comparator(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_disable_I_comparator(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_disable_I_comparator(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_disable_I_comparator(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_disable_I_comparator(driver.z2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_disable_I_comparator(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_disable_I_comparator(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_disable_I_comparator(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_disable_I_comparator(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_disable_I_comparator(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_disable_I_comparator(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_disable_I_comparator(driver.z2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_disable_I_comparator(driver.e[0]);
      #endif
    }
  }

  /**
   * M936: TMC set stealth_gradient.
   */
  inline void gcode_M936() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_stealth_gradient(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_stealth_gradient(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_stealth_gradient(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_stealth_gradient(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_stealth_gradient(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_stealth_gradient(driver.z2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_stealth_gradient(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_gradient(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_gradient(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_gradient(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_gradient(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_gradient(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_gradient(driver.z2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_gradient(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_gradient(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_gradient(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_gradient(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_gradient(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_gradient(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_gradient(driver.z2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_gradient(driver.e[0]);
      #endif
    }
  }

  /**
   * M937: TMC set stealth_amplitude.
   */
  inline void gcode_M937() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_stealth_amplitude(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_stealth_amplitude(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_stealth_amplitude(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_stealth_amplitude(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_stealth_amplitude(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_stealth_amplitude(driver.z2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_stealth_amplitude(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_amplitude(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_amplitude(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_amplitude(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_amplitude(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_amplitude(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_amplitude(driver.z2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_amplitude(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_amplitude(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_amplitude(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_amplitude(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_amplitude(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_amplitude(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_amplitude(driver.z2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_amplitude(driver.e[0]);
      #endif
    }
  }

  /**
   * M938: TMC set stealth_freq.
   */
  inline void gcode_M938() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_stealth_freq(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_stealth_freq(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_stealth_freq(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_stealth_freq(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_stealth_freq(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_stealth_freq(driver.z2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_stealth_freq(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_freq(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_freq(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_freq(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_freq(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_freq(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_freq(driver.z2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_freq(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_freq(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_freq(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_freq(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_freq(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_freq(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_freq(driver.z2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_freq(driver.e[0]);
      #endif
    }
  }

  /**
   * M939: TMC switch stealth_autoscale.
   */
  inline void gcode_M939() {
    if (parser.seenval('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.set_stealth_autoscale(driver.x, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.set_stealth_autoscale(driver.x2, parser.value_int());
      #endif
    }
    if (parser.seenval('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.set_stealth_autoscale(driver.y, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.set_stealth_autoscale(driver.y2, parser.value_int());
      #endif
    }
    if (parser.seenval('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.set_stealth_autoscale(driver.z, parser.value_int());
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.set_stealth_autoscale(driver.z2, parser.value_int());
      #endif
    }
    if (parser.seenval('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.set_stealth_autoscale(driver.e[0], parser.value_int());
      #endif
    }
    if (parser.seen('X')) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_autoscale(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_autoscale(driver.x2);
      #endif
    }
    if (parser.seen('Y')) {
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_autoscale(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_autoscale(driver.y2);
      #endif
    }
    if (parser.seen('Z')) {
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_autoscale(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_autoscale(driver.z2);
      #endif
    }
    if (parser.seen('E')) {
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_autoscale(driver.e[0]);
      #endif
    }
    if (!parser.seen_axis()) {
      #if AXIS_HAS_TMC(X)
        tmcManager.get_stealth_autoscale(driver.x);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmcManager.get_stealth_autoscale(driver.y);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmcManager.get_stealth_autoscale(driver.z);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmcManager.get_stealth_autoscale(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmcManager.get_stealth_autoscale(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmcManager.get_stealth_autoscale(driver.z2);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmcManager.get_stealth_autoscale(driver.e[0]);
      #endif
    }
  }

#endif // HAVE_DRV(TMC2130)
