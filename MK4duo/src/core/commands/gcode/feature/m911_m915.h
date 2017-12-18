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

#if HAS_TRINAMIC

  #define CODE_M911

  /**
   * M911: Report TMC stepper driver overtemperature pre-warn flag
   * The flag is held by the library and persist until manually cleared by M912
   */
  inline void gcode_M911(void) {
    #if ENABLED(X_IS_TMC2130) || (ENABLED(X_IS_TMC2208) && PIN_EXISTS(X_SERIAL_RX)) || ENABLED(IS_TRAMS)
      tmc_report_otpw(stepperX, extended_axis_codes[TMC_X]);
    #endif
    #if ENABLED(Y_IS_TMC2130) || (ENABLED(Y_IS_TMC2208) && PIN_EXISTS(Y_SERIAL_RX)) || ENABLED(IS_TRAMS)
      tmc_report_otpw(stepperY, extended_axis_codes[TMC_Y]);
    #endif
    #if ENABLED(Z_IS_TMC2130) || (ENABLED(Z_IS_TMC2208) && PIN_EXISTS(Z_SERIAL_RX)) || ENABLED(IS_TRAMS)
      tmc_report_otpw(stepperZ, extended_axis_codes[TMC_Z]);
    #endif
    #if ENABLED(E0_IS_TMC2130) || (ENABLED(E0_IS_TMC2208) && PIN_EXISTS(E0_SERIAL_RX)) || ENABLED(IS_TRAMS)
      tmc_report_otpw(stepperE0, extended_axis_codes[TMC_E0]);
    #endif
  }

  #define CODE_M912

  /**
   * M912: Clear TMC stepper driver overtemperature pre-warn flag held by the library
   */
  inline void gcode_M912(void) {
    const bool clearX = parser.seen(axis_codes[X_AXIS]), clearY = parser.seen(axis_codes[Y_AXIS]), clearZ = parser.seen(axis_codes[Z_AXIS]), clearE = parser.seen(axis_codes[E_AXIS]),
             clearAll = (!clearX && !clearY && !clearZ && !clearE) || (clearX && clearY && clearZ && clearE);
    #if ENABLED(X_IS_TMC2130) || ENABLED(IS_TRAMS) || (ENABLED(X_IS_TMC2208) && PIN_EXISTS(X_SERIAL_RX))
      if (clearX || clearAll) tmc_clear_otpw(stepperX, extended_axis_codes[TMC_X]);
    #endif
    #if ENABLED(X2_IS_TMC2130) || (ENABLED(X2_IS_TMC2208) && PIN_EXISTS(X_SERIAL_RX))
      if (clearX || clearAll) tmc_clear_otpw(stepperX, extended_axis_codes[TMC_X]);
    #endif

    #if ENABLED(Y_IS_TMC2130) || (ENABLED(Y_IS_TMC2208) && PIN_EXISTS(Y_SERIAL_RX))
      if (clearY || clearAll) tmc_clear_otpw(stepperY, extended_axis_codes[TMC_Y]);
    #endif

    #if ENABLED(Z_IS_TMC2130) || (ENABLED(Z_IS_TMC2208) && PIN_EXISTS(Z_SERIAL_RX))
      if (clearZ || clearAll) tmc_clear_otpw(stepperZ, extended_axis_codes[TMC_Z]);
    #endif

    #if ENABLED(E0_IS_TMC2130) || (ENABLED(E0_IS_TMC2208) && PIN_EXISTS(E0_SERIAL_RX))
      if (clearE || clearAll) tmc_clear_otpw(stepperE0, extended_axis_codes[TMC_E0]);
    #endif
  }

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  #if ENABLED(HYBRID_THRESHOLD)
    #define CODE_M913
    inline void gcode_M913(void) {
      uint16_t values[XYZE];
      LOOP_XYZE(i)
        values[i] = parser.intval(axis_codes[i]);

      #if X_IS_TRINAMIC
        if (values[X_AXIS]) tmc_set_pwmthrs(stepperX, extended_axis_codes[TMC_X], values[X_AXIS], mechanics.axis_steps_per_mm[X_AXIS]);
        else tmc_get_pwmthrs(stepperX, extended_axis_codes[TMC_X], mechanics.axis_steps_per_mm[X_AXIS]);
      #endif
      #if X2_IS_TRINAMIC
        if (values[X_AXIS]) tmc_set_pwmthrs(stepperX2, extended_axis_codes[TMC_X2], values[X_AXIS], mechanics.axis_steps_per_mm[X_AXIS]);
        else tmc_get_pwmthrs(stepperX, extended_axis_codes[TMC_X2], mechanics.axis_steps_per_mm[X_AXIS]);
      #endif

      #if Y_IS_TRINAMIC
        if (values[Y_AXIS]) tmc_set_pwmthrs(stepperY, extended_axis_codes[TMC_Y], values[Y_AXIS], mechanics.axis_steps_per_mm[Y_AXIS]);
        else tmc_get_pwmthrs(stepperY, extended_axis_codes[TMC_Y], mechanics.axis_steps_per_mm[Y_AXIS]);
      #endif
      #if Y2_IS_TRINAMIC
        if (values[Y_AXIS]) tmc_set_pwmthrs(stepperY2, extended_axis_codes[TMC_Y2], values[Y_AXIS], mechanics.axis_steps_per_mm[Y_AXIS]);
        else tmc_get_pwmthrs(stepperY, extended_axis_codes[TMC_Y2], mechanics.axis_steps_per_mm[Y_AXIS]);
      #endif

      #if Z_IS_TRINAMIC
        if (values[Z_AXIS]) tmc_set_pwmthrs(stepperZ, extended_axis_codes[TMC_Z], values[Z_AXIS], mechanics.axis_steps_per_mm[Z_AXIS]);
        else tmc_get_pwmthrs(stepperZ, extended_axis_codes[TMC_Z], mechanics.axis_steps_per_mm[Z_AXIS]);
      #endif
      #if Z2_IS_TRINAMIC
        if (values[Z_AXIS]) tmc_set_pwmthrs(stepperZ2, extended_axis_codes[TMC_Z2], values[Z_AXIS], mechanics.axis_steps_per_mm[Z_AXIS]);
        else tmc_get_pwmthrs(stepperZ, extended_axis_codes[TMC_Z2], mechanics.axis_steps_per_mm[Z_AXIS]);
      #endif

      #if E0_IS_TRINAMIC
        if (values[E_AXIS]) tmc_set_pwmthrs(stepperE0, extended_axis_codes[TMC_E0], values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS]);
        else tmc_get_pwmthrs(stepperE0, extended_axis_codes[TMC_E0], mechanics.axis_steps_per_mm[E_AXIS]);
      #endif
      #if E1_IS_TRINAMIC
        if (values[E_AXIS]) tmc_set_pwmthrs(stepperE1, extended_axis_codes[TMC_E1], values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS + 1]);
        else tmc_get_pwmthrs(stepperE1, extended_axis_codes[TMC_E1], mechanics.axis_steps_per_mm[E_AXIS + 1]);
      #endif
      #if E2_IS_TRINAMIC
        if (values[E_AXIS]) tmc_set_pwmthrs(stepperE2, extended_axis_codes[TMC_E2], values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS + 2]);
        else tmc_get_pwmthrs(stepperE2, extended_axis_codes[TMC_E2], mechanics.axis_steps_per_mm[E_AXIS + 2]);
      #endif
      #if E3_IS_TRINAMIC
        if (values[E_AXIS]) tmc_set_pwmthrs(stepperE3, extended_axis_codes[TMC_E3], values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS + 3]);
        else tmc_get_pwmthrs(stepperE3, extended_axis_codes[TMC_E3], mechanics.axis_steps_per_mm[E_AXIS + 3]);
      #endif
      #if E4_IS_TRINAMIC
        if (values[E_AXIS]) tmc_set_pwmthrs(stepperE4, extended_axis_codes[TMC_E4], values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS + 4]);
        else tmc_get_pwmthrs(stepperE4, extended_axis_codes[TMC_E4], mechanics.axis_steps_per_mm[E_AXIS + 4]);
      #endif
      #if E5_IS_TRINAMIC
        if (values[E_AXIS]) tmc_set_pwmthrs(stepperE5, extended_axis_codes[TMC_E5], values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS + 5]);
        else tmc_get_pwmthrs(stepperE4, extended_axis_codes[TMC_E4], mechanics.axis_steps_per_mm[E_AXIS + 5]);
      #endif
    }
  #endif // HYBRID_THRESHOLD

  /**
   * M914: Set SENSORLESS_HOMING sensitivity.
   */
  #if ENABLED(SENSORLESS_HOMING)
    #define CODE_M914
    inline void gcode_M914(void) {
      #if ENABLED(X_IS_TMC2130) || ENABLED(IS_TRAMS)
        if (parser.seen(axis_codes[X_AXIS])) tmc_set_sgt(stepperX, extended_axis_codes[TMC_X], parser.value_int());
        else tmc_get_sgt(stepperX, extended_axis_codes[TMC_X]);
      #endif
      #if ENABLED(X2_IS_TMC2130)
        if (parser.seen(axis_codes[X_AXIS])) tmc_set_sgt(stepperX2, extended_axis_codes[TMC_X2], parser.value_int());
        else tmc_get_sgt(stepperX2, extended_axis_codes[TMC_X2]);
      #endif
      #if ENABLED(Y_IS_TMC2130) || ENABLED(IS_TRAMS)
        if (parser.seen(axis_codes[Y_AXIS])) tmc_set_sgt(stepperY, extended_axis_codes[TMC_Y], parser.value_int());
        else tmc_get_sgt(stepperY, extended_axis_codes[TMC_Y]);
      #endif
      #if ENABLED(Y2_IS_TMC2130)
        if (parser.seen(axis_codes[Y_AXIS])) tmc_set_sgt(stepperY2, extended_axis_codes[TMC_Y2], parser.value_int());
        else tmc_get_sgt(stepperY2, extended_axis_codes[TMC_Y2]);
      #endif
    }
  #endif // SENSORLESS_HOMING

  /**
   * TMC Z axis calibration routine
   */
  #if ENABLED(TMC_Z_CALIBRATION) && (Z_IS_TRINAMIC || Z2_IS_TRINAMIC)
    #define CODE_M915
    inline void gcode_M915(void) {
      uint16_t _rms = parser.seenval('S') ? parser.value_int() : CALIBRATION_CURRENT;
      uint16_t _z = parser.seenval('Z') ? parser.value_int() : CALIBRATION_EXTRA_HEIGHT;

      if (!axis_known_position[Z_AXIS]) {
        SERIAL_EM("\nPlease home Z axis first");
        return;
      }

      uint16_t Z_current_1 = stepperZ.getCurrent();
      uint16_t Z2_current_1 = stepperZ.getCurrent();

      stepperZ.setCurrent(_rms, R_SENSE, HOLD_MULTIPLIER);
      stepperZ2.setCurrent(_rms, R_SENSE, HOLD_MULTIPLIER);
      SERIAL_MV("\nCalibration current: Z", _rms);

      printer.setSoftEndstop(false);

      mechanics.do_blocking_move_to_z(Z_MAX_POS + _z);

      stepperZ.setCurrent(Z_current_1, R_SENSE, HOLD_MULTIPLIER);
      stepperZ2.setCurrent(Z2_current_1, R_SENSE, HOLD_MULTIPLIER);

      mechanics.do_blocking_move_to_z(Z_MAX_POS);
      printer.setSoftEndstop(true);

      SERIAL_EM("\nHoming Z because we lost steps");
      mechanics.home_z_safely();
    }
  #endif

#endif // HAS_TRINAMIC
