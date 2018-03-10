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
      tmc_report_otpw(stepperX, TMC_X);
    #endif
    #if ENABLED(Y_IS_TMC2130) || (ENABLED(Y_IS_TMC2208) && PIN_EXISTS(Y_SERIAL_RX)) || ENABLED(IS_TRAMS)
      tmc_report_otpw(stepperY, TMC_Y);
    #endif
    #if ENABLED(Z_IS_TMC2130) || (ENABLED(Z_IS_TMC2208) && PIN_EXISTS(Z_SERIAL_RX)) || ENABLED(IS_TRAMS)
      tmc_report_otpw(stepperZ, TMC_Z);
    #endif
    #if ENABLED(E0_IS_TMC2130) || (ENABLED(E0_IS_TMC2208) && PIN_EXISTS(E0_SERIAL_RX)) || ENABLED(IS_TRAMS)
      tmc_report_otpw(stepperE0, TMC_E0);
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
      if (clearX || clearAll) tmc_clear_otpw(stepperX, TMC_X);
    #endif
    #if ENABLED(X2_IS_TMC2130) || (ENABLED(X2_IS_TMC2208) && PIN_EXISTS(X2_SERIAL_RX))
      if (clearX || clearAll) tmc_clear_otpw(stepperX2, TMC_X2);
    #endif

    #if ENABLED(Y_IS_TMC2130) || (ENABLED(Y_IS_TMC2208) && PIN_EXISTS(Y_SERIAL_RX))
      if (clearY || clearAll) tmc_clear_otpw(stepperY, TMC_Y);
    #endif
    #if ENABLED(Y2_IS_TMC2130) || (ENABLED(Y2_IS_TMC2208) && PIN_EXISTS(Y2_SERIAL_RX))
      if (clearY || clearAll) tmc_clear_otpw(stepperY2, TMC_Y2);
    #endif

    #if ENABLED(Z_IS_TMC2130) || (ENABLED(Z_IS_TMC2208) && PIN_EXISTS(Z_SERIAL_RX))
      if (clearZ || clearAll) tmc_clear_otpw(stepperZ, TMC_Z);
    #endif
    #if ENABLED(Z2_IS_TMC2130) || (ENABLED(Z2_IS_TMC2208) && PIN_EXISTS(Z2_SERIAL_RX))
      if (clearZ || clearAll) tmc_clear_otpw(stepperZ2, extended_axis_codes[TMC_Z2]);
    #endif

    #if ENABLED(E0_IS_TMC2130) || (ENABLED(E0_IS_TMC2208) && PIN_EXISTS(E0_SERIAL_RX))
      if (clearE || clearAll) tmc_clear_otpw(stepperE0, TMC_E0);
    #endif
    #if ENABLED(E1_IS_TMC2130) || (ENABLED(E1_IS_TMC2208) && PIN_EXISTS(E1_SERIAL_RX))
      if (clearE || clearAll) tmc_clear_otpw(stepperE1, TMC_E1);
    #endif
    #if ENABLED(E2_IS_TMC2130) || (ENABLED(E2_IS_TMC2208) && PIN_EXISTS(E2_SERIAL_RX))
      if (clearE || clearAll) tmc_clear_otpw(stepperE2, TMC_E2);
    #endif
    #if ENABLED(E3_IS_TMC2130) || (ENABLED(E3_IS_TMC2208) && PIN_EXISTS(E3_SERIAL_RX))
      if (clearE || clearAll) tmc_clear_otpw(stepperE3, TMC_E3);
    #endif
    #if ENABLED(E4_IS_TMC2130) || (ENABLED(E4_IS_TMC2208) && PIN_EXISTS(E4_SERIAL_RX))
      if (clearE || clearAll) tmc_clear_otpw(stepperE4, TMC_E4);
    #endif
    #if ENABLED(E5_IS_TMC2130) || (ENABLED(E5_IS_TMC2208) && PIN_EXISTS(E5_SERIAL_RX))
      if (clearE || clearAll) tmc_clear_otpw(stepperE5, TMC_E5);
    #endif
  }

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  #if ENABLED(HYBRID_THRESHOLD)

    #define CODE_M913

    inline void gcode_M913(void) {

      uint16_t values[XYZE];
      LOOP_XYZE(i) values[i] = parser.intval(axis_codes[i]);

      #define TMC_SET_GET_PWMTHRS(P,Q) do { \
        if (values[P##_AXIS]) tmc_set_pwmthrs(stepper##Q, TMC_##Q, values[P##_AXIS], planner.axis_steps_per_mm[P##_AXIS]); \
        else tmc_get_pwmthrs(stepper##Q, TMC_##Q, planner.axis_steps_per_mm[P##_AXIS]); } while(0)

      #if X_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(X,X);
      #endif
      #if X2_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(X,X2);
      #endif
      #if Y_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(Y,Y);
      #endif
      #if Y2_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(Y,Y2);
      #endif
      #if Z_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(Z,Z);
      #endif
      #if Z2_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(Z,Z2);
      #endif
      #if E0_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(E,E0);
      #endif
      #if E1_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(E,E1);
      #endif
      #if E2_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(E,E2);
      #endif
      #if E3_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(E,E3);
      #endif
      #if E4_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(E,E4);
      #endif
      #if E5_IS_TRINAMIC
        TMC_SET_GET_PWMTHRS(E,E5);
      #endif

    }

  #endif // HYBRID_THRESHOLD

  /**
   * M914: Set SENSORLESS_HOMING sensitivity.
   */
  #if ENABLED(SENSORLESS_HOMING)

    #define CODE_M914

    inline void gcode_M914(void) {

      #define TMC_SET_GET_SGT(P,Q) do { \
        if (parser.seen(axis_codes[P##_AXIS])) tmc_set_sgt(stepper##Q, TMC_##Q, parser.value_int()); \
        else tmc_get_sgt(stepper##Q, TMC_##Q); } while(0)

      #if ENABLED(X_HOMING_SENSITIVITY)
        #if ENABLED(X_IS_TMC2130) || ENABLED(IS_TRAMS)
          TMC_SET_GET_SGT(X,X);
        #endif
        #if ENABLED(X2_IS_TMC2130)
          TMC_SET_GET_SGT(X,X2);
        #endif
      #endif

      #if ENABLED(Y_HOMING_SENSITIVITY)
        #if ENABLED(Y_IS_TMC2130) || ENABLED(IS_TRAMS)
          TMC_SET_GET_SGT(Y,Y);
        #endif
        #if ENABLED(Y2_IS_TMC2130)
          TMC_SET_GET_SGT(Y,Y2);
        #endif
      #endif

      #if ENABLED(Z_HOMING_SENSITIVITY)
        #if ENABLED(Z_IS_TMC2130) || ENABLED(IS_TRAMS)
          TMC_SET_GET_SGT(Z,Z);
        #endif
        #if ENABLED(Z2_IS_TMC2130)
          TMC_SET_GET_SGT(Z,Z2);
        #endif
      #endif

    }

  #endif // SENSORLESS_HOMING

  /**
   * TMC Z axis calibration routine
   */
  #if ENABLED(TMC_Z_CALIBRATION)

    #define CODE_M915

    inline void gcode_M915(void) {

      uint16_t _rms = parser.seenval('S') ? parser.value_int() : CALIBRATION_CURRENT;
      uint16_t _z = parser.seenval('Z') ? parser.value_int() : CALIBRATION_EXTRA_HEIGHT;

      if (!printer.isZHomed()) {
        SERIAL_EM("\nPlease home Z axis first");
        return;
      }

      #if Z_IS_TRINAMIC
        uint16_t Z_current_1 = stepperZ.getCurrent();
        stepperZ.setCurrent(_rms, R_SENSE, HOLD_MULTIPLIER);
      #endif
      #if Z2_IS_TRINAMIC
        uint16_t Z2_current_1 = stepperZ2.getCurrent();
        stepperZ2.setCurrent(_rms, R_SENSE, HOLD_MULTIPLIER);
      #endif

      SERIAL_MV("\nCalibration current: Z", _rms);

      endstops.setSoftEndstop(false);

      mechanics.do_blocking_move_to_z(Z_MAX_POS + _z);

      #if Z_IS_TRINAMIC
        stepperZ.setCurrent(Z_current_1, R_SENSE, HOLD_MULTIPLIER);
      #endif
      #if Z2_IS_TRINAMIC
        stepperZ2.setCurrent(Z2_current_1, R_SENSE, HOLD_MULTIPLIER);
      #endif

      mechanics.do_blocking_move_to_z(Z_MAX_POS);
      endstops.setSoftEndstop(true);

      SERIAL_EM("\nHoming Z because we lost steps");
      commands.enqueue_and_echo_P(PSTR("G28 Z"));
    }

  #endif

#endif // HAS_TRINAMIC
