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
    #if X_IS_TRINAMIC
      tmc_report_otpw(stepperX, TMC_X);
    #endif
    #if X2_IS_TRINAMIC
      tmc_report_otpw(stepperX2, TMC_X2);
    #endif
    #if Y_IS_TRINAMIC
      tmc_report_otpw(stepperY, TMC_Y);
    #endif
    #if Y2_IS_TRINAMIC
      tmc_report_otpw(stepperY2, TMC_Y2);
    #endif
    #if Z_IS_TRINAMIC
      tmc_report_otpw(stepperZ, TMC_Z);
    #endif
    #if Z2_IS_TRINAMIC
      tmc_report_otpw(stepperZ2, TMC_Z2);
    #endif
    #if E0_IS_TRINAMIC
      tmc_report_otpw(stepperE0, TMC_E0);
    #endif
    #if E1_IS_TRINAMIC
      tmc_report_otpw(stepperE1, TMC_E1);
    #endif
    #if E2_IS_TRINAMIC
      tmc_report_otpw(stepperE2, TMC_E2);
    #endif
    #if E3_IS_TRINAMIC
      tmc_report_otpw(stepperE3, TMC_E3);
    #endif
    #if E4_IS_TRINAMIC
      tmc_report_otpw(stepperE4, TMC_E4);
    #endif
    #if E5_IS_TRINAMIC
      tmc_report_otpw(stepperE5, TMC_E5);
    #endif
  }

  #define CODE_M912

  /**
   * M912: Clear TMC stepper driver overtemperature pre-warn flag held by the library
   *       Specify one or more axes with X, Y, Z, X1, Y1, Z1, X2, Y2, Z2, and E[index].
   *       If no axes are given, clear all.
   *
   * Examples:
   *       M912 X   ; clear X and X2
   *       M912 X1  ; clear X1 only
   *       M912 X2  ; clear X2 only
   *       M912 X E ; clear X, X2, and all E
   *       M912 E1  ; clear E1 only
   */
  inline void gcode_M912(void) {
    const bool  hasX = parser.seen(axis_codes[X_AXIS]),
                hasY = parser.seen(axis_codes[Y_AXIS]),
                hasZ = parser.seen(axis_codes[Z_AXIS]),
                hasE = parser.seen(axis_codes[E_AXIS]),
                hasNone = !hasX && !hasY && !hasZ && !hasE;

    #if X_IS_TRINAMIC || X2_IS_TRINAMIC
      const uint8_t xval = parser.byteval(axis_codes[X_AXIS], 10);
      #if X_IS_TRINAMIC
        if (hasNone || xval == 1 || (hasX && xval == 10)) tmc_clear_otpw(stepperX, TMC_X);
      #endif
      #if X2_IS_TRINAMIC
        if (hasNone || xval == 2 || (hasX && xval == 10)) tmc_clear_otpw(stepperX2, TMC_X2);
      #endif
    #endif

    #if Y_IS_TRINAMIC || Y2_IS_TRINAMIC
      const uint8_t yval = parser.byteval(axis_codes[Y_AXIS], 10);
      #if Y_IS_TRINAMIC
        if (hasNone || yval == 1 || (hasY && yval == 10)) tmc_clear_otpw(stepperY, TMC_Y);
      #endif
      #if Y2_IS_TRINAMIC
        if (hasNone || yval == 2 || (hasY && yval == 10)) tmc_clear_otpw(stepperY2, TMC_Y2);
      #endif
    #endif

    #if Z_IS_TRINAMIC || Z2_IS_TRINAMIC
      const uint8_t zval = parser.byteval(axis_codes[Z_AXIS], 10);
      #if Z_IS_TRINAMIC
        if (hasNone || zval == 1 || (hasZ && zval == 10)) tmc_clear_otpw(stepperZ, TMC_Z);
      #endif
      #if Z2_IS_TRINAMIC
        if (hasNone || zval == 2 || (hasZ && zval == 10)) tmc_clear_otpw(stepperZ2, TMC_Z2);
      #endif
    #endif

    const uint8_t eval = parser.byteval(axis_codes[E_AXIS], 10);

    #if E0_IS_TRINAMIC
      if (hasNone || eval == 0 || (hasE && eval == 10)) tmc_clear_otpw(stepperE0, TMC_E0);
    #endif
    #if E1_IS_TRINAMIC
      if (hasNone || eval == 1 || (hasE && eval == 10)) tmc_clear_otpw(stepperE1, TMC_E1);
    #endif
    #if E2_IS_TRINAMIC
      if (hasNone || eval == 2 || (hasE && eval == 10)) tmc_clear_otpw(stepperE2, TMC_E2);
    #endif
    #if E3_IS_TRINAMIC
      if (hasNone || eval == 3 || (hasE && eval == 10)) tmc_clear_otpw(stepperE3, TMC_E3);
    #endif
    #if E4_IS_TRINAMIC
      if (hasNone || eval == 4 || (hasE && eval == 10)) tmc_clear_otpw(stepperE4, TMC_E4);
    #endif
    #if E5_IS_TRINAMIC
      if (hasNone || eval == 5 || (hasE && eval == 10)) tmc_clear_otpw(stepperE5, TMC_E5);
    #endif

  }

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  #if ENABLED(HYBRID_THRESHOLD)

    #define CODE_M913

    inline void gcode_M913(void) {

      if (commands.get_target_tool(913)) return;

      #define TMC_SAY_PWMTHRS(P,Q) tmc_get_pwmthrs(stepper##Q, TMC_##Q, mechanics.axis_steps_per_mm[P##_AXIS])
      #define TMC_SET_PWMTHRS(P,Q) tmc_set_pwmthrs(stepper##Q, value, mechanics.axis_steps_per_mm[P##_AXIS])
      #define TMC_SAY_PWMTHRS_E(E) do{ const uint8_t extruder = E; tmc_get_pwmthrs(stepperE##E, TMC_E##E, mechanics.axis_steps_per_mm[E_AXIS_N]); }while(0)
      #define TMC_SET_PWMTHRS_E(E) do{ const uint8_t extruder = E; tmc_set_pwmthrs(stepperE##E, value, mechanics.axis_steps_per_mm[E_AXIS_N]); }while(0)

      const uint8_t index = parser.byteval('I');
      LOOP_XYZE(i) {
        if (int32_t value = parser.longval(axis_codes[i])) {
          switch (i) {
            case X_AXIS:
              #if X_IS_TRINAMIC
                if (index < 2) TMC_SET_PWMTHRS(X,X);
              #endif
              #if X2_IS_TRINAMIC
                if (!(index & 1)) TMC_SET_PWMTHRS(X,X2);
              #endif
              break;
            case Y_AXIS:
              #if Y_IS_TRINAMIC
                if (index < 2) TMC_SET_PWMTHRS(Y,Y);
              #endif
              #if Y2_IS_TRINAMIC
                if (!(index & 1)) TMC_SET_PWMTHRS(Y,Y2);
              #endif
              break;
            case Z_AXIS:
              #if Z_IS_TRINAMIC
                if (index < 2) TMC_SET_PWMTHRS(Z,Z);
              #endif
              #if Z2_IS_TRINAMIC
                if (!(index & 1)) TMC_SET_PWMTHRS(Z,Z2);
              #endif
              break;
            case E_AXIS: {
              switch (TARGET_EXTRUDER) {
                #if E0_IS_TRINAMIC
                  case 0: TMC_SET_PWMTHRS_E(0); break;
                #endif
                #if DRIVER_EXTRUDERS > 1 && E1_IS_TRINAMIC
                  case 1: TMC_SET_PWMTHRS_E(1); break;
                #endif
                #if DRIVER_EXTRUDERS > 2 && E2_IS_TRINAMIC
                  case 2: TMC_SET_PWMTHRS_E(2); break;
                #endif
                #if DRIVER_EXTRUDERS > 3 && E3_IS_TRINAMIC
                  case 3: TMC_SET_PWMTHRS_E(3); break;
                #endif
                #if DRIVER_EXTRUDERS > 4 && E4_IS_TRINAMIC
                  case 4: TMC_SET_PWMTHRS_E(4); break;
                #endif
                #if DRIVER_EXTRUDERS > 5 && E5_IS_TRINAMIC
                  case 5: TMC_SET_PWMTHRS_E(5); break;
                #endif
              }
            } break;
          }
        }
      }

      #if X_IS_TRINAMIC
        TMC_SAY_PWMTHRS(X,X);
      #endif
      #if X2_IS_TRINAMIC
        TMC_SAY_PWMTHRS(X,X2);
      #endif
      #if Y_IS_TRINAMIC
        TMC_SAY_PWMTHRS(Y,Y);
      #endif
      #if Y2_IS_TRINAMIC
        TMC_SAY_PWMTHRS(Y,Y2);
      #endif
      #if Z_IS_TRINAMIC
        TMC_SAY_PWMTHRS(Z,Z);
      #endif
      #if Z2_IS_TRINAMIC
        TMC_SAY_PWMTHRS(Z,Z2);
      #endif
      #if E0_IS_TRINAMIC
        TMC_SAY_PWMTHRS_E(0);
      #endif
      #if DRIVER_EXTRUDERS > 1 && E1_IS_TRINAMIC
        TMC_SAY_PWMTHRS_E(1);
      #endif
      #if DRIVER_EXTRUDERS > 2 && E2_IS_TRINAMIC
        TMC_SAY_PWMTHRS_E(2);
      #endif
      #if DRIVER_EXTRUDERS > 3 && E3_IS_TRINAMIC
        TMC_SAY_PWMTHRS_E(3);
      #endif
      #if DRIVER_EXTRUDERS > 4 && E4_IS_TRINAMIC
        TMC_SAY_PWMTHRS_E(4);
      #endif
      #if DRIVER_EXTRUDERS > 5 && E5_IS_TRINAMIC
        TMC_SAY_PWMTHRS_E(5);
      #endif

    }

  #endif // HYBRID_THRESHOLD

  /**
   * M914: Set SENSORLESS_HOMING sensitivity.
   */
  #if ENABLED(SENSORLESS_HOMING)

    #define CODE_M914

    inline void gcode_M914(void) {

      #define TMC_SAY_SGT(Q) tmc_get_sgt(stepper##Q, TMC_##Q)
      #define TMC_SET_SGT(Q) tmc_set_sgt(stepper##Q, value)

      const uint8_t index = parser.byteval('I');
      LOOP_XYZ(i) {
        if (parser.seen(axis_codes[i])) {
          const int8_t value = (int8_t)constrain(parser.value_int(), -64, 63);
          switch (i) {
            #if X_SENSORLESS
              case X_AXIS:
                #if X_HAS_STALLGUARD
                  if (index < 2) TMC_SET_SGT(X);
                #endif
                #if X2_HAS_STALLGUARD
                  if (!(index & 1)) TMC_SET_SGT(X2);
                #endif
                break;
            #endif
            #if Y_SENSORLESS
              case Y_AXIS:
                #if Y_HAS_STALLGUARD
                  if (index < 2) TMC_SET_SGT(Y);
                #endif
                #if Y2_HAS_STALLGUARD
                  if (!(index & 1)) TMC_SET_SGT(Y2);
                #endif
                break;
            #endif
            #if Z_SENSORLESS
              case Z_AXIS:
                #if Z_HAS_STALLGUARD
                  if (index < 2) TMC_SET_SGT(Z);
                #endif
                #if Z2_HAS_STALLGUARD
                  if (!(index & 1)) TMC_SET_SGT(Z2);
                #endif
                break;
            #endif
          }
        }
      }

      #if X_SENSORLESS
        #if X_HAS_STALLGUARD
          TMC_SAY_SGT(X);
        #endif
        #if X2_HAS_STALLGUARD
          TMC_SAY_SGT(X2);
        #endif
      #endif
      #if Y_SENSORLESS
        #if Y_HAS_STALLGUARD
          TMC_SAY_SGT(Y);
        #endif
        #if Y2_HAS_STALLGUARD
          TMC_SAY_SGT(Y2);
        #endif
      #endif
      #if Z_SENSORLESS
        #if Z_HAS_STALLGUARD
          TMC_SAY_SGT(Z);
        #endif
        #if Z2_HAS_STALLGUARD
          TMC_SAY_SGT(Z2);
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

      const uint16_t  _rms  = parser.seenval('S') ? parser.value_int() : CALIBRATION_CURRENT,
                      _z    = parser.seenval('Z') ? parser.value_linear_units() : CALIBRATION_EXTRA_HEIGHT;

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
