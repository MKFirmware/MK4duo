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

#if HAS_TRINAMIC

  #if ENABLED(MONITOR_DRIVER_STATUS)

    #define CODE_M911

    /**
     * M911:  Report TMC stepper driver overtemperature pre-warn flag
     *        The flag is held by the library and persist until manually cleared by M912
     */
    inline void gcode_M911(void) {
      #if AXIS_HAS_TMC(X)
        tmc.report_otpw(stepperX);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.report_otpw(stepperX2);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.report_otpw(stepperY);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.report_otpw(stepperY2);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.report_otpw(stepperZ);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.report_otpw(stepperZ2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmc.report_otpw(stepperZ3);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.report_otpw(stepperE0);
      #endif
      #if AXIS_HAS_TMC(E1)
        tmc.report_otpw(stepperE1);
      #endif
      #if AXIS_HAS_TMC(E2)
        tmc.report_otpw(stepperE2);
      #endif
      #if AXIS_HAS_TMC(E3)
        tmc.report_otpw(stepperE3);
      #endif
      #if AXIS_HAS_TMC(E4)
        tmc.report_otpw(stepperE4);
      #endif
      #if AXIS_HAS_TMC(E5)
        tmc.report_otpw(stepperE5);
      #endif
    }

    #define CODE_M912

    /**
     * M912: Clear TMC stepper driver overtemperature pre-warn flag held by the library
     *       Specify one or more axes with X, Y, Z, X1, Y1, Z1, X2, Y2, Z2, Z3, and E[index].
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

      #if AXIS_HAS_TMC(X) || AXIS_HAS_TMC(X2)
        const int8_t xval = int8_t(parser.byteval(axis_codes[X_AXIS], 0xFF));
        #if AXIS_HAS_TMC(X)
          if (hasNone || xval == 1 || (hasX && xval < 0)) tmc.clear_otpw(stepperX);
        #endif
        #if AXIS_HAS_TMC(X2)
          if (hasNone || xval == 2 || (hasX && xval < 0)) tmc.clear_otpw(stepperX2);
        #endif
      #endif

      #if AXIS_HAS_TMC(Y) || AXIS_HAS_TMC(Y2)
        const int8_t yval = int8_t(parser.byteval(axis_codes[X_AXIS], 0xFF));
        #if AXIS_HAS_TMC(Y)
          if (hasNone || yval == 1 || (hasY && yval < 0)) tmc.clear_otpw(stepperY);
        #endif
        #if AXIS_HAS_TMC(Y2)
          if (hasNone || yval == 2 || (hasY && yval < 0)) tmc.clear_otpw(stepperY2);
        #endif
      #endif

      #if AXIS_HAS_TMC(Z) || AXIS_HAS_TMC(Z2) || AXIS_HAS_TMC(Z3)
        const int8_t zval = int8_t(parser.byteval(axis_codes[Z_AXIS], 0xFF));
        #if AXIS_HAS_TMC(Z)
          if (hasNone || zval == 1 || (hasZ && zval < 0)) tmc.clear_otpw(stepperZ);
        #endif
        #if AXIS_HAS_TMC(Z2)
          if (hasNone || zval == 2 || (hasZ && zval < 0)) tmc.clear_otpw(stepperZ2);
        #endif
        #if AXIS_HAS_TMC(Z3)
          if (hasNone || zval == 3 || (hasZ && zval < 0)) tmc.clear_otpw(stepperZ3);
        #endif
      #endif

      const uint8_t eval = int8_t(parser.byteval(axis_codes[E_AXIS], 0xFF));

      #if AXIS_HAS_TMC(E0)
        if (hasNone || eval == 0 || (hasE && eval < 0)) tmc.clear_otpw(stepperE0);
      #endif
      #if AXIS_HAS_TMC(E1)
        if (hasNone || eval == 1 || (hasE && eval < 0)) tmc.clear_otpw(stepperE1);
      #endif
      #if AXIS_HAS_TMC(E2)
        if (hasNone || eval == 2 || (hasE && eval < 0)) tmc.clear_otpw(stepperE2);
      #endif
      #if AXIS_HAS_TMC(E3)
        if (hasNone || eval == 3 || (hasE && eval < 0)) tmc.clear_otpw(stepperE3);
      #endif
      #if AXIS_HAS_TMC(E4)
        if (hasNone || eval == 4 || (hasE && eval < 0)) tmc.clear_otpw(stepperE4);
      #endif
      #if AXIS_HAS_TMC(E5)
        if (hasNone || eval == 5 || (hasE && eval < 0)) tmc.clear_otpw(stepperE5);
      #endif

    }

  #endif // ENABLED(MONITOR_DRIVER_STATUS)

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  #if ENABLED(HYBRID_THRESHOLD)

    #define CODE_M913

    inline void gcode_M913(void) {

      if (commands.get_target_tool(913)) return;

      #if DISABLED(DISABLE_M503)
        // No arguments? Show M913 report.
        if (!parser.seen("XYZE")) {
          tmc.print_M913();
          return;
        }
      #endif

      #define TMC_SET_PWMTHRS(P,ST) tmc.set_pwmthrs(stepper##ST, value, mechanics.data.axis_steps_per_mm[P##_AXIS])
      #define TMC_SET_PWMTHRS_E(E) do{ tmc.set_pwmthrs(stepperE##E, value, mechanics.data.axis_steps_per_mm[E_AXIS_N(E)]); }while(0)

      LOOP_XYZE(i) {
        if (int32_t value = parser.longval(axis_codes[i])) {
          switch (i) {
            case X_AXIS:
              #if AXIS_HAS_STEALTHCHOP(X)
                TMC_SET_PWMTHRS(X,X);
              #endif
              #if AXIS_HAS_STEALTHCHOP(X2)
                TMC_SET_PWMTHRS(X,X2);
              #endif
              break;
            case Y_AXIS:
              #if AXIS_HAS_STEALTHCHOP(Y)
                TMC_SET_PWMTHRS(Y,Y);
              #endif
              #if AXIS_HAS_STEALTHCHOP(Y2)
                TMC_SET_PWMTHRS(Y,Y2);
              #endif
              break;
            case Z_AXIS:
              #if AXIS_HAS_STEALTHCHOP(Z)
                TMC_SET_PWMTHRS(Z,Z);
              #endif
              #if AXIS_HAS_STEALTHCHOP(Z2)
                TMC_SET_PWMTHRS(Z,Z2);
              #endif
              #if AXIS_HAS_STEALTHCHOP(Z3)
                TMC_SET_PWMTHRS(Z,Z3);
              #endif
              break;
            case E_AXIS: {
              switch (TARGET_EXTRUDER) {
                #if AXIS_HAS_STEALTHCHOP(E0)
                  case 0: TMC_SET_PWMTHRS_E(0); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E1)
                  case 1: TMC_SET_PWMTHRS_E(1); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E2)
                  case 2: TMC_SET_PWMTHRS_E(2); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E3)
                  case 3: TMC_SET_PWMTHRS_E(3); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E4)
                  case 4: TMC_SET_PWMTHRS_E(4); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E5)
                  case 5: TMC_SET_PWMTHRS_E(5); break;
                #endif
              }
            } break;
          }
        }
      }
    }

  #endif // HYBRID_THRESHOLD

  /**
   * M914: Set StallGuard sensitivity.
   */
  #if HAS_SENSORLESS

    #define CODE_M914

    inline void gcode_M914(void) {

      #if DISABLED(DISABLE_M503)
        // No arguments? Show M914 report.
        if (!parser.seen("XYZ")) {
          tmc.print_M914();
          return;
        }
      #endif

      #define TMC_SET_SGT(ST) tmc.set_sgt(stepper##ST, value)

      LOOP_XYZ(i) {
        if (parser.seen(axis_codes[i])) {
          const int8_t value = (int8_t)constrain(parser.value_int(), -64, 63);
          switch (i) {
            #if X_HAS_SENSORLESS
              case X_AXIS:
                #if AXIS_HAS_STALLGUARD(X)
                  TMC_SET_SGT(X);
                #endif
                #if AXIS_HAS_STALLGUARD(X2)
                  TMC_SET_SGT(X2);
                #endif
                break;
            #endif
            #if Y_HAS_SENSORLESS
              case Y_AXIS:
                #if AXIS_HAS_STALLGUARD(Y)
                  TMC_SET_SGT(Y);
                #endif
                #if AXIS_HAS_STALLGUARD(Y2)
                  TMC_SET_SGT(Y2);
                #endif
                break;
            #endif
            #if Z_HAS_SENSORLESS
              case Z_AXIS:
                #if AXIS_HAS_STALLGUARD(Z)
                  TMC_SET_SGT(Z);
                #endif
                #if AXIS_HAS_STALLGUARD(Z2)
                  TMC_SET_SGT(Z2);
                #endif
                #if AXIS_HAS_STALLGUARD(Z3)
                  TMC_SET_SGT(Z3);
                #endif
                break;
            #endif
          }
        }
      }
    }

  #endif // HAS_SENSORLESS

  /**
   * TMC Z axis calibration routine
   */
  #if ENABLED(TMC_Z_CALIBRATION)

    #define CODE_M915

    inline void gcode_M915(void) {

      const uint16_t  _rms  = parser.seenval('S') ? parser.value_int() : CALIBRATION_CURRENT,
                      _z    = parser.seenval('Z') ? parser.value_linear_units() : CALIBRATION_EXTRA_HEIGHT;

      if (!mechanics.home_flag.ZHomed) {
        SERIAL_EM("\nPlease home Z axis first");
        return;
      }

      #if AXIS_HAS_TMC(Z)
        const uint16_t Z_current_1 = stepperZ->rms_current();
        stepperZ->rms_current(_rms);
      #endif
      #if AXIS_HAS_TMC(Z2)
        const uint16_t Z2_current_1 = stepperZ2->rms_current();
        stepperZ2->rms_current(_rms);
      #endif
      #if AXIS_HAS_TMC(Z3)
        const uint16_t Z3_current_1 = stepperZ3->rms_current();
        stepperZ3->rms_current(_rms);
      #endif

      SERIAL_MV("\nCalibration current: Z", _rms);

      endstops.setSoftEndstop(false);

      mechanics.do_blocking_move_to_z(Z_MAX_BED + _z);

      #if AXIS_HAS_TMC(Z)
        stepperZ->rms_current(Z_current_1);
      #endif
      #if AXIS_HAS_TMC(Z2)
        stepperZ->rms_current(Z2_current_1);
      #endif
      #if AXIS_HAS_TMC(Z3)
        stepperZ->rms_current(Z3_current_1);
      #endif

      mechanics.do_blocking_move_to_z(Z_MAX_BED);
      endstops.setSoftEndstop(true);

      SERIAL_EM("\nHoming Z because we lost steps");
      commands.enqueue_and_echo_P(PSTR("G28 Z"));
    }

  #endif

#endif // HAS_TRINAMIC
