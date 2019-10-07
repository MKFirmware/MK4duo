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

#if HAS_TRINAMIC

  #if ENABLED(MONITOR_DRIVER_STATUS)

    #define CODE_M911

    /**
     * M911:  Report TMC stepper driver overtemperature pre-warn flag
     *        The flag is held by the library and persist until manually cleared by M912
     */
    inline void gcode_M911() {
      #if AXIS_HAS_TMC(X)
        tmc.report_otpw(driver.x);
      #endif
      #if AXIS_HAS_TMC(X2)
        tmc.report_otpw(driver.x2);
      #endif
      #if AXIS_HAS_TMC(Y)
        tmc.report_otpw(driver.y);
      #endif
      #if AXIS_HAS_TMC(Y2)
        tmc.report_otpw(driver.y2);
      #endif
      #if AXIS_HAS_TMC(Z)
        tmc.report_otpw(driver.z);
      #endif
      #if AXIS_HAS_TMC(Z2)
        tmc.report_otpw(driver.z2);
      #endif
      #if AXIS_HAS_TMC(Z3)
        tmc.report_otpw(driver.z3);
      #endif
      #if AXIS_HAS_TMC(E0)
        tmc.report_otpw(driver.e[0]);
      #endif
      #if AXIS_HAS_TMC(E1)
        tmc.report_otpw(driver.e[1]);
      #endif
      #if AXIS_HAS_TMC(E2)
        tmc.report_otpw(driver.e[2]);
      #endif
      #if AXIS_HAS_TMC(E3)
        tmc.report_otpw(driver.e[3]);
      #endif
      #if AXIS_HAS_TMC(E4)
        tmc.report_otpw(driver.e[4]);
      #endif
      #if AXIS_HAS_TMC(E5)
        tmc.report_otpw(driver.e[5]);
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
    inline void gcode_M912() {
      const bool  hasX = parser.seen(axis_codes.x),
                  hasY = parser.seen(axis_codes.y),
                  hasZ = parser.seen(axis_codes.z),
                  hasE = parser.seen(axis_codes.e),
                  hasNone = !hasX && !hasY && !hasZ && !hasE;

      #if AXIS_HAS_TMC(X) || AXIS_HAS_TMC(X2)
        const int8_t xval = int8_t(parser.byteval(axis_codes.x, 0xFF));
        #if AXIS_HAS_TMC(X)
          if (hasNone || xval == 1 || (hasX && xval < 0)) tmc.clear_otpw(driver.x);
        #endif
        #if AXIS_HAS_TMC(X2)
          if (hasNone || xval == 2 || (hasX && xval < 0)) tmc.clear_otpw(driver.x2);
        #endif
      #endif

      #if AXIS_HAS_TMC(Y) || AXIS_HAS_TMC(Y2)
        const int8_t yval = int8_t(parser.byteval(axis_codes.x, 0xFF));
        #if AXIS_HAS_TMC(Y)
          if (hasNone || yval == 1 || (hasY && yval < 0)) tmc.clear_otpw(driver.y);
        #endif
        #if AXIS_HAS_TMC(Y2)
          if (hasNone || yval == 2 || (hasY && yval < 0)) tmc.clear_otpw(driver.y2);
        #endif
      #endif

      #if AXIS_HAS_TMC(Z) || AXIS_HAS_TMC(Z2) || AXIS_HAS_TMC(Z3)
        const int8_t zval = int8_t(parser.byteval(axis_codes.z, 0xFF));
        #if AXIS_HAS_TMC(Z)
          if (hasNone || zval == 1 || (hasZ && zval < 0)) tmc.clear_otpw(driver.z);
        #endif
        #if AXIS_HAS_TMC(Z2)
          if (hasNone || zval == 2 || (hasZ && zval < 0)) tmc.clear_otpw(driver.z2);
        #endif
        #if AXIS_HAS_TMC(Z3)
          if (hasNone || zval == 3 || (hasZ && zval < 0)) tmc.clear_otpw(driver.z3);
        #endif
      #endif

      const uint8_t eval = int8_t(parser.byteval(axis_codes.e, 0xFF));

      #if AXIS_HAS_TMC(E0)
        if (hasNone || eval == 0 || (hasE && eval < 0)) tmc.clear_otpw(driver.e[0]);
      #endif
      #if AXIS_HAS_TMC(E1)
        if (hasNone || eval == 1 || (hasE && eval < 0)) tmc.clear_otpw(driver.e[1]);
      #endif
      #if AXIS_HAS_TMC(E2)
        if (hasNone || eval == 2 || (hasE && eval < 0)) tmc.clear_otpw(driver.e[2]);
      #endif
      #if AXIS_HAS_TMC(E3)
        if (hasNone || eval == 3 || (hasE && eval < 0)) tmc.clear_otpw(driver.e[3]);
      #endif
      #if AXIS_HAS_TMC(E4)
        if (hasNone || eval == 4 || (hasE && eval < 0)) tmc.clear_otpw(driver.e[4]);
      #endif
      #if AXIS_HAS_TMC(E5)
        if (hasNone || eval == 5 || (hasE && eval < 0)) tmc.clear_otpw(driver.e[5]);
      #endif

    }

  #endif // ENABLED(MONITOR_DRIVER_STATUS)

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  #if ENABLED(HYBRID_THRESHOLD)

    #define CODE_M913

    inline void gcode_M913() {

      if (commands.get_target_tool(913)) return;

      #if DISABLED(DISABLE_M503)
        // No arguments? Show M913 report.
        if (!parser.seen("XYZE")) {
          tmc.print_M913();
          return;
        }
      #endif

      #define TMC_SET_PWMTHRS(ST)   driver[ST##_DRV]->tmc->set_pwm_thrs(value)
      #define TMC_SET_PWMTHRS_E(ST) driver.e[ST##_DRV]->tmc->set_pwm_thrs(value)

      LOOP_XYZE(i) {
        if (int32_t value = parser.longval(axis_codes[i])) {
          switch (i) {
            case X_AXIS:
              #if AXIS_HAS_STEALTHCHOP(X)
                TMC_SET_PWMTHRS(X);
              #endif
              #if AXIS_HAS_STEALTHCHOP(X2)
                TMC_SET_PWMTHRS(X2);
              #endif
              break;
            case Y_AXIS:
              #if AXIS_HAS_STEALTHCHOP(Y)
                TMC_SET_PWMTHRS(Y);
              #endif
              #if AXIS_HAS_STEALTHCHOP(Y2)
                TMC_SET_PWMTHRS(Y2);
              #endif
              break;
            case Z_AXIS:
              #if AXIS_HAS_STEALTHCHOP(Z)
                TMC_SET_PWMTHRS(Z);
              #endif
              #if AXIS_HAS_STEALTHCHOP(Z2)
                TMC_SET_PWMTHRS(Z2);
              #endif
              #if AXIS_HAS_STEALTHCHOP(Z3)
                TMC_SET_PWMTHRS(Z3);
              #endif
              break;
            case E_AXIS: {
              switch (tools.data.extruder.target) {
                #if AXIS_HAS_STEALTHCHOP(E0)
                  case 0: TMC_SET_PWMTHRS_E(E0); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E1)
                  case 1: TMC_SET_PWMTHRS_E(E1); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E2)
                  case 2: TMC_SET_PWMTHRS_E(E2); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E3)
                  case 3: TMC_SET_PWMTHRS_E(E3); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E4)
                  case 4: TMC_SET_PWMTHRS_E(E4); break;
                #endif
                #if AXIS_HAS_STEALTHCHOP(E5)
                  case 5: TMC_SET_PWMTHRS_E(E5); break;
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

    inline void gcode_M914() {

      #if DISABLED(DISABLE_M503)
        // No arguments? Show M914 report.
        if (!parser.seen("XYZ")) {
          tmc.print_M914();
          return;
        }
      #endif

      #define TMC_SET_SGT(ST) driver[ST##_DRV]->tmc->homing_threshold(value)

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

    inline void gcode_M915() {

      const uint16_t  _rms  = parser.seenval('S') ? parser.value_int() : CALIBRATION_CURRENT,
                      _z    = parser.seenval('Z') ? parser.value_linear_units() : CALIBRATION_EXTRA_HEIGHT;

      if (mechanics.axis_unhomed_error(HOME_Z)) {
        #if MECH(DELTA)
          mechanics.home();
        #else
          mechanics.home(HOME_Z);
        #endif
      }

      #if AXIS_HAS_TMC(Z)
        const uint16_t Z_current_1 = driver[driver.z]->tmc->rms_current();
        driver[driver.z]->tmc->rms_current(_rms);
      #endif
      #if AXIS_HAS_TMC(Z2)
        const uint16_t Z2_current_1 = driver.z2->tmc->rms_current();
        driver.z2->tmc->rms_current(_rms);
      #endif
      #if AXIS_HAS_TMC(Z3)
        const uint16_t Z3_current_1 = driver.z3->tmc->rms_current();
        driver.z3->tmc->rms_current(_rms);
      #endif

      SERIAL_MV("\nCalibration current: Z", _rms);

      endstops.setSoftEndstop(false);

      mechanics.do_blocking_move_to_z(Z_MAX_BED + _z);

      #if AXIS_HAS_TMC(Z)
        driver[driver.z]->tmc->rms_current(Z_current_1);
      #endif
      #if AXIS_HAS_TMC(Z2)
        driver[driver.z]->tmc->rms_current(Z2_current_1);
      #endif
      #if AXIS_HAS_TMC(Z3)
        driver[driver.z]->tmc->rms_current(Z3_current_1);
      #endif

      mechanics.do_blocking_move_to_z(Z_MAX_BED);
      endstops.setSoftEndstop(true);

      SERIAL_EM("\nHoming Z because we lost steps");
      #if MECH(DELTA)
        mechanics.home();
      #else
        mechanics.home(HOME_Z);
      #endif
    }

  #endif

#endif // HAS_TRINAMIC
