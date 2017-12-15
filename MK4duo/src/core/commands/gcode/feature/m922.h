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

#if ENABLED(TMC_DEBUG)

  #define CODE_M922
  
  enum TMC_debug_enum {
    TMC_CODES,
    TMC_ENABLED,
    TMC_CURRENT,
    TMC_RMS_CURRENT,
    TMC_MAX_CURRENT,
    TMC_IRUN,
    TMC_IHOLD,
    TMC_CS_ACTUAL,
    TMC_PWM_SCALE,
    TMC_VSENSE,
    TMC_STEALTHCHOP,
    TMC_MICROSTEPS,
    TMC_TSTEP,
    TMC_TPWMTHRS,
    TMC_TPWMTHRS_MMS,
    TMC_OTPW,
    TMC_OTPW_TRIGGERED,
    TMC_TOFF,
    TMC_TBL,
    TMC_HEND,
    TMC_HSTRT,
    TMC_SGT
  };
  enum TMC_drv_status_enum {
    TMC_DRV_CODES,
    TMC_STST,
    TMC_OLB,
    TMC_OLA,
    TMC_S2GB,
    TMC_S2GA,
    TMC_DRV_OTPW,
    TMC_OT,
    TMC_STALLGUARD,
    TMC_DRV_CS_ACTUAL,
    TMC_FSACTIVE,
    TMC_SG_RESULT,
    TMC_DRV_STATUS_HEX,
    TMC_T157,
    TMC_T150,
    TMC_T143,
    TMC_T120,
    TMC_STEALTH,
    TMC_S2VSB,
    TMC_S2VSA
  };

  static void drv_status_print_hex(const char name[], const uint32_t drv_status) {
    SERIAL_TXT(name);
    SERIAL_MSG(" = 0x");
    for (int B = 24; B >= 8; B -= 8) {
      SERIAL_VAL((drv_status >> (B + 4)) & 0xF);
      SERIAL_VAL((drv_status >> B) & 0xF);
      SERIAL_CHR(':');
    }
    SERIAL_VAL((drv_status >> 4) & 0xF);
    SERIAL_VAL((drv_status) & 0xF);
    SERIAL_EOL();
  }

  #if ENABLED(HAVE_TMC2130)
    static void tmc_status(TMC2130Stepper &st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_PWM_SCALE: SERIAL_VAL(st.PWM_SCALE(), DEC); break;
        case TMC_TSTEP: SERIAL_TXT(st.TSTEP()); break;
        case TMC_SGT: SERIAL_VAL(st.sgt(), DEC); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st.stealthChop() ? PSTR("true") : PSTR("false")); break;
        default: break;
      }
    }
    static void tmc_parse_drv_status(TMC2130Stepper &st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_STALLGUARD: if (st.stallguard()) SERIAL_MSG("X"); break;
        case TMC_SG_RESULT:  SERIAL_VAL(st.sg_result(), DEC);      break;
        case TMC_FSACTIVE:   if (st.fsactive())   SERIAL_MSG("X"); break;
        default: break;
      }
    }
  #endif
  #if ENABLED(HAVE_TMC2208)
    static void tmc_status(TMC2208Stepper &st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_TSTEP:
          {
            uint32_t data = 0;
            st.TSTEP(&data);
            SERIAL_VAL(data);
            break;
          }
        case TMC_PWM_SCALE: SERIAL_VAL(st.pwm_scale_sum()); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st.stealth() ? PSTR("true") : PSTR("false")); break;
        case TMC_S2VSA: if (st.s2vsa()) SERIAL_MSG("X"); break;
        case TMC_S2VSB: if (st.s2vsb()) SERIAL_MSG("X"); break;
        default: break;
      }
    }
    static void tmc_parse_drv_status(TMC2208Stepper &st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_T157: if (st.t157()) SERIAL_MSG("X"); break;
        case TMC_T150: if (st.t150()) SERIAL_MSG("X"); break;
        case TMC_T143: if (st.t143()) SERIAL_MSG("X"); break;
        case TMC_T120: if (st.t120()) SERIAL_MSG("X"); break;
        default: break;
      }
    }
  #endif
  template <typename TMC>
  static void tmc_status(TMC &st, TMC_AxisEnum axis, const TMC_debug_enum i, const float spmm) {
    SERIAL_CHR('\t');
    switch(i) {
      case TMC_CODES: SERIAL_TXT(extended_axis_codes[axis]); break;
      case TMC_ENABLED: SERIAL_PS(st.isEnabled() ? PSTR("true") : PSTR("false")); break;
      case TMC_CURRENT: SERIAL_TXT(st.getCurrent()); break;
      case TMC_RMS_CURRENT: SERIAL_VAL(st.rms_current()); break;
      case TMC_MAX_CURRENT: SERIAL_VAL((float)st.rms_current() * 1.41, 0); break;
      case TMC_IRUN:
        SERIAL_VAL(st.irun());
        SERIAL_MSG("/31");
        break;
      case TMC_IHOLD:
        SERIAL_VAL(st.ihold());
        SERIAL_MSG("/31");
        break;
      case TMC_CS_ACTUAL:
        SERIAL_VAL(st.cs_actual());
        SERIAL_MSG("/31");
        break;

      case TMC_VSENSE: SERIAL_PS(st.vsense() ? PSTR("1=.18") : PSTR("0=.325")); break;

      case TMC_MICROSTEPS: SERIAL_TXT(st.microsteps()); break;
      case TMC_TPWMTHRS:
        {
          uint32_t tpwmthrs_val = st.TPWMTHRS();
          SERIAL_TXT(tpwmthrs_val);
        }
        break;
      case TMC_TPWMTHRS_MMS:
        {
          uint32_t tpwmthrs_val = st.TPWMTHRS();
          tpwmthrs_val ? SERIAL_TXT(12650000UL * st.microsteps() / (256 * tpwmthrs_val * spmm)) : SERIAL_TXT('-');
        }
        break;
      case TMC_OTPW: SERIAL_PS(st.otpw() ? PSTR("true") : PSTR("false")); break;
      case TMC_OTPW_TRIGGERED: SERIAL_PS(st.getOTPW() ? PSTR("true") : PSTR("false")); break;
      case TMC_TOFF: SERIAL_VAL(st.toff()); break;
      case TMC_TBL: SERIAL_VAL(st.blank_time()); break;
      case TMC_HEND: SERIAL_VAL(st.hysterisis_end()); break;
      case TMC_HSTRT: SERIAL_VAL(st.hysterisis_start()); break;
      default: tmc_status(st, i); break;
    }
  }
  template <typename TMC>
  static void tmc_parse_drv_status(TMC &st, TMC_AxisEnum axis, const TMC_drv_status_enum i) {
    SERIAL_MSG("\t");
    switch(i) {
      case TMC_DRV_CODES:     SERIAL_TXT(extended_axis_codes[axis]);  break;
      case TMC_STST:          if (st.stst())         SERIAL_MSG("X"); break;
      case TMC_OLB:           if (st.olb())          SERIAL_MSG("X"); break;
      case TMC_OLA:           if (st.ola())          SERIAL_MSG("X"); break;
      case TMC_S2GB:          if (st.s2gb())         SERIAL_MSG("X"); break;
      case TMC_S2GA:          if (st.s2ga())         SERIAL_MSG("X"); break;
      case TMC_DRV_OTPW:      if (st.otpw())         SERIAL_MSG("X"); break;
      case TMC_OT:            if (st.ot())           SERIAL_MSG("X"); break;
      case TMC_DRV_CS_ACTUAL: SERIAL_VAL(st.cs_actual());             break;
      case TMC_DRV_STATUS_HEX:drv_status_print_hex(extended_axis_codes[axis], st.DRV_STATUS()); break;
      default: tmc_parse_drv_status(st, i); break;
    }
  }

  static void tmc_debug_loop(const TMC_debug_enum i) {
    #if X_IS_TRINAMIC
      tmc_status(stepperX, TMC_X, i, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif
    #if X2_IS_TRINAMIC
      tmc_status(stepperX2, TMC_X2, i, mechanics.axis_steps_per_mm[X_AXIS]);
    #endif

    #if Y_IS_TRINAMIC
      tmc_status(stepperY, TMC_Y, i, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif
    #if Y2_IS_TRINAMIC
      tmc_status(stepperY2, TMC_Y2, i, mechanics.axis_steps_per_mm[Y_AXIS]);
    #endif

    #if Z_IS_TRINAMIC
      tmc_status(stepperZ, TMC_Z, i, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif
    #if Z2_IS_TRINAMIC
      tmc_status(stepperZ2, TMC_Z2, i, mechanics.axis_steps_per_mm[Z_AXIS]);
    #endif

    #if E0_IS_TRINAMIC
      tmc_status(stepperE0, TMC_E0, i, mechanics.axis_steps_per_mm[E_AXIS]);
    #endif
    #if E1_IS_TRINAMIC
      tmc_status(stepperE1, TMC_E1, i, mechanics.axis_steps_per_mm[E_AXIS+1]);
    #endif
    #if E2_IS_TRINAMIC
      tmc_status(stepperE2, TMC_E2, i, mechanics.axis_steps_per_mm[E_AXIS+2]);
    #endif
    #if E3_IS_TRINAMIC
      tmc_status(stepperE3, TMC_E3, i, mechanics.axis_steps_per_mm[E_AXIS+3]);
    #endif
    #if E4_IS_TRINAMIC
      tmc_status(stepperE4, TMC_E4, i, mechanics.axis_steps_per_mm[E_AXIS+4]);
    #endif
    #if E5_IS_TRINAMIC
      tmc_status(stepperE5, TMC_E5, i, mechanics.axis_steps_per_mm[E_AXIS+5]);
    #endif

    SERIAL_EOL();
  }

  static void drv_status_loop(const TMC_drv_status_enum i) {
    #if X_IS_TRINAMIC
      tmc_parse_drv_status(stepperX, TMC_X, i);
    #endif
    #if X2_IS_TRINAMIC
      tmc_parse_drv_status(stepperX2, TMC_X2, i);
    #endif

    #if Y_IS_TRINAMIC
      tmc_parse_drv_status(stepperY, TMC_Y, i);
    #endif
    #if Y2_IS_TRINAMIC
      tmc_parse_drv_status(stepperY2, TMC_Y2, i);
    #endif

    #if Z_IS_TRINAMIC
      tmc_parse_drv_status(stepperZ, TMC_Z, i);
    #endif
    #if Z2_IS_TRINAMIC
      tmc_parse_drv_status(stepperZ2, TMC_Z2, i);
    #endif

    #if E0_IS_TRINAMIC
      tmc_parse_drv_status(stepperE0, TMC_E0, i);
    #endif
    #if E1_IS_TRINAMIC
      tmc_parse_drv_status(stepperE1, TMC_E1, i);
    #endif
    #if E2_IS_TRINAMIC
      tmc_parse_drv_status(stepperE2, TMC_E2, i);
    #endif
    #if E3_IS_TRINAMIC
      tmc_parse_drv_status(stepperE3, TMC_E3, i);
    #endif
    #if E4_IS_TRINAMIC
      tmc_parse_drv_status(stepperE4, TMC_E4, i);
    #endif
    #if E5_IS_TRINAMIC
      tmc_parse_drv_status(stepperE5, TMC_E5, i);
    #endif

    SERIAL_EOL();
  }

  inline void gcode_M922(void) {
    if (parser.seen('S')) {
      if (parser.value_bool()) {
        SERIAL_EM("axis:pwm_scale |status_response|");
        report_tmc_status = true;
      } else
        report_tmc_status = false;
    } else {
      SERIAL_MSG("\t");                 tmc_debug_loop(TMC_CODES);
      SERIAL_MSG("Enabled\t");          tmc_debug_loop(TMC_ENABLED);
      SERIAL_MSG("Set current");        tmc_debug_loop(TMC_CURRENT);
      SERIAL_MSG("RMS current");        tmc_debug_loop(TMC_RMS_CURRENT);
      SERIAL_MSG("MAX current");        tmc_debug_loop(TMC_MAX_CURRENT);
      SERIAL_MSG("Run current");        tmc_debug_loop(TMC_IRUN);
      SERIAL_MSG("Hold current");       tmc_debug_loop(TMC_IHOLD);
      SERIAL_MSG("CS actual\t");        tmc_debug_loop(TMC_CS_ACTUAL);
      SERIAL_MSG("PWM scale");          tmc_debug_loop(TMC_PWM_SCALE);
      SERIAL_MSG("vsense\t");           tmc_debug_loop(TMC_VSENSE);
      SERIAL_MSG("stealthChop");        tmc_debug_loop(TMC_STEALTHCHOP);
      SERIAL_MSG("msteps\t");           tmc_debug_loop(TMC_MICROSTEPS);
      SERIAL_MSG("tstep\t");            tmc_debug_loop(TMC_TSTEP);
      SERIAL_MSG("pwm\nthreshold\t");   tmc_debug_loop(TMC_TPWMTHRS);
      SERIAL_MSG("[mm/s]\t");           tmc_debug_loop(TMC_TPWMTHRS_MMS);
      SERIAL_MSG("OT prewarn");         tmc_debug_loop(TMC_OTPW);
      SERIAL_MSG("OT prewarn has\nbeen triggered"); tmc_debug_loop(TMC_OTPW_TRIGGERED);
      SERIAL_MSG("off time\t");         tmc_debug_loop(TMC_TOFF);
      SERIAL_MSG("blank time");         tmc_debug_loop(TMC_TBL);
      SERIAL_MSG("hysterisis\n-end\t"); tmc_debug_loop(TMC_HEND);
      SERIAL_MSG("-start\t");           tmc_debug_loop(TMC_HSTRT);
      SERIAL_MSG("Stallguard thrs");    tmc_debug_loop(TMC_SGT);

      SERIAL_MSG("DRVSTATUS");          drv_status_loop(TMC_DRV_CODES);
      #if ENABLED(HAVE_TMC2130)
        SERIAL_MSG("stallguard\t");     drv_status_loop(TMC_STALLGUARD);
        SERIAL_MSG("sg_result\t");      drv_status_loop(TMC_SG_RESULT);
        SERIAL_MSG("fsactive\t");       drv_status_loop(TMC_FSACTIVE);
      #endif
      SERIAL_MSG("stst\t");             drv_status_loop(TMC_STST);
      SERIAL_MSG("olb\t");              drv_status_loop(TMC_OLB);
      SERIAL_MSG("ola\t");              drv_status_loop(TMC_OLA);
      SERIAL_MSG("s2gb\t");             drv_status_loop(TMC_S2GB);
      SERIAL_MSG("s2ga\t");             drv_status_loop(TMC_S2GA);
      SERIAL_MSG("otpw\t");             drv_status_loop(TMC_DRV_OTPW);
      SERIAL_MSG("ot\t");               drv_status_loop(TMC_OT);
      #if ENABLED(HAVE_TMC2208)
        SERIAL_MSG("157C\t");           drv_status_loop(TMC_T157);
        SERIAL_MSG("150C\t");           drv_status_loop(TMC_T150);
        SERIAL_MSG("143C\t");           drv_status_loop(TMC_T143);
        SERIAL_MSG("120C\t");           drv_status_loop(TMC_T120);
        SERIAL_MSG("s2vsa\t");          drv_status_loop(TMC_S2VSA);
        SERIAL_MSG("s2vsb\t");          drv_status_loop(TMC_S2VSB);
      #endif
      SERIAL_EM("Driver registers:");   drv_status_loop(TMC_DRV_STATUS_HEX);
    }
  }

#endif // TMC_DEBUG
