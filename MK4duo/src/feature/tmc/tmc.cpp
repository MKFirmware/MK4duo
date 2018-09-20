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
 * tmc_util.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if HAS_TRINAMIC

bool report_tmc_status = false;

/*
 * Check for over temperature or short to ground error flags.
 * Report and log warning of overtemperature condition.
 * Reduce driver current in a persistent otpw condition.
 * Keep track of otpw counter so we don't reduce current on a single instance,
 * and so we don't repeatedly report warning before the condition is cleared.
 */
#if ENABLED(MONITOR_DRIVER_STATUS)
  struct TMC_driver_data {
    uint32_t drv_status;
    bool is_otpw;
    bool is_ot;
    bool is_error;
  };
  #if HAVE_DRV(TMC2130)
    static uint32_t get_pwm_scale(TMC2130Stepper &st) { return st.PWM_SCALE(); }
    static uint8_t get_status_response(TMC2130Stepper &st) { return st.status_response & 0xF; }
    static TMC_driver_data get_driver_data(TMC2130Stepper &st) {
      constexpr uint32_t OTPW_bm = 0x4000000UL;
      constexpr uint8_t OTPW_bp = 26;
      constexpr uint32_t OT_bm = 0x2000000UL;
      constexpr uint8_t OT_bp = 25;
      constexpr uint8_t DRIVER_ERROR_bm = 0x2UL;
      constexpr uint8_t DRIVER_ERROR_bp = 1;
      TMC_driver_data data;
      data.drv_status = st.DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_error = (st.status_response & DRIVER_ERROR_bm) >> DRIVER_ERROR_bp;
      return data;
    }
  #endif
  #if HAVE_DRV(TMC2208)
    static uint32_t get_pwm_scale(TMC2208Stepper &st) { return st.pwm_scale_sum(); }
    static uint8_t get_status_response(TMC2208Stepper &st) {
      uint32_t drv_status = st.DRV_STATUS();
      uint8_t gstat = st.GSTAT();
      uint8_t response = 0;
      response |= (drv_status >> (31-3)) & 0b1000;
      response |= gstat & 0b11;
      return response;
    }
    static TMC_driver_data get_driver_data(TMC2208Stepper &st) {
      constexpr uint32_t OTPW_bm = 0b1ul;
      constexpr uint8_t OTPW_bp = 0;
      constexpr uint32_t OT_bm = 0b10ul;
      constexpr uint8_t OT_bp = 1;
      TMC_driver_data data;
      data.drv_status = st.DRV_STATUS();
      data.is_otpw = (data.drv_status & OTPW_bm) >> OTPW_bp;
      data.is_ot = (data.drv_status & OT_bm) >> OT_bp;
      data.is_error = st.drv_err();
      return data;
    }
  #endif

  template<typename TMC>
  void monitor_tmc_driver(TMC &st, const TMC_AxisEnum axis, uint8_t &otpw_cnt) {
    TMC_driver_data data = get_driver_data(st);

    #if ENABLED(STOP_ON_ERROR)
      if (data.is_error) {
        SERIAL_EOL();
        _tmc_say_axis(axis);
        SERIAL_MSG(" driver error detected:");
        if (data.is_ot) SERIAL_EM("overtemperature");
        if (st.s2ga()) SERIAL_EM("short to ground (coil A)");
        if (st.s2gb()) SERIAL_EM("short to ground (coil B)");
        #if ENABLED(TMC_DEBUG)
          tmc_report_all();
        #endif
        printer.kill(PSTR("Driver error"));
      }
    #endif

    // Report if a warning was triggered
    if (data.is_otpw && otpw_cnt == 0) {
      char timestamp[10];
      duration_t elapsed = print_job_counter.duration();
      (void)elapsed.toDigital(timestamp, true);
      SERIAL_EOL();
      SERIAL_TXT(timestamp);
      SERIAL_MSG(": ");
      _tmc_say_axis(axis);
      SERIAL_MSG(" driver overtemperature warning! (");
      SERIAL_VAL(st.getCurrent());
      SERIAL_EM("mA)");
    }
    #if CURRENT_STEP_DOWN > 0
      // Decrease current if is_otpw is true and driver is enabled and there's been more than 4 warnings
      if (data.is_otpw && st.isEnabled() && otpw_cnt > 4) {
        st.setCurrent(st.getCurrent() - CURRENT_STEP_DOWN, R_SENSE, HOLD_MULTIPLIER);
        #if ENABLED(REPORT_CURRENT_CHANGE)
          _tmc_say_axis(axis);
          SERIAL_EMV(" current decreased to ", st.getCurrent());
        #endif
      }
    #endif

    if (data.is_otpw) {
      otpw_cnt++;
      st.flag_otpw = true;
    }
    else if (otpw_cnt > 0) otpw_cnt = 0;

    if (report_tmc_status) {
      const uint32_t pwm_scale = get_pwm_scale(st);
      _tmc_say_axis(axis);
      SERIAL_MV(":", pwm_scale);
      SERIAL_MV(" |0b", get_status_response(st));
      SERIAL_MSG("| ");
      if (data.is_error) SERIAL_CHR('E');
      else if (data.is_ot) SERIAL_CHR('O');
      else if (data.is_otpw) SERIAL_CHR('W');
      else if (otpw_cnt > 0) SERIAL_VAL(otpw_cnt);
      else if (st.flag_otpw) SERIAL_CHR('F');
      SERIAL_CHR('\t');
    }
  }

  #define HAS_HW_COMMS(ST)  ST##_HAS_DRV(TMC2130) || (ST##_HAS_DRV(TMC2208) && ENABLED(ST##_HARDWARE_SERIAL))

  void monitor_tmc_driver() {
    static millis_t next_cOT = 0;
    if (ELAPSED(millis(), next_cOT)) {
      next_cOT = millis() + 500;
      #if HAS_HW_COMMS(X)
        static uint8_t x_otpw_cnt = 0;
        monitor_tmc_driver(stepperX, TMC_X, x_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Y)
        static uint8_t y_otpw_cnt = 0;
        monitor_tmc_driver(stepperY, TMC_Y, y_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Z)
        static uint8_t z_otpw_cnt = 0;
        monitor_tmc_driver(stepperZ, TMC_Z, z_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(X2)
        static uint8_t x2_otpw_cnt = 0;
        monitor_tmc_driver(stepperX2, TMC_X, x2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Y2)
        static uint8_t y2_otpw_cnt = 0;
        monitor_tmc_driver(stepperY2, TMC_Y, y2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(Z2)
        static uint8_t z2_otpw_cnt = 0;
        monitor_tmc_driver(stepperZ2, TMC_Z, z2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E0)
        static uint8_t e0_otpw_cnt = 0;
        monitor_tmc_driver(stepperE0, TMC_E0, e0_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E1)
        static uint8_t e1_otpw_cnt = 0;
        monitor_tmc_driver(stepperE1, TMC_E1, e1_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E2)
        static uint8_t e2_otpw_cnt = 0;
        monitor_tmc_driver(stepperE2, TMC_E2, e2_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E3)
        static uint8_t e3_otpw_cnt = 0;
        monitor_tmc_driver(stepperE3, TMC_E3, e3_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E4)
        static uint8_t e4_otpw_cnt = 0;
        monitor_tmc_driver(stepperE4, TMC_E4, e4_otpw_cnt);
      #endif
      #if HAS_HW_COMMS(E5)
        static uint8_t e5_otpw_cnt = 0;
        monitor_tmc_driver(stepperE5, TMC_E5, e5_otpw_cnt);
      #endif

      if (report_tmc_status) SERIAL_EOL();
    }
  }

#endif // MONITOR_DRIVER_STATUS

void _tmc_say_axis(const TMC_AxisEnum axis) {
  const static char ext_X[]  PROGMEM = "X",  ext_X2[] PROGMEM = "X2",
                    ext_Y[]  PROGMEM = "Y",  ext_Y2[] PROGMEM = "Y2",
                    ext_Z[]  PROGMEM = "Z",  ext_Z2[] PROGMEM = "Z2",
                    ext_E0[] PROGMEM = "E0", ext_E1[] PROGMEM = "E1",
                    ext_E2[] PROGMEM = "E2", ext_E3[] PROGMEM = "E3",
                    ext_E4[] PROGMEM = "E4", ext_E5[] PROGMEM = "E5";
  const static char* const tmc_axes[] PROGMEM = { ext_X, ext_Y, ext_Z, ext_X2, ext_Y2, ext_Z2, ext_E0, ext_E1, ext_E2, ext_E3, ext_E4, ext_E5 };
  SERIAL_PS((char*)pgm_read_ptr(&tmc_axes[axis]));
}

void _tmc_say_current(const TMC_AxisEnum axis, const uint16_t curr) {
  _tmc_say_axis(axis);
  SERIAL_EMV(" driver current: ", curr);
}
void _tmc_say_otpw(const TMC_AxisEnum axis, const bool otpw) {
  _tmc_say_axis(axis);
  SERIAL_MSG(" temperature prewarn triggered: ");
  SERIAL_PS(otpw ? PSTR("true") : PSTR("false"));
  SERIAL_EOL();
}
void _tmc_say_otpw_cleared(const TMC_AxisEnum axis) {
  _tmc_say_axis(axis);
  SERIAL_EM(" prewarn flag cleared");
}
void _tmc_say_pwmthrs(const TMC_AxisEnum axis, const uint32_t thrs) {
  _tmc_say_axis(axis);
  SERIAL_EMV(" stealthChop max speed: ", thrs);
}
void _tmc_say_sgt(const TMC_AxisEnum axis, const int8_t sgt) {
  _tmc_say_axis(axis);
  SERIAL_MSG(" homing sensitivity: ");
  SERIAL_EV((int)sgt);
}

#if ENABLED(TMC_DEBUG)

  enum TMC_debug_enum : char {
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
  enum TMC_drv_status_enum : char {
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
  static void drv_status_print_hex(const TMC_AxisEnum axis, const uint32_t drv_status) {
    _tmc_say_axis(axis);
    SERIAL_MSG(" = 0x");
    for (int B = 24; B >= 8; B -= 8){
      SERIAL_VAL((drv_status >> (B + 4)) & 0xF);
      SERIAL_VAL((drv_status >> B) & 0xF);
      SERIAL_CHR(':');
    }
    SERIAL_VAL((drv_status >> 4) & 0xF);
    SERIAL_VAL(drv_status & 0xF);
    SERIAL_EOL();
  }

  #if HAVE_DRV(TMC2130)
    static void tmc_status(TMC2130Stepper &st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_PWM_SCALE: SERIAL_VAL(st.PWM_SCALE()); break;
        case TMC_TSTEP: SERIAL_TXT(st.TSTEP()); break;
        case TMC_SGT: SERIAL_VAL(st.sgt()); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st.stealthChop() ? PSTR("true") : PSTR("false")); break;
        default: break;
      }
    }
    static void tmc_parse_drv_status(TMC2130Stepper &st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_STALLGUARD: if (st.stallguard()) SERIAL_CHR('X'); break;
        case TMC_SG_RESULT:  SERIAL_VAL(st.sg_result());   break;
        case TMC_FSACTIVE:   if (st.fsactive())   SERIAL_CHR('X'); break;
        default: break;
      }
    }
  #endif

  #if HAVE_DRV(TMC2208)
    static void tmc_status(TMC2208Stepper &st, const TMC_debug_enum i) {
      switch(i) {
        case TMC_TSTEP: { uint32_t data = 0; st.TSTEP(&data); SERIAL_VAL(data); break; }
        case TMC_PWM_SCALE: SERIAL_VAL(st.pwm_scale_sum()); break;
        case TMC_STEALTHCHOP: SERIAL_PS(st.stealth() ? PSTR("true") : PSTR("false")); break;
        case TMC_S2VSA: if (st.s2vsa()) SERIAL_CHR('X'); break;
        case TMC_S2VSB: if (st.s2vsb()) SERIAL_CHR('X'); break;
        default: break;
      }
    }
    static void tmc_parse_drv_status(TMC2208Stepper &st, const TMC_drv_status_enum i) {
      switch(i) {
        case TMC_T157: if (st.t157()) SERIAL_CHR('X'); break;
        case TMC_T150: if (st.t150()) SERIAL_CHR('X'); break;
        case TMC_T143: if (st.t143()) SERIAL_CHR('X'); break;
        case TMC_T120: if (st.t120()) SERIAL_CHR('X'); break;
        default: break;
      }
    }
  #endif

  template <typename TMC>
  static void tmc_status(TMC &st, const TMC_AxisEnum axis, const TMC_debug_enum i, const float spmm) {
    SERIAL_CHR('\t');
    switch(i) {
      case TMC_CODES: _tmc_say_axis(axis); break;
      case TMC_ENABLED: SERIAL_PS(st.isEnabled() ? PSTR("true") : PSTR("false")); break;
      case TMC_CURRENT: SERIAL_VAL(st.getCurrent()); break;
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

      case TMC_MICROSTEPS: SERIAL_VAL(st.microsteps()); break;
      case TMC_TPWMTHRS: {
          uint32_t tpwmthrs_val = st.TPWMTHRS();
          SERIAL_VAL(tpwmthrs_val);
        }
        break;
      case TMC_TPWMTHRS_MMS: {
          uint32_t tpwmthrs_val = st.TPWMTHRS();
            if (tpwmthrs_val)
              SERIAL_VAL(12650000UL * st.microsteps() / (256 * tpwmthrs_val * spmm));
            else
              SERIAL_CHR('-');
          }
        break;
      case TMC_OTPW: SERIAL_PS(st.otpw() ? PSTR("true") : PSTR("false")); break;
      case TMC_OTPW_TRIGGERED: SERIAL_PS(st.getOTPW() ? PSTR("true") : PSTR("false")); break;
      case TMC_TOFF: SERIAL_VAL(st.toff()); break;
      case TMC_TBL: SERIAL_VAL(st.blank_time()); break;
      case TMC_HEND: SERIAL_VAL(st.hysteresis_end()); break;
      case TMC_HSTRT: SERIAL_VAL(st.hysteresis_start()); break;
      default: tmc_status(st, i); break;
    }
  }

  template <typename TMC>
  static void tmc_parse_drv_status(TMC &st, const TMC_AxisEnum axis, const TMC_drv_status_enum i) {
    SERIAL_CHR('\t');
    switch(i) {
      case TMC_DRV_CODES:     _tmc_say_axis(axis);                    break;
      case TMC_STST:          if (st.stst())         SERIAL_CHR('X'); break;
      case TMC_OLB:           if (st.olb())          SERIAL_CHR('X'); break;
      case TMC_OLA:           if (st.ola())          SERIAL_CHR('X'); break;
      case TMC_S2GB:          if (st.s2gb())         SERIAL_CHR('X'); break;
      case TMC_S2GA:          if (st.s2ga())         SERIAL_CHR('X'); break;
      case TMC_DRV_OTPW:      if (st.otpw())         SERIAL_CHR('X'); break;
      case TMC_OT:            if (st.ot())           SERIAL_CHR('X'); break;
      case TMC_DRV_CS_ACTUAL: SERIAL_VAL(st.cs_actual());             break;
      case TMC_DRV_STATUS_HEX:drv_status_print_hex(axis, st.DRV_STATUS()); break;
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

  /**
   * M922 report functions
   */
  void tmc_set_report_status(const bool status) {
    if ((report_tmc_status = status))
      SERIAL_EM("axis:pwm_scale |status_response|");
  }

  void tmc_report_all() {
    #define TMC_REPORT(LABEL, ITEM) do{ SERIAL_MSG(LABEL);  tmc_debug_loop(ITEM); }while(0)
    #define DRV_REPORT(LABEL, ITEM) do{ SERIAL_MSG(LABEL); drv_status_loop(ITEM); }while(0)
    TMC_REPORT("\t",                 TMC_CODES);
    TMC_REPORT("Enabled\t",          TMC_ENABLED);
    TMC_REPORT("Set current",        TMC_CURRENT);
    TMC_REPORT("RMS current",        TMC_RMS_CURRENT);
    TMC_REPORT("MAX current",        TMC_MAX_CURRENT);
    TMC_REPORT("Run current",        TMC_IRUN);
    TMC_REPORT("Hold current",       TMC_IHOLD);
    TMC_REPORT("CS actual\t",        TMC_CS_ACTUAL);
    TMC_REPORT("PWM scale\t",        TMC_PWM_SCALE);
    TMC_REPORT("vsense\t",           TMC_VSENSE);
    TMC_REPORT("stealthChop",        TMC_STEALTHCHOP);
    TMC_REPORT("msteps\t",           TMC_MICROSTEPS);
    TMC_REPORT("tstep\t",            TMC_TSTEP);
    TMC_REPORT("pwm\nthreshold\t",   TMC_TPWMTHRS);
    TMC_REPORT("[mm/s]\t",           TMC_TPWMTHRS_MMS);
    TMC_REPORT("OT prewarn",         TMC_OTPW);
    TMC_REPORT("OT prewarn has\n"
               "been triggered",     TMC_OTPW_TRIGGERED);
    TMC_REPORT("off time\t",         TMC_TOFF);
    TMC_REPORT("blank time",         TMC_TBL);
    TMC_REPORT("hysteresis\n-end\t", TMC_HEND);
    TMC_REPORT("-start\t",           TMC_HSTRT);
    TMC_REPORT("Stallguard thrs",    TMC_SGT);

    DRV_REPORT("DRVSTATUS",          TMC_DRV_CODES);
    #if HAVE_DRV(TMC2130)
      DRV_REPORT("stallguard\t",     TMC_STALLGUARD);
      DRV_REPORT("sg_result\t",      TMC_SG_RESULT);
      DRV_REPORT("fsactive\t",       TMC_FSACTIVE);
    #endif
    DRV_REPORT("stst\t",             TMC_STST);
    DRV_REPORT("olb\t",              TMC_OLB);
    DRV_REPORT("ola\t",              TMC_OLA);
    DRV_REPORT("s2gb\t",             TMC_S2GB);
    DRV_REPORT("s2ga\t",             TMC_S2GA);
    DRV_REPORT("otpw\t",             TMC_DRV_OTPW);
    DRV_REPORT("ot\t",               TMC_OT);
    #if HAVE_DRV(TMC2208)
      DRV_REPORT("157C\t",           TMC_T157);
      DRV_REPORT("150C\t",           TMC_T150);
      DRV_REPORT("143C\t",           TMC_T143);
      DRV_REPORT("120C\t",           TMC_T120);
      DRV_REPORT("s2vsa\t",          TMC_S2VSA);
      DRV_REPORT("s2vsb\t",          TMC_S2VSB);
    #endif
    DRV_REPORT("Driver registers:",  TMC_DRV_STATUS_HEX);
    SERIAL_EOL();
  }

#endif // TMC_DEBUG

#if ENABLED(SENSORLESS_HOMING)

  void tmc_sensorless_homing(TMC2130Stepper &st, const bool enable/*=true*/) {
    st.coolstep_min_speed(enable ? 1024UL * 1024UL - 1UL : 0);
    #if ENABLED(STEALTHCHOP)
      st.stealthChop(!enable);
    #endif
    st.diag1_stall(enable ? 1 : 0);
  }

#endif // SENSORLESS_HOMING

#endif // HAS_TRINAMIC