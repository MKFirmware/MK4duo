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
 * tmc.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _TMC_H_
#define _TMC_H_

#if HAS_TRINAMIC

#include <TMCStepper.h>

#if TMCSTEPPER_VERSION < 0x000202
  #error "Update TMCStepper library to 0.2.2 or newer."
#endif

#define TMC_X_LABEL "X", 0
#define TMC_Y_LABEL "Y", 1
#define TMC_Z_LABEL "Z", 2

#define TMC_X2_LABEL "X2", 3
#define TMC_Y2_LABEL "Y2", 4
#define TMC_Z2_LABEL "Z2", 5
#define TMC_Z3_LABEL "Z3", 6

#define TMC_E0_LABEL "E0", 10
#define TMC_E1_LABEL "E1", 11
#define TMC_E2_LABEL "E2", 12
#define TMC_E3_LABEL "E3", 13
#define TMC_E4_LABEL "E4", 14
#define TMC_E5_LABEL "E5", 15

#define TMC_AXIS 13

extern bool report_tmc_status;

#if HAVE_DRV(TMC2660)

  //
  // TMC2660 Driver Class
  //
  class MKTMC : public TMC2660Stepper {

    public: /** Constructor */

      MKTMC(char* AXIS_LETTER, uint8_t DRIVER_ID, uint16_t cs_pin, float RS) :
        TMC2660Stepper(cs_pin, RS),
        axis_letter(AXIS_LETTER),
        id(DRIVER_ID)
        {}

      MKTMC(char* AXIS_LETTER, uint8_t DRIVER_ID, uint16_t CS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) :
        TMC2660Stepper(CS, RS, pinMOSI, pinMISO, pinSCK),
        axis_letter(AXIS_LETTER),
        id(DRIVER_ID)
        {}

    protected: /** Protected Parameters */

      uint16_t val_mA = 0;

    public: /** Public Parameters */

      const char* axis_letter;
      const uint8_t id;

      #if ENABLED(MONITOR_DRIVER_STATUS)
        uint8_t otpw_count = 0;
        bool flag_otpw = false;
      #endif

    public: /** Public Function */

      uint16_t getMilliamps() { return val_mA; }

      uint16_t rms_current() { return TMC2660Stepper::rms_current(); }

      void printLabel() { SERIAL_TXT(axis_letter); }

      void rms_current(uint16_t mA) {
        val_mA = mA;
        TMC2660Stepper::rms_current(mA);
      }

      #if ENABLED(MONITOR_DRIVER_STATUS)
        bool getOTPW() { return flag_otpw; }
        void clear_otpw() { flag_otpw = 0; }
      #endif

  };

#elif HAVE_DRV(TMC2130)

  //
  // TMC2130 Driver Class
  //
  class MKTMC : public TMC2130Stepper {

    public: /** Constructor */

      MKTMC(char* AXIS_LETTER, uint8_t DRIVER_ID, uint16_t cs_pin, float RS) :
        TMC2130Stepper(cs_pin, RS),
        axis_letter(AXIS_LETTER),
        id(DRIVER_ID)
        {}

      MKTMC(char* AXIS_LETTER, uint8_t DRIVER_ID, uint16_t CS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) :
        TMC2130Stepper(CS, RS, pinMOSI, pinMISO, pinSCK),
        axis_letter(AXIS_LETTER),
        id(DRIVER_ID)
        {}

    protected: /** Protected Parameters */

      uint16_t val_mA = 0;

    public: /** Public Parameters */

      const char* axis_letter;
      const uint8_t id;

      #if ENABLED(MONITOR_DRIVER_STATUS)
        uint8_t otpw_count = 0;
        bool flag_otpw = false;
      #endif

    public: /** Public Function */

      uint16_t getMilliamps() { return val_mA; }

      uint16_t rms_current() { return TMC2130Stepper::rms_current(); }

      void printLabel() { SERIAL_TXT(axis_letter); }

      void rms_current(uint16_t mA) {
        val_mA = mA;
        TMC2130Stepper::rms_current(mA);
      }

      void rms_current(uint16_t mA, float mult) {
        val_mA = mA;
        TMC2130Stepper::rms_current(mA, mult);
      }

      #if ENABLED(MONITOR_DRIVER_STATUS)
        bool getOTPW() { return flag_otpw; }
        void clear_otpw() { flag_otpw = 0; }
      #endif

  };

#elif HAVE_DRV(TMC2208)

  //
  // TMC2208 Driver Class
  //
  class MKTMC : public TMC2208Stepper {

    public: /** Constructor */

      MKTMC(char* AXIS_LETTER, uint8_t DRIVER_ID, Stream * SerialPort, float RS, bool has_rx=true) :
        TMC2208Stepper(SerialPort, RS, has_rx=true),
        axis_letter(AXIS_LETTER),
        id(DRIVER_ID)
        {}

      MKTMC(char* AXIS_LETTER, uint8_t DRIVER_ID, uint16_t RX, uint16_t TX, float RS, bool has_rx=true) :
        TMC2208Stepper(RX, TX, RS, has_rx=true),
        axis_letter(AXIS_LETTER),
        id(DRIVER_ID)
        {}

    protected: /** Protected Parameters */

      uint16_t val_mA = 0;

    public: /** Public Parameters */

      const char* axis_letter;
      const uint8_t id;

      #if ENABLED(MONITOR_DRIVER_STATUS)
        uint8_t otpw_count = 0;
        bool flag_otpw = false;
      #endif

    public: /** Public Function */

      uint16_t getMilliamps() { return val_mA; }

      uint16_t rms_current() { return TMC2208Stepper::rms_current(); }

      void printLabel() { SERIAL_MSG(axis_letter); }

      void rms_current(uint16_t mA) {
        this->val_mA = mA;
        TMC2208Stepper::rms_current(mA);
      }

      void rms_current(uint16_t mA, float mult) {
        this->val_mA = mA;
        TMC2208Stepper::rms_current(mA, mult);
      }

      #if ENABLED(MONITOR_DRIVER_STATUS)
        bool getOTPW() { return flag_otpw; }
        void clear_otpw() { flag_otpw = 0; }
      #endif

  };

#endif

#if AXIS_HAS_TMC(X)
  extern MKTMC* stepperX;
#endif
#if AXIS_HAS_TMC(X2)
  extern MKTMC* stepperX2;
#endif
#if AXIS_HAS_TMC(Y)
  extern MKTMC* stepperY;
#endif
#if AXIS_HAS_TMC(Y2)
  extern MKTMC* stepperY2;
#endif
#if AXIS_HAS_TMC(Z)
  extern MKTMC* stepperZ;
#endif
#if AXIS_HAS_TMC(Z2)
  extern MKTMC* stepperZ2;
#endif
#if AXIS_HAS_TMC(Z3)
  extern MKTMC* stepperZ3;
#endif
#if AXIS_HAS_TMC(E0)
  extern MKTMC* stepperE0;
#endif
#if AXIS_HAS_TMC(E1)
  extern MKTMC* stepperE1;
#endif
#if AXIS_HAS_TMC(E2)
  extern MKTMC* stepperE2;
#endif
#if AXIS_HAS_TMC(E3)
  extern MKTMC* stepperE3;
#endif
#if AXIS_HAS_TMC(E4)
  extern MKTMC* stepperE4;
#endif
#if AXIS_HAS_TMC(E5)
  extern MKTMC* stepperE5;
#endif
    
struct TMC_driver_data {
  uint32_t drv_status;
  bool is_otpw;
  bool is_ot;
  bool is_error;
};

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
    TMC_SGT,
    TMC_NULL
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

#endif // ENABLED(TMC_DEBUG)

class TMC_Stepper {

  public: /** Constructor */

    TMC_Stepper() {}

  public: /** Public Parameters */

  private: /** Private Parameters */

    static bool report_status;

  public: /** Public Function */

    static void init();
    static void current_init_to_defaults();
    static void microstep_init_to_defaults();

    static void restore();

    #if ENABLED(MONITOR_DRIVER_STATUS)
      static void monitor_driver();
    #endif

    #if ENABLED(TMC_DEBUG)
      static void set_report_status(const bool status);
      static void report_all();
    #endif

    MKTMC* driver_by_index(const uint8_t index);

    FORCE_INLINE static uint32_t thrs(const uint16_t tmc_msteps, const int32_t tmc_thrs, const uint32_t tmc_spmm) {
      return 12650000UL * tmc_msteps / (256 * tmc_thrs * tmc_spmm);
    }
    
    FORCE_INLINE static void get_current(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" driver current: ", st->getMilliamps());
    }
    
    FORCE_INLINE static void set_current(MKTMC* st, const uint16_t mA) {
      st->rms_current(mA);
    }

    FORCE_INLINE static void get_microstep(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" driver microstep: ", st->microsteps());
    }

    FORCE_INLINE static void set_microstep(MKTMC* st, const uint16_t ms) {
      st->microsteps(ms);
    }

    #if ENABLED(MONITOR_DRIVER_STATUS)

      FORCE_INLINE static void report_otpw(MKTMC* st) {
        st->printLabel();
        SERIAL_MSG(" temperature prewarn triggered: ");
        SERIAL_PS(st->getOTPW() ? PSTR("true") : PSTR("false"));
        SERIAL_EOL();
      }

      FORCE_INLINE static void clear_otpw(MKTMC* st) {
        st->clear_otpw();
        st->printLabel();
        SERIAL_EM(" prewarn flag cleared");
      }

    #endif

    FORCE_INLINE static void get_pwmthrs(MKTMC* st, const uint32_t tmc_spmm) {
      st->printLabel();
      SERIAL_EMV(" stealthChop max speed: ", thrs(st->microsteps(), st->TPWMTHRS(), tmc_spmm));
    }

    FORCE_INLINE static void set_pwmthrs(MKTMC* st, const int32_t tmc_thrs, const uint32_t tmc_spmm) {
      st->TPWMTHRS(thrs(st->microsteps(), tmc_thrs, tmc_spmm));
    }

    FORCE_INLINE static void get_sgt(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" homing sensitivity: ", st->sgt(), DEC);
    }

    FORCE_INLINE static void set_sgt(MKTMC* st, const int8_t sgt_val) {
      st->sgt(sgt_val);
    }

    FORCE_INLINE static void get_off_time(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" off_time: ", st->toff());
    }

    FORCE_INLINE static void set_off_time(MKTMC* st, const uint8_t off_time_val) {
      st->toff(off_time_val);
    }

    FORCE_INLINE static void get_blank_time(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" blank_time: ", st->blank_time());
    }

    FORCE_INLINE static void set_blank_time(MKTMC* st, const uint8_t blank_time_val) {
      st->blank_time(blank_time_val);
    }

    FORCE_INLINE static void get_hysteresis_end(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" hysteresis_end: ", st->hysteresis_end());
    }

    FORCE_INLINE static void set_hysteresis_end(MKTMC* st, const int8_t hysteresis_end_val) {
      st->hysteresis_end(hysteresis_end_val);
    }

    FORCE_INLINE static void get_hysteresis_start(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" hysteresis_start: ", st->hysteresis_start());
    }

    FORCE_INLINE static void set_hysteresis_start(MKTMC* st, const uint8_t hysteresis_start_val) {
      st->hysteresis_start(hysteresis_start_val);
    }

    FORCE_INLINE static void get_disable_I_comparator(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" disable_I_comparator: ", st->disfdcc());
    }

    FORCE_INLINE static void set_disable_I_comparator(MKTMC* st, const bool onoff) {
      st->disfdcc(onoff);
    }

    FORCE_INLINE static void get_stealth_gradient(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" stealth_gradient: ", st->pwm_grad());
    }

    FORCE_INLINE static void set_stealth_gradient(MKTMC* st, const uint8_t stealth_gradient_val) {
      st->pwm_grad(stealth_gradient_val);
    }

    FORCE_INLINE static void get_stealth_amplitude(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" stealth_amplitude: ", st->pwm_ampl());
    }

    FORCE_INLINE static void set_stealth_amplitude(MKTMC* st, const uint8_t stealth_amplitude_val) {
      st->pwm_ampl(stealth_amplitude_val);
    }

    FORCE_INLINE static void get_stealth_freq(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" stealth_freq: ", st->pwm_freq());
    }

    FORCE_INLINE static void set_stealth_freq(MKTMC* st, const uint8_t stealth_freq_val) {
      st->pwm_freq(stealth_freq_val);
    }

    FORCE_INLINE static void get_stealth_autoscale(MKTMC* st) {
      st->printLabel();
      SERIAL_EMV(" stealth_autoscale: ", st->pwm_autoscale());
    }

    FORCE_INLINE static void set_stealth_autoscale(MKTMC* st, const bool onoff) {
      st->pwm_autoscale(onoff);
    }

  private: /** Private Function */

    #if HAVE_DRV(TMC2660)
      static void config(MKTMC* st, const int8_t tmc_sgt=0);
    #elif HAVE_DRV(TMC2130)
      static void config(MKTMC* st, const bool tmc_stealthchop=false, const int8_t tmc_sgt=0);
    #elif HAVE_DRV(TMC2208)
      static void config(MKTMC* st, const bool tmc_stealthchop=false);
    #endif

    #if ENABLED(MONITOR_DRIVER_STATUS)

      #if HAVE_DRV(TMC2660)
        #if ENABLED(TMC_DEBUG)
          FORCE_INLINE static uint32_t get_pwm_scale(MKTMC* st) { return 0; }
          FORCE_INLINE static uint8_t get_status_response(MKTMC* st) { return 0; }
        #endif
      #elif HAVE_DRV(TMC2130)
        #if ENABLED(TMC_DEBUG)
          FORCE_INLINE static uint32_t get_pwm_scale(MKTMC* st) { return st->PWM_SCALE(); }
          static uint8_t get_status_response(MKTMC* st);
        #endif
      #elif HAVE_DRV(TMC2208)
        #if ENABLED(TMC_DEBUG)
          FORCE_INLINE static uint32_t get_pwm_scale(MKTMC* st) { return st->pwm_scale_sum(); }
          static uint8_t get_status_response(MKTMC* st);
        #endif
      #endif

      static TMC_driver_data get_driver_data(MKTMC* st);
      static void monitor_driver(MKTMC* st);

    #endif

    #if ENABLED(TMC_DEBUG)

      FORCE_INLINE static void print_vsense(MKTMC* st) { SERIAL_PS(st->vsense() ? PSTR("1=.18") : PSTR("0=.325")); }

      static void drv_status_print_hex(const uint32_t drv_status);
      static void status(MKTMC* st, const TMC_debug_enum i);
      static void status(MKTMC* st, const TMC_debug_enum i, const float tmc_spmm);
      static void parse_type_drv_status(MKTMC* st, const TMC_drv_status_enum i);
      static void parse_drv_status(MKTMC* st, const TMC_drv_status_enum i);
      static void debug_loop(const TMC_debug_enum i);
      static void status_loop(const TMC_drv_status_enum i);

    #endif

};

extern TMC_Stepper tmc;

#endif // HAS_TRINAMIC

#endif /* _TMC_H_ */
