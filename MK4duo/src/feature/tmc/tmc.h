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

#if HAVE_DRV(TMC2130)

  #include <TMC2130Stepper.h>

  #if TMC2130STEPPER_VERSION < 0x020201
    #error "DEPENDENCY ERROR: Update TMC2130Stepper library to 2.2.1 or newer."
  #endif

  #define TMC_TYPE TMC2130Stepper

#elif HAVE_DRV(TMC2208)

  #include <TMC2208Stepper.h>
  #if ENABLED(__AVR__)
    #include <SoftwareSerial.h>
    #include <HardwareSerial.h>
  #endif

  #if TMC2208STEPPER_VERSION < 0x000101
    #error "DEPENDENCY ERROR: Update TMC2208Stepper library to 0.1.1 or newer."
  #endif

  #define TMC_TYPE TMC2208Stepper

#endif

#define TMC_AXES (13)

extern bool report_tmc_status;

enum TMC_AxisEnum : char {
  TMC_X, TMC_Y, TMC_Z
  #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER_DRIVERS)
    , TMC_X2
  #endif
  #if ENABLED(Y_TWO_STEPPER_DRIVERS)
    , TMC_Y2
  #endif
  #if ENABLED(Z_THREE_STEPPER_DRIVERS)
    , TMC_Z2, TMC_Z3
  #elif ENABLED(Z_TWO_STEPPER_DRIVERS)
    , TMC_Z2
  #endif
  #if DRIVER_EXTRUDERS > 0
    , TMC_E0
    #if DRIVER_EXTRUDERS > 1
      , TMC_E1
      #if DRIVER_EXTRUDERS > 2
        , TMC_E2
        #if DRIVER_EXTRUDERS > 3
          , TMC_E3
          #if DRIVER_EXTRUDERS > 4
            , TMC_E4
            #if DRIVER_EXTRUDERS > 5
              , TMC_E5
            #endif // DRIVER_EXTRUDERS > 5
          #endif // DRIVER_EXTRUDERS > 4
        #endif // DRIVER_EXTRUDERS > 3
      #endif // DRIVER_EXTRUDERS > 2
    #endif // DRIVER_EXTRUDERS > 1
  #endif // DRIVER_EXTRUDERS > 0

};

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

#endif // ENABLED(TMC_DEBUG)

class TMC_Stepper {

  public: /** Constructor */

    TMC_Stepper() {}

  public: /** Public Parameters */

    #if X_IS_TRINAMIC
      static TMC_TYPE* stepperX;
    #endif
    #if X2_IS_TRINAMIC
      static TMC_TYPE* stepperX2;
    #endif
    #if Y_IS_TRINAMIC
      static TMC_TYPE* stepperY;
    #endif
    #if Y2_IS_TRINAMIC
      static TMC_TYPE* stepperY2;
    #endif
    #if Z_IS_TRINAMIC
      static TMC_TYPE* stepperZ;
    #endif
    #if Z2_IS_TRINAMIC
      static TMC_TYPE* stepperZ2;
    #endif
    #if Z3_IS_TRINAMIC
      static TMC_TYPE* stepperZ3;
    #endif
    #if E0_IS_TRINAMIC
      static TMC_TYPE* stepperE0;
    #endif
    #if E1_IS_TRINAMIC
      static TMC_TYPE* stepperE1;
    #endif
    #if E2_IS_TRINAMIC
      static TMC_TYPE* stepperE2;
    #endif
    #if E3_IS_TRINAMIC
      static TMC_TYPE* stepperE3;
    #endif
    #if E4_IS_TRINAMIC
      static TMC_TYPE* stepperE4;
    #endif
    #if E5_IS_TRINAMIC
      static TMC_TYPE* stepperE5;
    #endif

  private: /** Private Parameters */

    static bool report_status;

  public: /** Public Function */

    static void init();
    static void current_init_to_defaults();
    static void microstep_init_to_defaults();

    static void restore();

    /**
     * TMC2130 specific sensorless homing using stallGuard2.
     * stallGuard2 only works when in spreadCycle mode.
     * spreadCycle and stealthChop are mutually exclusive.
     *
     * Defined here because of limitations with templates and headers.
     */
    #if HAVE_DRV(TMC2130) && ENABLED(SENSORLESS_HOMING)
      static void sensorless_homing(TMC2130Stepper* st, const bool enable=true);
    #endif

    #if HAVE_DRV(TMC2130)
      #if ENABLED(TMC2130_LINEARITY_CORRECTION)
        #define TMC2130_WAVE_FAC1000_MIN  -200
        #define TMC2130_WAVE_FAC1000_MAX   400
        static void reset_wave(TMC_TYPE* st);
        static void set_fixed_wave(TMC_TYPE* st, const uint8_t i);
        static void set_wave( TMC_TYPE* st, const uint8_t amp, int16_t fac1000,
                              const int8_t xoff=0, const int8_t yoff=10, const uint8_t wavetype=0,
                              const bool config=0, const uint8_t addto=0
        );
      #endif
    #endif

    #if ENABLED(MONITOR_DRIVER_STATUS)
      static void monitor_driver();
    #endif

    #if ENABLED(TMC_DEBUG)
      static void set_report_status(const bool status);
      static void report_all();
    #endif

    FORCE_INLINE static uint32_t thrs(const uint16_t tmc_msteps, const int32_t tmc_thrs, const uint32_t tmc_spmm) {
      return 12650000UL * tmc_msteps / (256 * tmc_thrs * tmc_spmm);
    }

    FORCE_INLINE static void get_current(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_current(axis, st->getCurrent());
    }
    FORCE_INLINE static void set_current(TMC_TYPE* st, const uint16_t mA) {
      st->setCurrent(mA, R_SENSE, HOLD_MULTIPLIER);
    }

    FORCE_INLINE static void get_microstep(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_microstep(axis, st->microsteps());
    }
    FORCE_INLINE static void set_microstep(TMC_TYPE* st, const uint16_t ms) {
      st->microsteps(ms);
    }

    FORCE_INLINE static void report_otpw(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_otpw(axis, st->getOTPW());
    }

    FORCE_INLINE static void clear_otpw(TMC_TYPE* st, const TMC_AxisEnum axis) {
      st->clear_otpw();
      say_otpw_cleared(axis);
    }

    FORCE_INLINE static void get_pwmthrs(TMC_TYPE* st, const TMC_AxisEnum axis, const uint32_t tmc_spmm) {
      say_pwmthrs(axis, thrs(st->microsteps(), st->stealth_max_speed(), tmc_spmm));
    }

    FORCE_INLINE static void set_pwmthrs(TMC_TYPE* st, const int32_t tmc_thrs, const uint32_t tmc_spmm) {
      st->stealth_max_speed(thrs(st->microsteps(), tmc_thrs, tmc_spmm));
    }

    FORCE_INLINE static void get_sgt(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_sgt(axis, st->sgt());
    }

    FORCE_INLINE static void set_sgt(TMC_TYPE* st, const int8_t sgt_val) {
      st->sgt(sgt_val);
    }

    FORCE_INLINE static void get_off_time(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_off_time(axis, st->off_time());
    }

    FORCE_INLINE static void set_off_time(TMC_TYPE* st, const uint8_t off_time_val) {
      st->off_time(off_time_val);
    }

    FORCE_INLINE static void get_fast_decay_time(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_fast_decay_time(axis, st->fast_decay_time());
      say_fast_decay_time(axis, st->hysteresis_start());
    }

    FORCE_INLINE static void set_fast_decay_time(TMC_TYPE* st, const uint8_t fast_decay_time_val) {
      st->fast_decay_time(fast_decay_time_val);
      st->hysteresis_start(fast_decay_time_val << 1);
    }

    FORCE_INLINE static void get_blank_time(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_blank_time(axis, st->blank_time());
    }

    FORCE_INLINE static void set_blank_time(TMC_TYPE* st, const uint8_t blank_time_val) {
      st->blank_time(blank_time_val);
    }

    FORCE_INLINE static void get_hysteresis_end(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_hysteresis_end(axis, st->hysteresis_end());
    }

    FORCE_INLINE static void set_hysteresis_end(TMC_TYPE* st, const int8_t hysteresis_end_val) {
      st->hysteresis_end(hysteresis_end_val);
    }

    FORCE_INLINE static void get_hysteresis_start(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_hysteresis_start(axis, st->hysteresis_start());
    }

    FORCE_INLINE static void set_hysteresis_start(TMC_TYPE* st, const uint8_t hysteresis_start_val) {
      st->hysteresis_start(hysteresis_start_val);
    }

    FORCE_INLINE static void get_stealth_gradient(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_stealth_gradient(axis, st->stealth_gradient());
    }

    FORCE_INLINE static void set_stealth_gradient(TMC_TYPE* st, const uint8_t stealth_gradient_val) {
      st->stealth_gradient(stealth_gradient_val);
    }

    FORCE_INLINE static void get_stealth_amplitude(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_stealth_amplitude(axis, st->stealth_amplitude());
    }

    FORCE_INLINE static void set_stealth_amplitude(TMC_TYPE* st, const uint8_t stealth_amplitude_val) {
      st->stealth_amplitude(stealth_amplitude_val);
    }

    FORCE_INLINE static void get_stealth_freq(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_stealth_freq(axis, st->stealth_freq());
    }

    FORCE_INLINE static void set_stealth_freq(TMC_TYPE* st, const uint8_t stealth_freq_val) {
      st->stealth_freq(stealth_freq_val);
    }

    FORCE_INLINE static void get_stealth_autoscale(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_stealth_autoscale(axis, st->stealth_autoscale());
    }

    FORCE_INLINE static void set_stealth_autoscale(TMC_TYPE* st, const bool stealth_autoscale_val) {
      st->stealth_autoscale(stealth_autoscale_val);
    }

    FORCE_INLINE static void get_disable_I_comparator(TMC_TYPE* st, const TMC_AxisEnum axis) {
      say_disable_I_comparator(axis, st->disable_I_comparator());
    }

    FORCE_INLINE static void set_disable_I_comparator(TMC_TYPE* st, const bool disable_I_comparator_val) {
      st->disable_I_comparator(disable_I_comparator_val);
    }

  private: /** Private Function */

    #if HAVE_DRV(TMC2130)
      static void config(TMC_TYPE* st, const bool tmc_stealthchop=false, const int8_t tmc_sgt=0);
    #elif HAVE_DRV(TMC2208)
      static void config(TMC_TYPE* st, const bool tmc_stealthchop=false);
    #endif

    static void say_axis(const TMC_AxisEnum axis);
    static void say_current(const TMC_AxisEnum axis, const uint16_t tmc_curr);
    static void say_microstep(const TMC_AxisEnum axis, const uint16_t tmc_ms);
    static void say_otpw(const TMC_AxisEnum axis, const bool tmc_otpw);
    static void say_otpw_cleared(const TMC_AxisEnum axis);
    static void say_pwmthrs(const TMC_AxisEnum axis, const uint32_t tmc_thrs);
    static void say_sgt(const TMC_AxisEnum axis, const int8_t tmc_sgt);
    static void say_off_time(const TMC_AxisEnum axis, const uint8_t off_time);
    static void say_fast_decay_time(const TMC_AxisEnum axis, const uint8_t fast_decay_time);
    static void say_blank_time(const TMC_AxisEnum axis, const uint8_t blank_time);
    static void say_hysteresis_end(const TMC_AxisEnum axis, const int8_t hysteresis_end);
    static void say_hysteresis_start(const TMC_AxisEnum axis, const uint8_t hysteresis_start);
    static void say_stealth_gradient(const TMC_AxisEnum axis, const uint8_t stealth_gradien);
    static void say_stealth_amplitude(const TMC_AxisEnum axis, const uint8_t stealth_amplitude);
    static void say_stealth_freq(const TMC_AxisEnum axis, const uint8_t stealth_freq);
    static void say_stealth_autoscale(const TMC_AxisEnum axis, const bool stealth_autoscale);
    static void say_disable_I_comparator(const TMC_AxisEnum axis, const bool disable_I_comparator);
    static void set_mslutstart(TMC_TYPE* st, const uint8_t start_sin, const uint8_t start_sin90);
    static void set_mslutsel(TMC_TYPE* st, uint8_t x1, uint8_t x2, uint8_t x3, int8_t w0, int8_t w1, int8_t w2, int8_t w3);
    static void set_mslut(TMC_TYPE* st, const uint8_t i, const uint32_t val);

    #if ENABLED(MONITOR_DRIVER_STATUS)

      #if HAVE_DRV(TMC2130)
        static TMC_driver_data get_driver_data(TMC2130Stepper* st);
        FORCE_INLINE static uint8_t get_status_response(TMC2130Stepper* st) { return st->status_response & 0xF; }
        FORCE_INLINE static uint32_t get_pwm_scale(TMC2130Stepper* st) { return st->PWM_SCALE(); }
      #elif HAVE_DRV(TMC2208)
        static TMC_driver_data get_driver_data(TMC2208Stepper* st);
        static uint8_t get_status_response(TMC2208Stepper* st);
        FORCE_INLINE static uint32_t get_pwm_scale(TMC2208Stepper* st) { return st->pwm_scale_sum(); }
      #endif

      static void monitor_driver(TMC_TYPE* st, const TMC_AxisEnum axis, uint8_t &otpw_cnt);

    #endif

    #if ENABLED(TMC_DEBUG)
      static void drv_status_print_hex(const TMC_AxisEnum axis, const uint32_t drv_status);
      static void status(TMC_TYPE* st, const TMC_debug_enum i);
      static void status(TMC_TYPE* st, const TMC_AxisEnum axis, const TMC_debug_enum i, const float tmc_spmm);
      static void parse_drv_status(TMC_TYPE* st, const TMC_drv_status_enum i);
      static void parse_drv_status(TMC_TYPE* st, const TMC_AxisEnum axis, const TMC_drv_status_enum i);
      static void debug_loop(const TMC_debug_enum i);
      static void status_loop(const TMC_drv_status_enum i);
    #endif

};

extern TMC_Stepper tmc;

#endif // HAS_TRINAMIC

#endif /* _TMC_H_ */
