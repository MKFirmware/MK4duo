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
#pragma once

/**
 * tmc.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if HAS_TRINAMIC

#include <TMCStepper.h>

#if TMCSTEPPER_VERSION < 0x000601
  #error "Update TMCStepper library to 0.6.1 or newer."
#endif

#if HAVE_DRV(TMC2130)
  #define TMC_MODEL_LIB TMC2130Stepper
#elif HAVE_DRV(TMC2160)
  #define TMC_MODEL_LIB TMC2160Stepper
#elif HAVE_DRV(TMC5130)
  #define TMC_MODEL_LIB TMC5130Stepper
#elif HAVE_DRV(TMC5160)
  #define TMC_MODEL_LIB TMC5160Stepper
#elif HAVE_DRV(TMC5161)
  #define TMC_MODEL_LIB TMC5160Stepper
#endif

#if ENABLED(MONITOR_DRIVER_STATUS) && DISABLED(MONITOR_DRIVER_STATUS_INTERVAL_MS)
  #define MONITOR_DRIVER_STATUS_INTERVAL_MS 500U
#endif

class Driver;

struct TMC_driver_data {
  uint32_t  drv_status;
  bool      is_otpw:  1,
            is_ot:    1,
            is_s2g:   1,
            is_error: 1
            #if ENABLED(TMC_DEBUG)
              , is_stall:             1
              , is_stealth:           1
              , is_standstill:        1
              , sg_result_reasonable: 1
            #endif
      ;
  #if ENABLED(TMC_DEBUG)
    #if HAS_TMCX1XX || HAVE_DRV(TMC2208)
      uint8_t cs_actual;
    #endif
    uint16_t sg_result;
  #endif
};

typedef struct {
  uint8_t toff;
  int8_t  hend;
  uint8_t hstrt;
} chopper_timing_t;

static constexpr chopper_timing_t chopper_timing = CHOPPER_TIMING;

static constexpr int8_t sgt_min = -64,
                        sgt_max =  63;

extern bool report_tmc_status;

constexpr uint16_t _tmc_thrs(const uint16_t msteps, const uint32_t thrs, const uint32_t spmm) {
  return 12650000UL * msteps / (256 * thrs * spmm);
}

class TMCStorage {

  protected: /** Protected Parameters */

    TMCStorage() {}

  public: /** Public Parameters */

    uint16_t  val_mA  = 0,
              val_ms  = 0;

    uint8_t hybrid_thrs = 0;

    #if TMC_HAS_STEALTHCHOP
      bool stealthChop_enabled = false;
    #endif

    #if HAS_SENSORLESS && HAS_LCD_MENU
      int16_t homing_thrs = 0;
    #endif

    #if ENABLED(MONITOR_DRIVER_STATUS)
      uint8_t  otpw_count = 0,
              error_count = 0;
      bool flag_otpw = false;
    #endif

  public: /** Public Function */

    inline uint16_t getMilliamps()  { return val_mA; }
    inline uint16_t getMicrosteps() { return val_ms; }

    #if ENABLED(MONITOR_DRIVER_STATUS)
      inline bool getOTPW() { return flag_otpw; }
      inline void clear_otpw() { flag_otpw = 0; }
    #endif

};

#if HAVE_DRV(TMC2208)

  //
  // TMC2208 Driver Class
  //
  class MKTMC : public TMC2208Stepper, public TMCStorage {

    public: /** Constructor */

      MKTMC(const uint8_t DRIVER_ID, Stream* SerialPort, const float RS) :
        TMC2208Stepper(SerialPort, RS),
        id(DRIVER_ID)
        {}

      MKTMC(const uint8_t DRIVER_ID, const uint16_t RX, const uint16_t TX, const float RS, const bool has_rx=true) :
        TMC2208Stepper(RX, TX, RS, has_rx),
        id(DRIVER_ID)
        {}

    public: /** Public Parameters */

      const uint8_t id;

    public: /** Public Function */

      inline uint16_t rms_current() { return TMC2208Stepper::rms_current(); }

      inline void rms_current(const uint16_t mA) {
        this->val_mA = mA;
        TMC2208Stepper::rms_current(mA);
      }

      inline void rms_current(const uint16_t mA, const float mult) {
        this->val_mA = mA;
        TMC2208Stepper::rms_current(mA, mult);
      }

      inline uint16_t microsteps() { return TMC2208Stepper::microsteps(); }

      inline void microsteps(const uint16_t ms) {
        this->val_ms = ms;
        TMC2208Stepper::microsteps(ms);
      }

      inline void refresh_stepping_mode()   { this->en_spreadCycle(!this->stealthChop_enabled); }
      inline bool get_stealthChop_status()  { return !this->en_spreadCycle(); }

      #if ENABLED(HYBRID_THRESHOLD)
        uint32_t get_pwm_thrs() {
          return _tmc_thrs(microsteps(), this->TPWMTHRS(), mechanics.data.axis_steps_per_mm[this->id]);
        }
        void set_pwm_thrs(const uint32_t thrs) {
          TMC2208Stepper::TPWMTHRS(_tmc_thrs(microsteps(), thrs, mechanics.data.axis_steps_per_mm[this->id]));
          #if HAS_LCD_MENU
            this->hybrid_thrs = thrs;
          #endif
        }
        uint32_t get_pwm_thrs_e() {
          return _tmc_thrs(microsteps(), this->TPWMTHRS(), extruders[this->id]->data.axis_steps_per_mm);
        }
        void set_pwm_thrs_e(const uint32_t thrs) {
          TMC2208Stepper::TPWMTHRS(_tmc_thrs(microsteps(), thrs, extruders[this->id]->data.axis_steps_per_mm));
          #if HAS_LCD_MENU
            this->hybrid_thrs = thrs;
          #endif
        }
      #endif

      #if HAS_LCD_MENU
        inline void refresh_stepper_current()   { rms_current(val_mA); }
        inline void refresh_stepper_microstep() { microsteps(val_ms); }

        #if ENABLED(HYBRID_THRESHOLD)
          inline void refresh_hybrid_thrs()   { set_pwm_thrs(this->hybrid_thrs); }
          inline void refresh_hybrid_thrs_e() { set_pwm_thrs_e(this->hybrid_thrs); }
        #endif
      #endif

  };

#elif HAVE_DRV(TMC2660)

  //
  // TMC2660 Driver Class
  //
  class MKTMC : public TMC2660Stepper, public TMCStorage {

    public: /** Constructor */

      MKTMC(const uint8_t DRIVER_ID, const uint16_t cs_pin, const float RS) :
        TMC2660Stepper(cs_pin, RS),
        id(DRIVER_ID)
        {}

      MKTMC(const uint8_t DRIVER_ID, const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
        TMC2660Stepper(CS, RS, pinMOSI, pinMISO, pinSCK),
        id(DRIVER_ID)
        {}

    public: /** Public Parameters */

      const uint8_t id;

    public: /** Public Function */

      inline uint16_t rms_current() { return TMC2660Stepper::rms_current(); }

      inline void rms_current(uint16_t mA) {
        this->val_mA = mA;
        TMC2660Stepper::rms_current(mA);
      }

      inline uint16_t microsteps() { return TMC2660Stepper::microsteps(); }

      inline void microsteps(const uint16_t ms) {
        this->val_ms = ms;
        TMC2660Stepper::microsteps(ms);
      }

      #if USE_SENSORLESS
        inline int16_t homing_threshold() { return TMC2660Stepper::sgt(); }
        void homing_threshold(int16_t sgt_val) {
          sgt_val = (int16_t)constrain(sgt_val, sgt_min, sgt_max);
          TMC2660Stepper::sgt(sgt_val);
          #if HAS_LCD_MENU
            this->homing_thrs = sgt_val;
          #endif
        }
      #endif

      #if HAS_LCD_MENU
        inline void refresh_stepper_current()   { rms_current(this->val_mA); }
        inline void refresh_stepper_microstep() { microsteps(this->val_ms); }

        #if HAS_SENSORLESS
          inline void refresh_homing_thrs() { homing_threshold(this->homing_thrs); }
        #endif
      #endif

  };

#elif HAS_TMCX1XX

  //
  // TMC Driver Class
  //
  class MKTMC : public TMC_MODEL_LIB, public TMCStorage {

    public: /** Constructor */

      MKTMC(const uint8_t DRIVER_ID, const uint16_t cs_pin, const float RS) :
        TMC_MODEL_LIB(cs_pin, RS),
        id(DRIVER_ID)
        {}

      MKTMC(const uint8_t DRIVER_ID, const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
        TMC_MODEL_LIB(CS, RS, pinMOSI, pinMISO, pinSCK),
        id(DRIVER_ID)
        {}

    public: /** Public Parameters */

      const uint8_t id;

    public: /** Public Function */

      inline uint16_t rms_current() { return TMC_MODEL_LIB::rms_current(); }

      inline void rms_current(const uint16_t mA) {
        this->val_mA = mA;
        TMC_MODEL_LIB::rms_current(mA);
      }

      inline void rms_current(const uint16_t mA, const float mult) {
        this->val_mA = mA;
        TMC_MODEL_LIB::rms_current(mA, mult);
      }

      inline uint16_t microsteps() { return TMC_MODEL_LIB::microsteps(); }

      inline void microsteps(const uint16_t ms) {
        this->val_ms = ms;
        TMC_MODEL_LIB::microsteps(ms);
      }

      inline void refresh_stepping_mode()   { this->en_pwm_mode(this->stealthChop_enabled); }
      inline bool get_stealthChop_status()  { return this->en_pwm_mode(); }

      #if ENABLED(HYBRID_THRESHOLD)
        uint32_t get_pwm_thrs() {
          return _tmc_thrs(microsteps(), this->TPWMTHRS(), mechanics.data.axis_steps_per_mm[this->id]);
        }
        void set_pwm_thrs(const uint32_t thrs) {
          TMC_MODEL_LIB::TPWMTHRS(_tmc_thrs(microsteps(), thrs, mechanics.data.axis_steps_per_mm[this->id]));
          #if HAS_LCD_MENU
            this->hybrid_thrs = thrs;
          #endif
        }
        uint32_t get_pwm_thrs_e() {
          return _tmc_thrs(microsteps(), this->TPWMTHRS(), mechanics.data.axis_steps_per_mm[this->id]);
        }
        void set_pwm_thrs_e(const uint32_t thrs) {
          TMC_MODEL_LIB::TPWMTHRS(_tmc_thrs(microsteps(), thrs, mechanics.data.axis_steps_per_mm[this->id]));
          #if HAS_LCD_MENU
            this->hybrid_thrs = thrs;
          #endif
        }
      #endif
      #if HAS_SENSORLESS
        inline int16_t homing_threshold() { return TMC_MODEL_LIB::sgt(); }
        void homing_threshold(int16_t sgt_val) {
          sgt_val = (int16_t)constrain(sgt_val, sgt_min, sgt_max);
          TMC_MODEL_LIB::sgt(sgt_val);
          #if HAS_LCD_MENU
            this->homing_thrs = sgt_val;
          #endif
        }
      #endif

      #if ENABLED(SPI_ENDSTOPS)

        bool test_stall_status() {
          TMC2130_n::DRV_STATUS_t drv_status{0};
          drv_status.sr = this->DRV_STATUS();
          return drv_status.stallGuard;
        }

      #endif // SPI_ENDSTOPS

      #if HAS_LCD_MENU
        inline void refresh_stepper_current()   { rms_current(this->val_mA); }
        inline void refresh_stepper_microstep() { microsteps(this->val_ms); }

        #if ENABLED(HYBRID_THRESHOLD)
          inline void refresh_hybrid_thrs()   { set_pwm_thrs(this->hybrid_thrs); }
          inline void refresh_hybrid_thrs_e() { set_pwm_thrs_e(this->hybrid_thrs); }
        #endif
        #if HAS_SENSORLESS
          inline void refresh_homing_thrs() { homing_threshold(this->homing_thrs); }
        #endif
      #endif

  };

#endif

class TMC_Manager {

  public: /** Constructor */

    TMC_Manager() {}

  public: /** Public Parameters */

    #if HAS_SENSORLESS && ENABLED(IMPROVE_HOMING_RELIABILITY)
      static millis_l sg_guard_period;
      static constexpr uint16_t default_sg_guard_duration = 400;
    #endif

  private: /** Private Parameters */

    static uint16_t report_status_interval;

  public: /** Public Function */

    static void init_cs_pins();

    static void create_tmc();

    static void factory_parameters();

    static void restore();

    static void test_connection(const bool test_x, const bool test_y, const bool test_z, const bool test_e);

    static void go_to_homing_phase(const AxisEnum axis, const feedrate_t fr_mm_s);

    #if ENABLED(MONITOR_DRIVER_STATUS)
      static void monitor_drivers();
    #endif

    #if HAS_SENSORLESS
      static bool enable_stallguard(Driver* drv);
      static void disable_stallguard(Driver* drv, const bool enable);
    #endif

    #if ENABLED(TMC_DEBUG)
      #if ENABLED(MONITOR_DRIVER_STATUS)
        static void set_report_interval(const uint16_t update_interval);
      #endif
      static void report_all(const bool print_x, const bool print_y, const bool print_z, const bool print_e);
      static void get_registers(const bool print_x, const bool print_y, const bool print_z, const bool print_e);
    #endif

    #if DISABLED(DISABLE_M503)
      static void print_M350();
      static void print_M906();
      static void print_M913();
      static void print_M914();
      static void print_M940();
    #endif

    #if ENABLED(MONITOR_DRIVER_STATUS)

      static void report_otpw(Driver* drv);
      static void clear_otpw(Driver* drv);

    #endif

    static void get_off_time(Driver* drv);
    static void set_off_time(Driver* drv, const uint8_t off_time_val);

    #if HAVE_DRV(TMC2130)

      static void get_blank_time(Driver* drv);
      static void set_blank_time(Driver* drv, const uint8_t blank_time_val);
      static void get_hysteresis_end(Driver* drv);
      static void set_hysteresis_end(Driver* drv, const int8_t hysteresis_end_val);
      static void get_hysteresis_start(Driver* drv);
      static void set_hysteresis_start(Driver* drv, const uint8_t hysteresis_start_val);
      static void get_disable_I_comparator(Driver* drv);
      static void set_disable_I_comparator(Driver* drv, const bool onoff);
      static void get_stealth_gradient(Driver* drv);
      static void set_stealth_gradient(Driver* drv, const uint8_t stealth_gradient_val);
      static void get_stealth_amplitude(Driver* drv);
      static void set_stealth_amplitude(Driver* drv, const uint8_t stealth_amplitude_val);
      static void get_stealth_freq(Driver* drv);
      static void set_stealth_freq(Driver* drv, const uint8_t stealth_freq_val);
      static void get_stealth_autoscale(Driver* drv);
      static void set_stealth_autoscale(Driver* drv, const bool onoff);

    #endif // HAVE_DRV(TMC2130)

  private: /** Private Function */

    static bool test_connection(Driver* drv);

    static void config(Driver* drv, const bool stealth=false);

    #if ENABLED(MONITOR_DRIVER_STATUS)

      #if ENABLED(TMC_DEBUG)
        #if HAVE_DRV(TMC2208)
          static uint32_t get_pwm_scale(Driver* drv);
        #elif HAVE_DRV(TMC2660)
          static uint32_t get_pwm_scale(Driver* drv);
        #elif HAS_TMCX1XX
          static uint32_t get_pwm_scale(Driver* drv);
        #endif
      #endif

      static TMC_driver_data get_driver_data(Driver* drv);
      static void monitor_driver(Driver* drv, const bool need_update_error_counters, const bool need_debug_reporting);

    #endif

    #if ENABLED(TMC_DEBUG)

      static void print_vsense(Driver* drv);
      static void status(Driver* drv, const TMCdebugEnum i);
      static void status(Driver* drv, const TMCdebugEnum i, const float spmm);
      static void parse_type_drv_status(Driver* drv, const TMCdrvStatusEnum i);
      static void parse_drv_status(Driver* drv, const TMCdrvStatusEnum i);
      static void debug_loop(const TMCdebugEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e);
      static void status_loop(const TMCdrvStatusEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e);
      static void get_ic_registers(Driver* drv, const TMCgetRegistersEnum i);
      static void get_registers(Driver* drv, const TMCgetRegistersEnum i);
      static void get_registers(const TMCgetRegistersEnum i, const bool print_x, const bool print_y, const bool print_z, const bool print_e);

    #endif

};

extern TMC_Manager tmcManager;

#endif // HAS_TRINAMIC
