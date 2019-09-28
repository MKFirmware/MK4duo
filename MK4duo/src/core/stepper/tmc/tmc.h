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
#pragma once

/**
 * tmc.h
 *
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_TRINAMIC

#include <TMCStepper.h>

#if TMCSTEPPER_VERSION < 0x000501
  #error "Update TMCStepper library to 0.5.1 or newer."
#endif

#if HAVE_DRV(TMC2130)
  #define TMC_MODEL_LIB TMC2130Stepper
#elif HAVE_DRV(TMC2160)
  #define TMC_MODEL_LIB TMC2160Stepper
#elif HAVE_DRV(TMC5130)
  #define TMC_MODEL_LIB TMC5130Stepper
#elif HAVE_DRV(TMC5160)
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
    #if HAS_TMCX1X0 || HAVE_DRV(TMC2208)
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

constexpr uint16_t tmc_thrs(const uint16_t msteps, const uint32_t thrs, const uint32_t spmm) {
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

    #if HAS_SENSORLESS
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

      MKTMC(Stream* SerialPort, const float RS) :
        id(DRIVER_ID),
        TMC2208Stepper(SerialPort, RS, /*has_rx=*/true)
        {}

      MKTMC(const uint16_t RX, const uint16_t TX, const float RS, const bool has_rx=true) :
        id(DRIVER_ID),
        TMC2208Stepper(RX, TX, RS, has_rx)
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
          return tmc_thrs(microsteps(), this->TPWMTHRS(), mechanics.data.axis_steps_per_mm[this->id]);
        }
        void set_pwm_thrs(const uint32_t thrs) {
          TMC2208Stepper::TPWMTHRS(tmc_thrs(microsteps(), thrs, mechanics.data.axis_steps_per_mm[this->id]));
          #if HAS_LCD_MENU
            this->hybrid_thrs = thrs;
          #endif
        }
      #endif

      #if HAS_LCD_MENU
        inline void refresh_stepper_current()   { rms_current(val_mA); }
        inline void refresh_stepper_microstep() { microsteps(val_ms); }

        #if ENABLED(HYBRID_THRESHOLD)
          inline void refresh_hybrid_thrs() { set_pwm_thrs(this->hybrid_thrs); }
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
        id(DRIVER_ID),
        TMC2660Stepper(cs_pin, RS)
        {}

      MKTMC(const uint8_t DRIVER_ID, const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
        id(DRIVER_ID),
        TMC2660Stepper(CS, RS, pinMOSI, pinMISO, pinSCK)
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

#elif HAS_TMCX1X0

  //
  // TMC Driver Class
  //
  class MKTMC : public TMC_MODEL_LIB, public TMCStorage {

    public: /** Constructor */

      MKTMC(const uint8_t DRIVER_ID, const uint16_t cs_pin, const float RS) :
        id(DRIVER_ID),
        TMC_MODEL_LIB(cs_pin, RS)
        {}

      MKTMC(const uint8_t DRIVER_ID, const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
        id(DRIVER_ID),
        TMC_MODEL_LIB(CS, RS, pinMOSI, pinMISO, pinSCK)
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
          return tmc_thrs(microsteps(), this->TPWMTHRS(), mechanics.data.axis_steps_per_mm[this->id]);
        }
        void set_pwm_thrs(const uint32_t thrs) {
          TMC_MODEL_LIB::TPWMTHRS(tmc_thrs(microsteps(), thrs, mechanics.data.axis_steps_per_mm[this->id]));
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
          uint16_t sg_result = 0;

          this->switchCSpin(LOW);

          if (this->TMC_SW_SPI != nullptr) {
            this->TMC_SW_SPI->transfer(TMC2130_n::DRV_STATUS_t::address);
            this->TMC_SW_SPI->transfer16(0);
            // We only care about the last 10 bits
            sg_result = this->TMC_SW_SPI->transfer(0);
            sg_result <<= 8;
            sg_result |= this->TMC_SW_SPI->transfer(0);
          }
          else {
            SPI.beginTransaction(SPISettings(16000000/8, MSBFIRST, SPI_MODE3));
            // Read DRV_STATUS
            SPI.transfer(TMC2130_n::DRV_STATUS_t::address);
            SPI.transfer16(0);
            // We only care about the last 10 bits
            sg_result = SPI.transfer(0);
            sg_result <<= 8;
            sg_result |= SPI.transfer(0);
            SPI.endTransaction();
          }
          this->switchCSpin(HIGH);

          return (sg_result & 0x3FF) == 0;
        }

      #endif // SPI_ENDSTOPS

      #if HAS_LCD_MENU
        inline void refresh_stepper_current()   { rms_current(this->val_mA); }
        inline void refresh_stepper_microstep() { microsteps(this->val_ms); }

        #if ENABLED(HYBRID_THRESHOLD)
          inline void refresh_hybrid_thrs() { set_pwm_thrs(this->hybrid_thrs); }
        #endif
        #if HAS_SENSORLESS
          inline void refresh_homing_thrs() { homing_threshold(this->homing_thrs); }
        #endif
      #endif

  };

#endif

// X Stepper
#if AXIS_HAS_TMC(X)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define X_ENABLE_INIT           NOOP
    #define X_ENABLE_WRITE(STATE)   driver.x->tmc->toff((STATE)==driver.x->isEnable() ? chopper_timing.toff : 0)
    #define X_ENABLE_READ()         driver.x->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(X)
    #define X_STEP_WRITE(STATE)     driver.x->step_toggle(STATE)
  #endif
#endif

// X2 Stepper
#if HAS_X2_ENABLE && AXIS_HAS_TMC(X2)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define X2_ENABLE_INIT          NOOP
    #define X2_ENABLE_WRITE(STATE)  driver.x2->tmc->toff((STATE)==driver.x2->isEnable() ? chopper_timing.toff : 0)
    #define X2_ENABLE_READ()        driver.x2->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(X2)
    #define X2_STEP_WRITE(STATE)    driver.x2->step_toggle(STATE)
  #endif
#endif

// Y Stepper
#if AXIS_HAS_TMC(Y)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define Y_ENABLE_INIT           NOOP
    #define Y_ENABLE_WRITE(STATE)   driver.y->tmc->toff((STATE)==driver.y->isEnable() ? chopper_timing.toff : 0)
    #define Y_ENABLE_READ()         driver.y->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(Y)
    #define Y_STEP_WRITE(STATE)     driver.y->step_toggle(STATE)
  #endif
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE && AXIS_HAS_TMC(Y2)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define Y2_ENABLE_INIT          NOOP
    #define Y2_ENABLE_WRITE(STATE)  driver.y2->tmc->toff((STATE)==driver.y2->isEnable() ? chopper_timing.toff : 0)
    #define Y2_ENABLE_READ()        driver.y2->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(Y2)
    #define Y2_STEP_WRITE(STATE)    driver.y2->step_toggle(STATE)
  #endif
#endif

// Z Stepper
#if AXIS_HAS_TMC(Z)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define Z_ENABLE_INIT           NOOP
    #define Z_ENABLE_WRITE(STATE)   driver.z->tmc->toff((STATE)==driver.z->isEnable() ? chopper_timing.toff : 0)
    #define Z_ENABLE_READ()         driver.z->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(Z)
    #define Z_STEP_WRITE(STATE)     driver.z->step_toggle(STATE)
  #endif
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE && AXIS_HAS_TMC(Z2)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define Z2_ENABLE_INIT          NOOP
    #define Z2_ENABLE_WRITE(STATE)  driver.z2->tmc->toff((STATE)==driver.z2->isEnable() ? chopper_timing.toff : 0)
    #define Z2_ENABLE_READ()        driver.z2->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(Z2)
    #define Z2_STEP_WRITE(STATE)    driver.z2->step_toggle(STATE)
  #endif
#endif

// Z3 Stepper
#if HAS_Z3_ENABLE && AXIS_HAS_TMC(Z3)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define Z3_ENABLE_INIT          NOOP
    #define Z3_ENABLE_WRITE(STATE)  driver.z3->toff((STATE)==driver.z3->isEnable() ? chopper_timing.toff : 0)
    #define Z3_ENABLE_READ()        driver.z3->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(Z3)
    #define Z3_STEP_WRITE(STATE)    driver.z3->step_toggle(STATE)
  #endif
#endif

// E0 Stepper
#if AXIS_HAS_TMC(E0)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define E0_ENABLE_INIT          NOOP
    #define E0_ENABLE_WRITE(STATE)  driver.e[E0_DRV]->tmc->toff((STATE)==driver.e[E0_DRV]->isEnable() ? chopper_timing.toff : 0)
    #define E0_ENABLE_READ()        driver.e[E0_DRV]->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(E0)
    #define E0_STEP_WRITE(STATE)    driver.e[E0_DRV]->step_toggle(STATE)
  #endif
#endif

// E1 Stepper
#if AXIS_HAS_TMC(E1)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define E1_ENABLE_INIT          NOOP
    #define E1_ENABLE_WRITE(STATE)  driver.e[E1_DRV]->tmc->toff((STATE)==driver.e[E1_DRV]->isEnable() ? chopper_timing.toff : 0)
    #define E1_ENABLE_READ()        driver.e[E1_DRV]->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(E1)
    #define E1_STEP_WRITE(STATE)    driver.e[E1_DRV]->step_toggle(STATE)
  #endif
#endif

// E2 Stepper
#if AXIS_HAS_TMC(E2)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define E2_ENABLE_INIT          NOOP
    #define E2_ENABLE_WRITE(STATE)  driver.e[E2_DRV]->tmc->toff((STATE)==driver.e[E2_DRV]->isEnable() ? chopper_timing.toff : 0)
    #define E2_ENABLE_READ()        driver.e[E2_DRV]->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(E2)
    #define E2_STEP_WRITE(STATE)    driver.e[E2_DRV]->step_toggle(STATE)
  #endif
#endif

// E3 Stepper
#if AXIS_HAS_TMC(E3)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define E3_ENABLE_INIT          NOOP
    #define E3_ENABLE_WRITE(STATE)  driver.e[E3_DRV]->tmc->toff((STATE)==driver.e[E3_DRV]->isEnable() ? chopper_timing.toff : 0)
    #define E3_ENABLE_READ()        driver.e[E3_DRV]->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(E3)
    #define E3_STEP_WRITE(STATE)    driver.e[E3_DRV]->step_toggle(STATE)
  #endif
#endif

// E4 Stepper
#if AXIS_HAS_TMC(E4)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define E4_ENABLE_INIT          NOOP
    #define E4_ENABLE_WRITE(STATE)  driver.e[E4_DRV]->tmc->toff((STATE)==driver.e[E4_DRV]->isEnable() ? chopper_timing.toff : 0)
    #define E4_ENABLE_READ()        driver.e[E4_DRV]->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(E4)
    #define E4_STEP_WRITE(STATE)    driver.e[E4_DRV]->step_toggle(STATE)
  #endif
#endif

// E5 Stepper
#if AXIS_HAS_TMC(E5)
  #if ENABLED(TMC_SOFTWARE_DRIVER_ENABLE)
    #define E5_ENABLE_INIT          NOOP
    #define E5_ENABLE_WRITE(STATE)  driver.e[E5_DRV]->tmc->toff((STATE)==driver.e[E5_DRV]->isEnable() ? chopper_timing.toff : 0)
    #define E5_ENABLE_READ()        driver.e[E5_DRV]->tmc->isEnabled()
  #endif
  #if AXIS_HAS_SQUARE_WAVE(E5)
    #define E5_STEP_WRITE(STATE)    driver.e[E5_DRV]->step_toggle(STATE)
  #endif
#endif

class TMC_Stepper {

  public: /** Constructor */

    TMC_Stepper() {}

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

    #if ENABLED(MONITOR_DRIVER_STATUS)
      static void monitor_driver();
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
        #elif HAS_TMCX1X0
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

extern TMC_Stepper tmc;

#endif // HAS_TRINAMIC
