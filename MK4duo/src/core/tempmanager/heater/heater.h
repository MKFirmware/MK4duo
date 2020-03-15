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
 * heater.h - heater object
 */

union heater_flag_t {
  uint8_t all;
  struct {
    bool  Active            : 1;
    bool  UsePid            : 1;
    bool  Pidtuned          : 1;
    bool  HWInvert          : 1;
    bool  HWpwm             : 1;
    bool  Thermalprotection : 1;
    bool  Idle              : 1;
    bool  Fault             : 1;
  };
  heater_flag_t() { all = 0x00; }
};

enum HeatertypeEnum : uint8_t { IS_HOTEND, IS_BED, IS_CHAMBER, IS_COOLER };
enum TRState        : uint8_t { TRInactive, TRFirstHeating, TRStable, TRRunaway };

// Struct Heater data
struct heater_data_t {
  uint8_t         ID;
  uint16_t        freq;
  pin_t           pin;
  heater_flag_t   flag;
  limit_int_t     temp;
  pid_data_t      pid;
  sensor_data_t   sensor;
};

class Heater {

  public: /** Constructor */

    Heater(HeatertypeEnum type_p, uint16_t temp_check_interval_p, uint8_t temp_hysteresis_p, uint8_t watch_period_p, uint8_t watch_increase_p) :
      type(type_p),
      temp_check_interval(temp_check_interval_p),
      temp_hysteresis(temp_hysteresis_p),
      watch_period(watch_period_p),
      watch_increase(watch_increase_p)
    {}

  public: /** Public Parameters */

    heater_data_t   data;

    uint8_t         pwm_value;

    int16_t         target_temperature,
                    idle_temperature;

    float           current_temperature;

    const HeatertypeEnum type;

  private: /** Private Parameters */

    const uint16_t  temp_check_interval;

    const uint8_t   temp_hysteresis,
                    watch_period,
                    watch_increase;

    uint8_t         consecutive_low_temp,
                    pwm_soft_pos;

    uint16_t        watch_target_temp;

    TRState         thermal_runaway_state;

    millis_l        idle_timeout_ms;

    short_timer_t   next_check_timer;
    long_timer_t    next_watch_timer;

    bool            Pidtuning;

  public: /** Public Function */

    void init();

    void set_target_temp(const int16_t celsius);
    void set_idle_temp(const int16_t celsius);
    void wait_for_target(bool no_wait_for_cooling=true);

    void get_output();
    void set_output_pwm();

    void check_and_power();

    void PID_autotune(const float target_temp, const uint8_t ncycles, const uint8_t method, const bool storeValues=false);

    void print_M301();
    void print_M305();
    void print_M306();
    #if HAS_AD8495 || HAS_AD595
      void print_M595();
    #endif

    void start_idle_timer(const millis_l &ms);
    void reset_idle_timer();

    void thermal_runaway_protection();
    void start_watching();

    FORCE_INLINE void update_current_temperature() { this->current_temperature = this->data.sensor.getTemperature(); }
    FORCE_INLINE int16_t deg_current()  { return this->current_temperature + 0.5f; }
    FORCE_INLINE int16_t deg_target()   { return this->target_temperature;  }
    FORCE_INLINE int16_t deg_idle()     { return this->idle_temperature;    }
    FORCE_INLINE bool tempisrange()     { return (WITHIN(this->current_temperature, this->data.temp.min, this->data.temp.max)); }
    FORCE_INLINE bool isHeating()       { return this->target_temperature > this->current_temperature;  }
    FORCE_INLINE bool isCooling()       { return this->target_temperature <= this->current_temperature; }

    FORCE_INLINE bool wait_for_heating() {
      return this->isActive() && ABS(this->current_temperature - this->target_temperature) > temp_hysteresis;
    }

    // Flag bit 0 Set Active
    FORCE_INLINE void setActive(const bool onoff) {
      if (!data.flag.Fault && data.sensor.type != 0 && onoff)
        data.flag.Active = true;
      else
        data.flag.Active = false;
    }
    FORCE_INLINE bool isActive() { return data.flag.Active; }

    // Flag bit 1 Set use Pid
    FORCE_INLINE void setUsePid(const bool onoff) { data.flag.UsePid = onoff; }
    FORCE_INLINE bool isUsePid() { return data.flag.UsePid; }

    // Flag bit 2 Set Set Pid Tuned
    FORCE_INLINE void setPidTuned(const bool onoff) { data.flag.Pidtuned = onoff; }
    FORCE_INLINE bool isPidTuned() { return data.flag.Pidtuned; }

    // Flag bit 3 Set Hardware inverted
    FORCE_INLINE void setHWinvert(const bool onoff) { data.flag.HWInvert = onoff; }
    FORCE_INLINE bool isHWinvert() { return data.flag.HWInvert; }

    // Flag bit 4 Set PWM Hardware
    FORCE_INLINE void setHWpwm(const bool onoff) { data.flag.HWpwm = onoff; }
    FORCE_INLINE bool isHWpwm() { return data.flag.HWpwm; }

    // Flag bit 5 Set Thermal Protection
    FORCE_INLINE void setThermalProtection(const bool onoff) { data.flag.Thermalprotection = onoff; }
    FORCE_INLINE bool isThermalProtection() { return data.flag.Thermalprotection; }

    // Flag bit 6 Set Idle
    FORCE_INLINE void setIdle(const bool onoff) {
      data.flag.Idle = onoff;
      if (onoff) thermal_runaway_state = TRInactive;
    }
    FORCE_INLINE bool isIdle() { return data.flag.Idle; }

    // Flag bit 7 Set Fault
    FORCE_INLINE void setFault() {
      pwm_value = 0;
      setActive(false);
      data.flag.Fault = true;
    }
    FORCE_INLINE void ResetFault() {
      data.flag.Fault = false;
      SwitchOff();
    }
    FORCE_INLINE bool isFault() { return data.flag.Fault; }

    FORCE_INLINE void resetFlag() { data.flag.all = false; }

    FORCE_INLINE void SwitchOff() {
      target_temperature = 0;
      pwm_value = 0;
      data.pid.reset();
      setActive(false);
    }

  private: /** Private Function */

    void temp_error(PGM_P const serial_msg, PGM_P const lcd_msg);
    void min_temp_error();
    void max_temp_error();

    void print_low_high_temp(const int16_t celsius, const bool min_temp);

    void update_idle_timer();

};

#if HAS_HOTENDS
  extern Heater* hotends[MAX_HOTEND];
#endif
#if HAS_BEDS
  extern Heater* beds[MAX_BED];
#endif
#if HAS_CHAMBERS
  extern Heater* chambers[MAX_CHAMBER];
#endif
#if HAS_COOLERS
  extern Heater* coolers[MAX_COOLER];
#endif
