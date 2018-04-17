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
 * heater.h - heater object
 */

#ifndef _HEATER_H_
#define _HEATER_H_

#if HOTENDS <= 1
  #define HOTEND_INDEX      0
  #define EXTRUDER_IDX      0
  #define TRG_EXTRUDER_IDX  0
#else
  #define HOTEND_INDEX      h
  #define EXTRUDER_IDX      tools.active_extruder
  #define TRG_EXTRUDER_IDX  tools.target_extruder
#endif

#include "sensor/sensor.h"

#if HEATER_COUNT > 0

  enum FlagHeaters {
    heater_flag_use_pid,
    heater_flag_tuning,
    heater_flag_hardware_inverted,
    heater_flag_idle
  };

  typedef enum { IS_HOTEND = 0, IS_BED = 1, IS_CHAMBER = 2, IS_COOLER = 3 } Heater_type;

  constexpr uint16_t  temp_check_interval[HEATER_TYPE]  = { 0, BED_CHECK_INTERVAL, CHAMBER_CHECK_INTERVAL, COOLER_CHECK_INTERVAL };
  constexpr uint8_t   temp_hysteresis[HEATER_TYPE]      = { 0, BED_HYSTERESIS, CHAMBER_HYSTERESIS, COOLER_HYSTERESIS };

  class Heater {

    public: /** Public Parameters */

      Heater_type type;
      pin_t       pin;
      uint8_t     ID,
                  soft_pwm,
                  pwm_pos,
                  pidDriveMin,
                  pidDriveMax,
                  pidMax;
      int16_t     target_temperature,
                  mintemp,
                  maxtemp;
      float       current_temperature,
                  Kp,
                  Ki,
                  Kd,
                  Kc,
                  tempIState,
                  tempIStateLimitMin,
                  tempIStateLimitMax,
                  last_temperature,
                  temperature_1s;
      millis_t    next_check_ms;
      uint8_t     HeaterFlag;

      #if HEATER_IDLE_HANDLER
        millis_t  idle_timeout_ms;
      #endif

      #if WATCH_THE_HEATER
        uint16_t  watch_target_temp;
        millis_t  watch_next_ms;
      #endif

      TemperatureSensor sensor;

    private: /** Private Parameters */

    public: /** Public Function */

      void init();

      void setTarget(int16_t celsius);
      void updatePID();
      void get_pid_output(const bool cycle_1s);
      void print_PID();
      void print_parameters();
      void sensor_print_parameters();

      #if HARDWARE_PWM
        void SetHardwarePwm();
      #endif

      FORCE_INLINE void updateCurrentTemperature() { this->current_temperature = this->sensor.getTemperature(); }
      FORCE_INLINE bool isON()        { return (this->sensor.type != 0 && this->target_temperature > 0); }
      FORCE_INLINE bool isOFF()       { return (!isON()); }
      FORCE_INLINE bool tempisrange() { return (WITHIN(this->current_temperature, this->mintemp, this->maxtemp)); }
      FORCE_INLINE bool isHeating()   { return this->target_temperature > this->current_temperature; }
      FORCE_INLINE bool isCooling()   { return this->target_temperature <= this->current_temperature; }

      FORCE_INLINE bool wait_for_heating() {
        return this->isON() && this->target_temperature > (this->current_temperature + TEMP_HYSTERESIS);
      }

      #if WATCH_THE_HEATER
        void start_watching();
      #endif

      FORCE_INLINE void setUsePid(const bool onoff) {
        SET_BIT(HeaterFlag, heater_flag_use_pid, onoff);
      }
      FORCE_INLINE bool isUsePid() { return TEST(HeaterFlag, heater_flag_use_pid); }

      FORCE_INLINE void setTuning(const bool onoff) {
        SET_BIT(HeaterFlag, heater_flag_tuning, onoff);
      }
      FORCE_INLINE bool isTuning() { return TEST(HeaterFlag, heater_flag_tuning); }

      FORCE_INLINE void setHWInverted(const bool onoff) {
        SET_BIT(HeaterFlag, heater_flag_hardware_inverted, onoff);
      }
      FORCE_INLINE bool isHWInverted() { return TEST(HeaterFlag, heater_flag_hardware_inverted); }

      FORCE_INLINE void setIdle(const bool onoff) {
        SET_BIT(HeaterFlag, heater_flag_idle, onoff);
      }
      FORCE_INLINE bool isIdle() { return TEST(HeaterFlag, heater_flag_idle); }

      FORCE_INLINE bool resetFlag() { HeaterFlag = 0; }

      #if HEATER_IDLE_HANDLER
        void start_idle_timer(const millis_t timeout_ms);
        void reset_idle_timer();
      #endif

    private: /** Private Function */

  };

  extern Heater heaters[HEATER_COUNT];

#endif // HEATER_COUNT > 0

#endif /* _HEATER_H_ */
