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

#include "sensor/sensor.h"
#include "pid/pid.h"

constexpr uint16_t  temp_check_interval[HEATER_TYPE]  = { 0, BED_CHECK_INTERVAL, CHAMBER_CHECK_INTERVAL, COOLER_CHECK_INTERVAL };
constexpr uint8_t   temp_hysteresis[HEATER_TYPE]      = { 0, BED_HYSTERESIS, CHAMBER_HYSTERESIS, COOLER_HYSTERESIS };

// Struct Heater data
typedef struct {

  heater_t    type;

  pin_t       pin;

  flagbyte_t  flag;

  uint8_t     ID;

  int16_t     mintemp,
              maxtemp;

} heater_data_t;

class Heater {

  public: /** Public Parameters */

    heater_data_t data;
    pid_data_t    pid;
    sensor_data_t sensor;

    uint8_t       soft_pwm,
                  pwm_pos;

    int16_t       target_temperature,
                  idle_temperature;

    float         current_temperature;

    millis_t      next_check_ms,
                  idle_timeout_ms;

    #if WATCH_THE_HEATER
      uint16_t    watch_target_temp;
      millis_t    watch_next_ms;
    #endif

  private: /** Private Parameters */

  public: /** Public Function */

    void init();

    void setTarget(int16_t celsius);
    void get_output();
    void print_sensor_parameters();
    void print_heater_parameters();
    void print_PID_parameters();
    #if ENABLED(SUPPORT_AD8495) || ENABLED(SUPPORT_AD595)
      void print_AD595_parameters();
    #endif
    void start_idle_timer(const millis_t timeout_ms);
    void reset_idle_timer();

    #if HARDWARE_PWM
      void SetHardwarePwm();
    #endif

    FORCE_INLINE void updateCurrentTemperature() { this->current_temperature = this->sensor.getTemperature(); }
    FORCE_INLINE bool tempisrange() { return (WITHIN(this->current_temperature, this->data.mintemp, this->data.maxtemp)); }
    FORCE_INLINE bool isHeating()   { return this->target_temperature > this->current_temperature; }
    FORCE_INLINE bool isCooling()   { return this->target_temperature <= this->current_temperature; }

    FORCE_INLINE bool wait_for_heating() {
      return this->isActive() && ABS(this->current_temperature - this->target_temperature) > TEMP_HYSTERESIS;
    }

    #if WATCH_THE_HEATER
      void start_watching();
    #endif

    // Flag bit 0 Set Active
    FORCE_INLINE void setActive(const bool onoff) {
      if (!isFault() && sensor.type != 0 && onoff)
        data.flag.bit0 = true;
      else
        data.flag.bit0 = false;
    }
    FORCE_INLINE bool isActive() { return data.flag.bit0; }

    // Flag bit 1 Set use Pid
    FORCE_INLINE void setUsePid(const bool onoff) { data.flag.bit1 = onoff; }
    FORCE_INLINE bool isUsePid() { return data.flag.bit1; }

    // Flag bit 2 Set tuning
    FORCE_INLINE void setTuning(const bool onoff) { data.flag.bit2 = onoff; }
    FORCE_INLINE bool isTuning() { return data.flag.bit2; }

    // Flag bit 3 Set Hardware inverted
    FORCE_INLINE void setHWInverted(const bool onoff) { data.flag.bit3 = onoff; }
    FORCE_INLINE bool isHWInverted() { return data.flag.bit3; }

    // Flag bit 4 Set Hardware inverted
    FORCE_INLINE void setIdle(const bool onoff, const int16_t idle_temp=0) {
      data.flag.bit4 = onoff;
      idle_temperature = idle_temp;
    }
    FORCE_INLINE bool isIdle() { return data.flag.bit4; }

    // Flag bit 5 Set Hardware inverted
    FORCE_INLINE void setFault() {
      soft_pwm = 0;
      setActive(false);
      data.flag.bit5 = true;
    }
    FORCE_INLINE bool isFault() { return data.flag.bit5; }

    FORCE_INLINE void resetFlag() { data.flag._byte = 0; }

    FORCE_INLINE void SwitchOff() {
      target_temperature = 0;
      soft_pwm = 0;
      setActive(false);
    }

    FORCE_INLINE void ResetFault() {
      if (isFault()) {
        data.flag.bit5 = false;
        SwitchOff();
      }
    }

  private: /** Private Function */

};

#if HEATER_COUNT > 0
  extern Heater heaters[HEATER_COUNT];
#endif // HEATER_COUNT > 0

#endif /* _HEATER_H_ */
