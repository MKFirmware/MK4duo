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

#ifndef _HEATER_H
#define _HEATER_H

#if HOTENDS <= 1
  #define HOTEND_INDEX      0
  #define EXTRUDER_IDX      0
  #define TRG_EXTRUDER_IDX  0
#else
  #define HOTEND_INDEX      h
  #define EXTRUDER_IDX      tools.active_extruder
  #define TRG_EXTRUDER_IDX  tools.target_extruder
#endif

typedef enum {
  IS_HOTEND   = 0,
  IS_BED      = 1,
  IS_CHAMBER  = 2,
  IS_COOLER   = 3
} Heater_type;

#if HEATER_COUNT > 0

  class Heater {

    public: /** Public Parameters */

      Heater_type type;
      Pin         output_pin,
                  sensor_pin;
      int16_t     sensor_type;
      uint8_t     soft_pwm,
                  pwm_pos,
                  pid_min,
                  pid_max;
      int16_t     current_temperature_raw,
                  target_temperature,
                  mintemp,
                  maxtemp;
      float       current_temperature,
                  Kp,
                  Ki,
                  Kd,
                  Kc;
      bool        use_pid,
                  pwm_hardware,
                  hardwareInverted;

      #if HEATER_USES_AD595
        float     ad595_offset,
                  ad595_gain;
      #endif

      #if WATCH_THE_HEATER
        uint16_t  watch_target_temp;
        millis_t  watch_next_ms;
      #endif

    public: /** Public Function */

      void init();

      #if PWM_HARDWARE
        void SetHardwarePwm();
      #endif

      void setTarget(int16_t celsius);

      bool isON()         { return this->sensor_type != 0; }
      bool tempisrange()  { return (WITHIN(this->current_temperature, this->mintemp, this->maxtemp)); }
      bool isHeating()    { return this->target_temperature > this->current_temperature; }
      bool isCooling()    { return this->target_temperature < this->current_temperature; }

    private: /** Private Parameters */

    private: /** Private Function */

  };

  extern Heater heaters[HEATER_COUNT];

#endif // HEATER_COUNT > 0

#endif /* _HEATER_H */
