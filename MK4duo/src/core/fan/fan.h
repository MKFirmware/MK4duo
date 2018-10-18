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
 * fan.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#pragma once

#if ENABLED(TACHOMETRIC)
  #include "tachometric.h"
#endif

// Struct Fan data
typedef struct {

  pin_t       pin;

  flagbyte_t  flag;

  uint8_t     ID,
              min_Speed,
              autoMonitored;

  uint16_t    triggerTemperature,
              freq;

} fan_data_t;

class Fan {

  public: /** Constructor */

    Fan() {}

  public: /** Public Parameters */

    fan_data_t  data;

    #if ENABLED(TACHOMETRIC)
      tacho_data_t tacho;
    #endif

    uint8_t     Speed,
                paused_Speed,
                Kickstart,
                pwm_pos;

    int16_t     lastpwm;

  public: /** Public Function */

    void init();
    void setAutoMonitored(const int8_t h);
    void spin();
    void print_parameters();

    // Fan flag bit 0 Hardware inverted
    FORCE_INLINE void setHWInverted(const bool onoff) {
      data.flag.bit0 = onoff;
    }
    FORCE_INLINE bool isHWInverted() { return data.flag.bit0; }

    // Fan flag bit 1 Idle
    FORCE_INLINE void setIdle(const bool onoff) {
      if (onoff != isIdle()) {
        data.flag.bit1 = onoff;
        if (onoff) {
          paused_Speed = Speed;
          Speed = 0;
        }
        else
          Speed = paused_Speed;
      }
    }
    FORCE_INLINE bool isIdle() { return data.flag.bit1; }

    #if HARDWARE_PWM
      void SetHardwarePwm();
    #endif

};

#if FAN_COUNT > 0
  extern Fan fans[FAN_COUNT];
#endif // FAN_COUNT > 0
