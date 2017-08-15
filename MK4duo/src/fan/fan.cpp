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
 * fan.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"

#if FAN_COUNT > 0

  Fan fans;

  uint16_t  Fan::Speed[FAN_COUNT] = { 0 },
            Fan::paused_Speed[FAN_COUNT] = { 0 };
  uint8_t   Fan::Kickstart[FAN_COUNT] = { 0 },
            Fan::pwm_val[FAN_COUNT]  = { 0 },
            Fan::pwm_pos[FAN_COUNT] = { 0 };
  bool      Fan::pwm_hardware[FAN_COUNT]  = { false },
            Fan::hardwareInverted[FAN_COUNT] = { false },
            Fan::paused = false;
  Pin       Fan::pin[FAN_COUNT] = { -1 };
  int16_t   Fan::lastSpeed[FAN_COUNT] = { -1 };

  void Fan::init(const uint8_t fan, Pin p_pin, const bool hwInverted, const bool hardwarepwm/*=false*/) {
    Speed[fan]             = 0;
    paused_Speed[fan]      = 0;
    Kickstart[fan]         = 0;
    pwm_val[fan]           = 0;
    pwm_pos[fan]           = 0;
    pwm_hardware[fan]      = hardwarepwm;
    hardwareInverted[fan]  = hwInverted;
    pin[fan]               = p_pin;
    lastSpeed[fan]         = -1;

    HAL::pinMode(p_pin, OUTPUT);
  }

  void Fan::pause(const uint8_t fan, const bool p) {

    if (p != paused) {
      paused = p;
      if (p) {
        paused_Speed[fan] = Speed[fan];
        Speed[fan] = 0;
      }
      else
        Speed[fan] = paused_Speed[fan];
    }
  }

  #if ENABLED(PWM_HARDWARE)

    void Fan::SetHardwarePwm() {

      LOOP_FAN() {
        if (pwm_hardware[f] && lastSpeed[f] != Speed[f]) {

          if (hardwareInverted[f])
            pwm_val[f] = 255 - Speed[f];
          else
            pwm_val[f] = Speed[f];

          pwm_hardware[f] = HAL::analogWrite(fans.pin[f], fans.pwm_val[f], FAN_PWM_FREQ);

          lastSpeed[f] = Speed[f];

        }
      }
    }

  #endif

#endif // FAN_COUNT > 0
