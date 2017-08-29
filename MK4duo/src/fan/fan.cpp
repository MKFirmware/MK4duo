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

  Fan fans[FAN_COUNT];

  /**
   * Initialize Fans
   */
  void fan_init() {
    constexpr Pin fan_channel[] = FANS_CHANNELS;
    LOOP_FAN() fans[f].init(fan_channel[f], FAN_INVERTED);
  }

  void Fan::init(Pin p_pin, const bool hwInverted) {

    this->pin               = p_pin;
    this->hardwareInverted  = hwInverted;
    this->pwm_hardware      = PWM_HARDWARE;

    HAL::pinMode(p_pin, OUTPUT);
  }

  void Fan::pause(const bool p) {

    if (p != this->paused) {
      this->paused = p;
      if (p) {
        this->paused_Speed = this->Speed;
        this->Speed = 0;
      }
      else
        this->Speed = this->paused_Speed;
    }
  }

  #if PWM_HARDWARE

    void Fan::SetHardwarePwm() {
      uint8_t pwm_val = 0;

      if (this->pwm_hardware && this->lastSpeed != this->Speed) {

        if (this->hardwareInverted)
          pwm_val = 255 - this->Speed;
        else
          pwm_val = this->Speed;

        this->pwm_hardware = HAL::analogWrite(this->pin, pwm_val, FAN_PWM_FREQ);

        this->lastSpeed = this->Speed;
      }
    }

  #endif

#endif // FAN_COUNT > 0
