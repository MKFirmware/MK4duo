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

  void Fan::init(Pin p_pin, const bool hwInverted, const bool hardwarepwm/*=false*/) {
    this->Speed             = 0;
    this->paused_Speed      = 0;
    this->Kickstart         = 0;
    this->pwm_val           = 0;
    this->pwm_pos           = 0;
    this->pwm_hardware      = hardwarepwm;
    this->hardwareInverted  = hwInverted;
    this->paused            = false;
    this->pin               = p_pin;
    this->lastSpeed         = -1;

    HAL::pinMode(p_pin, OUTPUT);
  }

  void Fan::setSpeed(const uint8_t speed) {

    if (this->lastSpeed != speed) {

      this->Speed = speed;

      if (this->hardwareInverted)
        this->pwm_val = 255 - speed;
      else
        this->pwm_val = speed;

      this->lastSpeed = speed;
    }
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

#endif // FAN_COUNT > 0
