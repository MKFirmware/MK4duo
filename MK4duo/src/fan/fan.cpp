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

#include "../../MK4duo.h"

#if FAN_COUNT > 0

  Fan fans[FAN_COUNT];

  /**
   * Initialize Fans
   */
  void Fan::init() {

    this->Speed             = 0;
    this->paused_Speed      = 0;
    this->Kickstart         = 0;
    this->pwm_pos           = 0;
    this->paused            = false;
    this->lastpwm           = -1;

    if (this->pin > NoPin)
      HAL::pinMode(this->pin, OUTPUT);
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

  #if HARDWARE_PWM
    void Fan::SetHardwarePwm() {
      if (this->pin > NoPin) {
        if (this->hardwareInverted)
          this->pwm_pos = 255 - this->Speed;
        else
          this->pwm_pos = this->Speed;

        if (this->pwm_pos != this->lastpwm) {
          this->lastpwm = this->pwm_pos;
          HAL::analogWrite(this->pin, this->pwm_pos, this->freq);
        }
      }
    }
  #endif

#endif // FAN_COUNT > 0
