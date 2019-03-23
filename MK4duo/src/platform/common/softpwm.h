/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * softpwm.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#define SOFTPWM_MAXCHANNELS (HEATER_COUNT + FAN_COUNT + 3)

typedef struct {
  // hardware I/O port and pin for this channel
  pin_t   pin;
  uint8_t pwm_value,
          pwm_pos;
} softPWMChannel;

class SoftPWM {

  public: /** Constructor */

    SoftPWM() { init(); }

  private: /** Private Parameters */

    static uint8_t used_channel;

    static softPWMChannel channels[SOFTPWM_MAXCHANNELS];

  public: /** Public Function */

    static void init();

    static void spin(void);

    static void set(const pin_t pin, const uint8_t value);

};

extern SoftPWM softpwm;

