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

#ifndef _FAN_H_
#define _FAN_H_

#if FAN_COUNT > 0

  class Fan {

    public: /** Constructor */

      Fan() {};

    public: /** Public Parameters */

      uint16_t  Speed;
      uint8_t   paused_Speed,
                Kickstart,
                pwm_val,
                pwm_pos;
      bool      pwm_hardware,
                hardwareInverted,
                paused;
      Pin       pin;

    public: /** Public Function */

      void init(Pin p_pin, const bool hwInverted, const bool hardwarepwm=false);
      void setSpeed(const uint8_t speed);
      void pause(const bool p);

    private: /** Private Parameters */

      int16_t lastSpeed;

    private: /** Private Function */

  };

  extern Fan fans[FAN_COUNT];

#endif // FAN_COUNT > 0

#endif /* _FAN_H_ */
