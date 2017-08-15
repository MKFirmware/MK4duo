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

      static uint16_t Speed[FAN_COUNT],
                      paused_Speed[FAN_COUNT];
      static uint8_t  Kickstart[FAN_COUNT],
                      pwm_val[FAN_COUNT],
                      pwm_pos[FAN_COUNT];
      static bool     pwm_hardware[FAN_COUNT],
                      hardwareInverted[FAN_COUNT],
                      paused;
      static Pin      pin[FAN_COUNT];

    public: /** Public Function */

      static void init(const uint8_t fan, Pin p_pin, const bool hwInverted, const bool hardwarepwm=false);
      static void pause(const uint8_t fan, const bool p);

      #if ENABLED(PWM_HARDWARE)
        static void SetHardwarePwm();
      #endif

    private: /** Private Parameters */

      static int16_t lastSpeed[FAN_COUNT];

    private: /** Private Function */

  };

  extern Fan fans;

#endif // FAN_COUNT > 0

#endif /* _FAN_H_ */
