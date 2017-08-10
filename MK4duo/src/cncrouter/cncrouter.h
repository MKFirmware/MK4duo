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
 * cncrouter.h - CNC Router control for MK4duo  Version 1
 * Copyright (c) 2017 Franco (nextime) Lanza
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _CNCROUTER_H_
#define _CNCROUTER_H_

#if ENABLED(CNCROUTER)

  class Cncrouter {

    public: /** Constructor */

      Cncrouter() {};

    public: /** Public Parameters */

    public: /** Public Function */

      static void init();    // initialize cnc router
      static void manage();  // management loop for CNC

      #if ENABLED(FAST_PWM_CNCROUTER)
        static uint32_t get_Speed();
        static void print_Speed();
      #else
        #if ENABLED(INVERTED_CNCROUTER_PIN)
          #define get_Speed() READ(CNCROUTER_PIN)
        #else
          #define get_Speed() !READ(CNCROUTER_PIN)
        #endif // INVERTED_CNCROUTER_PIN
      #endif // FAST_PWM_CNCROUTER

      static void setRouterSpeed(uint32_t rpm, bool clockwise=false);
      static void disable_router();

    private: /** Private Parameters */

      #if ENABLED(CNCROUTER_SLOWSTART) && ENABLED(FAST_PWM_CNCROUTER)
        static uint32_t rpm_target;
        static millis_t next_speed_step; 
      #endif

      #if ENABLED(FAST_PWM_CNCROUTER)
        static uint32_t rpm_instant;
      #endif

    private: /** Private Function */

      static void speed_step();
      static uint8_t calcPWM(uint32_t rpm);
      static void setPwm(uint8_t pwm);

  };

  extern Cncrouter cnc;

#endif // CNCROUTER

#endif /* _CNCROUTER_H_ */
