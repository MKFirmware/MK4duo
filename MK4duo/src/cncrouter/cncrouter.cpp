/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * cncrouter.cpp - CNC Router control for MK4duo  Version 1
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

#include "../../base.h"

void cnc_init() {

   #if ENABLED(CNCROUTER)
      SET_OUTPUT(CNCROUTER_PIN);
      #if ENABLED(FAST_PWM_CNCROUTER)
         setPwmFrequency(CNCROUTER_PIN, 2); // No prescaling. Pwm frequency = F_CPU/256/64
      #endif
   #endif

}


#if ENABLED(CNCROUTER)

#if ENABLED(FAST_PWM_CNCROUTER)

unsigned char fast_pwm_cncrouter;
void setPwmCNCRouter(unsigned char pwm); // XXX pwm level or cnc router speed?

void setPwmCNCRouter(unsigned char pwm) {
  fast_pwm_cncrouter = pwm;
  analogWrite(CNCROUTER_PIN, pwm);
}

int getCNCSpeed() {
   return fast_pwm_cncrouter;
}


#endif

void disable_cncrouter() {
   #if ENABLED(FAST_PWM_CNCROUTER)
      setPwmCNCRouter(0);
   #else
      WRITE_CNCROUTER(LOW);
   #endif
}

void setCNCRouterSpeed(unsigned long rpm, bool clockwise) {
   #if ENABLED(FAST_PWM_CNCROUTER)
     if(rpm > MAX_CNCROUTER_SPEED) rpm = MAX_CNCROUTER_SPEED;
     setPwmCNCRouter(rpm*255/MAX_CNCROUTER_SPEED);
   #else
     WRITE_CNCROUTER((rpm) ? 1 : 0);  
   #endif

}


#endif // CNCROUTER
