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

#if ENABLED(CNCROUTER)

  void cnc_init() {
    SET_OUTPUT(CNCROUTER_PIN);
    #if ENABLED(FAST_PWM_CNCROUTER)
      HAL::setPwmFrequency(CNCROUTER_PIN, 2); // No prescaling. Pwm frequency = F_CPU/256/64
    #endif
  }

  #if ENABLED(CNCROUTER_SLOWSTART) && ENABLED(FAST_PWM_CNCROUTER)
    unsigned long rpm_target;
    static millis_t next_speed_step = 0; 
    void cncrouter_speed_step();
  #endif

  #if ENABLED(FAST_PWM_CNCROUTER)
    unsigned char cncrouter_calcPWM(unsigned long rpm);
    unsigned long rpm_instant = 0;
    void setPwmCNCRouter(unsigned char pwm); // XXX pwm level or cnc router speed?
  #endif

  void cnc_manage() {
    #if ENABLED(CNCROUTER_SLOWSTART) && ENABLED(FAST_PWM_CNCROUTER)
      if (rpm_target != rpm_instant) {
        millis_t ms = millis();
        if (ELAPSED(ms, next_speed_step)) {
          next_speed_step = ms + (CNCROUTER_SLOWSTART_INTERVAL * 1000L);
          cncrouter_speed_step();   
        }
      }
    #endif
  }

  #if ENABLED(FAST_PWM_CNCROUTER)

    void setPwmCNCRouter(unsigned char pwm) {
      analogWrite(CNCROUTER_PIN, pwm);
    }

    unsigned long getCNCSpeed() {
      return rpm_instant;
    }

    void print_cncspeed() {
      SERIAL_MV(" CNC speed: ", getCNCSpeed());
      SERIAL_MSG(" rpm ");
    }

  #endif

  void disable_cncrouter() {
    #if ENABLED(FAST_PWM_CNCROUTER)
      #if ENABLED(CNCROUTER_SLOWSTART)
        rpm_target = 0;
      #endif
      rpm_instant = 0;
      setPwmCNCRouter(0);
    #else
      WRITE_CNCROUTER(LOW);
    #endif
  }

  #if ENABLED(FAST_PWM_CNCROUTER)
    unsigned char cncrouter_calcPWM(unsigned long rpm) {
      unsigned char pwm;
      pwm = rpm * 255 / MAX_CNCROUTER_SPEED;
      NOMORE(pwm, MAX_CNCROUTER_PWM_VAL);
      return pwm;
    }
  #endif

  #if ENABLED(FAST_PWM_CNCROUTER) && ENABLED(CNCROUTER_SLOWSTART)

    void cncrouter_speed_step() {
      if (rpm_target < rpm_instant) {
        rpm_instant = rpm_target;
        setPwmCNCRouter(cncrouter_calcPWM(rpm_instant));
      }
      else if (rpm_target > rpm_instant) {
        rpm_instant = ((rpm_target - rpm_instant) > CNCROUTER_SLOWSTART_STEP) ?  rpm_instant + CNCROUTER_SLOWSTART_STEP : rpm_target;  
        setPwmCNCRouter(cncrouter_calcPWM(rpm_instant));
      }
    }

  #endif

  // XXX TODO: support for CNCROUTER_ANTICLOCKWISE 
  void setCNCRouterSpeed(unsigned long rpm, bool clockwise/*=false*/) {
    #if ENABLED(FAST_PWM_CNCROUTER)
      NOMORE(rpm, MAX_CNCROUTER_SPEED);
      if (rpm > 0) NOLESS(rpm, MIN_CNCROUTER_SPEED);
      #if ENABLED(CNCROUTER_SLOWSTART)
        rpm_target = rpm;
        cncrouter_speed_step();
      #else
        rpm_instant = rpm;
        setPwmCNCRouter(cncrouter_calcPWM(rpm));
      #endif
    #else
      WRITE_CNCROUTER(!!rpm);  
    #endif
  }

#endif // CNCROUTER
