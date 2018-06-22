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

#include "../../../MK4duo.h"

#if ENABLED(CNCROUTER)

  Cncrouter cnc;

  uint8_t Cncrouter::active_tool = 0;

  #if ENABLED(CNCROUTER_SLOWSTART) && ENABLED(FAST_PWM_CNCROUTER)
    uint32_t Cncrouter::rpm_target = 0;
    millis_t Cncrouter::next_speed_step = 0; 
  #endif

  #if ENABLED(FAST_PWM_CNCROUTER)
    uint32_t Cncrouter::rpm_instant = 0;
  #endif

  void Cncrouter::init() {
    SET_OUTPUT(CNCROUTER_PIN);
    #if ENABLED(FAST_PWM_CNCROUTER) && ENABLED(__AVR__)
      HAL::setPwmFrequency(CNCROUTER_PIN, 2); // No prescaling. Pwm frequency = F_CPU/256/64
    #endif
  }

  void Cncrouter::manage() {
    #if ENABLED(CNCROUTER_SLOWSTART) && ENABLED(FAST_PWM_CNCROUTER)
      if (rpm_target != rpm_instant) {
        millis_t ms = millis();
        if (ELAPSED(ms, next_speed_step)) {
          next_speed_step = ms + (CNCROUTER_SLOWSTART_INTERVAL * 1000L);
          speed_step();   
        }
      }
    #endif
  }

  void Cncrouter::tool_change(uint8_t tool_id, bool wait/*=true*/, bool raise_z/*=true*/) {

    #if !ENABLED(CNCROUTER_AUTO_TOOL_CHANGE)
      unsigned long saved_speed;
      float saved_z;
    #endif

    if (tool_id != active_tool) {

      if (wait) {
        SERIAL_STR(PAUSE);
        SERIAL_EOL();
      }

      planner.synchronize();

      #if !ENABLED(CNCROUTER_AUTO_TOOL_CHANGE)
        if (raise_z) {
          saved_speed = get_Speed();
          saved_z = mechanics.current_position[Z_AXIS];
          mechanics.do_blocking_move_to_z(CNCROUTER_SAFE_Z);
        }
      #endif

      disable_router();
      printer.safe_delay(300);

      if (wait) {
        // LCD click or M108 will clear this
        printer.setWaitForUser(true);

        printer.keepalive(PausedforUser);

        #if HAS_BUZZER
          millis_t next_buzz = millis();
        #endif

        while (printer.isWaitForUser()) {
          #if HAS_BUZZER
            if (millis() - next_buzz > 60000) {
              for (uint8_t i = 0; i < 3; i++) BUZZ(300, 1000);
              next_buzz = millis();
            }
          #endif
          printer.idle(true);
        } // while (printer.isWaitForUser())
      } // if (wait)

      if (tool_id != CNC_M6_TOOL_ID) active_tool = tool_id;
      #if !ENABLED(CNCROUTER_AUTO_TOOL_CHANGE)
        else cnc.setRouterSpeed(saved_speed);
        if (raise_z)
          mechanics.do_blocking_move_to_z(saved_z);
      #endif

      planner.synchronize();

      if (wait) {
        printer.keepalive(InHandler);

        SERIAL_STR(RESUME);
        SERIAL_EOL();
      }
    }
  }

  #if ENABLED(FAST_PWM_CNCROUTER)

    void Cncrouter::setPwm(uint8_t pwm) { HAL::analogWrite(CNCROUTER_PIN, pwm); }

    uint32_t Cncrouter::get_Speed() { return rpm_instant; }

    void Cncrouter::print_Speed() {
      SERIAL_MV(" CNC speed: ", get_Speed());
      SERIAL_EM(" rpm ");
    }

  #endif

  void Cncrouter::disable_router() {
    #if ENABLED(FAST_PWM_CNCROUTER)
      #if ENABLED(CNCROUTER_SLOWSTART)
        rpm_target = 0;
      #endif
      rpm_instant = 0;
      setPwm(0);
    #else
      WRITE_CNCROUTER(LOW);
    #endif
  }

  #if ENABLED(FAST_PWM_CNCROUTER)
    uint8_t Cncrouter::calcPWM(uint32_t rpm) {
      uint8_t pwm;
      pwm = rpm * 255 / MAX_CNCROUTER_SPEED;
      NOMORE(pwm, MAX_CNCROUTER_PWM_VAL);
      return pwm;
    }
  #endif

  #if ENABLED(FAST_PWM_CNCROUTER) && ENABLED(CNCROUTER_SLOWSTART)

    void Cncrouter::speed_step() {
      if (rpm_target < rpm_instant) {
        rpm_instant = rpm_target;
        setPwm(calcPWM(rpm_instant));
      }
      else if (rpm_target > rpm_instant) {
        rpm_instant = ((rpm_target - rpm_instant) > CNCROUTER_SLOWSTART_STEP) ? rpm_instant + CNCROUTER_SLOWSTART_STEP : rpm_target;  
        setPwm(calcPWM(rpm_instant));
      }
    }

  #endif

  // XXX TODO: support for CNCROUTER_ANTICLOCKWISE 
  void Cncrouter::setRouterSpeed(uint32_t rpm, bool clockwise/*=false*/) {
    #if ENABLED(FAST_PWM_CNCROUTER)
      NOMORE(rpm, MAX_CNCROUTER_SPEED);
      if (rpm > 0) NOLESS(rpm, MIN_CNCROUTER_SPEED);
      #if ENABLED(CNCROUTER_SLOWSTART)
        rpm_target = rpm;
        speed_step();
      #else
        rpm_instant = rpm;
        setPwm(calcPWM(rpm));
      #endif
    #else
      WRITE_CNCROUTER(!!rpm);  
    #endif
  }

#endif // CNCROUTER
