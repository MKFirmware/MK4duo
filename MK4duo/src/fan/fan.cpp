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

    Speed               = 0;
    paused_Speed        = 0;
    Kickstart           = 0;
    pwm_pos             = 0;
    lastpwm             = -1;
    paused              = false;
    triggerTemperatures = (HOTEND_AUTO_FAN_TEMPERATURE);

    #if ENABLED(__AVR__)
      if (pin > NoPin) HAL::pinMode(pin, OUTPUT);
    #endif

  }

  void Fan::SetAutoMonitored(const int8_t h) {
    if (WITHIN(h, 0, HOTENDS -1) || h == 7)
      SBI(autoMonitored, (unsigned int)h);
    else      
      autoMonitored = 0;
    Check();
  }

  void Fan::Check() {
    static millis_t next_auto_fan_check_ms  = 0,
                    lastMotorOn             = 0;

    millis_t ms = millis();

    if (autoMonitored == 0) return;

    if (ELAPSED(ms, next_auto_fan_check_ms)) {
      // Check for Hotend temperature
      LOOP_HOTEND() {
        if (TEST(autoMonitored, h))
          Speed = ((int)heaters[h].current_temperature > triggerTemperatures) ? HOTEND_AUTO_FAN_SPEED : HOTEND_AUTO_FAN_MIN_SPEED;
      }

      // Check for Controller fan
      if (TEST(autoMonitored, 7)) {
        if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON
          || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
          #if EXTRUDERS > 1
            || E1_ENABLE_READ == E_ENABLE_ON
            #if HAS_X2_ENABLE
              || X2_ENABLE_READ == X_ENABLE_ON
            #endif
            #if EXTRUDERS > 2
              || E2_ENABLE_READ == E_ENABLE_ON
              #if EXTRUDERS > 3
                || E3_ENABLE_READ == E_ENABLE_ON
                #if EXTRUDERS > 4
                  || E4_ENABLE_READ == E_ENABLE_ON
                  #if EXTRUDERS > 5
                    || E5_ENABLE_READ == E_ENABLE_ON
                  #endif
                #endif
              #endif
            #endif
          #endif
        ) {
          lastMotorOn = ms; // set time to NOW so the fan will turn on
        }

        // Fan off if no steppers have been enabled for CONTROLLERFAN_SECS seconds
        Speed = (!lastMotorOn || ELAPSED(ms, lastMotorOn + (CONTROLLERFAN_SECS) * 1000UL)) ? CONTROLLERFAN_MIN_SPEED : CONTROLLERFAN_SPEED;
      }

      next_auto_fan_check_ms = ms + 2500UL; // Not a time critical function, so only check every 2.5s
    }
  }

  void Fan::pause(const bool p) {

    if (p != paused) {
      paused = p;
      if (p) {
        paused_Speed = Speed;
        Speed = 0;
      }
      else
        Speed = paused_Speed;
    }
  }

  #if HARDWARE_PWM
    void Fan::SetHardwarePwm() {
      if (pin > NoPin) {
        if (hardwareInverted)
          pwm_pos = 255 - Speed;
        else
          pwm_pos = Speed;

        if (pwm_pos != lastpwm) {
          lastpwm = pwm_pos;
          HAL::analogWrite(pin, pwm_pos, freq);
        }
      }
    }
  #endif

#endif // FAN_COUNT > 0
