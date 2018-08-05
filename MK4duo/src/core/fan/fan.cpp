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

#include "../../../MK4duo.h"

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
    triggerTemperatures = (HOTEND_AUTO_FAN_TEMPERATURE);

    setIdle(false);

    if (printer.isRunning()) return; // All running not reinitialize

    if (pin > 0) HAL::pinMode(pin, isHWInverted() ? OUTPUT_HIGH : OUTPUT_LOW);

  }

  void Fan::setAutoMonitored(const int8_t h) {
    if (WITHIN(h, 0, HOTENDS - 1) || h == 7)
      SBI(autoMonitored, (uint8_t)h);
    else      
      autoMonitored = 0;
    spin();
  }

  void Fan::spin() {

    static watch_t controller_fan_watch(CONTROLLERFAN_SECS * 1000UL);

    if (autoMonitored == 0) return;

    // Check for Hotend temperature
    LOOP_HOTEND() {
      if (TEST(autoMonitored, h)) {
        if (heaters[h].current_temperature > triggerTemperatures) {
          Speed = HOTEND_AUTO_FAN_SPEED;
          break;
        }
        else
          Speed = HOTEND_AUTO_FAN_MIN_SPEED;
      }
    }

    // Check for Controller fan
    if (TEST(autoMonitored, 7)) {

      // Check Heaters
      if (thermalManager.heaters_isActive()) controller_fan_watch.start();

      // Check Motors
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON
        || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
        #if DRIVER_EXTRUDERS > 1
          || E1_ENABLE_READ == E_ENABLE_ON
          #if HAS_X2_ENABLE
            || X2_ENABLE_READ == X_ENABLE_ON
          #endif
          #if DRIVER_EXTRUDERS > 2
            || E2_ENABLE_READ == E_ENABLE_ON
            #if DRIVER_EXTRUDERS > 3
              || E3_ENABLE_READ == E_ENABLE_ON
              #if DRIVER_EXTRUDERS > 4
                || E4_ENABLE_READ == E_ENABLE_ON
                #if DRIVER_EXTRUDERS > 5
                  || E5_ENABLE_READ == E_ENABLE_ON
                #endif
              #endif
            #endif
          #endif
        #endif
      ) {
        controller_fan_watch.start();
      }

      // Fan off if no steppers or heaters have been enabled for CONTROLLERFAN_SECS seconds
      Speed = controller_fan_watch.elapsed() ? CONTROLLERFAN_MIN_SPEED : CONTROLLERFAN_SPEED;
    }
  }

  void Fan::print_parameters() {
    SERIAL_LM(CFG, "Fans: P<Fan> U<Pin> L<Min Speed> F<Freq> I<Hardware Inverted 0-1> H<Auto mode> :");
    SERIAL_SMV(CFG, "  M106 P", (int)ID);
    SERIAL_MV(" U", pin);
    SERIAL_MV(" L", min_Speed);
    SERIAL_MV(" F", freq);
    SERIAL_MV(" I", isHWInverted());
    LOOP_HOTEND() if (TEST(autoMonitored, h)) SERIAL_MV(" H", (int)h);
    if (TEST(autoMonitored, 7)) SERIAL_MSG(" H7");
    SERIAL_EOL();
  }

  #if HARDWARE_PWM
    void Fan::SetHardwarePwm() {
      if (pin > NoPin) {
        if (isHWInverted())
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
