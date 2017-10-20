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
 * laser.cpp - Laser control library for Arduino using 16 bit timers- Version 1
 * Copyright (c) 2013 Timothy Schmidt.  All right reserved.
 * Copyright (c) 2016 Franco (nextime) Lanza
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

#if ENABLED(LASER) && ENABLED(ARDUINO_ARCH_SAM)

  Laser laser;

  void Laser::Init() {

    #if LASER_CONTROL == 1
      HAL::analogWrite(LASER_PWR_PIN, 0, LASER_PWM);
    #elif LASER_CONTROL == 2
      HAL::analogWrite(LASER_PWM_PIN, 0, LASER_PWM);
    #endif

    #if ENABLED(LASER_PERIPHERALS)
      OUT_WRITE(LASER_PERIPHERALS_PIN, HIGH); // Laser peripherals are active LOW, so preset the pin
      SET_INPUT_PULLUP(LASER_PERIPHERALS_STATUS_PIN);   // Set the peripherals status pin to pull-up.
    #endif // LASER_PERIPHERALS

    #if LASER_CONTROL == 2
      OUT_WRITE(LASER_PWR_PIN, LASER_UNARM);  // Laser FIRING is active LOW, so preset the pin
    #endif

    // initialize state to some sane defaults
    laser.intensity = 100.0;
    laser.ppm = 0.0;
    laser.duration = 0;
    laser.status = LASER_OFF;
    laser.firing = LASER_ON;
    laser.mode = CONTINUOUS;
    laser.last_firing = 0;
    laser.diagnostics = false;
    laser.time = 0;

    #if ENABLED(LASER_RASTER)
      laser.raster_aspect_ratio = LASER_RASTER_ASPECT_RATIO;
      laser.raster_mm_per_pulse = LASER_RASTER_MM_PER_PULSE;
      laser.raster_direction = 1;
    #endif // LASER_RASTER
    
    #if DISABLED(LASER_PULSE_METHOD)
      laser.extinguish();
    #endif
  }

  void Laser::fire(float intensity/*=100.0*/) {

    laser.firing = LASER_ON;
    laser.last_firing = micros(); // microseconds of last laser firing

    // restrict intensity between 0 and 100
    NOMORE(intensity, 100.0);
    NOLESS(intensity, 0.0);

    #if ENABLED(LASER_PWM_INVERT)
      intensity = 100 - intensity;
    #endif

    #if LASER_CONTROL == 1
      HAL::analogWrite(LASER_PWR_PIN, (255 * intensity * 0.01), LASER_PWM); // Range 0-255
    #elif LASER_CONTROL == 2
      HAL::analogWrite(LASER_PWM_PIN, (255 * intensity * 0.01), LASER_PWM); // Range 0-255
      HAL::digitalWrite(LASER_PWR_PIN, LASER_ARM);
    #endif

    if (laser.diagnostics) SERIAL_EM("Laser fired");
  }

  void Laser::extinguish() {
    if (laser.firing == LASER_ON) {
      laser.firing = LASER_OFF;

      if (laser.diagnostics) SERIAL_EM("Laser being extinguished");

      #if LASER_CONTROL == 1
        #if ENABLED(LASER_PWM_INVERT)
          HAL::analogWrite(LASER_PWR_PIN, 255, LASER_PWM);
        #else
          HAL::analogWrite(LASER_PWR_PIN, 0, LASER_PWM);
        #endif
      #elif LASER_CONTROL == 2
        #if ENABLED(LASER_PWM_INVERT)
          HAL::analogWrite(LASER_PWM_PIN, 255, LASER_PWM);
        #else
          HAL::analogWrite(LASER_PWM_PIN, 0, LASER_PWM);
        #endif
        HAL::digitalWrite(LASER_PWR_PIN, LASER_UNARM);
      #endif

      laser.time += millis() - (laser.last_firing / 1000);

      if (laser.diagnostics)
        SERIAL_EM("Laser extinguished");
    }
  }

  void Laser::set_mode(uint8_t mode) {
    switch(mode) {
      case 0:
        laser.mode = CONTINUOUS;
        return;
      case 1:
        laser.mode = PULSED;
        return;
      case 2:
        laser.mode = RASTER;
        return;
    }
  }

  #if ENABLED(LASER_PERIPHERALS)
    bool Laser::peripherals_ok() { return !HAL::digitalRead(LASER_PERIPHERALS_STATUS_PIN); }

    void Laser::peripherals_on() {
      HAL::digitalWrite(LASER_PERIPHERALS_PIN, LOW);
      if (laser.diagnostics)
        SERIAL_EM("Laser Peripherals Enabled");
    }

    void Laser::peripherals_off() {
      if (!peripherals_ok()) {
        HAL::digitalWrite(LASER_PERIPHERALS_PIN, HIGH);
        if (laser.diagnostics)
          SERIAL_EM("Laser Peripherals Disabled");
      }
    }

    void Laser::wait_for_peripherals() {
      unsigned long timeout = millis() + LASER_PERIPHERALS_TIMEOUT;
      if (laser.diagnostics)
        SERIAL_EM("Waiting for peripheral control board signal...");

      while(!peripherals_ok()) {
        if (millis() > timeout) {
          if (laser.diagnostics)
            SERIAL_LM(ER, "Peripheral control board failed to respond");

          printer.Stop();
          break;
        }
      }
    }
  #endif // LASER_PERIPHERALS

#endif // ENABLED(LASER) && ENABLED(ARDUINO_ARCH_SAM)
