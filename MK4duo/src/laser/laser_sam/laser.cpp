/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#include "../../../base.h"

#if ENABLED(LASERBEAM) && ENABLED(ARDUINO_ARCH_SAM)

  laser_t laser;

  void laser_init() {

    #if LASER_CONTROL == 1
      HAL_laser_init_pwm(LASER_PWR_PIN, LASER_PWM);
    #elif LASER_CONTROL == 2
      HAL_laser_init_pwm(LASER_TTL_PIN, LASER_PWM);
    #endif

    #if ENABLED(LASER_PERIPHERALS)
      OUT_WRITE(LASER_PERIPHERALS_PIN, HIGH); // Laser peripherals are active LOW, so preset the pin
      PULLUP(LASER_PERIPHERALS_STATUS_PIN);   // Set the peripherals status pin to pull-up.
    #endif // LASER_PERIPHERALS

    #if LASER_CONTROL == 2
      OUT_WRITE(LASER_PWR_PIN, LASER_UNARM);  // Laser FIRING is active LOW, so preset the pin
    #endif

    // initialize state to some sane defaults
    laser.intensity = 50.0;
    laser.ppm = 0.0;
    laser.duration = 0;
    laser.status = LASER_OFF;
    laser.firing = LASER_OFF;
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
      laser_extinguish();
    #endif
  }

  void laser_fire(float intensity = 100.0) { // Fire with range 0-100
    laser.firing = LASER_ON;
    laser.last_firing = micros(); // microseconds of last laser firing
    if (intensity > 100.0) intensity = 100.0; // restrict intensity between 0 and 100
    if (intensity < 0) intensity = 0;

    HAL_laser_intensity(LASER_PWM_MAX_DUTY_CYCLE * intensity / 100.0); // Range 0-255

    #if LASER_CONTROL == 2
      digitalWrite(LASER_PWR_PIN, LASER_ARM);
    #endif

    if (laser.diagnostics)
      SERIAL_EM("Laser fired");
  }

  void laser_fire_byte(uint8_t intensity) { // Fire with byte-range 0-255
    laser.firing = LASER_ON;
    laser.last_firing = micros(); // microseconds of last laser firing

    HAL_laser_intensity(LASER_PWM_MAX_DUTY_CYCLE * intensity / 255.0); // Range 0-255

    #if LASER_CONTROL == 2
      digitalWrite(LASER_PWR_PIN, LASER_ARM);
    #endif

    if (laser.diagnostics)
      SERIAL_EM("Laser_byte fired");
  }

  void laser_extinguish() {
    if (laser.firing == LASER_ON) {
      laser.firing = LASER_OFF;

      if (laser.diagnostics)
        SERIAL_EM("Laser being extinguished");

      HAL_laser_intensity(0);

      #if LASER_CONTROL == 2
        digitalWrite(LASER_PWR_PIN, LASER_UNARM);
      #endif

      laser.time += millis() - (laser.last_firing / 1000);

      if (laser.diagnostics)
        SERIAL_EM("Laser extinguished");
    }
  }

  void laser_set_mode(int mode) {
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

  void laser_diagnose() {
    if (!laser.diagnostics)
      return;

    SERIAL_LMV(CFG, "Intensity ", laser.intensity);
    SERIAL_LMV(CFG, "Duration  ", laser.duration);
    SERIAL_LMV(CFG, "Ppm       ", laser.ppm);
    SERIAL_LMV(CFG, "Mode      ", laser.mode);
    SERIAL_LMV(CFG, "Status    ", laser.status);
    SERIAL_LMV(CFG, "Fired     ", laser.fired);
    HAL::serialFlush();
  }

  #if ENABLED(LASER_PERIPHERALS)
    bool laser_peripherals_ok() { return !digitalRead(LASER_PERIPHERALS_STATUS_PIN); }

    void laser_peripherals_on() {
      digitalWrite(LASER_PERIPHERALS_PIN, LOW);
      if (laser.diagnostics)
        SERIAL_EM("Laser Peripherals Enabled");
    }

    void laser_peripherals_off() {
      if (!digitalRead(LASER_PERIPHERALS_STATUS_PIN)) {
        digitalWrite(LASER_PERIPHERALS_PIN, HIGH);
        if (laser.diagnostics)
          SERIAL_EM("Laser Peripherals Disabled");
      }
    }

    void laser_wait_for_peripherals() {
      unsigned long timeout = millis() + LASER_PERIPHERALS_TIMEOUT;
      if (laser.diagnostics)
        SERIAL_EM("Waiting for peripheral control board signal...");

      while(!laser_peripherals_ok()) {
        if (millis() > timeout) {
          if (laser.diagnostics)
            SERIAL_LM(ER, "Peripheral control board failed to respond");

          Stop();
          break;
        }
      }
    }
  #endif // LASER_PERIPHERALS

#endif // LASERBEAM
