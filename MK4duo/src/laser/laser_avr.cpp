/**
 * MK & MK4due 3D Printer Firmware
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

#include "../../base.h"

#if ENABLED(LASER) && ENABLED(__AVR__)

  #include <Arduino.h>
  #include <avr/interrupt.h>

  #define PWM_PULSE_FREQUENCY   (F_CPU / LASER_PWM)
  #define PWM_MAPPED_INTENSITY  labs((intensity / 100.0) * (PWM_PULSE_FREQUENCY))

  #if ENABLED(LASER_PWM_INVERT)
    #define OCR3_INIT      ICR3
    #define OCR4_INIT      ICR4
    #define OCR_EXTINGUISH labs(PWM_PULSE_FREQUENCY)
  #else
    #define OCR3_INIT      0
    #define OCR4_INIT      0
    #define OCR_EXTINGUISH 0
  #endif

  #define IS_TIMER_3_PWR (LASER_PWR_PIN == 2 || LASER_PWR_PIN == 3 || LASER_PWR_PIN == 5)
  #define IS_TIMER_4_PWR (LASER_PWR_PIN == 6 || LASER_PWR_PIN == 7 || LASER_PWR_PIN == 8)

  #define IS_TIMER_3_PWM (LASER_PWM_PIN == 2 || LASER_PWM_PIN == 3 || LASER_PWM_PIN == 5)
  #define IS_TIMER_4_PWM (LASER_PWM_PIN == 6 || LASER_PWM_PIN == 7 || LASER_PWM_PIN == 8)

  Laser laser;

  void Laser::timer3_init(Pin pin) {

    HAL::pinMode(pin, OUTPUT);
    HAL::digitalWrite(pin, HIGH);

    // Setup timer3 to fast PWM with OC w/ ICR3 as TOP
    noInterrupts();

    TCCR3A = 0x00;
    TCCR3B = 0x00;                  // stop Timer3 clock for register updates
    ICR3 = labs(PWM_PULSE_FREQUENCY); // set clock cycles per PWM pulse (OC Top value)

    if (pin == 2) {
      TCCR3A = _BV(COM3B1) | _BV(COM3B0) | _BV(WGM31); // Fast PWM (WGM31) / (Clear OC3B/pin 2 on compare match (set output to low level)
      OCR3B = OCR3_INIT;
    }
    else if (pin == 3) {
      TCCR3A = _BV(COM3C1) | _BV(COM3C0) | _BV(WGM31); // Fast PWM (WGM31) / Clear OC3C/pin 3 on compare match (set output to low level)
      OCR3C = OCR3_INIT;
    }
    else if (pin == 5) {
      TCCR3A = _BV(COM3A1) | _BV(COM3A0) | _BV(WGM31); // Fast PWM (WGM31) / Clear OC3A/pin 5 on compare match (set output to low level)
      OCR3A = OCR3_INIT;
    }

    TCCR3B = _BV(CS30) | _BV(WGM33) |  _BV(WGM32); // Fast PWM / clkIo/1 (No prescaling) 

    interrupts();
  }

  void Laser::timer4_init(Pin pin) {

    HAL::pinMode(pin, OUTPUT);
    HAL::digitalWrite(pin, HIGH);

    // Setup timer4 to fast PWM with OC w/ ICR4 as TOP
    noInterrupts();

    TCCR4A = 0x00;
    TCCR4B = 0x00;  // stop Timer4 clock for register updates
    TCCR4C = 0x00;

    ICR4 = labs(PWM_PULSE_FREQUENCY); // set clock cycles per PWM pulse (OC Top value)

    if (pin == 6) {
      TCCR4A = _BV(COM4A1) |  _BV(COM4A0) | _BV(WGM41); // Fast PWM (WGM41) / Clear OC4A/pin 5 on compare match (set output to low level)
      OCR4A = OCR4_INIT;
    }
    else if (pin == 7) {
      TCCR4A = _BV(COM4B1) | _BV(COM4B0) | _BV(WGM41); // Fast PWM (WGM41) / (Clear OC4B/pin 2 on compare match (set output to low level)
      OCR4B = OCR4_INIT;
    }
    else if (pin == 8) {
      TCCR4A = _BV(COM4C1) | _BV(COM4C0) | _BV(WGM41); // Fast PWM (WGM41) / Clear OC4C/pin 4 on compare match (set output to low level)
      OCR4C = OCR4_INIT;
    }

    TCCR4B = _BV(CS40) | _BV(WGM43) |  _BV(WGM42); // Fast PWM / clkIo/1 (No prescaling) 

    interrupts();
    
  }

  void Laser::Init() {

    // Initialize timers for laser intensity control
    #if LASER_CONTROL == 1

      #if IS_TIMER_3_PWR
        timer3_init(LASER_PWR_PIN);
      #elif IS_TIMER_4_PWR
        timer4_init(LASER_PWR_PIN);
      #endif

    #elif LASER_CONTROL == 2

      #if IS_TIMER_3_PWM
        timer3_init(LASER_PWM_PIN);
      #elif IS_TIMER_4_PWM
        timer4_init(LASER_PWM_PIN);
      #endif

    #endif

    #if ENABLED(LASER_PERIPHERALS)
      OUT_WRITE(LASER_PERIPHERALS_PIN, HIGH);         // Laser peripherals are active LOW, so preset the pin
      OUT_WRITE(LASER_PERIPHERALS_STATUS_PIN, HIGH);  // Set the peripherals status pin to pull-up.
    #endif

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

    laser.extinguish();

  }

  void Laser::fire(float intensity/*=100.0*/){

    laser.firing = LASER_ON;
    laser.last_firing = micros(); // microseconds of last laser firing

    NOMORE(intensity, 100.0);
    NOLESS(intensity, 0.0);

    #if ENABLED(LASER_PWM_INVERT)
      intensity = 100 - intensity;
    #endif

    #if LASER_CONTROL == 1

      #if LASER_PWR_PIN == 2
        OCR3B = PWM_MAPPED_INTENSITY;
      #elif LASER_PWR_PIN == 3
        OCR3C = PWM_MAPPED_INTENSITY;
      #elif LASER_PWR_PIN == 5
        OCR3A = PWM_MAPPED_INTENSITY;
      #elif LASER_PWR_PIN == 6
        OCR4A = PWM_MAPPED_INTENSITY;
      #elif LASER_PWR_PIN == 7
        OCR4B = PWM_MAPPED_INTENSITY;
      #elif LASER_PWR_PIN == 8
        OCR4C = PWM_MAPPED_INTENSITY;
      #endif

    #elif LASER_CONTROL == 2

      #if LASER_PWM_PIN == 2
        OCR3B = PWM_MAPPED_INTENSITY;
      #elif LASER_PWM_PIN == 3
        OCR3C = PWM_MAPPED_INTENSITY;
      #elif LASER_PWM_PIN == 5
        OCR3A = PWM_MAPPED_INTENSITY;
      #elif LASER_PWM_PIN == 6
        OCR4A = PWM_MAPPED_INTENSITY;
      #elif LASER_PWM_PIN == 7
        OCR4B = PWM_MAPPED_INTENSITY;
      #elif LASER_PWM_PIN == 8
        OCR4C = PWM_MAPPED_INTENSITY;
      #endif

      WRITE(LASER_PWR_PIN, LASER_ARM);

    #endif /* LASER_CONTROL */

    if (laser.diagnostics) SERIAL_EM("Laser_byte fired");

  }

  void Laser::extinguish() {

    if (laser.firing == LASER_ON) {
      laser.firing = LASER_OFF;

      #if LASER_CONTROL == 1

        #if LASER_PWR_PIN == 2
          OCR3B = OCR_EXTINGUISH;
        #elif LASER_PWR_PIN == 3
          OCR3C = OCR_EXTINGUISH;
        #elif LASER_PWR_PIN == 5
          OCR3A = OCR_EXTINGUISH;
        #elif LASER_PWR_PIN == 6
          OCR4A = OCR_EXTINGUISH;
        #elif LASER_PWR_PIN == 7
          OCR4B = OCR_EXTINGUISH;
        #elif LASER_PWR_PIN == 8
          OCR4C = OCR_EXTINGUISH;
        #endif

      #elif LASER_CONTROL == 2

       #if LASER_PWM_PIN == 2
          OCR3B = OCR_EXTINGUISH;
        #elif LASER_PWM_PIN == 3
          OCR3C = OCR_EXTINGUISH;
        #elif LASER_PWM_PIN == 5
          OCR3A = OCR_EXTINGUISH;
        #elif LASER_PWM_PIN == 6
          OCR4A = OCR_EXTINGUISH;
        #elif LASER_PWM_PIN == 7
          OCR4B = OCR_EXTINGUISH;
        #elif LASER_PWM_PIN == 8
          OCR4C = OCR_EXTINGUISH;
        #endif

        WRITE(LASER_PWR_PIN, LASER_UNARM);

      #endif /* LASER_CONTROL */

      laser.time += millis() - (laser.last_firing / 1000);

      if (laser.diagnostics) SERIAL_EM("Laser extinguished");

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
        SERIAL_LM(ECHO, "Laser Peripherals Enabled");
    }

    void Laser::peripherals_off() {
      if (!peripherals_ok()) {
        HAL::digitalWrite(LASER_PERIPHERALS_PIN, HIGH);
        if (laser.diagnostics)
          SERIAL_LM(ECHO, "Laser Peripherals Disabled");
      }
    }

    void Laser::wait_for_peripherals() {
      unsigned long timeout = millis() + LASER_PERIPHERALS_TIMEOUT;
      if (laser.diagnostics)
        SERIAL_LM(ECHO, "Waiting for peripheral control board signal...");

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

#endif // ENABLED(LASER) && ENABLED(__AVR__)
