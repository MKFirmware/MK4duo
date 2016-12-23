/**
 * MK & MK4due 3D Printer Firmware
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

#if ENABLED(LASERBEAM) && ENABLED(ARDUINO_ARCH_AVR)

  #include <Arduino.h>
  #include <avr/interrupt.h>

  laser_t laser;

  #if ENABLED(LASER_PULSE_METHOD)
    #define pulsebit(x) (1 << x)
    #ifndef PE3                   // Undef'd in fastio.h.
      #define PE3 3
    #endif
  #endif

  void timer3_init(int pin) {
    #if ENABLED(LASER_PULSE_METHOD)
      TCCR3A = 0;                 // clear control register A 
      TCCR3B = pulsebit(WGM33);   // set mode as phase and frequency correct pwm, stop the timer

      ICR3 = F_CPU / LASER_PWM / 2; // the counter runs backwards after TOP
      TCCR3B &= ~(bit(CS30) | bit(CS31) | bit(CS32)); // Stop timer

      TCCR3A |= pulsebit(COM3A1); // Connect pin5 to timer register
      DDRE |= pulsebit(PORTE3);   // Actually output on pin 5

      OCR3A = 0;                // Zero duty cycle = OFF
      TCCR3B |= pulsebit(CS30); // No prescaler, start timer

      // Use timer4 to end laser pulse
      /*
      Prescaler  CS42  CS41  CS40  Range
         1         0     0     1   0 - 4.08 msec
         8         0     1     0   0 - 32.7 ms  <=====
        64         0     1     1   0 - 261 ms
       256         1     0     0   0 - 1046 ms
      1024         1     0     1   0 - 4183 ms
      6000 mm/min at 508 dpi = 0.5 ms pulse
      300 mm/min at 254 dpi = 20 ms pulse
      For the moment a prescaler of 8 is used which
      allows up to 32.7 ms pulses with a theoretical
      resolution of 0.5 µs. 
      Waveform generation mode 4: CTC top in OCR4A
      ============================================

      WGN43, WGM42, WGM41, WGM40 = 0, 1, 0, 0
      TCCR4A
      ======
      COM4A1, COM4A0 = 0,0 = Normal operation, OC4A disconnected
      COM4B1, COM4B0 = 0,0 = Normal operation, OC4B disconnected
      COM4C1, COM4C0 = 0,0 = Normal operation, OC4C disconnected
      WGM41, WGM40 = 0,0 (See above)
      TCCR4B
      ======
      ICN4, IEC4 = 0,0 = Not applicable without input
      WGM43, WGM42 = 0,1 (See above)
      CS42, CS41, CS40 = 0,1,0 (See above)
      CS42, CS41, CS40 = 0,0,0 = Clock stopped
      TCCR4C
      ======
      FOC4A, FOC4B, FOS4B = 0,0,0 = Not used
      OCR4A
      =====
      16-bit value when timer overflows = generated interrupt
      This is set in laser_pulse()
      TIMSK4
      ======
      OCIE4A = 1 = Generate interrupt when timer reach OCR4A

      TIFR4
      =====
      OCF4A: When set, the interrupt will be executed. To clear, write 1 here
      When reloading the timer in laser_pulse, an expired interrupt is cleared.
      */
      // Prepare laser pulse shutdown timer
      TCCR4A = 0; 
      TCCR4B = pulsebit(WGM42);    // CTC
      TIMSK4 |= pulsebit(OCIE4A);  // Enable interrupt on OCR4A

    #else

      pinMode(pin, OUTPUT);
      analogWrite(pin, 1);  // let Arduino setup do it's thing to the PWM pin

      TCCR3B = 0x00;  // stop Timer4 clock for register updates
      TCCR3A = 0x82; // Clear OC3A on match, fast PWM mode, lower WGM3x=14
      ICR3 = labs(F_CPU / LASER_PWM); // clock cycles per PWM pulse
      OCR3A = labs(F_CPU / LASER_PWM) - 1; // ICR3 - 1 force immediate compare on next tick
      TCCR3B = 0x18 | 0x01; // upper WGM4x = 14, clock sel = prescaler, start running

      noInterrupts();
      TCCR3B &= 0xf8; // stop timer, OC3A may be active now
      TCNT3 = labs(F_CPU / LASER_PWM); // force immediate compare on next tick
      ICR3 = labs(F_CPU / LASER_PWM); // set new PWM period
      TCCR3B |= 0x01; // start the timer with proper prescaler value
      interrupts();

    #endif
  }

  #if ENABLED(LASER_PULSE_METHOD)
    ISR(TIMER4_COMPA_vect) {
      OCR3A = 0;              // 0 Duty cycle

      // Stop pulse shutdown timer
      TCCR4B &= ~(pulsebit(CS40) | pulsebit(CS41) | pulsebit(CS42)); // Stop timer.
    }

    void laser_pulse(uint32_t ulValue, unsigned long usec) {
      OCR3A = ulValue;        // Duty cycle of pulse

      // Start timer4 to end pulse
      OCR4A = 2*usec;         // Ticks until IRQ, "2" comes from prescaler
      TCNT4 = 0;              // Count from 0
      TCCR4B |= pulsebit(CS41);    // Start timer
      TIFR4 = pulsebit(OCF4A);     // Clear any pending interrupt
    }

  #else // LASER_PULSE_METHOD

    void timer4_init(int pin) {
      pinMode(pin, OUTPUT);
      analogWrite(pin, 1);  // let Arduino setup do it's thing to the PWM pin

      TCCR4B = 0x00;  // stop Timer4 clock for register updates
      TCCR4A = 0x82; // Clear OC4A on match, fast PWM mode, lower WGM4x=14
      ICR4 = labs(F_CPU / LASER_PWM); // clock cycles per PWM pulse
      OCR4A = labs(F_CPU / LASER_PWM) - 1; // ICR4 - 1 force immediate compare on next tick
      TCCR4B = 0x18 | 0x01; // upper WGM4x = 14, clock sel = prescaler, start running

      noInterrupts();
      TCCR4B &= 0xf8; // stop timer, OC4A may be active now
      TCNT4 = labs(F_CPU / LASER_PWM); // force immediate compare on next tick
      ICR4 = labs(F_CPU / LASER_PWM); // set new PWM period
      TCCR4B |= 0x01; // start the timer with proper prescaler value
      interrupts();
    }

  #endif // LASER_PULSE_METHOD

  void laser_init() {

    #if ENABLED(LASER_PULSE_METHOD)
      // Initialize timers for laser intensity control 
      // ONLY laser_firing on pin 5. Can't use pin 6 for output, used by timer4.
      timer3_init(LASER_PWR_PIN);
    #else
      // Initialize timers for laser intensity control
      #if LASER_CONTROL == 1
        if (LASER_PWR_PIN == 2 || LASER_PWR_PIN == 3 || LASER_PWR_PIN == 5) timer3_init(LASER_PWR_PIN);
        if (LASER_PWR_PIN == 6 || LASER_PWR_PIN == 7 || LASER_PWR_PIN == 8) timer4_init(LASER_PWR_PIN);
      #endif
      #if LASER_CONTROL == 2
        if (LASER_TTL_PIN == 2 || LASER_TTL_PIN == 3 || LASER_TTL_PIN == 5) timer3_init(LASER_TTL_PIN);
        if (LASER_TTL_PIN == 6 || LASER_TTL_PIN == 7 || LASER_TTL_PIN == 8) timer4_init(LASER_TTL_PIN);
      #endif
    #endif // LASER_PULSE_METHOD

    #if ENABLED(LASER_PERIPHERALS)
      pinMode(LASER_PERIPHERALS_PIN, OUTPUT);
      digitalWrite(LASER_PERIPHERALS_PIN, HIGH);        // Laser peripherals are active LOW, so preset the pin
      pinMode(LASER_PERIPHERALS_STATUS_PIN, INPUT);
      digitalWrite(LASER_PERIPHERALS_STATUS_PIN, HIGH); // Set the peripherals status pin to pull-up.
    #endif // LASER_PERIPHERALS

    #if LASER_CONTROL == 2
      pinMode(LASER_PWR_PIN, OUTPUT);
      digitalWrite(LASER_PWR_PIN, LASER_UNARM);         // Laser FIRING is active LOW, so preset the pin
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
    
    #if !ENABLED(LASER_PULSE_METHOD)
      laser_extinguish();
    #endif
  }

  void laser_fire(float intensity = 100.0){
    laser.firing = LASER_ON;
    laser.last_firing = micros(); // microseconds of last laser firing
    if (intensity > 100.0) intensity = 100.0; // restrict intensity between 0 and 100
    if (intensity < 0) intensity = 0;

    // In the case that the laserdriver need at least a certain level "LASER_REMAP_INTENSITY"
    // to give anything, the intensity can be remapped to start at "LASER_REMAP_INTENSITY"
    // At least some CO2-drivers need it, not sure about laserdiode drivers.

    #if(ENABLED(LASER_REMAP_INTENSITY) && ENABLED(LASER_PULSE_METHOD))
      #if LASER_REMAP_INTENSITY != 0
        float OldRange, NewRange;
        OldRange = (255.0 - 0.0);
        NewRange = (intensity - LASER_REMAP_INTENSITY);  
        intensity = (float)(((((float)intensity - 0) * NewRange) / OldRange) + LASER_REMAP_INTENSITY);
     #endif
    #endif

    #if (!ENABLED(LASER_PULSE_METHOD))
      pinMode(LASER_PWR_PIN, OUTPUT);
    #endif

    #if LASER_CONTROL == 1
      #if ENABLED(LASER_PULSE_METHOD)
        OCR3A = labs((intensity / 100.0) * (F_CPU / LASER_PWM / 2));
      #else
        analogWrite(LASER_PWR_PIN, labs((intensity / 100.0) * (F_CPU / LASER_PWM)));
      #endif
    #endif

    #if LASER_CONTROL == 2
      analogWrite(LASER_TTL_PIN, labs((intensity / 100.0) * (F_CPU / LASER_PWM)));
      digitalWrite(LASER_PWR_PIN, LASER_ARM);
    #endif

    if (laser.diagnostics)
      SERIAL_EM("Laser_byte fired");
  }

  void laser_extinguish() {
    if (laser.firing == LASER_ON) {
      laser.firing = LASER_OFF;
       
      #if ENABLED(LASER_PULSE_METHOD)
        OCR3A = 0; // Zero duty cycle = OFF
      #else
        // Engage the pullup resistor for TTL laser controllers which don't turn off entirely without it.
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
