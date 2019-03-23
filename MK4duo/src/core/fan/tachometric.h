/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * tachometric.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#pragma once

extern void tacho_interrupt0();
#if FAN_COUNT > 1
  extern void tacho_interrupt1();
  #if FAN_COUNT > 2
    void tacho_interrupt2() { fans[2].tacho.interrupt(); }
    #if FAN_COUNT > 3
      void tacho_interrupt3() { fans[3].tacho.interrupt(); }
      #if FAN_COUNT > 4
        void tacho_interrupt4() { fans[4].tacho.interrupt(); }
        #if FAN_COUNT > 5
          void tacho_interrupt5() { fans[5].tacho.interrupt(); }
        #endif
      #endif
    #endif
  #endif
#endif

typedef struct {
  void (* function) ();	
} tacho_interrupt_t;

constexpr tacho_interrupt_t tacho_table[FAN_COUNT] = {
  tacho_interrupt0,
  #if FAN_COUNT > 1
    tacho_interrupt1,
    #if FAN_COUNT > 2
      tacho_interrupt2,
      #if FAN_COUNT > 3
        tacho_interrupt3,
        #if FAN_COUNT > 4
          tacho_interrupt4,
          #if FAN_COUNT > 5
            tacho_interrupt5
          #endif
        #endif
      #endif
    #endif
  #endif
};

// Struct Tacho data
typedef struct {

  public: /** Public Parameters */

    pin_t   pin;

  private: /** Private Parameters */

    static constexpr uint32_t MaxInterruptCount = 32;  // number of tacho interrupts that we average over

    uint32_t InterruptCount;          // accessed only in ISR, so no need to declare it volatile
    volatile millis_t LastResetTime,  // time (microseconds) at which we last reset the interrupt count, accessed inside and outside ISR
                      Interval;       // written by ISR, read outside the ISR

  public: /** Public Function */

    void init(const uint8_t index) {
      if (pin > 0) {
        HAL::pinMode(pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pin), tacho_table[index].function, FALLING);
      }
    }

    void interrupt() {
      ++InterruptCount;
      if (InterruptCount == MaxInterruptCount) {
        const uint32_t now = HAL_timer_get_current_count(STEPPER_TIMER);
        Interval = now - LastResetTime;
        LastResetTime = now;
        InterruptCount = 0;
      }
    }

    uint32_t GetRPM() {
      return (Interval != 0 && HAL_timer_get_current_count(STEPPER_TIMER) - LastResetTime < 3 * STEPPER_CLOCK_RATE)
        ? (STEPPER_CLOCK_RATE * MaxInterruptCount) / (2 * Interval)
        : 0;
    }

} tacho_data_t;
