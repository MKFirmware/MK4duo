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
 * tachometric.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if FAN_COUNT > 0 && ENABLED(TACHOMETRIC)

  Tachometric tachometrics[FAN_COUNT];

  void TachoInterrupt0() { tachometrics[0].Interrupt(); }
  #if FAN_COUNT > 1
    void TachoInterrupt1() { tachometrics[1].Interrupt(); }
    #if FAN_COUNT > 2
      void TachoInterrupt2() { tachometrics[2].Interrupt(); }
      #if FAN_COUNT > 3
        void TachoInterrupt3() { tachometrics[3].Interrupt(); }
        #if FAN_COUNT > 4
          void TachoInterrupt4() { tachometrics[4].Interrupt(); }
          #if FAN_COUNT > 5
            void TachoInterrupt5() { tachometrics[5].Interrupt(); }
          #endif
        #endif
      #endif
    #endif
  #endif

  typedef struct {
    void (* function) ();	
  } Tacho_t;

  constexpr Tacho_t Tacho_Table[FAN_COUNT] = {
    TachoInterrupt0,
    #if FAN_COUNT > 1
      TachoInterrupt1,
      #if FAN_COUNT > 2
        TachoInterrupt2,
        #if FAN_COUNT > 3
          TachoInterrupt3,
          #if FAN_COUNT > 4
            TachoInterrupt4,
            #if FAN_COUNT > 5
              TachoInterrupt5
            #endif
          #endif
        #endif
      #endif
    #endif
  };

  Tachometric::Tachometric() : InterruptCount(0), LastResetTime(0), Interval(0), pin(NoPin) {}

  /**
   * Initialize Tachometric
   */
  void Tachometric::init(const uint8_t tacho) {
    if (pin > 0) {
      HAL::pinMode(pin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(pin), Tacho_Table[tacho].function, FALLING);
    }
  }

  uint32_t Tachometric::GetRPM() {
    return (Interval != 0 && HAL_timer_get_current_count(STEPPER_TIMER) - LastResetTime < 3 * STEPPER_CLOCK_RATE)
      ? (STEPPER_CLOCK_RATE * MaxInterruptCount) / (2 * Interval)
      : 0;
  }

  void Tachometric::Interrupt() {
    ++InterruptCount;
    if (InterruptCount == MaxInterruptCount) {
      const uint32_t now = HAL_timer_get_current_count(STEPPER_TIMER);
      Interval = now - LastResetTime;
      LastResetTime = now;
      InterruptCount = 0;
    }
  }

#endif // FAN_COUNT > 0 && ENABLED(TACHOMETRIC)
