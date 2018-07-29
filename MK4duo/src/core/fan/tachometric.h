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
 * tachometric.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _TACHOMETRIC_H_
#define _TACHOMETRIC_H_

#if FAN_COUNT > 0 && ENABLED(TACHOMETRIC)

  class Tachometric {

    public: /** Constructor */

      Tachometric();

    public: /** Public Parameters */

      pin_t   pin;

    private: /** Private Parameters */

      static constexpr uint32_t MaxInterruptCount = 32;  // number of tacho interrupts that we average over

      uint32_t InterruptCount;          // accessed only in ISR, so no need to declare it volatile
      volatile millis_t LastResetTime,  // time (microseconds) at which we last reset the interrupt count, accessed inside and outside ISR
                        Interval;       // written by ISR, read outside the ISR

    public: /** Public Function */

      void init(const uint8_t tacho);
      void Interrupt();

      uint32_t GetRPM();

  };

  extern Tachometric tachometrics[FAN_COUNT];

#endif // FAN_COUNT > 0 && ENABLED(TACHOMETRIC)

#endif /* _TACHOMETRIC_H_ */
