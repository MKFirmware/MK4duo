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

#pragma once

/**
 * Optimized math functions for SAMD
 */

static FORCE_INLINE uint32_t MultiU32X32toH32(uint32_t longIn1,uint32_t longIn2) {
	return ((uint64_t)longIn1 * longIn2) >> 32;
}

// Class to perform averaging of values read from the ADC
// numAveraged should be a power of 2 for best efficiency
template <size_t numAveraged> class AveragingFilter {

  public: /** Constructor */

    AveragingFilter() { Init(0); }

  private: /** Private Parameters */

    uint16_t  readings[numAveraged];
    size_t    index;
    uint32_t  sum;
    bool      valid;

  public: /** Public Function */

    void Init(uint16_t val) volatile {

      
      sum = (uint32_t)val * (uint32_t)numAveraged;
      index = 0;
      valid = false;
      for (size_t i = 0; i < numAveraged; ++i)
        readings[i] = val;
    
    }

    void ProcessReading(const uint16_t read) {
      sum = sum - readings[index] + read;
      readings[index] = read;
      if (++index == numAveraged) {
        index = 0;
        valid = true;
      }
    }

    uint32_t GetSum() const volatile { return sum; }

    bool IsValid() const volatile	{ return valid; }

};
