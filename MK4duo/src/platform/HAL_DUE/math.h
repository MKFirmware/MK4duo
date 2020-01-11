/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * Optimized math functions for DUE
 */

static FORCE_INLINE uint32_t MultiU32X24toH32(uint32_t longIn1, uint32_t longIn2) {
	return ((uint64_t)longIn1 * longIn2 + 0x00800000) >> 24;
}

// Class to perform averaging of values read from the ADC
// numAveraged should be a power of 2 for best efficiency
template <size_t numAveraged>
class AveragingFilter {

  public: /** Constructor */

    AveragingFilter() { init(0); }

  private: /** Private Parameters */

    uint16_t  sample[numAveraged];
    size_t    index;
    uint32_t  sum;
    bool      valid;

  public: /** Public Function */

    void init(uint16_t val) volatile {
      sum = (uint32_t)val * (uint32_t)numAveraged;
      index = 0;
      valid = false;
      for (size_t i = 0; i < numAveraged; ++i)
        sample[i] = val;
    }

    void process_reading(const uint16_t read_adc) {
      sum += read_adc - sample[index];
      sample[index] = read_adc;
      if (++index == numAveraged) {
        index = 0;
        valid = true;
      }
    }

    uint32_t GetSum() const volatile { return sum / numAveraged; }

    bool IsValid() const volatile { return valid; }

};
