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

// Processor-level delays for hardware interfaces
#define nop() __asm__ __volatile__("nop;\n\t":::)

FORCE_INLINE static void HAL_delay_4cycles(uint8_t cy) {
  __asm__ __volatile__(
    L("1")
    A("dec %[cnt]")
    A("nop")
    A("brne 1b")
    : [cnt] "+r"(cy)  // output: +r means input+output
    :                 // input:
    : "cc"            // clobbers:
  );
}

FORCE_INLINE static void HAL_delay_cycles(uint16_t cycles) {

  if (__builtin_constant_p(cycles)) {
    #define MAXNOPS 4

    if (cycles <= (MAXNOPS)) {
      switch (cycles) { case 4: nop(); case 3: nop(); case 2: nop(); case 1: nop(); }
    }
    else {
      const uint32_t rem = (cycles) % (MAXNOPS);
      switch (rem) { case 3: nop(); case 2: nop(); case 1: nop(); }
      if ((cycles = (cycles) / (MAXNOPS)))
        HAL_delay_4cycles(cycles); // if need more then 4 nop loop is more optimal
    }

    #undef MAXNOPS
  }
  else if (cycles >>= 2)
    HAL_delay_4cycles(cycles);
}

#undef nop
