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
#pragma once

typedef uint32_t  millis_t;
typedef int8_t    pin_t;

#ifdef __AVR__
  typedef uint8_t       mixer_color_t;
  typedef int8_t        mixer_accu_t;
  typedef int8_t        mixer_perc_t;
#else
  typedef uint_fast16_t mixer_color_t;
  typedef uint_fast16_t mixer_accu_t;
  typedef int8_t        mixer_perc_t;
#endif

union flagbyte_t {
  uint8_t _byte;
  struct {
    bool bit0 : 1;
    bool bit1 : 1;
    bool bit2 : 1;
    bool bit3 : 1;
    bool bit4 : 1;
    bool bit5 : 1;
    bool bit6 : 1;
    bool bit7 : 1;
  };
  flagbyte_t() { _byte = 0; }
};

union flagword_t {
  uint16_t _word;
  struct {
    bool bit0   : 1;
    bool bit1   : 1;
    bool bit2   : 1;
    bool bit3   : 1;
    bool bit4   : 1;
    bool bit5   : 1;
    bool bit6   : 1;
    bool bit7   : 1;
    bool bit8   : 1;
    bool bit9   : 1;
    bool bit10  : 1;
    bool bit11  : 1;
    bool bit12  : 1;
    bool bit13  : 1;
    bool bit14  : 1;
    bool bit15  : 1;
  };
  flagword_t() { _word = 0; }
};

