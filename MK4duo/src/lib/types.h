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

#ifndef _TYPES_H_
#define _TYPES_H_

typedef uint32_t  millis_t;
typedef int8_t    pin_t;

union flagdir_t {
  int8_t dir;
  struct {
    int8_t X : 2;
    int8_t Y : 2;
    int8_t Z : 2;
    int8_t E : 2;
 };
  flagdir_t(const int8_t dirx=0, const int8_t diry=0, const int8_t dirz=0) { X = dirx; Y = diry; Z = dirz; E = -1; }
};

union flagbyte_t {
  uint8_t _byte;
  struct {
    byte bit0 : 1;
    byte bit1 : 1;
    byte bit2 : 1;
    byte bit3 : 1;
    byte bit4 : 1;
    byte bit5 : 1;
    byte bit6 : 1;
    byte bit7 : 1;
  };
  flagbyte_t() { _byte = 0; }
};

union flagword_t {
  uint16_t _word;
  struct {
    byte bit0   : 1;
    byte bit1   : 1;
    byte bit2   : 1;
    byte bit3   : 1;
    byte bit4   : 1;
    byte bit5   : 1;
    byte bit6   : 1;
    byte bit7   : 1;
    byte bit8   : 1;
    byte bit9   : 1;
    byte bit10  : 1;
    byte bit11  : 1;
    byte bit12  : 1;
    byte bit13  : 1;
    byte bit14  : 1;
    byte bit15  : 1;
  };
  flagword_t() { _word = 0; }
};

#endif /* _ENUM_H_ */
