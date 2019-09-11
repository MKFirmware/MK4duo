/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

typedef uint32_t  millis_l;
typedef uint16_t  millis_s;
typedef int8_t    pin_t;

/**
 * Mixer color for AVR or 32 bit
 */
#ifdef __AVR__
  typedef uint8_t       mixer_color_t;
  typedef int8_t        mixer_accu_t;
  typedef int8_t        mixer_perc_t;
#else
  typedef uint_fast16_t mixer_color_t;
  typedef uint_fast16_t mixer_accu_t;
  typedef int8_t        mixer_perc_t;
#endif

/**
 * Val limit min max
 */
template<typename T>
struct limit_val_t {
  union {
    struct { T min, max; };
    T val[2];
  };
  void set(const T pmin, const T pmax)    { min = pmin; max = pmax; }
  void reset()                            { min = max = 0; }
            operator T* ()                { return val; }
        T&  operator[](const int i)       { return val[i]; }
  const T&  operator[](const int i) const { return val[i]; }
};
typedef struct limit_val_t<uint8_t> uint8_t_limit_t;
typedef struct limit_val_t<int16_t>   int16_limit_t;
typedef struct limit_val_t<float>     float_limit_t;
