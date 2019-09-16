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
 * Conditional type assignment magic. For example...
 *
 * typename IF<(MYOPT==12), int, float>::type myvar;
 */
template <bool, class L, class R>
struct IF { typedef R type; };
template <class L, class R>
struct IF<true, L, R> { typedef L type; };

/**
 * Val limit min max
 */
template<typename T>
struct MinMaxVal {
  union {
    struct { T min, max; };
    T val[2];
  };
  void set(const T pmin)                                  { min = pmin; }
  void set(const T pmin, const T pmax)                    { min = pmin; max = pmax; }
  void reset()                                            { min = max = 0; }
  T length()                                        const { return (T)sqrtf(sq(min) + sq(max)); }
  operator T* ()                                          { return val; }
  operator bool()                                         { return min || max; }
  MinMaxVal<short> asInt()                                { MinMaxVal<short> o = { short(min), short(max) }; return o; }
  MinMaxVal<long>  asLong()                               { MinMaxVal<long>  o = {  long(min),  long(max) }; return o; }
  MinMaxVal<float> asFloat()                              { MinMaxVal<float> o = { float(min), float(max) }; return o; }
  MinMaxVal<short> asInt()                          const { MinMaxVal<short> o = { short(min), short(max) }; return o; }
  MinMaxVal<long>  asLong()                         const { MinMaxVal<long>  o = {  long(min),  long(max) }; return o; }
  MinMaxVal<float> asFloat()                        const { MinMaxVal<float> o = { float(min), float(max) }; return o; }
        T&  operator[](const int i)                       { return val[i]; }
  const T&  operator[](const int i)                 const { return val[i]; }
  MinMaxVal<T>& operator= (const MinMaxVal<T> &lh)        { set(lh.min, lh.max); return *this; }
  MinMaxVal<T>  operator+ (const MinMaxVal<T> &lh)  const { MinMaxVal<T> ls = *this; ls.min += lh.min; ls.max += lh.max; return ls; }
  MinMaxVal<T>  operator+ (const MinMaxVal<T> &lh)        { MinMaxVal<T> ls = *this; ls.min += lh.min; ls.max += lh.max; return ls; }
  MinMaxVal<T>  operator- (const MinMaxVal<T> &lh)  const { MinMaxVal<T> ls = *this; ls.min -= lh.min; ls.max -= lh.max; return ls; }
  MinMaxVal<T>  operator- (const MinMaxVal<T> &lh)        { MinMaxVal<T> ls = *this; ls.min -= lh.min; ls.max -= lh.max; return ls; }
  MinMaxVal<T>  operator* (const MinMaxVal<T> &lh)  const { MinMaxVal<T> ls = *this; ls.min *= lh.min; ls.max *= lh.max; return ls; }
  MinMaxVal<T>  operator* (const MinMaxVal<T> &lh)        { MinMaxVal<T> ls = *this; ls.min *= lh.min; ls.max *= lh.max; return ls; }
  MinMaxVal<T>  operator/ (const MinMaxVal<T> &lh)  const { MinMaxVal<T> ls = *this; ls.min /= lh.min; ls.max /= lh.max; return ls; }
  MinMaxVal<T>  operator/ (const MinMaxVal<T> &lh)        { MinMaxVal<T> ls = *this; ls.min /= lh.min; ls.max /= lh.max; return ls; }
  MinMaxVal<T>  operator* (const float &v)          const { MinMaxVal<T> ls = *this; ls.min *= v;      ls.max *= v;      return ls; }
  MinMaxVal<T>  operator* (const float &v)                { MinMaxVal<T> ls = *this; ls.min *= v;      ls.max *= v;      return ls; }
  MinMaxVal<T>  operator* (const int &v)            const { MinMaxVal<T> ls = *this; ls.min *= v;      ls.max *= v;      return ls; }
  MinMaxVal<T>  operator* (const int &v)                  { MinMaxVal<T> ls = *this; ls.min *= v;      ls.max *= v;      return ls; }
  MinMaxVal<T>  operator/ (const float &v)          const { MinMaxVal<T> ls = *this; ls.min /= v;      ls.max /= v;      return ls; }
  MinMaxVal<T>  operator/ (const float &v)                { MinMaxVal<T> ls = *this; ls.min /= v;      ls.max /= v;      return ls; }
  MinMaxVal<T>  operator/ (const int &v)            const { MinMaxVal<T> ls = *this; ls.min /= v;      ls.max /= v;      return ls; }
  MinMaxVal<T>  operator/ (const int &v)                  { MinMaxVal<T> ls = *this; ls.min /= v;      ls.max /= v;      return ls; }
  MinMaxVal<T>& operator+=(const MinMaxVal<T> &lh)        { min += lh.min;  max += lh.max;  return *this; }
  MinMaxVal<T>& operator-=(const MinMaxVal<T> &lh)        { min -= lh.min;  max -= lh.max;  return *this; }
  MinMaxVal<T>& operator*=(const MinMaxVal<T> &lh)        { min *= lh.min;  max *= lh.max;  return *this; }
  MinMaxVal<T>& operator*=(const float &lh)               { min *= lh;      max *= lh;      return *this; }
  MinMaxVal<T>& operator*=(const int &lh)                 { min *= lh;      max *= lh;      return *this; }
  bool          operator==(const MinMaxVal<T> &lh)        { return min == lh.min && max == lh.max; }
  bool          operator==(const MinMaxVal<T> &lh)  const { return min == lh.min && max == lh.max; }
  bool          operator!=(const MinMaxVal<T> &lh)        { return !operator==(lh); }
  bool          operator!=(const MinMaxVal<T> &lh)  const { return !operator==(lh); }
  MinMaxVal<T>       operator-()                          { MinMaxVal<T> o = *this; o.min = -min; o.max = -max; return o; }
  const MinMaxVal<T> operator-()                    const { MinMaxVal<T> o = *this; o.min = -min; o.max = -max; return o; }
};

typedef struct MinMaxVal<uint8_t> uint8_t_limit_t;
typedef struct MinMaxVal<int16_t>   int16_limit_t;
typedef struct MinMaxVal<float>     float_limit_t;
