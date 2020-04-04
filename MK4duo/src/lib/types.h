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

class __FlashStringHelper;
typedef const __FlashStringHelper *progmem_str;

/**
 * Type for pins
 */
typedef int8_t    pin_t;

/**
 * Type for millis
 */
typedef uint32_t  millis_l;
typedef uint16_t  millis_s;

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

#define _RECIP(N) ((N) ? 1.0f / float(N) : 0.0f)
#define _ABS(N)   ((N) < 0 ? -(N) : (N))
#define _LS(N)    (N = (T)(uint32_t(N) << v))
#define _RS(N)    (N = (T)(uint32_t(N) >> v))

/**
 * FeedRate_t
 */
typedef float feedrate_t;

// Feedrate scaling and conversion
#define MMM_TO_MMS(MM_M)  feedrate_t(float(MM_M)/60.0f)
#define MMS_TO_MMM(MM_S)  (float(MM_S)*60.0f)
#define MMS_SCALED(MM_S)  ((MM_S)* 0.01f * mechanics.feedrate_percentage)
/***********************************************************/

template<typename T> struct                 XYval;
template<typename T> struct                XYZval;
template<typename T> struct               XYZEval;

typedef struct XYval<bool>              xy_bool_t;
typedef struct XYZval<bool>            xyz_bool_t;
typedef struct XYZEval<bool>          xyze_bool_t;

typedef struct XYval<char>               xy_char_t;
typedef struct XYZval<char>             xyz_char_t;
typedef struct XYZEval<char>           xyze_char_t;

typedef struct XYval<unsigned char>     xy_uchar_t;
typedef struct XYZval<unsigned char>   xyz_uchar_t;
typedef struct XYZEval<unsigned char> xyze_uchar_t;

typedef struct XYval<int8_t>            xy_int8_t;
typedef struct XYZval<int8_t>          xyz_int8_t;
typedef struct XYZEval<int8_t>        xyze_int8_t;

typedef struct XYval<uint8_t>          xy_uint8_t;
typedef struct XYZval<uint8_t>        xyz_uint8_t;
typedef struct XYZEval<uint8_t>      xyze_uint8_t;

typedef struct XYval<int16_t>            xy_int_t;
typedef struct XYZval<int16_t>          xyz_int_t;
typedef struct XYZEval<int16_t>        xyze_int_t;

typedef struct XYval<uint16_t>          xy_uint_t;
typedef struct XYZval<uint16_t>        xyz_uint_t;
typedef struct XYZEval<uint16_t>      xyze_uint_t;

typedef struct XYval<int32_t>           xy_long_t;
typedef struct XYZval<int32_t>         xyz_long_t;
typedef struct XYZEval<int32_t>       xyze_long_t;

typedef struct XYval<uint32_t>         xy_ulong_t;
typedef struct XYZval<uint32_t>       xyz_ulong_t;
typedef struct XYZEval<uint32_t>     xyze_ulong_t;

typedef struct XYZval<volatile int32_t>   xyz_vlong_t;
typedef struct XYZEval<volatile int32_t> xyze_vlong_t;

typedef struct XYval<float>            xy_float_t;
typedef struct XYZval<float>          xyz_float_t;
typedef struct XYZEval<float>        xyze_float_t;

typedef struct XYval<float>         xy_feedrate_t;
typedef struct XYZval<float>       xyz_feedrate_t;
typedef struct XYZEval<float>     xyze_feedrate_t;

typedef xy_uint8_t                      xy_byte_t;
typedef xyz_uint8_t                    xyz_byte_t;
typedef xyze_uint8_t                  xyze_byte_t;

typedef xyz_long_t                     abc_long_t;
typedef xyze_long_t                   abce_long_t;
typedef xyz_ulong_t                   abc_ulong_t;
typedef xyze_ulong_t                 abce_ulong_t;

typedef xy_float_t                       xy_pos_t;
typedef xyz_float_t                     xyz_pos_t;
typedef xyze_float_t                   xyze_pos_t;

typedef xy_float_t                     ab_float_t;
typedef xyz_float_t                   abc_float_t;
typedef xyze_float_t                 abce_float_t;

typedef ab_float_t                       ab_pos_t;
typedef abc_float_t                     abc_pos_t;
typedef abce_float_t                   abce_pos_t;

#if ENABLED(WORKSPACE_OFFSETS)

  #define NATIVE_TO_LOGICAL(POS, AXIS)    ((POS) + mechanics.workspace_offset[AXIS])
  #define LOGICAL_TO_NATIVE(POS, AXIS)    ((POS) - mechanics.workspace_offset[AXIS])
  inline void toLogical(xy_pos_t &raw)    { raw += mechanics.workspace_offset; }
  inline void toLogical(xyz_pos_t &raw)   { raw += mechanics.workspace_offset; }
  inline void toLogical(xyze_pos_t &raw)  { raw += mechanics.workspace_offset; }
  inline void toNative(xy_pos_t &raw)     { raw -= mechanics.workspace_offset; }
  inline void toNative(xyz_pos_t &raw)    { raw -= mechanics.workspace_offset; }
  inline void toNative(xyze_pos_t &raw)   { raw -= mechanics.workspace_offset; }

#else

  #define NATIVE_TO_LOGICAL(POS, AXIS)    (POS)
  #define LOGICAL_TO_NATIVE(POS, AXIS)    (POS)
  inline void toLogical(xy_pos_t &raw)    { UNUSED(raw); }
  inline void toLogical(xyz_pos_t &raw)   { UNUSED(raw); }
  inline void toLogical(xyze_pos_t &raw)  { UNUSED(raw); }
  inline void toNative(xy_pos_t &raw)     { UNUSED(raw); }
  inline void toNative(xyz_pos_t &raw)    { UNUSED(raw); }
  inline void toNative(xyze_pos_t &raw)   { UNUSED(raw); }

#endif

#define LOGICAL_X_POSITION(POS) NATIVE_TO_LOGICAL(POS, X_AXIS)
#define LOGICAL_Y_POSITION(POS) NATIVE_TO_LOGICAL(POS, Y_AXIS)
#define LOGICAL_Z_POSITION(POS) NATIVE_TO_LOGICAL(POS, Z_AXIS)
#define NATIVE_X_POSITION(POS)  LOGICAL_TO_NATIVE(POS, X_AXIS)
#define NATIVE_Y_POSITION(POS)  LOGICAL_TO_NATIVE(POS, Y_AXIS)
#define NATIVE_Z_POSITION(POS)  LOGICAL_TO_NATIVE(POS, Z_AXIS)

/**
 * XY coordinates, counters, etc.
 */
template<typename T>
struct XYval {
  union {
    struct { T x, y; };
    struct { T a, b; };
    T pos[2];
  };
  FORCE_INLINE void set(const T px)                               { x = px; }
  FORCE_INLINE void set(const T px, const T py)                   { x = px; y = py; }
  FORCE_INLINE void set(const T (&arr)[XY])                       { x = arr[0]; y = arr[1]; }
  FORCE_INLINE void set(const T (&arr)[XYZ])                      { x = arr[0]; y = arr[1]; }
  FORCE_INLINE void set(const T (&arr)[XYZE])                     { x = arr[0]; y = arr[1]; }
  FORCE_INLINE void reset()                                       { x = y = 0; }
  FORCE_INLINE T magnitude()                                const { return (T)sqrtf(x*x + y*y); }
  FORCE_INLINE operator T* ()                                     { return pos; }
  FORCE_INLINE operator bool()                                    { return x || y; }
  FORCE_INLINE XYval<T>           copy()                    const { return *this; }
  FORCE_INLINE XYval<T>            ABS()                    const { return { T(_ABS(x)), T(_ABS(y)) }; }
  FORCE_INLINE XYval<int16_t>    asInt()                          { return { int16_t(x), int16_t(y) }; }
  FORCE_INLINE XYval<int16_t>    asInt()                    const { return { int16_t(x), int16_t(y) }; }
  FORCE_INLINE XYval<int32_t>   asLong()                          { return { int32_t(x), int32_t(y) }; }
  FORCE_INLINE XYval<int32_t>   asLong()                    const { return { int32_t(x), int32_t(y) }; }
  FORCE_INLINE XYval<int32_t>   ROUNDL()                          { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
  FORCE_INLINE XYval<int32_t>   ROUNDL()                    const { return { int32_t(LROUND(x)), int32_t(LROUND(y)) }; }
  FORCE_INLINE XYval<float>    asFloat()                          { return {   float(x),   float(y) }; }
  FORCE_INLINE XYval<float>    asFloat()                    const { return {   float(x),   float(y) }; }
  FORCE_INLINE XYval<float> reciprocal()                    const { return {  _RECIP(x),  _RECIP(y) }; }
  FORCE_INLINE XYval<float>  asLogical()                    const { XYval<float> o = asFloat(); toLogical(o); return o; }
  FORCE_INLINE XYval<float>   asNative()                    const { XYval<float> o = asFloat(); toNative(o);  return o; }
  FORCE_INLINE operator XYZval<T>()                               { return { x, y }; }
  FORCE_INLINE operator XYZval<T>()                         const { return { x, y }; }
  FORCE_INLINE operator XYZEval<T>()                              { return { x, y }; }
  FORCE_INLINE operator XYZEval<T>()                        const { return { x, y }; }
  FORCE_INLINE       T&  operator[](const int i)                  { return pos[i]; }
  FORCE_INLINE const T&  operator[](const int i)            const { return pos[i]; }
  FORCE_INLINE XYval<T>& operator= (const T v)                    { set(v,    v   ); return *this; }
  FORCE_INLINE XYval<T>& operator= (const XYZval<T>  &rs)         { set(rs.x, rs.y); return *this; }
  FORCE_INLINE XYval<T>& operator= (const XYZEval<T> &rs)         { set(rs.x, rs.y); return *this; }
  FORCE_INLINE XYval<T>  operator+ (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator+ (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator- (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator- (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator* (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator* (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator/ (const XYval<T>   &rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator/ (const XYval<T>   &rs)         { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator+ (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator+ (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator- (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator- (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator* (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator* (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator/ (const XYZval<T>  &rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator/ (const XYZval<T>  &rs)         { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator+ (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator+ (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator- (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator- (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator* (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator* (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator/ (const XYZEval<T> &rs)   const { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator/ (const XYZEval<T> &rs)         { XYval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; return ls; }
  FORCE_INLINE XYval<T>  operator* (const float &v)         const { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FORCE_INLINE XYval<T>  operator* (const float &v)               { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FORCE_INLINE XYval<T>  operator* (const int &v)           const { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FORCE_INLINE XYval<T>  operator* (const int &v)                 { XYval<T> ls = *this; ls.x *= v;    ls.y *= v;    return ls; }
  FORCE_INLINE XYval<T>  operator/ (const float &v)         const { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FORCE_INLINE XYval<T>  operator/ (const float &v)               { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FORCE_INLINE XYval<T>  operator/ (const int &v)           const { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FORCE_INLINE XYval<T>  operator/ (const int &v)                 { XYval<T> ls = *this; ls.x /= v;    ls.y /= v;    return ls; }
  FORCE_INLINE XYval<T>  operator>>(const int &v)           const { XYval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    return ls; }
  FORCE_INLINE XYval<T>  operator>>(const int &v)                 { XYval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    return ls; }
  FORCE_INLINE XYval<T>  operator<<(const int &v)           const { XYval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    return ls; }
  FORCE_INLINE XYval<T>  operator<<(const int &v)                 { XYval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    return ls; }
  FORCE_INLINE XYval<T>& operator+=(const XYval<T>   &rs)         { x += rs.x; y += rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator-=(const XYval<T>   &rs)         { x -= rs.x; y -= rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator*=(const XYval<T>   &rs)         { x *= rs.x; y *= rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator+=(const XYZval<T>  &rs)         { x += rs.x; y += rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator-=(const XYZval<T>  &rs)         { x -= rs.x; y -= rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator*=(const XYZval<T>  &rs)         { x *= rs.x; y *= rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator+=(const XYZEval<T> &rs)         { x += rs.x; y += rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator-=(const XYZEval<T> &rs)         { x -= rs.x; y -= rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator*=(const XYZEval<T> &rs)         { x *= rs.x; y *= rs.y; return *this; }
  FORCE_INLINE XYval<T>& operator*=(const float &v)               { x *= v;    y *= v;    return *this; }
  FORCE_INLINE XYval<T>& operator*=(const int &v)                 { x *= v;    y *= v;    return *this; }
  FORCE_INLINE XYval<T>& operator/=(const float &v)               { x /= v;    y /= v;    return *this; }
  FORCE_INLINE XYval<T>& operator/=(const int &v)                 { x /= v;    y /= v;    return *this; }
  FORCE_INLINE XYval<T>& operator+=(const float &v)               { x += v;    y += v;    return *this; }
  FORCE_INLINE XYval<T>& operator+=(const int &v)                 { x += v;    y += v;    return *this; }
  FORCE_INLINE XYval<T>& operator-=(const float &v)               { x -= v;    y -= v;    return *this; }
  FORCE_INLINE XYval<T>& operator-=(const int &v)                 { x -= v;    y -= v;    return *this; }
  FORCE_INLINE XYval<T>& operator>>=(const int &v)                { _RS(x);    _RS(y);    return *this; }
  FORCE_INLINE XYval<T>& operator<<=(const int &v)                { _LS(x);    _LS(y);    return *this; }
  FORCE_INLINE bool      operator==(const XYval<T>   &rs)         { return x == rs.x && y == rs.y; }
  FORCE_INLINE bool      operator==(const XYZval<T>  &rs)         { return x == rs.x && y == rs.y; }
  FORCE_INLINE bool      operator==(const XYZEval<T> &rs)         { return x == rs.x && y == rs.y; }
  FORCE_INLINE bool      operator==(const XYval<T>   &rs)   const { return x == rs.x && y == rs.y; }
  FORCE_INLINE bool      operator==(const XYZval<T>  &rs)   const { return x == rs.x && y == rs.y; }
  FORCE_INLINE bool      operator==(const XYZEval<T> &rs)   const { return x == rs.x && y == rs.y; }
  FORCE_INLINE bool      operator!=(const XYval<T>   &rs)         { return !operator==(rs); }
  FORCE_INLINE bool      operator!=(const XYZval<T>  &rs)         { return !operator==(rs); }
  FORCE_INLINE bool      operator!=(const XYZEval<T> &rs)         { return !operator==(rs); }
  FORCE_INLINE bool      operator!=(const XYval<T>   &rs)   const { return !operator==(rs); }
  FORCE_INLINE bool      operator!=(const XYZval<T>  &rs)   const { return !operator==(rs); }
  FORCE_INLINE bool      operator!=(const XYZEval<T> &rs)   const { return !operator==(rs); }
  FORCE_INLINE XYval<T>       operator-()                         { XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
  FORCE_INLINE const XYval<T> operator-()                   const { XYval<T> o = *this; o.x = -x; o.y = -y; return o; }
};

/**
 * XYZ coordinates, counters, etc.
 */
template<typename T>
struct XYZval {
  union {
    struct { T x, y, z; };
    struct { T a, b, c; };
    T pos[3];
  };
  FORCE_INLINE void set(const T px)                               { x = px; }
  FORCE_INLINE void set(const T px, const T py)                   { x = px; y = py; }
  FORCE_INLINE void set(const T px, const T py, const T pz)       { x = px; y = py; z = pz; }
  FORCE_INLINE void set(const XYval<T> pxy, const T pz)           { x = pxy.x; y = pxy.y; z = pz; }
  FORCE_INLINE void set(const T (&arr)[XY])                       { x = arr[0]; y = arr[1]; }
  FORCE_INLINE void set(const T (&arr)[XYZ])                      { x = arr[0]; y = arr[1]; z = arr[2]; }
  FORCE_INLINE void set(const T (&arr)[XYZE])                     { x = arr[0]; y = arr[1]; z = arr[2]; }
  FORCE_INLINE void reset()                                       { x = y = z = 0; }
  FORCE_INLINE T magnitude()                                const { return (T)sqrtf(x*x + y*y + z*z); }
  FORCE_INLINE operator T* ()                                     { return pos; }
  FORCE_INLINE operator bool()                                    { return z || x || y; }
  FORCE_INLINE XYZval<T>          copy()                    const { XYZval<T> o = *this; return o; }
  FORCE_INLINE XYZval<T>           ABS()                    const { return { T(_ABS(x)), T(_ABS(y)), T(_ABS(z)) }; }
  FORCE_INLINE XYZval<int16_t>   asInt()                          { return { int16_t(x), int16_t(y), int16_t(z) }; }
  FORCE_INLINE XYZval<int16_t>   asInt()                    const { return { int16_t(x), int16_t(y), int16_t(z) }; }
  FORCE_INLINE XYZval<int32_t>  asLong()                          { return { int32_t(x), int32_t(y), int32_t(z) }; }
  FORCE_INLINE XYZval<int32_t>  asLong()                    const { return { int32_t(x), int32_t(y), int32_t(z) }; }
  FORCE_INLINE XYZval<int32_t>  ROUNDL()                          { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)) }; }
  FORCE_INLINE XYZval<int32_t>  ROUNDL()                    const { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)) }; }
  FORCE_INLINE XYZval<float>   asFloat()                          { return {   float(x),   float(y),   float(z) }; }
  FORCE_INLINE XYZval<float>   asFloat()                    const { return {   float(x),   float(y),   float(z) }; }
  FORCE_INLINE XYZval<float> reciprocal()                   const { return {  _RECIP(x),  _RECIP(y),  _RECIP(z) }; }
  FORCE_INLINE XYZval<float> asLogical()                    const { XYZval<float> o = asFloat(); toLogical(o); return o; }
  FORCE_INLINE XYZval<float>  asNative()                    const { XYZval<float> o = asFloat(); toNative(o);  return o; }
  FORCE_INLINE operator       XYval<T>&()                         { return *(XYval<T>*)this; }
  FORCE_INLINE operator const XYval<T>&()                   const { return *(const XYval<T>*)this; }
  FORCE_INLINE operator       XYZEval<T>()                  const { return { x, y, z }; }
  FORCE_INLINE       T&   operator[](const int i)                 { return pos[i]; }
  FORCE_INLINE const T&   operator[](const int i)           const { return pos[i]; }
  FORCE_INLINE XYZval<T>& operator= (const T v)                   { set(v,    v,    v   ); return *this; }
  FORCE_INLINE XYZval<T>& operator= (const XYval<T>   &rs)        { set(rs.x, rs.y      ); return *this; }
  FORCE_INLINE XYZval<T>& operator= (const XYZEval<T> &rs)        { set(rs.x, rs.y, rs.z); return *this; }
  FORCE_INLINE XYZval<T>  operator+ (const XYval<T>   &rs)  const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator+ (const XYval<T>   &rs)        { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator- (const XYval<T>   &rs)  const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator- (const XYval<T>   &rs)        { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator* (const XYval<T>   &rs)  const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator* (const XYval<T>   &rs)        { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const XYval<T>   &rs)  const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const XYval<T>   &rs)        { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;               return ls; }
  FORCE_INLINE XYZval<T>  operator+ (const XYZval<T>  &rs)  const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator+ (const XYZval<T>  &rs)        { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator- (const XYZval<T>  &rs)  const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator- (const XYZval<T>  &rs)        { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator* (const XYZval<T>  &rs)  const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator* (const XYZval<T>  &rs)        { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const XYZval<T>  &rs)  const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const XYZval<T>  &rs)        { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator+ (const XYZEval<T> &rs)  const { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator+ (const XYZEval<T> &rs)        { XYZval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator- (const XYZEval<T> &rs)  const { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator- (const XYZEval<T> &rs)        { XYZval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator* (const XYZEval<T> &rs)  const { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator* (const XYZEval<T> &rs)        { XYZval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const XYZEval<T> &rs)  const { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const XYZEval<T> &rs)        { XYZval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; return ls; }
  FORCE_INLINE XYZval<T>  operator* (const float &v)        const { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator* (const float &v)              { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator* (const int &v)          const { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator* (const int &v)                { XYZval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const float &v)        const { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const float &v)              { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const int &v)          const { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator/ (const int &v)                { XYZval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= z;    return ls; }
  FORCE_INLINE XYZval<T>  operator>>(const int &v)          const { XYZval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); return ls; }
  FORCE_INLINE XYZval<T>  operator>>(const int &v)                { XYZval<T> ls = *this; _RS(ls.x); _RS(ls.y); _RS(ls.z); return ls; }
  FORCE_INLINE XYZval<T>  operator<<(const int &v)          const { XYZval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); return ls; }
  FORCE_INLINE XYZval<T>  operator<<(const int &v)                { XYZval<T> ls = *this; _LS(ls.x); _LS(ls.y); _LS(ls.z); return ls; }
  FORCE_INLINE XYZval<T>& operator+=(const XYval<T>   &rs)        { x += rs.x; y += rs.y;            return *this; }
  FORCE_INLINE XYZval<T>& operator-=(const XYval<T>   &rs)        { x -= rs.x; y -= rs.y;            return *this; }
  FORCE_INLINE XYZval<T>& operator*=(const XYval<T>   &rs)        { x *= rs.x; y *= rs.y;            return *this; }
  FORCE_INLINE XYZval<T>& operator/=(const XYval<T>   &rs)        { x /= rs.x; y /= rs.y;            return *this; }
  FORCE_INLINE XYZval<T>& operator+=(const XYZval<T>  &rs)        { x += rs.x; y += rs.y; z += rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator-=(const XYZval<T>  &rs)        { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator*=(const XYZval<T>  &rs)        { x *= rs.x; y *= rs.y; z *= rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator/=(const XYZval<T>  &rs)        { x /= rs.x; y /= rs.y; z /= rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator+=(const XYZEval<T> &rs)        { x += rs.x; y += rs.y; z += rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator-=(const XYZEval<T> &rs)        { x -= rs.x; y -= rs.y; z -= rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator*=(const XYZEval<T> &rs)        { x *= rs.x; y *= rs.y; z *= rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator/=(const XYZEval<T> &rs)        { x /= rs.x; y /= rs.y; z /= rs.z; return *this; }
  FORCE_INLINE XYZval<T>& operator*=(const float &v)              { x *= v;    y *= v;    z *= v;    return *this; }
  FORCE_INLINE XYZval<T>& operator*=(const int &v)                { x *= v;    y *= v;    z *= v;    return *this; }
  FORCE_INLINE XYZval<T>& operator/=(const float &v)              { x /= v;    y /= v;    z /= v;    return *this; }
  FORCE_INLINE XYZval<T>& operator/=(const int &v)                { x /= v;    y /= v;    z /= v;    return *this; }
  FORCE_INLINE XYZval<T>& operator+=(const float &v)              { x += v;    y += v;    z += v;    return *this; }
  FORCE_INLINE XYZval<T>& operator+=(const int &v)                { x += v;    y += v;    z += v;    return *this; }
  FORCE_INLINE XYZval<T>& operator-=(const float &v)              { x -= v;    y -= v;    z -= v;    return *this; }
  FORCE_INLINE XYZval<T>& operator-=(const int &v)                { x -= v;    y -= v;    z -= v;    return *this; }
  FORCE_INLINE XYZval<T>& operator>>=(const int &v)               { _RS(x);   _RS(y);   _RS(z);   return *this; }
  FORCE_INLINE XYZval<T>& operator<<=(const int &v)               { _LS(x);   _LS(y);   _LS(z);   return *this; }
  FORCE_INLINE bool       operator==(const XYZEval<T> &rs)        { return x == rs.x && y == rs.y && z == rs.z; }
  FORCE_INLINE bool       operator!=(const XYZEval<T> &rs)        { return !operator==(rs); }
  FORCE_INLINE bool       operator==(const XYZEval<T> &rs)  const { return x == rs.x && y == rs.y && z == rs.z; }
  FORCE_INLINE bool       operator!=(const XYZEval<T> &rs)  const { return !operator==(rs); }
  FORCE_INLINE XYZval<T>       operator-()                        { XYZval<T> o = *this; o.x = -x; o.y = -y; o.z = -z; return o; }
  FORCE_INLINE const XYZval<T> operator-()                  const { XYZval<T> o = *this; o.x = -x; o.y = -y; o.z = -z; return o; }
};

/**
 * XYZE coordinates, counters, etc.
 */
template<typename T>
struct XYZEval {
  union {
    struct { T x, y, z, e; };
    struct { T a, b, c; };
    T pos[4];
  };
  FORCE_INLINE void reset()                                             { x = y = z = e = 0; }
  FORCE_INLINE T magnitude()                                      const { return (T)sqrtf(x*x + y*y + z*z + e*e); }
  FORCE_INLINE operator T* ()                                           { return pos; }
  FORCE_INLINE operator bool()                                          { return e || z || x || y; }
  FORCE_INLINE void set(const T px)                                     { x = px;                                         }
  FORCE_INLINE void set(const T px, const T py)                         { x = px;     y = py;                             }
  FORCE_INLINE void set(const T px, const T py, const T pz)             { x = px;     y = py;     z = pz;                 }
  FORCE_INLINE void set(const T px, const T py, const T pz, const T pe) { x = px;     y = py;     z = pz;     e = pe;     }
  FORCE_INLINE void set(const XYval<T> pxy)                             { x = pxy.x;  y = pxy.y;                          }
  FORCE_INLINE void set(const XYval<T> pxy, const T pz)                 { x = pxy.x;  y = pxy.y;  z = pz;                 }
  FORCE_INLINE void set(const XYZval<T> pxyz)                           { x = pxyz.x; y = pxyz.y; z = pxyz.z;             }
  FORCE_INLINE void set(const XYval<T> pxy, const T pz, const T pe)     { x = pxy.x;  y = pxy.y;  z = pz;     e = pe;     }
  FORCE_INLINE void set(const XYval<T> pxy, const XYval<T> pze)         { x = pxy.x;  y = pxy.y;  z = pze.z;  e = pze.e;  }
  FORCE_INLINE void set(const XYZval<T> pxyz, const T pe)               { x = pxyz.x; y = pxyz.y; z = pxyz.z; e = pe;     }
  FORCE_INLINE void set(const T (&arr)[XY])                             { x = arr[0]; y = arr[1]; }
  FORCE_INLINE void set(const T (&arr)[XYZ])                            { x = arr[0]; y = arr[1]; z = arr[2]; }
  FORCE_INLINE void set(const T (&arr)[XYZE])                           { x = arr[0]; y = arr[1]; z = arr[2]; e = arr[3]; }
  FORCE_INLINE XYZEval<T>          copy()                         const { return *this; }
  FORCE_INLINE XYZEval<T>           ABS()                         const { return { T(_ABS(x)), T(_ABS(y)), T(_ABS(z)), T(_ABS(e)) }; }
  FORCE_INLINE XYZEval<int16_t>   asInt()                               { return { int16_t(x), int16_t(y), int16_t(z), int16_t(e) }; }
  FORCE_INLINE XYZEval<int16_t>   asInt()                         const { return { int16_t(x), int16_t(y), int16_t(z), int16_t(e) }; }
  FORCE_INLINE XYZEval<int32_t>  asLong()                               { return { int32_t(x), int32_t(y), int32_t(z), int32_t(e) }; }
  FORCE_INLINE XYZEval<int32_t>  asLong()                         const { return { int32_t(x), int32_t(y), int32_t(z), int32_t(e) }; }
  FORCE_INLINE XYZEval<int32_t>  ROUNDL()                               { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(e)) }; }
  FORCE_INLINE XYZEval<int32_t>  ROUNDL()                         const { return { int32_t(LROUND(x)), int32_t(LROUND(y)), int32_t(LROUND(z)), int32_t(LROUND(e)) }; }
  FORCE_INLINE XYZEval<float>   asFloat()                               { return {   float(x),   float(y),   float(z),   float(e) }; }
  FORCE_INLINE XYZEval<float>   asFloat()                         const { return {   float(x),   float(y),   float(z),   float(e) }; }
  FORCE_INLINE XYZEval<float> reciprocal()                        const { return {  _RECIP(x),  _RECIP(y),  _RECIP(z),  _RECIP(e) }; }
  FORCE_INLINE XYZEval<float> asLogical()                         const { XYZEval<float> o = asFloat(); toLogical(o); return o; }
  FORCE_INLINE XYZEval<float>  asNative()                         const { XYZEval<float> o = asFloat(); toNative(o);  return o; }
  FORCE_INLINE operator       XYval<T>&()                               { return *(XYval<T>*)this; }
  FORCE_INLINE operator const XYval<T>&()                         const { return *(const XYval<T>*)this; }
  FORCE_INLINE operator       XYZval<T>&()                              { return *(XYZval<T>*)this; }
  FORCE_INLINE operator const XYZval<T>&()                        const { return *(const XYZval<T>*)this; }
  FORCE_INLINE       T&    operator[](const int i)                      { return pos[i]; }
  FORCE_INLINE const T&    operator[](const int i)                const { return pos[i]; }
  FORCE_INLINE XYZEval<T>& operator= (const T v)                        { set(v, v, v, v); return *this; }
  FORCE_INLINE XYZEval<T>& operator= (const XYval<T>   &rs)             { set(rs.x, rs.y); return *this; }
  FORCE_INLINE XYZEval<T>& operator= (const XYZval<T>  &rs)             { set(rs.x, rs.y, rs.z); return *this; }
  FORCE_INLINE XYZEval<T>  operator+ (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator+ (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator- (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator- (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const XYval<T>   &rs)       const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const XYval<T>   &rs)             { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y;                             return ls; }
  FORCE_INLINE XYZEval<T>  operator+ (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator+ (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator- (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator- (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const XYZval<T>  &rs)       const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const XYZval<T>  &rs)             { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z;               return ls; }
  FORCE_INLINE XYZEval<T>  operator+ (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; ls.e += rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator+ (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x += rs.x; ls.y += rs.y; ls.z += rs.z; ls.e += rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator- (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; ls.e -= rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator- (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x -= rs.x; ls.y -= rs.y; ls.z -= rs.z; ls.e -= rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; ls.e *= rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x *= rs.x; ls.y *= rs.y; ls.z *= rs.z; ls.e *= rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const XYZEval<T> &rs)       const { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; ls.e /= rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const XYZEval<T> &rs)             { XYZEval<T> ls = *this; ls.x /= rs.x; ls.y /= rs.y; ls.z /= rs.z; ls.e /= rs.e; return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const float &v)             const { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const float &v)                   { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const int &v)               const { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator* (const int &v)                     { XYZEval<T> ls = *this; ls.x *= v;    ls.y *= v;    ls.z *= v;    ls.e *= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const float &v)             const { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const float &v)                   { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const int &v)               const { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator/ (const int &v)                     { XYZEval<T> ls = *this; ls.x /= v;    ls.y /= v;    ls.z /= v;    ls.e /= v;    return ls; }
  FORCE_INLINE XYZEval<T>  operator>>(const int &v)               const { XYZEval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    _RS(ls.z);    _RS(ls.e);    return ls; }
  FORCE_INLINE XYZEval<T>  operator>>(const int &v)                     { XYZEval<T> ls = *this; _RS(ls.x);    _RS(ls.y);    _RS(ls.z);    _RS(ls.e);    return ls; }
  FORCE_INLINE XYZEval<T>  operator<<(const int &v)               const { XYZEval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    _LS(ls.z);    _LS(ls.e);    return ls; }
  FORCE_INLINE XYZEval<T>  operator<<(const int &v)                     { XYZEval<T> ls = *this; _LS(ls.x);    _LS(ls.y);    _LS(ls.z);    _LS(ls.e);    return ls; }
  FORCE_INLINE XYZEval<T>& operator+=(const XYval<T>   &rs)             { x += rs.x; y += rs.y;                       return *this; }
  FORCE_INLINE XYZEval<T>& operator-=(const XYval<T>   &rs)             { x -= rs.x; y -= rs.y;                       return *this; }
  FORCE_INLINE XYZEval<T>& operator*=(const XYval<T>   &rs)             { x *= rs.x; y *= rs.y;                       return *this; }
  FORCE_INLINE XYZEval<T>& operator/=(const XYval<T>   &rs)             { x /= rs.x; y /= rs.y;                       return *this; }
  FORCE_INLINE XYZEval<T>& operator+=(const XYZval<T>  &rs)             { x += rs.x; y += rs.y; z += rs.z;            return *this; }
  FORCE_INLINE XYZEval<T>& operator-=(const XYZval<T>  &rs)             { x -= rs.x; y -= rs.y; z -= rs.z;            return *this; }
  FORCE_INLINE XYZEval<T>& operator*=(const XYZval<T>  &rs)             { x *= rs.x; y *= rs.y; z *= rs.z;            return *this; }
  FORCE_INLINE XYZEval<T>& operator/=(const XYZval<T>  &rs)             { x /= rs.x; y /= rs.y; z /= rs.z;            return *this; }
  FORCE_INLINE XYZEval<T>& operator+=(const XYZEval<T> &rs)             { x += rs.x; y += rs.y; z += rs.z; e += rs.e; return *this; }
  FORCE_INLINE XYZEval<T>& operator-=(const XYZEval<T> &rs)             { x -= rs.x; y -= rs.y; z -= rs.z; e -= rs.e; return *this; }
  FORCE_INLINE XYZEval<T>& operator*=(const XYZEval<T> &rs)             { x *= rs.x; y *= rs.y; z *= rs.z; e *= rs.e; return *this; }
  FORCE_INLINE XYZEval<T>& operator/=(const XYZEval<T> &rs)             { x /= rs.x; y /= rs.y; z /= rs.z; e /= rs.e; return *this; }
  FORCE_INLINE XYZEval<T>& operator*=(const T &v)                       { x *= v;    y *= v;    z *= v;    e *= v;    return *this; }
  FORCE_INLINE XYZEval<T>& operator/=(const T &v)                       { x /= v;    y /= v;    z /= v;    e /= v;    return *this; }
  FORCE_INLINE XYZEval<T>& operator+=(const T &v)                       { x += v;    y += v;    z += v;    e += v;    return *this; }
  FORCE_INLINE XYZEval<T>& operator-=(const T &v)                       { x -= v;    y -= v;    z -= v;    e -= v;    return *this; }
  FORCE_INLINE XYZEval<T>& operator>>=(const int &v)                    { _RS(x);    _RS(y);    _RS(z);    _RS(e);    return *this; }
  FORCE_INLINE XYZEval<T>& operator<<=(const int &v)                    { _LS(x);    _LS(y);    _LS(z);    _LS(e);    return *this; }
  FORCE_INLINE bool        operator==(const XYZval<T>  &rs)             { return x == rs.x && y == rs.y && z == rs.z; }
  FORCE_INLINE bool        operator!=(const XYZval<T>  &rs)             { return !operator==(rs); }
  FORCE_INLINE bool        operator==(const XYZval<T>  &rs)       const { return x == rs.x && y == rs.y && z == rs.z; }
  FORCE_INLINE bool        operator!=(const XYZval<T>  &rs)       const { return !operator==(rs); }
  FORCE_INLINE       XYZEval<T> operator-()                             { return { -x, -y, -z, -e }; }
  FORCE_INLINE const XYZEval<T> operator-()                       const { return { -x, -y, -z, -e }; }
};

/**
 * Val limit min max
 */
template<typename T>
struct MinMaxVal {
  union {
    struct { T min, max; };
    T val[2];
  };
  FORCE_INLINE void set(const T pmin)                                    { min = pmin; }
  FORCE_INLINE void set(const T pmin, const T pmax)                      { min = pmin; max = pmax; }
  FORCE_INLINE void reset()                                              { min = max = 0; }
  FORCE_INLINE T magnitude()                                       const { return (T)sqrtf(min*min + max*max); }
  FORCE_INLINE operator T* ()                                            { return val; }
  FORCE_INLINE operator bool()                                           { return min || max; }
  FORCE_INLINE MinMaxVal<T>           copy()                       const { MinMaxVal<T> o = *this; return o; }
  FORCE_INLINE MinMaxVal<T>            ABS()                       const { MinMaxVal<T> o; o.set(_ABS(min), _ABS(max)); return o; }
  FORCE_INLINE MinMaxVal<int16_t>    asInt()                             { MinMaxVal<int16_t> o = { int16_t(min), int16_t(max) }; return o; }
  FORCE_INLINE MinMaxVal<int16_t>    asInt()                       const { MinMaxVal<int16_t> o = { int16_t(min), int16_t(max) }; return o; }
  FORCE_INLINE MinMaxVal<int32_t>   asLong()                             { MinMaxVal<int32_t> o = { int32_t(min), int32_t(max) }; return o; }
  FORCE_INLINE MinMaxVal<int32_t>   asLong()                       const { MinMaxVal<int32_t> o = { int32_t(min), int32_t(max) }; return o; }
  FORCE_INLINE MinMaxVal<float>    asFloat()                             { MinMaxVal<float>   o = {   float(min),   float(max) }; return o; }
  FORCE_INLINE MinMaxVal<float>    asFloat()                       const { MinMaxVal<float>   o = {   float(min),   float(max) }; return o; }
  FORCE_INLINE operator XYZval<T>()                                      { MinMaxVal<T> o = { min, max }; return o; }
  FORCE_INLINE operator XYZval<T>()                                const { MinMaxVal<T> o = { min, max }; return o; }
  FORCE_INLINE operator XYZEval<T>()                                     { XYZEval<T> o = { min, max }; return o; }
  FORCE_INLINE operator XYZEval<T>()                               const { XYZEval<T> o = { min, max }; return o; }
  FORCE_INLINE       T&      operator[](const int i)                     { return val[i]; }
  FORCE_INLINE const T&      operator[](const int i)               const { return val[i]; }
  FORCE_INLINE MinMaxVal<T>& operator= (const T v)                       { set(v, v);           return *this; }
  FORCE_INLINE MinMaxVal<T>& operator= (const MinMaxVal<T>   &rs)        { set(rs.min, rs.max); return *this; }
  FORCE_INLINE MinMaxVal<T>  operator+ (const MinMaxVal<T>   &rs)  const { MinMaxVal<T> ls = *this; ls.min += rs.min; ls.max += rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator+ (const MinMaxVal<T>   &rs)        { MinMaxVal<T> ls = *this; ls.min += rs.min; ls.max += rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator- (const MinMaxVal<T>   &rs)  const { MinMaxVal<T> ls = *this; ls.min -= rs.min; ls.max -= rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator- (const MinMaxVal<T>   &rs)        { MinMaxVal<T> ls = *this; ls.min -= rs.min; ls.max -= rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator* (const MinMaxVal<T>   &rs)  const { MinMaxVal<T> ls = *this; ls.min *= rs.min; ls.max *= rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator* (const MinMaxVal<T>   &rs)        { MinMaxVal<T> ls = *this; ls.min *= rs.min; ls.max *= rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator/ (const MinMaxVal<T>   &rs)  const { MinMaxVal<T> ls = *this; ls.min /= rs.min; ls.max /= rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator/ (const MinMaxVal<T>   &rs)        { MinMaxVal<T> ls = *this; ls.min /= rs.min; ls.max /= rs.max; return ls; }
  FORCE_INLINE MinMaxVal<T>  operator* (const float &v)            const { MinMaxVal<T> ls = *this; ls.min *= v;    ls.max *= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator* (const float &v)                  { MinMaxVal<T> ls = *this; ls.min *= v;    ls.max *= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator* (const int &v)              const { MinMaxVal<T> ls = *this; ls.min *= v;    ls.max *= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator* (const int &v)                    { MinMaxVal<T> ls = *this; ls.min *= v;    ls.max *= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator/ (const float &v)            const { MinMaxVal<T> ls = *this; ls.min /= v;    ls.max /= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator/ (const float &v)                  { MinMaxVal<T> ls = *this; ls.min /= v;    ls.max /= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator/ (const int &v)              const { MinMaxVal<T> ls = *this; ls.min /= v;    ls.max /= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator/ (const int &v)                    { MinMaxVal<T> ls = *this; ls.min /= v;    ls.max /= v;    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator>>(const int &v)              const { MinMaxVal<T> ls = *this; _RS(ls.min);    _RS(ls.max);    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator>>(const int &v)                    { MinMaxVal<T> ls = *this; _RS(ls.min);    _RS(ls.max);    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator<<(const int &v)              const { MinMaxVal<T> ls = *this; _LS(ls.min);    _LS(ls.max);    return ls; }
  FORCE_INLINE MinMaxVal<T>  operator<<(const int &v)                    { MinMaxVal<T> ls = *this; _LS(ls.min);    _LS(ls.max);    return ls; }
  FORCE_INLINE MinMaxVal<T>& operator+=(const MinMaxVal<T>   &rs)        { min += rs.min; max += rs.max; return *this; }
  FORCE_INLINE MinMaxVal<T>& operator-=(const MinMaxVal<T>   &rs)        { min -= rs.min; max -= rs.max; return *this; }
  FORCE_INLINE MinMaxVal<T>& operator*=(const MinMaxVal<T>   &rs)        { min *= rs.min; max *= rs.max; return *this; }
  FORCE_INLINE MinMaxVal<T>& operator*=(const T &v)                      { min *= v;    max *= v;    return *this; }
  FORCE_INLINE MinMaxVal<T>& operator/=(const T &v)                      { min /= v;    max /= v;    return *this; }
  FORCE_INLINE MinMaxVal<T>& operator+=(const T &v)                      { min += v;    max += v;    return *this; }
  FORCE_INLINE MinMaxVal<T>& operator-=(const T &v)                      { min -= v;    max -= v;    return *this; }
  FORCE_INLINE MinMaxVal<T>& operator>>=(const int &v)                   { _RS(min);    _RS(max);    return *this; }
  FORCE_INLINE MinMaxVal<T>& operator<<=(const int &v)                   { _LS(min);    _LS(max);    return *this; }
  FORCE_INLINE bool          operator==(const MinMaxVal<T>   &rs)        { return min == rs.min && max == rs.max; }
  FORCE_INLINE bool          operator==(const MinMaxVal<T>   &rs)  const { return min == rs.min && max == rs.max; }
  FORCE_INLINE bool          operator!=(const MinMaxVal<T>   &rs)        { return !operator==(rs); }
  FORCE_INLINE bool          operator!=(const MinMaxVal<T>   &rs)  const { return !operator==(rs); }
  FORCE_INLINE MinMaxVal<T>       operator-()                            { MinMaxVal<T> o = *this; o.min = -min; o.max = -max; return o; }
  FORCE_INLINE const MinMaxVal<T> operator-()                      const { MinMaxVal<T> o = *this; o.min = -min; o.max = -max; return o; }
};

typedef struct MinMaxVal<uint8_t>   limit_uchar_t;
typedef struct MinMaxVal<int16_t>     limit_int_t;
typedef struct MinMaxVal<float>     limit_float_t;

/**
 * XYY Limit
 */
template<typename T>
struct XYZlimit {
  XYZval<T> min, max;
};

typedef struct XYZlimit<float>  xyz_limit_float_t;

/**
 * Axix code constant
 */
const xyze_char_t axis_codes { 'X', 'Y', 'Z', 'E' };

#undef _RECIP
#undef _ABS
#undef _LS
#undef _RS
