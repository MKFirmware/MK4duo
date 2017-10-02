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

#ifndef MACROS_H
#define MACROS_H

// The axis order in all axis related arrays is X, Y, Z, E
#define NUM_AXIS  4
#define XYZE      4
#define ABCE      4
#define ABC       3
#define XYZ       3

// Function macro
#define FORCE_INLINE  __attribute__((always_inline)) inline
#define _UNUSED       __attribute__((unused))
#define _O0           __attribute__((optimize("O0")))
#define _Os           __attribute__((optimize("Os")))
#define _O1           __attribute__((optimize("O1")))
#define _O2           __attribute__((optimize("O2")))
#define _O3           __attribute__((optimize("O3")))

/**
 * Macros for mechanics type
 */
#define MECH_UNKNOWN        -1
#define MECH_CARTESIAN       0
#define MECH_COREXY          1
#define MECH_COREYX          2
#define MECH_DELTA           3
#define MECH_MORGAN_SCARA    4
#define MECH_MAKERARM_SCARA  5
#define MECH_COREXZ          8
#define MECH_COREZX          9
#define MECH_COREYZ         10
#define MECH_COREZY         11
#define MECH_MUVE3D         21

#define MECH(mech)    (MECHANISM == MECH_##mech)
#define NOMECH(mech)  (MECHANISM != MECH_##mech)

#define IS_CARTESIAN  (MECH(CARTESIAN))

#define IS_DELTA      (MECH(DELTA))
#define IS_SCARA      (MECH(MORGAN_SCARA) || MECH(MAKERARM_SCARA))
#define IS_KINEMATIC  (IS_DELTA || IS_SCARA)

#define CORE_IS_XY    (MECH(COREXY) || MECH(COREYX))
#define CORE_IS_XZ    (MECH(COREXZ) || MECH(COREZX))
#define CORE_IS_YZ    (MECH(COREYZ) || MECH(COREZY))
#define IS_CORE       (CORE_IS_XY || CORE_IS_XZ || CORE_IS_YZ)

#define IS_MUVE3D     (MECH(MUVE3D))
/********************************************************************/

// Compiler warning on unused varable.
#define UNUSED(x) (void) (x)

// Macros to make a string from a macro
#define STRINGIFY_(M) #M
#define STRINGIFY(M)  STRINGIFY_(M)

// Macros for communication
#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

// Macros for bit masks
#ifndef _BV
  #define _BV(b)            (1 << (b))
#endif
#define TEST(n,b)           (((n)&_BV(b))!=0)
#define SBI(n,b)            (n |= _BV(b))
#define CBI(n,b)            (n &= ~_BV(b))
#define SET_BIT(n,b,value)  (n) ^= ((-value)^(n)) & (_BV(b))

// Macros for maths shortcuts
#ifndef M_PI 
  #define M_PI      3.14159265358979323846
#endif
#define RADIANS(d)  ((d)*M_PI/180.0)
#define DEGREES(r)  ((r)*180.0/M_PI)
#define HYPOT2(x,y) (sq(x)+sq(y))
#define HYPOT(x,y)  SQRT(HYPOT2(x,y))
#define SQUARE(x)   ((x)*(x))
#define SIN_60      0.8660254037844386
#define COS_60      0.5

#define CIRCLE_AREA(R)    (M_PI * sq(R))
#define CIRCLE_CIRC(R)    (2.0 * M_PI * (R))

#define SIGN(a)           ((a>0)-(a<0))
#define IS_POWER_OF_2(x)  ((x) && !((x) & ((x) - 1)))

// Macros to contrain values
#define NOLESS(v,n)       v = (v < n) ? n : v
#define NOMORE(v,n)       v = (v > n) ? n : v

// Macros to support option testing
#define ENABLED defined
#define DISABLED !defined

#define WITHIN(V,L,H)     ((V) >= (L) && (V) <= (H))
#define NUMERIC(a)        WITHIN(a, '0', '9')
#define DECIMAL(a)        (NUMERIC(a) || a == '.')
#define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-' || (a) == '+')
#define DECIMAL_SIGNED(a) (DECIMAL(a) || (a) == '-' || (a) == '+')
#define COUNT(a)          (sizeof(a) / sizeof(*a))
#define ZERO(a)           memset(a, 0, sizeof(a))
#define COPY_ARRAY(a,b)   memcpy(a, b, min(sizeof(a), sizeof(b)))

// Macros for initializing arrays
#define ARRAY_12(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, ...)  { v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12 }
#define ARRAY_11(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, ...)       { v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11 }
#define ARRAY_10(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, ...)            { v1, v2, v3, v4, v5, v6, v7, v8, v9, v10 }
#define ARRAY_9(v1, v2, v3, v4, v5, v6, v7, v8, v9, ...)                  { v1, v2, v3, v4, v5, v6, v7, v8, v9 }
#define ARRAY_8(v1, v2, v3, v4, v5, v6, v7, v8, ...)                      { v1, v2, v3, v4, v5, v6, v7, v8 }
#define ARRAY_7(v1, v2, v3, v4, v5, v6, v7, ...)                          { v1, v2, v3, v4, v5, v6, v7 }
#define ARRAY_6(v1, v2, v3, v4, v5, v6, ...)                              { v1, v2, v3, v4, v5, v6 }
#define ARRAY_5(v1, v2, v3, v4, v5, ...)                                  { v1, v2, v3, v4, v5 }
#define ARRAY_4(v1, v2, v3, v4, ...)                                      { v1, v2, v3, v4 }
#define ARRAY_3(v1, v2, v3, ...)                                          { v1, v2, v3 }
#define ARRAY_2(v1, v2, ...)                                              { v1, v2 }
#define ARRAY_1(v1, ...)                                                  { v1 }
#define ARRAY_0(...)                                                      { }

#define _ARRAY_N(N, ...) ARRAY_ ##N(__VA_ARGS__)
#define ARRAY_N(N, ...) _ARRAY_N(N, __VA_ARGS__)

// ARRAY_BY_N based
#define ARRAY_BY_N_N(N, ...) ARRAY_N(N, __VA_ARGS__)
#define ARRAY_BY_N(N, v1) ARRAY_BY_N_N(N, v1, v1, v1, v1, v1, v1, v1, v1, v1, v1, v1, v1)

// ARRAY_BY_EXTRUDERS based on EXTRUDERS
#define ARRAY_BY_EXTRUDERS_N(...) ARRAY_N(EXTRUDERS, __VA_ARGS__)
#define ARRAY_BY_EXTRUDERS(v1) ARRAY_BY_EXTRUDERS_N(v1, v1, v1, v1, v1, v1, v1, v1, v1, v1, v1, v1)

// ARRAY_BY_HOTENDS based on HOTENDS
#define ARRAY_BY_HOTENDS_N(...) ARRAY_N(HOTENDS, __VA_ARGS__)
#define ARRAY_BY_HOTENDS(v1) ARRAY_BY_HOTENDS_N(v1, v1, v1, v1, v1, v1)

// ARRAY_BY_FAN based on FAN_COUNT
#define ARRAY_BY_FANS_N(...) ARRAY_N(FAN_COUNT, __VA_ARGS__)
#define ARRAY_BY_FANS(v1) ARRAY_BY_FANS_N(v1, v1, v1, v1, v1, v1)

#define PIN_EXISTS(PN) (defined(PN##_PIN) && PN##_PIN >= 0)

#define PENDING(NOW,SOON) ((long)(NOW-(SOON))<0)
#define ELAPSED(NOW,SOON) (!PENDING(NOW,SOON))

#define NOOP do{}while(0)

#define CEILING(x,y) (((x) + (y) - 1) / (y))

#define MIN3(a, b, c)     min(min(a, b), c)
#define MIN4(a, b, c, d)  min(min(a, b), min(c, d))
#define MAX3(a, b, c)     max(max(a, b), c)
#define MAX4(a, b, c, d)  max(max(a, b), max(c, d))

#define UNEAR_ZERO(x)     ((x) < 0.000001)
#define NEAR_ZERO(x)      ((x) > -0.000001 && (x) < 0.000001)
#define NEAR(x,y)         NEAR_ZERO((x)-(y))

#define RECIPROCAL(x)     (NEAR_ZERO(x) ? 0.0 : 1.0 / (x))
#define FIXFLOAT(f)       (f + 0.00001)

// LOOP MACROS
#define LOOP_S_LE_N(VAR, S, N)  for (uint8_t VAR=S; VAR<=N; VAR++)
#define LOOP_S_L_N(VAR, S, N)   for (uint8_t VAR=S; VAR<N; VAR++)
#define LOOP_LE_N(VAR, N)       LOOP_S_LE_N(VAR, 0, N)
#define LOOP_L_N(VAR, N)        LOOP_S_L_N(VAR, 0, N)

#define LOOP_NA(VAR)            LOOP_L_N(VAR, NUM_AXIS)
#define LOOP_XY(VAR)            LOOP_S_LE_N(VAR, X_AXIS, Y_AXIS)
#define LOOP_XYZ(VAR)           LOOP_S_LE_N(VAR, X_AXIS, Z_AXIS)
#define LOOP_XYZE(VAR)          LOOP_S_LE_N(VAR, X_AXIS, E_AXIS)
#define LOOP_XYZE_N(VAR)        LOOP_S_L_N(VAR, X_AXIS, XYZE_N)
#define LOOP_HOTEND()           LOOP_L_N(h, HOTENDS)
#define LOOP_HEATER()           LOOP_L_N(h, HEATER_COUNT)
#define LOOP_FAN()              LOOP_L_N(f, FAN_COUNT)

// Feedrate scaling and conversion
#define MMM_TO_MMS(MM_M) ((MM_M) / 60.0)
#define MMS_TO_MMM(MM_S) ((MM_S) * 60.0)
#define MMS_SCALED(MM_S) ((MM_S) * mechanics.feedrate_percentage * 0.01)

#endif //__MACROS_H
