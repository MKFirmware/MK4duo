/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

// Clock speed factor
#define CYCLES_PER_MICROSECOND (F_CPU / 1000000UL) // 16 or 20

// Compiler warning on unused varable.
#define UNUSED(x) (void) (x)

// Macros to make a string from a macro
#define STRINGIFY_(M) #M
#define STRINGIFY(M) STRINGIFY_(M)

// Macros for communication
#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

// Macros for bit masks
#ifndef _BV
  #define _BV(b) (1<<(b))
#endif
#define TEST(n,b) (((n)&_BV(b))!=0)
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))
#define SET_BIT(n,b,value) (n) ^= ((-value)^(n)) & (_BV(b))

// Macros for maths shortcuts
#ifndef M_PI 
  #define M_PI 3.1415926536
#endif
#define RADIANS(d)  ((d)*M_PI/180.0)
#define DEGREES(r)  ((r)*180.0/M_PI)
#define HYPOT2(x,y) (sq(x)+sq(y))
#define HYPOT(x,y)  sqrt(HYPOT2(x,y))
#define SQUARE(x)   ((x)*(x))
#define SIN_60 0.8660254037844386
#define COS_60 0.5

// Macros to support option testing
#define ENABLED defined
#define DISABLED !defined

#define HAS(FE) (HAS_##FE)
#define HASNT(FE) (!(HAS_##FE))

// Macros to contrain values
#define NUMERIC(a) ((a) >= '0' && '9' >= (a))
#define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-')
#define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
#define NOMORE(v,n) do{ if (v > n) v = n; }while(0)
#define COUNT(a) (sizeof(a)/sizeof(*a))
#define ZERO(a) memset(a,0,sizeof(a))

// Function macro
#define FORCE_INLINE __attribute__((always_inline)) inline

// Macro for debugging
#define DEBUGGING(F) (mk_debug_flags & (DEBUG_## F))

// Macros for initializing arrays
#define ARRAY_9(v1, v2, v3, v4, v5, v6, v7, v8, v9, ...)  { v1, v2, v3, v4, v5, v6, v7, v8, v9 }
#define ARRAY_8(v1, v2, v3, v4, v5, v6, v7, v8, ...)      { v1, v2, v3, v4, v5, v6, v7, v8 }
#define ARRAY_7(v1, v2, v3, v4, v5, v6, v7, ...)          { v1, v2, v3, v4, v5, v6, v7 }
#define ARRAY_6(v1, v2, v3, v4, v5, v6, ...)              { v1, v2, v3, v4, v5, v6 }
#define ARRAY_5(v1, v2, v3, v4, v5, ...)                  { v1, v2, v3, v4, v5 }
#define ARRAY_4(v1, v2, v3, v4, ...)                      { v1, v2, v3, v4 }
#define ARRAY_3(v1, v2, v3, ...)                          { v1, v2, v3 }
#define ARRAY_2(v1, v2, ...)                              { v1, v2 }
#define ARRAY_1(v1, ...)                                  { v1 }

#define _ARRAY_N(N, ...) ARRAY_ ##N(__VA_ARGS__)
#define ARRAY_N(N, ...) _ARRAY_N(N, __VA_ARGS__)

// ARRAY_BY_N based
#define ARRAY_BY_N_N(N, ...) ARRAY_N(N, __VA_ARGS__)
#define ARRAY_BY_N(N, v1) ARRAY_BY_N_N(N, v1, v1, v1, v1, v1, v1, v1, v1, v1)

// ARRAY_BY_EXTRUDERS based on EXTRUDERS
#define ARRAY_BY_EXTRUDERS_N(...) ARRAY_N(EXTRUDERS, __VA_ARGS__)
#define ARRAY_BY_EXTRUDERS(v1) ARRAY_BY_EXTRUDERS_N(v1, v1, v1, v1, v1, v1)

// ARRAY_BY_HOTENDS based on HOTENDS
#define ARRAY_BY_HOTENDS_N(...) ARRAY_N(HOTENDS, __VA_ARGS__)
#define ARRAY_BY_HOTENDS(v1) ARRAY_BY_HOTENDS_N(v1, v1, v1, v1, v1, v1)

// Macros for adding
#define INC_0 1
#define INC_1 2
#define INC_2 3
#define INC_3 4
#define INC_4 5
#define INC_5 6
#define INC_6 7
#define INC_7 8
#define INC_8 9
#define INCREMENT_(n) INC_ ##n
#define INCREMENT(n) INCREMENT_(n)

// Macros for subtracting
#define DEC_1 0
#define DEC_2 1
#define DEC_3 2
#define DEC_4 3
#define DEC_5 4
#define DEC_6 5
#define DEC_7 6
#define DEC_8 7
#define DEC_9 8
#define DECREMENT_(n) DEC_ ##n
#define DECREMENT(n) DECREMENT_(n)

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

#define _AXIS(AXIS) AXIS ##_AXIS

#define LOOP_XYZ(VAR)     for (uint8_t VAR = X_AXIS; VAR <= Z_AXIS; VAR++)
#define LOOP_XYZE(VAR)    for (uint8_t VAR = X_AXIS; VAR <= E_AXIS; VAR++)
#define LOOP_XYZE_N(VAR)  for (uint8_t VAR = X_AXIS; VAR < XYZE_N; VAR++)

// Feedrate scaling and conversion
#define MMM_TO_MMS(MM_M) ((MM_M) / 60.0)
#define MMS_TO_MMM(MM_S) ((MM_S) * 60.0)
#define MMS_SCALED(MM_S) ((MM_S) * feedrate_percentage * 0.01)

#endif //__MACROS_H
