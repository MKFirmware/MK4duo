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

// Models
#define _A4988              0x4988
#define _A5984              0x5984
#define _DRV8825            0x8825
#define _LV8729             0x8729
#define _L6470              0x6470
#define _L6474              0x6474
#define _L6480              0x6480
#define _TB6560             0x6560
#define _TB6600             0x6600
#define _TMC2100            0x2100
#define _TMC2130            0x2130A
#define _TMC2130_STANDALONE 0x2130B
#define _TMC2160            0x2160A
#define _TMC2160_STANDALONE 0x2160B
#define _TMC2208            0x2208A
#define _TMC2208_STANDALONE 0x2208B
#define _TMC2660            0x2660A
#define _TMC2660_STANDALONE 0x2660B
#define _TMC5130            0x5130A
#define _TMC5130_STANDALONE 0x5130B
#define _TMC5160            0x5160A
#define _TMC5160_STANDALONE 0x5160B
#define _TMC5161            0x5161A
#define _TMC5161_STANDALONE 0x5161B

// Type
#define _ACTUAL(V)          _CAT(_, V)
#define AXIS_DRV_TYPE(ST,T) (_ACTUAL(ST##_DRIVER_TYPE) == _CAT(_, T))
#define  X_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(X,TYPE))
#define  Y_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(Y,TYPE))
#define  Z_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(Z,TYPE))
#define X2_HAS_DRV(TYPE)    ((ENABLED(X_TWO_STEPPER_DRIVERS) || ENABLED(DUAL_X_CARRIAGE))         && AXIS_DRV_TYPE(X2,TYPE))
#define Y2_HAS_DRV(TYPE)    (ENABLED(Y_TWO_STEPPER_DRIVERS)                                       && AXIS_DRV_TYPE(Y2,TYPE))
#define Z2_HAS_DRV(TYPE)    ((ENABLED(Z_TWO_STEPPER_DRIVERS) || ENABLED(Z_THREE_STEPPER_DRIVERS)) && AXIS_DRV_TYPE(Z2,TYPE))
#define Z3_HAS_DRV(TYPE)    (ENABLED(Z_THREE_STEPPER_DRIVERS)                                     && AXIS_DRV_TYPE(Z3,TYPE))
#define E0_HAS_DRV(TYPE)    (MAX_DRIVER_E > 0 && AXIS_DRV_TYPE(E0,TYPE))
#define E1_HAS_DRV(TYPE)    (MAX_DRIVER_E > 1 && AXIS_DRV_TYPE(E1,TYPE))
#define E2_HAS_DRV(TYPE)    (MAX_DRIVER_E > 2 && AXIS_DRV_TYPE(E2,TYPE))
#define E3_HAS_DRV(TYPE)    (MAX_DRIVER_E > 3 && AXIS_DRV_TYPE(E3,TYPE))
#define E4_HAS_DRV(TYPE)    (MAX_DRIVER_E > 4 && AXIS_DRV_TYPE(E4,TYPE))
#define E5_HAS_DRV(TYPE)    (MAX_DRIVER_E > 5 && AXIS_DRV_TYPE(E5,TYPE))

#define HAVE_E_DRV(TYPE)    ( E0_HAS_DRV(TYPE) || E1_HAS_DRV(TYPE) || E2_HAS_DRV(TYPE)  \
                           || E3_HAS_DRV(TYPE) || E4_HAS_DRV(TYPE) || E5_HAS_DRV(TYPE) )

#define HAVE_DRV(TYPE)      ( X_HAS_DRV(TYPE) ||  Y_HAS_DRV(TYPE) ||  Z_HAS_DRV(TYPE)   \
                          || X2_HAS_DRV(TYPE) || Y2_HAS_DRV(TYPE) || Z2_HAS_DRV(TYPE)   \
                                                                  || Z3_HAS_DRV(TYPE)   \
                          || HAVE_E_DRV(TYPE) )

#define HAS_TRINAMIC              (HAVE_DRV(TMC2130)      || HAVE_DRV(TMC2160)      || HAVE_DRV(TMC2208)    || HAVE_DRV(TMC2660)    || HAVE_DRV(TMC5130)    || HAVE_DRV(TMC5160)    || HAVE_DRV(TMC5161))
#define AXIS_HAS_TMC(ST)          (ST##_HAS_DRV(TMC2130)  || ST##_HAS_DRV(TMC2160)  || ST##_HAS_DRV(TMC2208)|| ST##_HAS_DRV(TMC2660)|| ST##_HAS_DRV(TMC5130)|| ST##_HAS_DRV(TMC5160)|| ST##_HAS_DRV(TMC5161))
#define HAS_TMCX1XX               (HAVE_DRV(TMC2130)      || HAVE_DRV(TMC2160)      || HAVE_DRV(TMC5130)    || HAVE_DRV(TMC5160)    || HAVE_DRV(TMC5161))
#define TMC_HAS_SPI               (HAS_TMCX1XX            || HAVE_DRV(TMC2660))
#define TMC_HAS_STALLGUARD        (HAS_TMCX1XX            || HAVE_DRV(TMC2660))
#define TMC_HAS_STEALTHCHOP       (HAS_TMCX1XX            || HAVE_DRV(TMC2208))
#define AXIS_HAS_SPI(ST)          (ST##_HAS_DRV(TMC2130)  || ST##_HAS_DRV(TMC2160)  || ST##_HAS_DRV(TMC2660)  || ST##_HAS_DRV(TMC5130)  || ST##_HAS_DRV(TMC5160)  || ST##_HAS_DRV(TMC5161))
#define AXIS_HAS_UART(ST)         (ST##_HAS_DRV(TMC2208))
#define AXIS_HAS_STALLGUARD(ST)   (ST##_HAS_DRV(TMC2130)  || ST##_HAS_DRV(TMC2160)  || ST##_HAS_DRV(TMC2660)  || ST##_HAS_DRV(TMC5130)  || ST##_HAS_DRV(TMC5160)  || ST##_HAS_DRV(TMC5161))
#define AXIS_HAS_STEALTHCHOP(ST)  (ST##_HAS_DRV(TMC2130)  || ST##_HAS_DRV(TMC2160)  || ST##_HAS_DRV(TMC2208)  || ST##_HAS_DRV(TMC5130)  || ST##_HAS_DRV(TMC5160)  || ST##_HAS_DRV(TMC5161))

#define HAS_SENSORLESS            (TMC_HAS_STALLGUARD && (ENABLED(SENSORLESS_HOMING) || ENABLED(PROBE_SENSORLESS)))

#define HAS_L64XX                 (HAVE_DRV(L6470) || HAVE_DRV(L6474) || HAVE_DRV(L6480))
#define HAS_L64XX_NOT_L6474       (HAS_L64XX && !HAVE_DRV(L6474))

#define AXIS_HAS_L64XX(ST)        (ST##_HAS_DRV(L6470) || ST##_HAS_DRV(L6474) || ST##_HAS_DRV(L6480))

// Disable Z axis sensorless homing if a probe is used to home the Z axis
#if HAS_TRINAMIC
  #if HOMING_Z_WITH_PROBE
    #undef Z_STALL_SENSITIVITY
    #define Z_STALL_SENSITIVITY 0
  #endif
  #define X_HAS_SENSORLESS    (AXIS_HAS_STALLGUARD(X) && ENABLED(X_STALL_SENSITIVITY))
  #define Y_HAS_SENSORLESS    (AXIS_HAS_STALLGUARD(Y) && ENABLED(Y_STALL_SENSITIVITY))
  #define Z_HAS_SENSORLESS    (AXIS_HAS_STALLGUARD(Z) && ENABLED(Z_STALL_SENSITIVITY))
  #define X2_HAS_SENSORLESS   (AXIS_HAS_STALLGUARD(X2) && ENABLED(X_STALL_SENSITIVITY))
  #define Y2_HAS_SENSORLESS   (AXIS_HAS_STALLGUARD(Y2) && ENABLED(Y_STALL_SENSITIVITY))
  #define Z2_HAS_SENSORLESS   (AXIS_HAS_STALLGUARD(Z2) && ENABLED(Z_STALL_SENSITIVITY))
  #define Z3_HAS_SENSORLESS   (AXIS_HAS_STALLGUARD(Z3) && ENABLED(Z_STALL_SENSITIVITY))
  #if ENABLED(SPI_ENDSTOPS)
    #define X_SPI_SENSORLESS  X_HAS_SENSORLESS
    #define Y_SPI_SENSORLESS  Y_HAS_SENSORLESS
    #define Z_SPI_SENSORLESS  Z_HAS_SENSORLESS
  #endif
#endif
