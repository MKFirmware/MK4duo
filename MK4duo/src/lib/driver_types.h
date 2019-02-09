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

// Models
#define A4988               0x001
#define A5984               0x002
#define DRV8825             0x003
#define LV8729              0x004
#define L6470               0x105
#define TB6560              0x006
#define TB6600              0x007
#define TMC2100             0x008
#define TMC2130             2130
#define TMC2130_STANDALONE  0x009
#define TMC2208             2208
#define TMC2208_STANDALONE  0x00A
#define TMC26X              0x10B
#define TMC26X_STANDALONE   0x00B
#define TMC2660             0x10C
#define TMC2660_STANDALONE  0x00C

// Type
#define AXIS_DRV_TYPE(A,T)  (A##_DRIVER_TYPE == T)
#define  X_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(X,TYPE))
#define  Y_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(Y,TYPE))
#define  Z_HAS_DRV(TYPE)    (AXIS_DRV_TYPE(Z,TYPE))
#define X2_HAS_DRV(TYPE)    ((ENABLED(X_TWO_STEPPER_DRIVERS) || ENABLED(DUAL_X_CARRIAGE))         && AXIS_DRV_TYPE(X2,TYPE))
#define Y2_HAS_DRV(TYPE)    (ENABLED(Y_TWO_STEPPER_DRIVERS)                                       && AXIS_DRV_TYPE(Y2,TYPE))
#define Z2_HAS_DRV(TYPE)    ((ENABLED(Z_TWO_STEPPER_DRIVERS) || ENABLED(Z_THREE_STEPPER_DRIVERS)) && AXIS_DRV_TYPE(Z2,TYPE))
#define Z3_HAS_DRV(TYPE)    (ENABLED(Z_THREE_STEPPER_DRIVERS)                                     && AXIS_DRV_TYPE(Z3,TYPE))
#define E0_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 0 && AXIS_DRV_TYPE(E0,TYPE))
#define E1_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 1 && AXIS_DRV_TYPE(E1,TYPE))
#define E2_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 2 && AXIS_DRV_TYPE(E2,TYPE))
#define E3_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 3 && AXIS_DRV_TYPE(E3,TYPE))
#define E4_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 4 && AXIS_DRV_TYPE(E4,TYPE))
#define E5_HAS_DRV(TYPE)    (DRIVER_EXTRUDERS > 5 && AXIS_DRV_TYPE(E5,TYPE))

#define HAVE_DRV(TYPE) ( \
            X_HAS_DRV(TYPE) ||  Y_HAS_DRV(TYPE) ||  Z_HAS_DRV(TYPE)   \
        || X2_HAS_DRV(TYPE) || Y2_HAS_DRV(TYPE) || Z2_HAS_DRV(TYPE)   \
                                                || Z3_HAS_DRV(TYPE)   \
        || E0_HAS_DRV(TYPE) || E1_HAS_DRV(TYPE) || E2_HAS_DRV(TYPE)   \
        || E3_HAS_DRV(TYPE) || E4_HAS_DRV(TYPE) || E5_HAS_DRV(TYPE))

#define HAS_TRINAMIC              (HAVE_DRV(TMC2130)      || HAVE_DRV(TMC2208)      || HAVE_DRV(TMC2660)    || HAVE_DRV(TMC5130))
#define AXIS_HAS_TMC(A)           (A##_HAS_DRV(TMC2130)   || A##_HAS_DRV(TMC2208)   || A##_HAS_DRV(TMC2660) || A##_HAS_DRV(TMC5130))
#define TMC_HAS_SPI               (HAVE_DRV(TMC2130)      || HAVE_DRV(TMC5130)      || HAVE_DRV(TMC2660))
#define TMC_HAS_STALLGUARD        (HAVE_DRV(TMC2130)      || HAVE_DRV(TMC5130)      || HAVE_DRV(TMC2660))
#define TMC_HAS_STEALTHCHOP       (HAVE_DRV(TMC2130)      || HAVE_DRV(TMC5130)      || HAVE_DRV(TMC2208))
#define AXIS_HAS_SPI(ST)          (ST##_HAS_DRV(TMC2130)  || ST##_HAS_DRV(TMC5130)  || ST##_HAS_DRV(TMC2660))
#define AXIS_HAS_STALLGUARD(ST)   (ST##_HAS_DRV(TMC2130)  || ST##_HAS_DRV(TMC5130)  || ST##_HAS_DRV(TMC2660))
#define AXIS_HAS_STEALTHCHOP(ST)  (ST##_HAS_DRV(TMC2130)  || ST##_HAS_DRV(TMC5130)  || ST##_HAS_DRV(TMC2208))
