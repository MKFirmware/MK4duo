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
#define  X_HAS_DRV(TYPE)  ( X_DRIVER_TYPE == TYPE)
#define  Y_HAS_DRV(TYPE)  ( Y_DRIVER_TYPE == TYPE)
#define  Z_HAS_DRV(TYPE)  ( Z_DRIVER_TYPE == TYPE)
#define X2_HAS_DRV(TYPE)  (X2_DRIVER_TYPE == TYPE)
#define Y2_HAS_DRV(TYPE)  (Y2_DRIVER_TYPE == TYPE)
#define Z2_HAS_DRV(TYPE)  (Z2_DRIVER_TYPE == TYPE)
#define Z3_HAS_DRV(TYPE)  (Z3_DRIVER_TYPE == TYPE)
#define E0_HAS_DRV(TYPE)  (DRIVER_EXTRUDERS > 0 && E0_DRIVER_TYPE == TYPE)
#define E1_HAS_DRV(TYPE)  (DRIVER_EXTRUDERS > 1 && E1_DRIVER_TYPE == TYPE)
#define E2_HAS_DRV(TYPE)  (DRIVER_EXTRUDERS > 2 && E2_DRIVER_TYPE == TYPE)
#define E3_HAS_DRV(TYPE)  (DRIVER_EXTRUDERS > 3 && E3_DRIVER_TYPE == TYPE)
#define E4_HAS_DRV(TYPE)  (DRIVER_EXTRUDERS > 4 && E4_DRIVER_TYPE == TYPE)
#define E5_HAS_DRV(TYPE)  (DRIVER_EXTRUDERS > 5 && E5_DRIVER_TYPE == TYPE)

#define HAVE_DRV(TYPE) ( \
            X_HAS_DRV(TYPE) ||  Y_HAS_DRV(TYPE) ||  Z_HAS_DRV(TYPE)   \
        || X2_HAS_DRV(TYPE) || Y2_HAS_DRV(TYPE) || Z2_HAS_DRV(TYPE)   \
                                                || Z3_HAS_DRV(TYPE)   \
        || E0_HAS_DRV(TYPE) || E1_HAS_DRV(TYPE) || E2_HAS_DRV(TYPE)   \
        || E3_HAS_DRV(TYPE) || E4_HAS_DRV(TYPE) || E5_HAS_DRV(TYPE))

#define    HAS_STEALTHCHOP  ( HAVE_DRV(TMC2130)   ||  HAVE_DRV(TMC2208))
#define  X_HAS_STEALTHCHOP  ( X_HAS_DRV(TMC2130)  ||  X_HAS_DRV(TMC2208))
#define  Y_HAS_STEALTHCHOP  ( Y_HAS_DRV(TMC2130)  ||  Y_HAS_DRV(TMC2208))
#define  Z_HAS_STEALTHCHOP  ( Z_HAS_DRV(TMC2130)  ||  Z_HAS_DRV(TMC2208))
#define X2_HAS_STEALTHCHOP  (X2_HAS_DRV(TMC2130)  || X2_HAS_DRV(TMC2208))
#define Y2_HAS_STEALTHCHOP  (Y2_HAS_DRV(TMC2130)  || Y2_HAS_DRV(TMC2208))
#define Z2_HAS_STEALTHCHOP  (Z2_HAS_DRV(TMC2130)  || Z2_HAS_DRV(TMC2208))
#define Z3_HAS_STEALTHCHOP  (Z3_HAS_DRV(TMC2130)  || Z3_HAS_DRV(TMC2208))
#define E0_HAS_STEALTHCHOP  (E0_HAS_DRV(TMC2130)  || E0_HAS_DRV(TMC2208))
#define E1_HAS_STEALTHCHOP  (E1_HAS_DRV(TMC2130)  || E1_HAS_DRV(TMC2208))
#define E2_HAS_STEALTHCHOP  (E2_HAS_DRV(TMC2130)  || E2_HAS_DRV(TMC2208))
#define E3_HAS_STEALTHCHOP  (E3_HAS_DRV(TMC2130)  || E3_HAS_DRV(TMC2208))
#define E4_HAS_STEALTHCHOP  (E4_HAS_DRV(TMC2130)  || E4_HAS_DRV(TMC2208))
#define E5_HAS_STEALTHCHOP  (E5_HAS_DRV(TMC2130)  || E5_HAS_DRV(TMC2208))

#define    HAS_STALLGUARD   ( HAVE_DRV(TMC2130)   ||  HAVE_DRV(TMC5130))
#define  X_HAS_STALLGUARD   ( X_HAS_DRV(TMC2130)  ||  X_HAS_DRV(TMC5130))
#define  Y_HAS_STALLGUARD   ( Y_HAS_DRV(TMC2130)  ||  Y_HAS_DRV(TMC5130))
#define  Z_HAS_STALLGUARD   ( Z_HAS_DRV(TMC2130)  ||  Z_HAS_DRV(TMC5130))
#define X2_HAS_STALLGUARD   (X2_HAS_DRV(TMC2130)  || X2_HAS_DRV(TMC5130))
#define Y2_HAS_STALLGUARD   (Y2_HAS_DRV(TMC2130)  || Y2_HAS_DRV(TMC5130))
#define Z2_HAS_STALLGUARD   (Z2_HAS_DRV(TMC2130)  || Z2_HAS_DRV(TMC5130))
#define Z3_HAS_STALLGUARD   (Z3_HAS_DRV(TMC2130)  || Z3_HAS_DRV(TMC5130))
#define E0_HAS_STALLGUARD   (E0_HAS_DRV(TMC2130)  || E0_HAS_DRV(TMC5130))
#define E1_HAS_STALLGUARD   (E1_HAS_DRV(TMC2130)  || E1_HAS_DRV(TMC5130))
#define E2_HAS_STALLGUARD   (E2_HAS_DRV(TMC2130)  || E2_HAS_DRV(TMC5130))
#define E3_HAS_STALLGUARD   (E3_HAS_DRV(TMC2130)  || E3_HAS_DRV(TMC5130))
#define E4_HAS_STALLGUARD   (E4_HAS_DRV(TMC2130)  || E4_HAS_DRV(TMC5130))
#define E5_HAS_STALLGUARD   (E5_HAS_DRV(TMC2130)  || E5_HAS_DRV(TMC5130))

#define   HAS_TRINAMIC      ( HAVE_DRV(TMC2130)   ||  HAVE_DRV(TMC2208)   ||  HAVE_DRV(TMC5130))
#define  X_IS_TRINAMIC      ( X_HAS_DRV(TMC2130)  ||  X_HAS_DRV(TMC2208)  ||  X_HAS_DRV(TMC5130))
#define  Y_IS_TRINAMIC      ( Y_HAS_DRV(TMC2130)  ||  Y_HAS_DRV(TMC2208)  ||  Y_HAS_DRV(TMC5130))
#define  Z_IS_TRINAMIC      ( Z_HAS_DRV(TMC2130)  ||  Z_HAS_DRV(TMC2208)  ||  Z_HAS_DRV(TMC5130))
#define X2_IS_TRINAMIC      (X2_HAS_DRV(TMC2130)  || X2_HAS_DRV(TMC2208)  || X2_HAS_DRV(TMC5130))
#define Y2_IS_TRINAMIC      (Y2_HAS_DRV(TMC2130)  || Y2_HAS_DRV(TMC2208)  || Y2_HAS_DRV(TMC5130))
#define Z2_IS_TRINAMIC      (Z2_HAS_DRV(TMC2130)  || Z2_HAS_DRV(TMC2208)  || Z2_HAS_DRV(TMC5130))
#define Z3_IS_TRINAMIC      (Z3_HAS_DRV(TMC2130)  || Z3_HAS_DRV(TMC2208)  || Z3_HAS_DRV(TMC5130))
#define E0_IS_TRINAMIC      (E0_HAS_DRV(TMC2130)  || E0_HAS_DRV(TMC2208)  || E0_HAS_DRV(TMC5130))
#define E1_IS_TRINAMIC      (E1_HAS_DRV(TMC2130)  || E1_HAS_DRV(TMC2208)  || E1_HAS_DRV(TMC5130))
#define E2_IS_TRINAMIC      (E2_HAS_DRV(TMC2130)  || E2_HAS_DRV(TMC2208)  || E2_HAS_DRV(TMC5130))
#define E3_IS_TRINAMIC      (E3_HAS_DRV(TMC2130)  || E3_HAS_DRV(TMC2208)  || E3_HAS_DRV(TMC5130))
#define E4_IS_TRINAMIC      (E4_HAS_DRV(TMC2130)  || E4_HAS_DRV(TMC2208)  || E4_HAS_DRV(TMC5130))
#define E5_IS_TRINAMIC      (E5_HAS_DRV(TMC2130)  || E5_HAS_DRV(TMC2208)  || E5_HAS_DRV(TMC5130))
