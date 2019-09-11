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

/**
 * l6470.cpp
 * Stepper driver indirection for L6470 drivers
 */

#include "../../../../MK4duo.h"
#include "sanitycheck.h"

#if HAVE_DRV(L6470)

#include "l6470.h"

#define _L6470_DEFINE(ST) L6470 stepper##ST(ST##_CS_PIN)

// L6470 Stepper objects
#if X_HAS_DRV(L6470)
  _L6470_DEFINE(X);
#endif
#if X2_HAS_DRV(L6470)
  _L6470_DEFINE(X2);
#endif
#if Y_HAS_DRV(L6470)
  _L6470_DEFINE(Y);
#endif
#if Y2_HAS_DRV(L6470)
  _L6470_DEFINE(Y2);
#endif
#if Z_HAS_DRV(L6470)
  _L6470_DEFINE(Z);
#endif
#if Z2_HAS_DRV(L6470)
  _L6470_DEFINE(Z2);
#endif
#if Z3_HAS_DRV(L6470)
  _L6470_DEFINE(Z3);
#endif
#if E0_HAS_DRV(L6470)
  _L6470_DEFINE(E0);
#endif
#if E1_HAS_DRV(L6470)
  _L6470_DEFINE(E1);
#endif
#if E2_HAS_DRV(L6470)
  _L6470_DEFINE(E2);
#endif
#if E3_HAS_DRV(L6470)
  _L6470_DEFINE(E3);
#endif
#if E4_HAS_DRV(L6470)
  _L6470_DEFINE(E4);
#endif

#define _L6470_INIT(A) do{ \
  stepper##A.init(); \
  stepper##A.softFree(); \
  stepper##A.setMicroSteps(A##_MICROSTEPS); \
  stepper##A.setOverCurrent(A##_OVERCURRENT); \
  stepper##A.setStallCurrent(A##_STALLCURRENT); \
} while(0)

void L6470_init_to_defaults() {
  #if X_HAS_DRV(L6470)
    _L6470_INIT(X);
  #endif
  #if X2_HAS_DRV(L6470)
    _L6470_INIT(X2);
  #endif
  #if Y_HAS_DRV(L6470)
    _L6470_INIT(Y);
  #endif
  #if Y2_HAS_DRV(L6470)
    _L6470_INIT(Y2);
  #endif
  #if Z_HAS_DRV(L6470)
    _L6470_INIT(Z);
  #endif
  #if Z2_HAS_DRV(L6470)
    _L6470_INIT(Z2);
  #endif
  #if Z3_HAS_DRV(L6470)
    _L6470_INIT(Z3);
  #endif
  #if E0_HAS_DRV(L6470)
    _L6470_INIT(E0);
  #endif
  #if E1_HAS_DRV(L6470)
    _L6470_INIT(E1);
  #endif
  #if E2_HAS_DRV(L6470)
    _L6470_INIT(E2);
  #endif
  #if E3_HAS_DRV(L6470)
    _L6470_INIT(E3);
  #endif
  #if E4_HAS_DRV(L6470)
    _L6470_INIT(E4);
  #endif
}

#endif // HAVE_DRV(L6470)
