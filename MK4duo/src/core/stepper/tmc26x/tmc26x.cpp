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
 * tmc26x.cpp
 * Stepper driver indirection for TMC26X drivers
 */

#include "../../../../MK4duo.h"
#include "sanitycheck.h"

#if HAVE_DRV(TMC26X)

#include "tmc26x.h"

#define _TMC26X_DEFINE(ST) TMC26XStepper stepper##ST(200, ST##_CS_PIN, ST##_STEP_PIN, ST##_DIR_PIN, ST##_MAX_CURRENT, ST##_SENSE_RESISTOR)

#if X_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(X);
#endif
#if X2_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(X2);
#endif
#if Y_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(Y);
#endif
#if Y2_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(Y2);
#endif
#if Z_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(Z);
#endif
#if Z2_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(Z2);
#endif
#if Z3_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(Z3);
#endif
#if E0_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(E0);
#endif
#if E1_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(E1);
#endif
#if E2_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(E2);
#endif
#if E3_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(E3);
#endif
#if E4_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(E4);
#endif
#if E5_HAS_DRV(TMC26X)
  _TMC26X_DEFINE(E5);
#endif

#define _TMC26X_INIT(A) do{ \
  stepper##A.setMicrosteps(A##_MICROSTEPS); \
  stepper##A.start(); \
} while(0)

void tmc26x_init_to_defaults() {
  #if X_HAS_DRV(TMC26X)
    _TMC26X_INIT(X);
  #endif
  #if X2_HAS_DRV(TMC26X)
    _TMC26X_INIT(X2);
  #endif
  #if Y_HAS_DRV(TMC26X)
    _TMC26X_INIT(Y);
  #endif
  #if Y2_HAS_DRV(TMC26X)
    _TMC26X_INIT(Y2);
  #endif
  #if Z_HAS_DRV(TMC26X)
    _TMC26X_INIT(Z);
  #endif
  #if Z2_HAS_DRV(TMC26X)
    _TMC26X_INIT(Z2);
  #endif
  #if Z3_HAS_DRV(TMC26X)
    _TMC26X_INIT(Z3);
  #endif
  #if E0_HAS_DRV(TMC26X)
    _TMC26X_INIT(E0);
  #endif
  #if E1_HAS_DRV(TMC26X)
    _TMC26X_INIT(E1);
  #endif
  #if E2_HAS_DRV(TMC26X)
    _TMC26X_INIT(E2);
  #endif
  #if E3_HAS_DRV(TMC26X)
    _TMC26X_INIT(E3);
  #endif
  #if E4_HAS_DRV(TMC26X)
    _TMC26X_INIT(E4);
  #endif
  #if E5_HAS_DRV(TMC26X)
    _TMC26X_INIT(E5);
  #endif
}

#endif // HAVE_DRV(TMC26X)
