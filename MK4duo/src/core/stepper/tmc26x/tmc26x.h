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

/**
 * tmc26x.h
 * Stepper driver indirection for TMC26X drivers
 */

#if HAVE_DRV(TMC26X)

// TMC26X drivers have STEP/DIR on normal pins, but ENABLE via SPI
#include <SPI.h>
#include <TMC26XStepper.h>

void tmc26x_init_to_defaults();

// X Stepper
#if X_HAS_DRV(TMC26X)
  extern TMC26XStepper driver[X_DRV];
  #define X_ENABLE_INIT           NOOP
  #define X_ENABLE_WRITE(STATE)   driver[X_DRV].setEnabled(STATE)
  #define X_ENABLE_READ()         driver[X_DRV].isEnabled()
#endif

// Y Stepper
#if Y_HAS_DRV(TMC26X)
  extern TMC26XStepper driver[Y_DRV];
  #define Y_ENABLE_INIT           NOOP
  #define Y_ENABLE_WRITE(STATE)   driver[Y_DRV].setEnabled(STATE)
  #define Y_ENABLE_READ()         driver[Y_DRV].isEnabled()
#endif

// Z Stepper
#if Z_HAS_DRV(TMC26X)
  extern TMC26XStepper driver[Z_DRV];
  #define Z_ENABLE_INIT           NOOP
  #define Z_ENABLE_WRITE(STATE)   driver[Z_DRV].setEnabled(STATE)
  #define Z_ENABLE_READ()         driver[Z_DRV].isEnabled()
#endif

// X2 Stepper
#if HAS_X2_ENABLE && X2_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.x2;
  #define X2_ENABLE_INIT          NOOP
  #define X2_ENABLE_WRITE(STATE)  driver.x2.setEnabled(STATE)
  #define X2_ENABLE_READ()        driver.x2.isEnabled()
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE && Y2_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.y2;
  #define Y2_ENABLE_INIT          NOOP
  #define Y2_ENABLE_WRITE(STATE)  driver.y2.setEnabled(STATE)
  #define Y2_ENABLE_READ()        driver.y2.isEnabled()
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE && Z2_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.z2;
  #define Z2_ENABLE_INIT          NOOP
  #define Z2_ENABLE_WRITE(STATE)  driver.z2.setEnabled(STATE)
  #define Z2_ENABLE_READ()        driver.z2.isEnabled()
#endif

// Z3 Stepper
#if HAS_Z3_ENABLE && Z3_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.z3;
  #define Z3_ENABLE_INIT          NOOP
  #define Z3_ENABLE_WRITE(STATE)  driver.z3.setEnabled(STATE)
  #define Z3_ENABLE_READ()        driver.z3.isEnabled()
#endif

// Z4 Stepper
#if HAS_Z4_ENABLE && Z4_HAS_DRV(TMC26X)
  extern TMC26XStepper stepperZ4;
  #define Z4_ENABLE_INIT()        NOOP
  #define Z4_ENABLE_WRITE(STATE)  stepperZ4.setEnabled(STATE)
  #define Z4_ENABLE_READ()        stepperZ4.isEnabled()
#endif

// E0 Stepper
#if E0_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.e[E0_DRV];
  #define E0_ENABLE_INIT          NOOP
  #define E0_ENABLE_WRITE(STATE)  driver.e[E0_DRV].setEnabled(STATE)
  #define E0_ENABLE_READ()        driver.e[E0_DRV].isEnabled()
#endif

// E1 Stepper
#if E1_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.e[E1_DRV];
  #define E1_ENABLE_INIT()        NOOP
  #define E1_ENABLE_WRITE(STATE)  driver.e[E1_DRV]->setEnabled(STATE)
  #define E1_ENABLE_READ()        driver.e[E1_DRV]->isEnabled()
#endif

// E2 Stepper
#if E2_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.e[E2_DRV];
  #define E2_ENABLE_INIT()        NOOP
  #define E2_ENABLE_WRITE(STATE)  driver.e[E2_DRV]->setEnabled(STATE)
  #define E2_ENABLE_READ()        driver.e[E2_DRV]->isEnabled()
#endif

// E3 Stepper
#if E3_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.e[E3_DRV];
  #define E3_ENABLE_INIT()        NOOP
  #define E3_ENABLE_WRITE(STATE)  driver.e[E3_DRV]->setEnabled(STATE)
  #define E3_ENABLE_READ()        driver.e[E3_DRV]->isEnabled()
#endif

// E4 Stepper
#if E4_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.e[E4_DRV];
  #define E4_ENABLE_INIT()        NOOP
  #define E4_ENABLE_WRITE(STATE)  driver.e[E4_DRV]->setEnabled(STATE)
  #define E4_ENABLE_READ()        driver.e[E4_DRV]->isEnabled()
#endif

// E5 Stepper
#if E5_HAS_DRV(TMC26X)
  extern TMC26XStepper driver.e[E5_DRV];
  #define E5_ENABLE_INIT()        NOOP
  #define E5_ENABLE_WRITE(STATE)  driver.e[E5_DRV]->setEnabled(STATE)
  #define E5_ENABLE_READ()        driver.e[E5_DRV]->isEnabled()
#endif

#endif // HAVE_DRV(TMC26X)
