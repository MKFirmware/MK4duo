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
 * l6470.h
 * Stepper driver indirection for L6470 drivers
 */

#if HAVE_DRV(L6470)

// L6470 has STEP on normal pins, but DIR/ENABLE via SPI
#include <L6470.h>

void L6470_init_to_defaults();

// X Stepper
#if X_HAS_DRV(L6470)
  extern L6470 driver[X_DRV];
  #define X_ENABLE_INIT()             NOOP
  #define X_ENABLE_WRITE(STATE)       do{ if(STATE) driver[X_DRV].Step_Clock(driver[X_DRV].getStatus() & STATUS_HIZ); else driver[X_DRV].softFree(); }while(0)
  #define X_ENABLE_READ()             (driver[X_DRV].getStatus() & STATUS_HIZ)
  #define X_DIR_INIT()                NOOP
  #define X_DIR_WRITE(STATE)          driver[X_DRV].Step_Clock(STATE)
  #define X_DIR_READ()                (driver[X_DRV].getStatus() & STATUS_DIR)
#endif

// Y Stepper
#if Y_HAS_DRV(L6470)
  extern L6470 driver[Y_DRV];
  #define Y_ENABLE_INIT()             NOOP
  #define Y_ENABLE_WRITE(STATE)       do{ if(STATE) driver[Y_DRV].Step_Clock(driver[Y_DRV].getStatus() & STATUS_HIZ); else driver[Y_DRV].softFree(); }while(0)
  #define Y_ENABLE_READ()             (driver[Y_DRV].getStatus() & STATUS_HIZ)
  #define Y_DIR_INIT()                NOOP
  #define Y_DIR_WRITE(STATE)          driver[Y_DRV].Step_Clock(STATE)
  #define Y_DIR_READ()                (driver[Y_DRV].getStatus() & STATUS_DIR)
#endif

// Z Stepper
#if Z_HAS_DRV(L6470)
  extern L6470 driver[Z_DRV];
  #define Z_ENABLE_INIT()             NOOP
  #define Z_ENABLE_WRITE(STATE)       do{ if(STATE) driver[Z_DRV].Step_Clock(driver[Z_DRV].getStatus() & STATUS_HIZ); else driver[Z_DRV].softFree(); }while(0)
  #define Z_ENABLE_READ()             (driver[Z_DRV].getStatus() & STATUS_HIZ)
  #define Z_DIR_INIT()                NOOP
  #define Z_DIR_WRITE(STATE)          driver[Z_DRV].Step_Clock(STATE)
  #define Z_DIR_READ()                (driver[Z_DRV].getStatus() & STATUS_DIR)
#endif

// X2 Stepper
#if HAS_X2_ENABLE && X2_HAS_DRV(L6470)
  extern L6470 driver.x2;
  #define X2_ENABLE_INIT()            NOOP
  #define X2_ENABLE_WRITE(STATE)      do{ if(STATE) driver.x2.Step_Clock(driver.x2.getStatus() & STATUS_HIZ); else driver.x2.softFree(); }while(0)
  #define X2_ENABLE_READ()            (driver.x2.getStatus() & STATUS_HIZ)
  #define X2_DIR_INIT()               NOOP
  #define X2_DIR_WRITE(STATE)         driver.x2.Step_Clock(STATE)
  #define X2_DIR_READ()               (driver.x2.getStatus() & STATUS_DIR)
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE && Y2_HAS_DRV(L6470)
  extern L6470 driver.y2;
  #define Y2_ENABLE_INIT()            NOOP
  #define Y2_ENABLE_WRITE(STATE)      do{ if(STATE) driver.y2.Step_Clock(driver.y2.getStatus() & STATUS_HIZ); else driver.y2.softFree(); }while(0)
  #define Y2_ENABLE_READ()            (driver.y2.getStatus() & STATUS_HIZ)
  #define Y2_DIR_INIT()               NOOP
  #define Y2_DIR_WRITE(STATE)         driver.y2.Step_Clock(STATE)
  #define Y2_DIR_READ()               (driver.y2.getStatus() & STATUS_DIR)
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE && Z2_HAS_DRV(L6470)
  extern L6470 driver.z2;
  #define Z2_ENABLE_INIT()            NOOP
  #define Z2_ENABLE_WRITE(STATE)      do{ if(STATE) driver.z2.Step_Clock(driver.z2.getStatus() & STATUS_HIZ); else driver.z2.softFree(); }while(0)
  #define Z2_ENABLE_READ()            (driver.z2.getStatus() & STATUS_HIZ)
  #define Z2_DIR_INIT()               NOOP
  #define Z2_DIR_WRITE(STATE)         driver.z2.Step_Clock(STATE)
  #define Z2_DIR_READ()               (driver.z2.getStatus() & STATUS_DIR)
#endif

// Z3 Stepper
#if HAS_Z3_ENABLE && Z3_HAS_DRV(L6470)
  extern L6470 driver.z3;
  #define Z3_ENABLE_INIT()            NOOP
  #define Z3_ENABLE_WRITE(STATE)      do{ if(STATE) driver.z3.Step_Clock(driver.z3.getStatus() & STATUS_HIZ); else driver.z3.softFree(); }while(0)
  #define Z3_ENABLE_READ()            (driver.z3.getStatus() & STATUS_HIZ)
  #define Z3_DIR_INIT()               NOOP
  #define Z3_DIR_WRITE(STATE)         driver.z3.Step_Clock(STATE)
  #define Z3_DIR_READ()               (driver.z3.getStatus() & STATUS_DIR)
#endif

// Z4 Stepper
#if HAS_Z4_ENABLE && Z4_HAS_DRV(L6470)
  extern L6470 stepperZ4;
  #define Z4_ENABLE_INIT()            NOOP
  #define Z4_ENABLE_WRITE(STATE)      do{ if(STATE) stepperZ4.Step_Clock(stepperZ4.getStatus() & STATUS_HIZ); else stepperZ4.softFree(); }while(0)
  #define Z4_ENABLE_READ()            (stepperZ4.getStatus() & STATUS_HIZ)
  #define Z4_DIR_INIT()               NOOP
  #define Z4_DIR_WRITE(STATE)         stepperZ4.Step_Clock(STATE)
  #define Z4_DIR_READ()               (stepperZ4.getStatus() & STATUS_DIR)
#endif

// E0 Stepper
#if E0_HAS_DRV(L6470)
  extern L6470 driver.e[E0_DRV];
  #define E0_ENABLE_INIT()            NOOP
  #define E0_ENABLE_WRITE(STATE)      do{ if(STATE) driver.e[E0_DRV].Step_Clock(driver.e[E0_DRV].getStatus() & STATUS_HIZ); else driver.e[E0_DRV].softFree(); }while(0)
  #define E0_ENABLE_READ()            (driver.e[E0_DRV].getStatus() & STATUS_HIZ)
  #define E0_DIR_INIT()               NOOP
  #define E0_DIR_WRITE(STATE)         driver.e[E0_DRV].Step_Clock(STATE)
  #define E0_DIR_READ()               (driver.e[E0_DRV].getStatus() & STATUS_DIR)
#endif

// E1 Stepper
#if E1_HAS_DRV(L6470)
  extern L6470 driver.e[E1_DRV];
  #define E1_ENABLE_INIT()            NOOP
  #define E1_ENABLE_WRITE(STATE)      do{if(STATE) driver.e[E1_DRV].Step_Clock(driver.e[E1_DRV].getStatus() & STATUS_HIZ); else driver.e[E1_DRV].softFree();}while(0)
  #define E1_ENABLE_READ()            (driver.e[E1_DRV].getStatus() & STATUS_HIZ)
  #define E1_DIR_INIT()               NOOP
  #define E1_DIR_WRITE(STATE)         driver.e[E1_DRV].Step_Clock(STATE)
  #define E1_DIR_READ()               (driver.e[E1_DRV].getStatus() & STATUS_DIR)
#endif

// E2 Stepper
#if E2_HAS_DRV(L6470)
  extern L6470 driver.e[E2_DRV];
  #define E2_ENABLE_INIT()            NOOP
  #define E2_ENABLE_WRITE(STATE)      do{if(STATE) driver.e[E2_DRV].Step_Clock(driver.e[E2_DRV].getStatus() & STATUS_HIZ); else driver.e[E2_DRV].softFree();}while(0)
  #define E2_ENABLE_READ()            (driver.e[E2_DRV].getStatus() & STATUS_HIZ)
  #define E2_DIR_INIT()               NOOP
  #define E2_DIR_WRITE(STATE)         driver.e[E2_DRV].Step_Clock(STATE)
  #define E2_DIR_READ()               (driver.e[E2_DRV].getStatus() & STATUS_DIR)
#endif

// E3 Stepper
#if E3_HAS_DRV(L6470)
  extern L6470 driver.e[E3_DRV];
  #define E3_ENABLE_INIT()            NOOP
  #define E3_ENABLE_WRITE(STATE)      do{if(STATE) driver.e[E3_DRV].Step_Clock(driver.e[E3_DRV].getStatus() & STATUS_HIZ); else driver.e[E3_DRV].softFree();}while(0)
  #define E3_ENABLE_READ()            (driver.e[E3_DRV].getStatus() & STATUS_HIZ)
  #define E3_DIR_INIT()               NOOP
  #define E3_DIR_WRITE(STATE)         driver.e[E3_DRV].Step_Clock(STATE)
  #define E3_DIR_READ()               (driver.e[E3_DRV].getStatus() & STATUS_DIR)
#endif

// E4 Stepper
#if E4_HAS_DRV(L6470)
  extern L6470 driver.e[E4_DRV];
  #define E4_ENABLE_INIT()            NOOP
  #define E4_ENABLE_WRITE(STATE)      do{ if (STATE) driver.e[E4_DRV].Step_Clock(driver.e[E4_DRV].getStatus() & STATUS_HIZ); else driver.e[E4_DRV].softFree(); }while(0)
  #define E4_ENABLE_READ()            (driver.e[E4_DRV].getStatus() & STATUS_HIZ)
  #define E4_DIR_INIT()               NOOP
  #define E4_DIR_WRITE(STATE)         driver.e[E4_DRV].Step_Clock(STATE)
  #define E4_DIR_READ()               (driver.e[E4_DRV].getStatus() & STATUS_DIR)
#endif

// E5 Stepper
#if E5_HAS_DRV(L6470)
  extern L6470 driver.e[E5_DRV];
  #define E5_ENABLE_INIT()            NOOP
  #define E5_ENABLE_WRITE(STATE)      do{if(STATE) driver.e[E5_DRV].Step_Clock(driver.e[E5_DRV].getStatus() & STATUS_HIZ); else driver.e[E5_DRV].softFree();}while(0)
  #define E5_ENABLE_READ()            (driver.e[E5_DRV].getStatus() & STATUS_HIZ)
  #define E5_DIR_INIT()               NOOP
  #define E5_DIR_WRITE(STATE)         driver.e[E5_DRV].Step_Clock(STATE)
  #define E5_DIR_READ()               (driver.e[E5_DRV].getStatus() & STATUS_DIR)
#endif

#endif // HAVE_DRV(TMC26X)
