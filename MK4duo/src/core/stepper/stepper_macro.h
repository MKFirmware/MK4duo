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
 * stepper/indirection.h
 *
 * Stepper motor driver indirection to allow some stepper functions to
 * be done via SPI/I2c instead of direct pin manipulation.
 *
 * Copyright (c) 2015 Dominik Wenger
 */

#include "l6470/l6470.h"
#include "tmc/tmc.h"
#include "tmc26x/tmc26x.h"
#include "driver/driver.h"

// X Stepper
#ifndef X_ENABLE_INIT()
  #define X_ENABLE_INIT()           driver[X_DRV]->enable_init()
  #define X_ENABLE_WRITE(STATE)     driver[X_DRV]->enable_write(STATE)
  #define X_ENABLE_READ()           driver[X_DRV]->enable_read()
#endif
#ifndef X_DIR_INIT()
  #define X_DIR_INIT()              driver[X_DRV]->dir_init()
  #define X_DIR_WRITE(STATE)        driver[X_DRV]->dir_write(STATE)
  #define X_DIR_READ()              driver[X_DRV]->dir_read()
#endif
#ifndef X_STEP_INIT()
  #define X_STEP_INIT()             driver[X_DRV]->step_init()
#endif
#ifndef X_STEP_WRITE()
  #define X_STEP_WRITE(STATE)       driver[X_DRV]->step_write(STATE)
#endif
#ifndef X_STEP_READ()
  #define X_STEP_READ()             driver[X_DRV]->step_read()
#endif

// Y Stepper
#ifndef Y_ENABLE_INIT()
  #define Y_ENABLE_INIT()           driver[Y_DRV]->enable_init()
  #define Y_ENABLE_WRITE(STATE)     driver[Y_DRV]->enable_write(STATE)
  #define Y_ENABLE_READ()           driver[Y_DRV]->enable_read()
#endif
#ifndef Y_DIR_INIT()
  #define Y_DIR_INIT()              driver[Y_DRV]->dir_init()
  #define Y_DIR_WRITE(STATE)        driver[Y_DRV]->dir_write(STATE)
  #define Y_DIR_READ()              driver[Y_DRV]->dir_read()
#endif
#ifndef Y_STEP_INIT()
  #define Y_STEP_INIT()             driver[Y_DRV]->step_init()
#endif
#ifndef Y_STEP_WRITE()
  #define Y_STEP_WRITE(STATE)       driver[Y_DRV]->step_write(STATE)
#endif
#ifndef Y_STEP_READ()
  #define Y_STEP_READ()             driver[Y_DRV]->step_read()
#endif

// Z Stepper
#ifndef Z_ENABLE_INIT()
  #define Z_ENABLE_INIT()           driver[Z_DRV]->enable_init()
  #define Z_ENABLE_WRITE(STATE)     driver[Z_DRV]->enable_write(STATE)
  #define Z_ENABLE_READ()           driver[Z_DRV]->enable_read()
#endif
#ifndef Z_DIR_INIT()
  #define Z_DIR_INIT()              driver[Z_DRV]->dir_init()
  #define Z_DIR_WRITE(STATE)        driver[Z_DRV]->dir_write(STATE)
  #define Z_DIR_READ()              driver[Z_DRV]->dir_read()
#endif
#ifndef Z_STEP_INIT()
  #define Z_STEP_INIT()             driver[Z_DRV]->step_init()
#endif
#ifndef Z_STEP_WRITE()
  #define Z_STEP_WRITE(STATE)       driver[Z_DRV]->step_write(STATE)
#endif
#ifndef Z_STEP_READ()
  #define Z_STEP_READ()             driver[Z_DRV]->step_read()
#endif

// X2 Stepper
#ifndef X2_ENABLE_INIT()
  #define X2_ENABLE_INIT()          driver[X2_DRV]->enable_init()
  #define X2_ENABLE_WRITE(STATE)    driver[X2_DRV]->enable_write(STATE)
  #define X2_ENABLE_READ()          driver[X2_DRV]->enable_read()
#endif
#ifndef X2_DIR_INIT()
  #define X2_DIR_INIT()             driver[X2_DRV]->dir_init()
  #define X2_DIR_WRITE(STATE)       driver[X2_DRV]->dir_write(STATE)
  #define X2_DIR_READ()             driver[X2_DRV]->dir_read()
#endif
#ifndef X2_STEP_INIT()
  #define X2_STEP_INIT()            driver[X2_DRV]->step_init()
#endif
#ifndef X2_STEP_WRITE()
  #define X2_STEP_WRITE(STATE)      driver[X2_DRV]->step_write(STATE)
#endif
#ifndef X2_STEP_READ()
  #define X2_STEP_READ()            driver[X2_DRV]->step_read()
#endif

// Y2 Stepper
#ifndef Y2_ENABLE_INIT()
  #define Y2_ENABLE_INIT()          driver[Y2_DRV]->enable_init()
  #define Y2_ENABLE_WRITE(STATE)    driver[Y2_DRV]->enable_write(STATE)
  #define Y2_ENABLE_READ()          driver[Y2_DRV]->enable_read()
#endif
#ifndef Y2_DIR_INIT()
  #define Y2_DIR_INIT()             driver[Y2_DRV]->dir_init()
  #define Y2_DIR_WRITE(STATE)       driver[Y2_DRV]->dir_write(STATE)
  #define Y2_DIR_READ()             driver[Y2_DRV]->dir_read()
#endif
#ifndef Y2_STEP_INIT()
  #define Y2_STEP_INIT()            driver[Y2_DRV]->step_init()
#endif
#ifndef Y2_STEP_WRITE()
  #define Y2_STEP_WRITE(STATE)      driver[Y2_DRV]->step_write(STATE)
#endif
#ifndef Y2_STEP_READ()
  #define Y2_STEP_READ()            driver[Y2_DRV]->step_read()
#endif

// Z2 Stepper
#ifndef Z2_ENABLE_INIT()
  #define Z2_ENABLE_INIT()          driver[Z2_DRV]->enable_init()
  #define Z2_ENABLE_WRITE(STATE)    driver[Z2_DRV]->enable_write(STATE)
  #define Z2_ENABLE_READ()          driver[Z2_DRV]->enable_read()
#endif
#ifndef Z2_DIR_INIT()
  #define Z2_DIR_INIT()             driver[Z2_DRV]->dir_init()
  #define Z2_DIR_WRITE(STATE)       driver[Z2_DRV]->dir_write(STATE)
  #define Z2_DIR_READ()             driver[Z2_DRV]->dir_read()
#endif
#ifndef Z2_STEP_INIT()
  #define Z2_STEP_INIT()            driver[Z2_DRV]->step_init()
#endif
#ifndef Z2_STEP_WRITE()
  #define Z2_STEP_WRITE(STATE)      driver[Z2_DRV]->step_write(STATE)
#endif
#ifndef Z2_STEP_READ()
  #define Z2_STEP_READ()            driver[Z2_DRV]->step_read()
#endif

// Z3 Stepper
#ifndef Z3_ENABLE_INIT()
  #define Z3_ENABLE_INIT()          driver[Z3_DRV]->enable_init()
  #define Z3_ENABLE_WRITE(STATE)    driver[Z3_DRV]->enable_write(STATE)
  #define Z3_ENABLE_READ()          driver[Z3_DRV]->enable_read()
#endif
#ifndef Z3_DIR_INIT()
  #define Z3_DIR_INIT()             driver[Z3_DRV]->dir_init()
  #define Z3_DIR_WRITE(STATE)       driver[Z3_DRV]->dir_write(STATE)
  #define Z3_DIR_READ()             driver[Z3_DRV]->dir_read()
#endif
#ifndef Z3_STEP_INIT()
  #define Z3_STEP_INIT()            driver[Z3_DRV]->step_init()
#endif
#ifndef Z3_STEP_WRITE()
  #define Z3_STEP_WRITE(STATE)      driver[Z3_DRV]->step_write(STATE)
#endif
#ifndef Z3_STEP_READ()
  #define Z3_STEP_READ()            driver[Z3_DRV]->step_read()
#endif

// Z4 Stepper
#ifndef Z4_ENABLE_INIT()
  #define Z4_ENABLE_INIT()          stepperZ4->enable_init()
  #define Z4_ENABLE_WRITE(STATE)    stepperZ4->enable_write(STATE)
  #define Z4_ENABLE_READ()          stepperZ4->enable_read()
#endif
#ifndef Z4_DIR_INIT()
  #define Z4_DIR_INIT()             stepperZ4->dir_init()
  #define Z4_DIR_WRITE(STATE)       stepperZ4->dir_write(STATE)
  #define Z4_DIR_READ()             stepperZ4->dir_read()
#endif
#ifndef Z4_STEP_INIT()
  #define Z4_STEP_INIT()            stepperZ4->step_init()
#endif
#ifndef Z4_STEP_WRITE()
  #define Z4_STEP_WRITE(STATE)      stepperZ4->step_write(STATE)
#endif
#ifndef Z4_STEP_READ()
  #define Z4_STEP_READ()            stepperZ4->step_read()
#endif

// E0 Stepper
#ifndef E0_ENABLE_INIT()
  #define E0_ENABLE_INIT()          driver[E0_DRV]->enable_init()
  #define E0_ENABLE_WRITE(STATE)    driver[E0_DRV]->enable_write(STATE)
  #define E0_ENABLE_READ()          driver[E0_DRV]->enable_read()
#endif
#ifndef E0_DIR_INIT()
  #define E0_DIR_INIT()             driver[E0_DRV]->dir_init()
  #define E0_DIR_WRITE(STATE)       driver[E0_DRV]->dir_write(STATE)
  #define E0_DIR_READ()             driver[E0_DRV]->dir_read()
#endif
#ifndef E0_STEP_INIT()
  #if HAS_DAV_SYSTEM
    #define E0_STEP_INIT()          do{ driver[E0_DRV]->step_init(); SET_OUTPUT(FIL_RUNOUT_DAV_PIN); }while(0)
  #else
    #define E0_STEP_INIT()          driver[E0_DRV]->step_init()
  #endif
#endif
#ifndef E0_STEP_WRITE()
  #if HAS_DAV_SYSTEM
    #define E0_STEP_WRITE(STATE)    do{ driver[E0_DRV]->step_write(STATE); WRITE(FIL_RUNOUT_DAV_PIN,STATE); }while(0)
  #else  
    #define E0_STEP_WRITE(STATE)    driver[E0_DRV]->step_write(STATE)
  #endif
#endif
#ifndef E0_STEP_READ()
  #define E0_STEP_READ()            driver[E0_DRV]->step_read()
#endif

// E1 Stepper
#ifndef E1_ENABLE_INIT()
  #define E1_ENABLE_INIT()          driver[E1_DRV]->enable_init()
  #define E1_ENABLE_WRITE(STATE)    driver[E1_DRV]->enable_write(STATE)
  #define E1_ENABLE_READ()          driver[E1_DRV]->enable_read()
#endif
#ifndef E1_DIR_INIT()
  #define E1_DIR_INIT()             driver[E1_DRV]->dir_init()
  #define E1_DIR_WRITE(STATE)       driver[E1_DRV]->dir_write(STATE)
  #define E1_DIR_READ()             driver[E1_DRV]->dir_read()
#endif
#ifndef E1_STEP_INIT()
  #define E1_STEP_INIT()            driver[E1_DRV]->step_init()
#endif
#ifndef E1_STEP_WRITE()
  #define E1_STEP_WRITE(STATE)      driver[E1_DRV]->step_write(STATE)
#endif
#ifndef E1_STEP_READ()
  #define E1_STEP_READ()            driver[E1_DRV]->step_read()
#endif

// E2 Stepper
#ifndef E2_ENABLE_INIT()
  #define E2_ENABLE_INIT()          driver[E2_DRV]->enable_init()
  #define E2_ENABLE_WRITE(STATE)    driver[E2_DRV]->enable_write(STATE)
  #define E2_ENABLE_READ()          driver[E2_DRV]->enable_read()
#endif
#ifndef E2_DIR_INIT()
  #define E2_DIR_INIT()             driver[E2_DRV]->dir_init()
  #define E2_DIR_WRITE(STATE)       driver[E2_DRV]->dir_write(STATE)
  #define E2_DIR_READ()             driver[E2_DRV]->dir_read()
#endif
#ifndef E2_STEP_INIT()
  #define E2_STEP_INIT()            driver[E2_DRV]->step_init()
#endif
#ifndef E2_STEP_WRITE()
  #define E2_STEP_WRITE(STATE)      driver[E2_DRV]->step_write(STATE)
#endif
#ifndef E2_STEP_READ()
  #define E2_STEP_READ()            driver[E2_DRV]->step_read()
#endif

// E3 Stepper
#ifndef E3_ENABLE_INIT()
  #define E3_ENABLE_INIT()          driver[E3_DRV]->enable_init()
  #define E3_ENABLE_WRITE(STATE)    driver[E3_DRV]->enable_write(STATE)
  #define E3_ENABLE_READ()          driver[E3_DRV]->enable_read()
#endif
#ifndef E3_DIR_INIT()
  #define E3_DIR_INIT()             driver[E3_DRV]->dir_init()
  #define E3_DIR_WRITE(STATE)       driver[E3_DRV]->dir_write(STATE)
  #define E3_DIR_READ()             driver[E3_DRV]->dir_read()
#endif
#ifndef E3_STEP_INIT()
  #define E3_STEP_INIT()            driver[E3_DRV]->step_init()
#endif
#ifndef E3_STEP_WRITE()
  #define E3_STEP_WRITE(STATE)      driver[E3_DRV]->step_write(STATE)
#endif
#ifndef E3_STEP_READ()
  #define E3_STEP_READ()            driver[E3_DRV]->step_read()
#endif

// E4 Stepper
#ifndef E4_ENABLE_INIT()
  #define E4_ENABLE_INIT()          driver[E4_DRV]->enable_init()
  #define E4_ENABLE_WRITE(STATE)    driver[E4_DRV]->enable_write(STATE)
  #define E4_ENABLE_READ()          driver[E4_DRV]->enable_read()
#endif
#ifndef E4_DIR_INIT()
  #define E4_DIR_INIT()             driver[E4_DRV]->dir_init()
  #define E4_DIR_WRITE(STATE)       driver[E4_DRV]->dir_write(STATE)
  #define E4_DIR_READ()             driver[E4_DRV]->dir_read()
#endif
#ifndef E4_STEP_INIT()
  #define E4_STEP_INIT()            driver[E4_DRV]->step_init()
#endif
#ifndef E4_STEP_WRITE()
  #define E4_STEP_WRITE(STATE)      driver[E4_DRV]->step_write(STATE)
#endif
#ifndef E4_STEP_READ()
  #define E4_STEP_READ()            driver[E4_DRV]->step_read()
#endif

// E5 Stepper
#ifndef E5_ENABLE_INIT()
  #define E5_ENABLE_INIT()          driver[E5_DRV]->enable_init()
  #define E5_ENABLE_WRITE(STATE)    driver[E5_DRV]->enable_write(STATE)
  #define E5_ENABLE_READ()          driver[E5_DRV]->enable_read()
#endif
#ifndef E5_DIR_INIT()
  #define E5_DIR_INIT()             driver[E5_DRV]->dir_init()
  #define E5_DIR_WRITE(STATE)       driver[E5_DRV]->dir_write(STATE)
  #define E5_DIR_READ()             driver[E5_DRV]->dir_read()
#endif
#ifndef E5_STEP_INIT()
  #define E5_STEP_INIT()            driver[E5_DRV]->step_init()
#endif
#ifndef E5_STEP_WRITE()
  #define E5_STEP_WRITE(STATE)      driver[E5_DRV]->step_write(STATE)
#endif
#ifndef E5_STEP_READ()
  #define E5_STEP_READ()            driver[E5_DRV]->step_read()
#endif

/**
 * Extruder Step for the single E axis
 */
#if DRIVER_EXTRUDERS > 5
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; case 4: E4_STEP_WRITE(V); break; case 5: E5_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 4
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); break; case 4: E4_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 3
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); break; case 3: E3_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 2
  #define E_STEP_WRITE(E,V)     do{ switch (E) { case 0: E0_STEP_WRITE(V); break; case 1: E1_STEP_WRITE(V); break; case 2: E2_STEP_WRITE(V); } }while(0)
#elif DRIVER_EXTRUDERS > 1
  #if ENABLED(DUAL_X_CARRIAGE)
    #define E_STEP_WRITE(E,V)   do{ if (mechanics.extruder_duplication_enabled) { E0_STEP_WRITE(V); E1_STEP_WRITE(V); } else if ((E) == 0) { E0_STEP_WRITE(V); } else { E1_STEP_WRITE(V); } }while(0)
  #else
    #define E_STEP_WRITE(E,V)   do{ if (E == 0) { E0_STEP_WRITE(V); } else { E1_STEP_WRITE(V); } }while(0)
  #endif
#elif DRIVER_EXTRUDERS > 0
  #define E_STEP_WRITE(E,V)     E0_STEP_WRITE(V)
#endif // DRIVER_EXTRUDERS
