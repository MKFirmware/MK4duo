/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _ENDSTOP_SANITYCHECK_H_
#define _ENDSTOP_SANITYCHECK_H_

  #if ENABLED(X_TWO_ENDSTOPS) && DISABLED(X_TWO_STEPPER_DRIVERS)
    #error "DEPENDENCY ERROR: X_TWO_ENDSTOPS requires X_TWO_STEPPER_DRIVERS"
  #endif

  #if ENABLED(Y_TWO_ENDSTOPS) && DISABLED(Y_TWO_STEPPER_DRIVERS)
    #error "DEPENDENCY ERROR: Y_TWO_ENDSTOPS requires Y_TWO_STEPPER_DRIVERS"
  #endif

  #if ENABLED(Z_TWO_ENDSTOPS) && DISABLED(Z_TWO_STEPPER_DRIVERS)
    #error "DEPENDENCY ERROR: Z_TWO_ENDSTOPS requires Z_TWO_STEPPER_DRIVERS"
  #endif

#endif /* _ENDSTOP_SANITYCHECK_H_ */
