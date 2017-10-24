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

/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _ENDSTOP_SANITYCHECK_H_
#define _ENDSTOP_SANITYCHECK_H_

// Endstop logic
#if DISABLED(X_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting X_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Y_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Y_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Z_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z2_MIN_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Z2_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z3_MIN_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z3_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(Z4_MIN_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z4_MIN_ENDSTOP_LOGIC
#endif
#if DISABLED(X_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting X_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Y_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Y_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z2_MAX_ENDSTOP_LOGIC) && !IS_DELTA
  #error DEPENDENCY ERROR: Missing setting Z2_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z3_MAX_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z3_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z4_MAX_ENDSTOP_LOGIC) && !IS_KINEMATIC
  #error DEPENDENCY ERROR: Missing setting Z4_MAX_ENDSTOP_LOGIC
#endif
#if DISABLED(Z_PROBE_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_PROBE_ENDSTOP_LOGIC
#endif
#if DISABLED(E_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting E_MIN_ENDSTOP_LOGIC
#endif

#endif /* _ENDSTOP_SANITYCHECK_H_ */
