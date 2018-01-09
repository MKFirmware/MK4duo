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

#ifndef _SERVO_SANITYCHECK_H_
#define _SERVO_SANITYCHECK_H_

// Limited number of servos
#if NUM_SERVOS > 4
  #error "DEPENDENCY ERROR: The maximum number of SERVOS in MK4duo is 4."
#endif

#if ENABLED(ENABLE_SERVOS)
  #if NUM_SERVOS < 1
    #error "DEPENDENCY ERROR: NUM_SERVOS has to be at least one if you enable ENABLE_SERVOS."
  #endif
  #if Z_ENDSTOP_SERVO_NR >= 0
    #if Z_ENDSTOP_SERVO_NR >= NUM_SERVOS
      #error "DEPENDENCY ERROR: Z_ENDSTOP_SERVO_NR must be smaller than NUM_SERVOS."
    #endif
  #endif
#endif

#if ENABLED(ENABLE_SERVOS)
  #if NUM_SERVOS > 0 && !(HAS_SERVO_0)
    #error "DEPENDENCY ERROR: You have to set SERVO0_PIN to a valid pin if you enable ENABLE_SERVOS."
  #endif
  #if NUM_SERVOS > 1 && !(HAS_SERVO_1)
    #error "DEPENDENCY ERROR: You have to set SERVO1_PIN to a valid pin if you enable ENABLE_SERVOS."
  #endif
  #if NUM_SERVOS > 2 && !(HAS_SERVO_2)
    #error "DEPENDENCY ERROR: You have to set SERVO2_PIN to a valid pin if you enable ENABLE_SERVOS."
  #endif
  #if NUM_SERVOS > 3 && !(HAS_SERVO_3)
    #error "DEPENDENCY ERROR: You have to set SERVO3_PIN to a valid pin if you enable ENABLE_SERVOS."
  #endif
#endif

#endif /* _SERVO_SANITYCHECK_H_ */
