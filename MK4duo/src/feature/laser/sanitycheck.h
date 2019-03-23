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

#ifndef _LASER_SANITYCHECK_H_
#define _LASER_SANITYCHECK_H_

#if ENABLED(LASER)
  #if ENABLED(LASER_PERIPHERALS)
    #if !PIN_EXISTS(LASER_PERIPHERALS)
      #error "DEPENDENCY ERROR: You have to set LASER_PERIPHERALS_PIN to a valid pin if you enable LASER_PERIPHERALS."
    #endif
    #if !PIN_EXISTS(LASER_PERIPHERALS_STATUS)
      #error "DEPENDENCY ERROR: You have to set LASER_PERIPHERALS_STATUS_PIN to a valid pin if you enable LASER_PERIPHERALS."
    #endif
  #endif
  #if (DISABLED(LASER_CONTROL) || ((LASER_CONTROL != 1) && (LASER_CONTROL != 2)))
     #error "DEPENDENCY ERROR: You have to set LASER_CONTROL to 1 or 2."
  #else
    #if(LASER_CONTROL == 1)
      #if(!HAS_LASER_POWER)
        #error "DEPENDENCY ERROR: You have to set LASER_PWR_PIN."
      #endif
    #else
      #if(!HAS_LASER_POWER || !HAS_LASER_PWM)
        #error "DEPENDENCY ERROR: You have to set LASER_PWR_PIN and LASER_PWM_PIN to a valid pin if you enable LASER."
      #endif
    #endif
  #endif
#endif

#endif /* _LASER_SANITYCHECK_H_ */
