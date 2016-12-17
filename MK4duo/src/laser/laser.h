/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#ifndef _LASER_H
  #define _LASER_H

  #ifdef HIGH_TO_FIRE // Some cutters fire on high, some on low.
    #define LASER_ARM HIGH
    #define LASER_UNARM LOW
  #else
    #define LASER_ARM LOW
    #define LASER_UNARM HIGH
  #endif

  // Laser constants
  #define LASER_OFF 0
  #define LASER_ON 1

  #define CONTINUOUS 0
  #define PULSED 1
  #define RASTER 2

  #define LASER_PWM_MAX_DUTY_CYCLE 255

  #if ENABLED(ARDUINO_ARCH_SAM)
    #include "laser_sam/laser.h"
  #elif ENABLED(ARDUINO_ARCH_AVR)
    #include "laser_avr/laser.h"
  #endif

#endif // LASER_H
