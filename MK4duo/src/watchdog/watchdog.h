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

#ifndef WATCHDOG_H
  #define WATCHDOG_H

  #ifdef ARDUINO_ARCH_SAM
    #if ENABLED(USE_WATCHDOG)
      // Initialize watchdog with a 4 second interrupt time
      void watchdog_init();
      // pad the dog/reset watchdog. MUST be called at least every second after the first watchdog_init or AVR will go into emergency procedures..
      void watchdog_reset();
    #else
      // If we do not have a watchdog, then we can have empty functions which are optimized away.
      FORCE_INLINE void watchdog_init() {};
      FORCE_INLINE void watchdog_reset() {};
    #endif
  #else
    #include <avr/wdt.h>
    #if ENABLED(USE_WATCHDOG)
      // Initialize watchdog with a 4 second interrupt time
      void watchdog_init();

      // Reset watchdog. MUST be called at least every 4 seconds after the
      // first watchdog_init or AVR will go into emergency procedures.
      inline void watchdog_reset() { wdt_reset(); }
    #else
      // If we do not have a watchdog, then we can have empty functions which are optimized away.
      FORCE_INLINE void watchdog_init() {};
      FORCE_INLINE void watchdog_reset() {};
    #endif
  #endif
#endif
