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

#include "../../../base.h"

#if ENABLED(ARDUINO_ARCH_SAM)

  #if ENABLED(USE_WATCHDOG)

    #include "watchdog_Due.h"

    // Initialize watchdog with a 4 second interrupt time
    void watchdogSetup(void) { watchdogEnable(4000); }

    // TODO: implement for Due
    void watchdog_init() {
      // this is a stub
    }

  #endif // USE_WATCHDOG

#endif
