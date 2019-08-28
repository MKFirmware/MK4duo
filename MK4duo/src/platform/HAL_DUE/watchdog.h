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

#define WDTO_15MS 1500

// Arduino Due core now has watchdog support
class Watchdog {

  public: /** Constructor */

    Watchdog() {}

  public: /** Public Function */

    // Initialize watchdog with a 4 second interrupt time
    FORCE_INLINE static void init(void) { WDT_Restart(WDT); }

    // Reset watchdog. MUST be called at least every 4 seconds.
    FORCE_INLINE static void reset(void) { WDT_Restart(WDT); }

    // Enable the watchdog with the specified timeout.
    FORCE_INLINE static void enable(uint32_t timeout) {
      timeout = (timeout << 8) / 1000;
      if (timeout == 0) timeout = 1;
      else if (timeout > 0xFFF) timeout = 0xFFF;
      // We want to enable the watchdog with the specified timeout
      uint32_t value = WDT_MR_WDRSTEN | WDT_MR_WDV(timeout) | WDT_MR_WDD(timeout);
      WDT_Enable(WDT, value);
      WDT_Restart(WDT);
    }

};

extern Watchdog watchdog;
