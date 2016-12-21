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

#include "../../base.h"

#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"

  #ifdef ARDUINO_ARCH_SAM
    // Initialize watchdog with a 4 sec interrupt time
    void watchdog_init() {
      const uint32_t wdtTicks = 256;	// number of watchdog ticks @ 32768Hz/128 before the watchdog times out (max 4095)
      WDT_Enable(WDT, (wdtTicks << WDT_MR_WDV_Pos) | (wdtTicks << WDT_MR_WDD_Pos) | WDT_MR_WDRSTEN);	// enable watchdog, reset the mcu if it times out
    }

    // Reset watchdog. MUST be called every 1s after init or avr will reset.
    void watchdog_reset() {
      WDT_Restart(WDT);
    }
  #else
    // Initialize watchdog with a 4 sec interrupt time
    void watchdog_init() {
      #if ENABLED(WATCHDOG_RESET_MANUAL)
        // We enable the watchdog timer, but only for the interrupt.
        // Take care, as this requires the correct order of operation, with interrupts disabled. See the datasheet of any AVR chip for details.
        wdt_reset();
        _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
        _WD_CONTROL_REG = _BV(WDIE) | WDTO_4S;
      #else
        wdt_enable(WDTO_4S);
      #endif
    }

    //===========================================================================
    //=================================== ISR ===================================
    //===========================================================================

    // Watchdog timer interrupt, called if main program blocks >1sec and manual reset is enabled.
    #if ENABLED(WATCHDOG_RESET_MANUAL)
      ISR(WDT_vect) {
        ECHO_LM(ER, "Something is wrong, please turn off the printer.");
        kill(PSTR("ERR:Please Reset")); //kill blocks //16 characters so it fits on a 16x2 display
        while (1); //wait for user or serial reset
      }
    #endif // WATCHDOG_RESET_MANUAL
  #endif
#endif // USE_WATCHDOG
