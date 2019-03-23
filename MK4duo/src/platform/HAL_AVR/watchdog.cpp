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

#ifdef __AVR__

  #include "../../../MK4duo.h"

  Watchdog watchdog;

  // Initialize watchdog with a 4 sec interrupt time
  void Watchdog::init(void) {
    #if ENABLED(USE_WATCHDOG)
      #if ENABLED(WATCHDOG_DURATION_8S) && ENABLED(WDTO_8S)
        #define WDTO_NS WDTO_8S
      #else
        #define WDTO_NS WDTO_4S
      #endif
      #if ENABLED(WATCHDOG_RESET_MANUAL)
        // We enable the watchdog timer, but only for the interrupt.
        // Take care, as this requires the correct order of operation, with interrupts disabled.
        // See the datasheet of any AVR chip for details.
        wdt_reset();
        cli();
        _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
        _WD_CONTROL_REG = _BV(WDIE) | (WDTO_NS & 0x07) | ((WDTO_NS & 0x08) << 2); // WDTO_NS directly does not work. bit 0-2 are consecutive in the register but the highest value bit is at bit 5
                                                                                  // So worked for up to WDTO_2S
        sei();
        wdt_reset();
      #else
        wdt_enable(WDTO_NS); // The function handles the upper bit correct.
      #endif
    #endif // USE_WATCHDOG
  }

  void Watchdog::reset(void) {
    #if ENABLED(USE_WATCHDOG)
      wdt_reset();
    #endif
  }

  //===========================================================================
  //=================================== ISR ===================================
  //===========================================================================

  // Watchdog timer interrupt, called if main program blocks >4sec and manual reset is enabled.
  #if ENABLED(USE_WATCHDOG) && ENABLED(WATCHDOG_RESET_MANUAL)
    ISR(WDT_vect) {
      sei();  // With the interrupt driven serial we need to allow interrupts.
      SERIAL_LM(ER, "Watchdog timeout. Reset required.");
      printer.minikill();
    }
  #endif // WATCHDOG_RESET_MANUAL

#endif // __AVR__
