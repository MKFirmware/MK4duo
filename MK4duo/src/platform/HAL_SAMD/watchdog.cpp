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

#include "../../../MK4duo.h"

#if ENABLED(ARDUINO_ARCH_SAMD)

    #include "watchdog.h"
    #define WDT_CONFIG_PER_7_Val 0x9u
    #define WDT_CONFIG_PER_Pos 0
    #define WDT_CONFIG_PER_7 (WDT_CONFIG_PER_7_Val << WDT_CONFIG_PER_Pos)

    void Watchdog::init(void) { 
       #if ENABLED(USE_WATCHDOG)
        // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
        REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |            // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                          GCLK_GENDIV_ID(2);              // Select Generic Clock (GCLK) 2
        while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

        REG_GCLK_GENCTRL =  GCLK_GENCTRL_DIVSEL |         // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                            GCLK_GENCTRL_IDC |            // Set the duty cycle to 50/50 HIGH/LOW
                            GCLK_GENCTRL_GENEN |          // Enable GCLK2
                            GCLK_GENCTRL_SRC_OSCULP32K |  // Set the clock source to the ultra low power oscillator (OSCULP32K)
                            GCLK_GENCTRL_ID(2);           // Select GCLK2         
        while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

        // Feed GCLK2 to WDT (Watchdog Timer)
        REG_GCLK_CLKCTRL =  GCLK_CLKCTRL_CLKEN |          // Enable GCLK2 to the WDT
                            GCLK_CLKCTRL_GEN_GCLK2 |      // Select GCLK2
                            GCLK_CLKCTRL_ID_WDT;          // Feed the GCLK2 to the WDT
        while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

        REG_WDT_CONFIG = WDT_CONFIG_PER_7;                // Set the WDT reset timeout to 4 seconds
        while(WDT->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
        REG_WDT_CTRL = WDT_CTRL_ENABLE;                   // Enable the WDT in normal mode
        while(WDT->STATUS.bit.SYNCBUSY);                  // Wait for synchronization
      #endif
    }
    
    void Watchdog::reset(void) { 
      #if ENABLED(USE_WATCHDOG) 
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY; 
        while(WDT->STATUS.bit.SYNCBUSY);
      #endif
    }

  Watchdog watchdog;

#endif
