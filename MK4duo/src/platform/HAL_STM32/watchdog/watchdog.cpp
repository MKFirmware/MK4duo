/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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

#ifdef ARDUINO_ARCH_STM32

#include "../../../../MK4duo.h"

void Watchdog::init() {
  #if ENABLED(USE_WATCHDOG)
    IWatchdog.begin(4000000); // 4 sec timeout
  #endif
}

void Watchdog::reset() {
  #if ENABLED(USE_WATCHDOG)
    IWatchdog.reload();
  #endif
}

void Watchdog::enable(uint32_t timeout) {
  NVIC_SystemReset();
}

Watchdog watchdog;

#endif // ARDUINO_ARCH_STM32
