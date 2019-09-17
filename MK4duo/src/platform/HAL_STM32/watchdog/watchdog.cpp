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

#ifdef ARDUINO_ARCH_STM32

#include "../../../../MK4duo.h"

void Watchdog::init() {
  #if ENABLED(USE_WATCHDOG)
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32; // 32kHz LSI clock and 32x prescalar = 1024Hz IWDG clock
    hiwdg.Init.Reload = 4095;                 // 4095 counts = 4 seconds at 1024Hz
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
      //Error_Handler();
    }
  #endif
}

void Watchdog::reset() {
  #if ENABLED(USE_WATCHDOG)
    /* Refresh IWDG: reload counter */
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
      /* Refresh Error */
      //Error_Handler();
    }
  #endif
}

Watchdog watchdog;

#endif // ARDUINO_ARCH_STM32
