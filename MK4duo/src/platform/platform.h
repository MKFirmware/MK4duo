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
#pragma once

/**
 * Description:
 *
 * Supports platforms:
 *    ARDUINO_ARCH_SAM  : For Arduino Due and other boards based on Atmel SAM3X8E
 *    __AVR__           : For all Atmel AVR boards
 */

#include "common/memory_store.h"
#include "common/communication.h"
#include "common/debug.h"
#include "common/host_action.h"
#include "common/servo.h"
#include "common/softpwm.h"

#if ENABLED(ARDUINO_ARCH_SAM)
  #define CPU_32_BIT
  #include "HAL_DUE/spi_pins.h"
  #include "HAL_DUE/HAL.h"
#elif ENABLED(ARDUINO_ARCH_SAMD)
  #define CPU_32_BIT
  #include "HAL_SAMD/spi_pins.h"
  #include "HAL_SAMD/HAL.h"
#elif ENABLED(__AVR__)
  #include "HAL_AVR/spi_pins.h"
  #include "HAL_AVR/HAL.h"
#else
  #error "Unsupported Platform!"
#endif
