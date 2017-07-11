/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

/**
 * Description:
 *
 * Supports platforms:
 *    ARDUINO_ARCH_SAM : For Arduino Due and other boards based on Atmel SAM3X8E
 *    ARDUINO_ARCH_AVR : For all Atmel AVR boards
 */

#ifndef _HAL_H_
#define _HAL_H_

#include <stdint.h>

#if ENABLED(ARDUINO_ARCH_SAM)
  #define CPU_32_BIT
  #include "HAL_DUE/HAL_Due.h"
  #include "HAL_DUE/communication.h"
#elif ENABLED(ARDUINO_ARCH_AVR)
  #include "HAL_AVR/HAL_AVR.h"
  #include "HAL_AVR/communication.h"
#else
  #error "Unsupported Platform!"
#endif

#endif /* _HAL_H_ */
