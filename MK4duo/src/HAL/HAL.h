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

/**
 *
 * Description:
 *
 * Supports platforms :
 *     ARDUINO CPU SAM
 *     ARDUINO CPU AVR
 */

#ifndef _HAL_H
#define _HAL_H

#define REFERENCE_F_CPU 16000000 // 16MHz MEGA2560

/**
 * Timers
 */
#if ENABLED(ADVANCE) || ENABLED(LIN_ADVANCE)
  #define REFERENCE_EXTRUDER_TIMER_PRESCALE 64
  #define HAL_REFERENCE_EXTRUDER_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_EXTRUDER_TIMER_PRESCALE) // 250KHz
  #define REFERENCE_EXTRUDER_TIMER_FREQUENCY (HAL_REFERENCE_EXTRUDER_TIMER_RATE / 200)            // 1.25KHz for start
#endif

#define REFERENCE_STEPPER_TIMER_PRESCALE 8
#define HAL_REFERENCE_STEPPER_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_STEPPER_TIMER_PRESCALE)     // 2MHz
#define REFERENCE_STEPPER_TIMER_FREQUENCY (HAL_REFERENCE_STEPPER_TIMER_RATE / 2000)               // 1KHz for start

#define REFERENCE_TEMP_TIMER_PRESCALE 64
#define HAL_REFERENCE_TEMP_TIMER_RATE (REFERENCE_F_CPU / REFERENCE_TEMP_TIMER_PRESCALE)           // 250KHz
#define REFERENCE_TEMP_TIMER_FREQUENCY (HAL_REFERENCE_TEMP_TIMER_RATE / 256)                      // 976.5625Hz

#if ENABLED(ARDUINO_ARCH_SAM)
  #include "HAL_SAM/HAL.h"
  #include "HAL_SAM/communication.h"
#elif ENABLED(ARDUINO_ARCH_AVR)
  #include "HAL_AVR/HAL.h"
  #include "HAL_AVR/communication.h"
#else
  #error "error:Unsupported CPU"
#endif

#endif // _HAL_H
