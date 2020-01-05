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
#pragma once

/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

// Advanced motion
#if DISABLED(DEFAULT_STEPPER_DEACTIVE_TIME)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_STEPPER_DEACTIVE_TIME."
#endif

#if ENABLED(STEPPER_HIGH_LOW)
  #if DISABLED(STEPPER_HIGH_LOW_DELAY)
    #error "DEPENDENCY ERROR: Missing setting STEPPER_HIGH_LOW_DELAY."
  #endif
#endif

#if ENABLED(DIGIPOT_I2C)
  #if DISABLED(DIGIPOT_I2C_NUM_CHANNELS)
    #error "DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_NUM_CHANNELS."
  #endif
  #if DISABLED(DIGIPOT_I2C_MOTOR_CURRENTS)
    #error "DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_MOTOR_CURRENTS."
  #endif
#endif
