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

// Power consumption sensor
#if ENABLED(POWER_CONSUMPTION) && !PIN_EXISTS(POWER_CONSUMPTION)
  #error "DEPENDENCY ERROR: You have to set POWER_CONSUMPTION_PIN to a valid pin if you enable POWER_CONSUMPTION."
#endif

#if ENABLED(POWER_CONSUMPTION)
  #if DISABLED(POWER_VOLTAGE)
    #error "DEPENDENCY ERROR: Missing setting POWER_VOLTAGE."
  #endif
  #if DISABLED(POWER_SENSITIVITY)
    #error "DEPENDENCY ERROR: Missing setting POWER_SENSITIVITY."
  #endif
  #if DISABLED(POWER_OFFSET)
    #error "DEPENDENCY ERROR: Missing setting POWER_OFFSET."
  #endif
  #if DISABLED(POWER_ZERO)
    #error "DEPENDENCY ERROR: Missing setting POWER_ZERO."
  #endif
  #if DISABLED(POWER_ERROR)
    #error "DEPENDENCY ERROR: Missing setting POWER_ERROR."
  #endif
  #if DISABLED(POWER_EFFICIENCY)
    #error "DEPENDENCY ERROR: Missing setting POWER_EFFICIENCY."
  #endif
#endif
