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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#if ENABLED(CASE_LIGHT)
  #if !PIN_EXISTS(CASE_LIGHT) && DISABLED(CASE_LIGHT_USE_NEOPIXEL)
    #error "DEPENDENCY ERROR: You have to set CASE_LIGHT_PIN to a valid pin if you enable CASE_LIGHT."
  #endif
  #if DISABLED(INVERT_CASE_LIGHT)
    #error "DEPENDENCY ERROR: Missing setting INVERT_CASE_LIGHT is needed by CASE_LIGHT."
  #endif
  #if DISABLED(CASE_LIGHT_DEFAULT_ON)
    #error "DEPENDENCY ERROR: Missing setting CASE_LIGHT_DEFAULT_ON is needed by CASE_LIGHT."
  #endif
  #if DISABLED(CASE_LIGHT_DEFAULT_BRIGHTNESS)
    #error "DEPENDENCY ERROR: Missing setting CASE_LIGHT_DEFAULT_BRIGHTNESS is needed by CASE_LIGHT."
  #endif
#endif
