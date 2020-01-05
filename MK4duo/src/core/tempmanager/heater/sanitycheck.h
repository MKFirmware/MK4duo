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

// Test required HEATER defines
#if HOTENDS > 3
  #if !HAS_HEATER_HE3
    #error "DEPENDENCY ERROR: HEATER_HE3_PIN not EXIST for this board."
  #endif
#elif HOTENDS > 2
  #if !HAS_HEATER_HE2
    #error "DEPENDENCY ERROR: HEATER_HE2_PIN not EXIST for this board."
  #endif
#elif HOTENDS > 1
  #if !HAS_HEATER_HE1
    #error "DEPENDENCY ERROR: HEATER_HE1_PIN not EXIST for this board."
  #endif
#elif HOTENDS > 0
  #if !HAS_HEATER_HE0
    #error "DEPENDENCY ERROR: HEATER_HE0_PIN not EXIST for this board."
  #endif
#endif

// Thermistors
#if DISABLED(TEMP_SENSOR_HE0)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_HE0."
#endif
#if DISABLED(TEMP_SENSOR_HE1)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_HE1."
#endif
#if DISABLED(TEMP_SENSOR_HE2)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_HE2."
#endif
#if DISABLED(TEMP_SENSOR_HE3)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_HE3."
#endif
#if DISABLED(TEMP_SENSOR_HE4)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_HE4."
#endif
#if DISABLED(TEMP_SENSOR_HE5)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_HE5."
#endif
#if DISABLED(TEMP_SENSOR_BED0)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED0."
#endif
#if DISABLED(TEMP_SENSOR_BED1)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED1."
#endif
#if DISABLED(TEMP_SENSOR_BED2)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED2."
#endif
#if DISABLED(TEMP_SENSOR_BED3)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED3."
#endif
#if DISABLED(TEMP_SENSOR_CHAMBER0)
  #error "DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER0."
#endif
#if DISABLED(TEMP_SENSOR_CHAMBER1)
  #error "DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER1."
#endif
#if DISABLED(TEMP_SENSOR_CHAMBER2)
  #error "DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER2."
#endif
#if DISABLED(TEMP_SENSOR_CHAMBER3)
  #error "DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER3."
#endif

// Every hotend needs a temp sensor
#if HAS_HOTENDS
  #if TEMP_SENSOR_HE0 == 0
    #error "DEPENDENCY ERROR: Hotend 0 needs a temp sensor."
  #endif
  #if HOTENDS > 1
    #if TEMP_SENSOR_HE1 == 0
      #error "DEPENDENCY ERROR: Hotend 1 needs a temp sensor."
    #endif
    #if HOTENDS > 2
      #if TEMP_SENSOR_HE2 == 0
        #error "DEPENDENCY ERROR: Hotend 2 needs a temp sensor."
      #endif
      #if HOTENDS > 3
        #if TEMP_SENSOR_HE3 == 0
          #error "DEPENDENCY ERROR: Hotend 3 needs a temp sensor."
        #endif
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1
#endif // HAS_HOTENDS
