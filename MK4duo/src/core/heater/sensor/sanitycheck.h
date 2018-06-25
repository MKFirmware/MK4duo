/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _TEMP_SENSOR_SANITYCHECK_H_
#define _TEMP_SENSOR_SANITYCHECK_H_

// Thermistors
#if DISABLED(TEMP_SENSOR_0)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_0."
#endif
#if DISABLED(TEMP_SENSOR_1)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_1."
#endif
#if DISABLED(TEMP_SENSOR_2)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_2."
#endif
#if DISABLED(TEMP_SENSOR_3)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_3."
#endif
#if DISABLED(TEMP_SENSOR_BED)
  #error "DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED."
#endif
#if DISABLED(TEMP_SENSOR_CHAMBER)
  #error "DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER."
#endif
#if DISABLED(TEMP_SENSOR_COOLER)
  #error "DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_COOLER."
#endif

#if TEMP_SENSOR_BED <= -3
  #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_BED"
#endif

#if TEMP_SENSOR_CHAMBER <= -3
  #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_CHAMBER"
#endif

#if TEMP_SENSOR_COOLER <= -3
  #error "MAX6675 / MAX31855 Thermocouples not supported for TEMP_SENSOR_COOLER"
#endif

// Every hotend needs a temp sensor
#if HOTENDS > 0
  #if TEMP_SENSOR_0 == 0
    #error "DEPENDENCY ERROR: Hotend 0 needs a temp sensor."
  #endif
  #if HOTENDS > 1
    #if TEMP_SENSOR_1 == 0
      #error "DEPENDENCY ERROR: Hotend 1 needs a temp sensor."
    #endif
    #if HOTENDS > 2
      #if TEMP_SENSOR_2 == 0
        #error "DEPENDENCY ERROR: Hotend 2 needs a temp sensor."
      #endif
      #if HOTENDS > 3
        #if TEMP_SENSOR_3 == 0
          #error "DEPENDENCY ERROR: Hotend 3 needs a temp sensor."
        #endif
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1
#endif // HOTENDS > 0

#endif /* _TEMP_SENSOR_SANITYCHECK_H_ */
