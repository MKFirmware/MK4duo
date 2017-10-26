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
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_0
#endif
#if DISABLED(TEMP_SENSOR_1)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_1
#endif
#if DISABLED(TEMP_SENSOR_2)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_2
#endif
#if DISABLED(TEMP_SENSOR_3)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_3
#endif
#if DISABLED(TEMP_SENSOR_BED)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED
#endif
#if DISABLED(TEMP_SENSOR_CHAMBER)
  #error DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER
#endif
#if DISABLED(TEMP_SENSOR_COOLER)
  #error DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_COOLER
#endif
#if (THERMISTORHEATER_0 == 998) || (THERMISTORHEATER_1 == 998) || (THERMISTORHEATER_2 == 998) || (THERMISTORHEATER_3 == 998) || (THERMISTORBED == 998) || (THERMISTORCHAMBER == 998) || (THERMISTORCOOLER == 998) // User EXIST table
  #if DISABLED(DUMMY_THERMISTOR_998_VALUE)
    #define DUMMY_THERMISTOR_998_VALUE 25
  #endif
#endif
#if (THERMISTORHEATER_0 == 999) || (THERMISTORHEATER_1 == 999) || (THERMISTORHEATER_2 == 999) || (THERMISTORHEATER_3 == 999) || (THERMISTORBED == 999) || (THERMISTORCHAMBER == 999) || (THERMISTORCOOLER == 999)// User EXIST table
  #if DISABLED(DUMMY_THERMISTOR_999_VALUE)
    #define DUMMY_THERMISTOR_999_VALUE 25
  #endif
#endif

// Every hotend needs a temp sensor
#if HOTENDS > 0
  #if TEMP_SENSOR_0 == 0
    #error DEPENDENCY ERROR: Hotend 0 needs a temp sensor!
  #endif
  #if HOTENDS > 1
    #if TEMP_SENSOR_1 == 0
      #error DEPENDENCY ERROR: Hotend 1 needs a temp sensor!
    #endif
    #if HOTENDS > 2
      #if TEMP_SENSOR_2 == 0
        #error DEPENDENCY ERROR: Hotend 2 needs a temp sensor!
      #endif
      #if HOTENDS > 3
        #if TEMP_SENSOR_3 == 0
          #error DEPENDENCY ERROR: Hotend 3 needs a temp sensor!
        #endif
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1
#endif // HOTENDS > 0

#endif /* _TEMP_SENSOR_SANITYCHECK_H_ */
