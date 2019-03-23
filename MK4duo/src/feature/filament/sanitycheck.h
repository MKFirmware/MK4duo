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

/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _FILAMENT_SANITYCHECK_H_
#define _FILAMENT_SANITYCHECK_H_

#if ENABLED(FILAMENT_WIDTH_SENSOR) && !PIN_EXISTS(FILWIDTH)
  #error "DEPENDENCY ERROR: You have to set FILWIDTH_PIN to a valid pin if you enable FILAMENT_WIDTH_SENSOR."
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  #if DISABLED(VOLUMETRIC_EXTRUSION)
    #error "DEPENDENCY ERROR: Missing setting VOLUMETRIC_EXTRUSION."
  #endif
  #if DISABLED(FILAMENT_SENSOR_EXTRUDER_NUM)
    #error "DEPENDENCY ERROR: Missing setting FILAMENT_SENSOR_EXTRUDER_NUM."
  #endif
  #if DISABLED(MEASUREMENT_DELAY_CM)
    #error "DEPENDENCY ERROR: Missing setting MEASUREMENT_DELAY_CM."
  #endif
  #if DISABLED(DEFAULT_NOMINAL_FILAMENT_DIA)
    #error "DEPENDENCY ERROR: Missing setting DEFAULT_NOMINAL_FILAMENT_DIA."
  #endif
  #if DISABLED(FILWIDTH_ERROR_MARGIN)
    #error "DEPENDENCY ERROR: Missing setting FILWIDTH_ERROR_MARGIN."
  #endif
  #if DISABLED(MAX_MEASUREMENT_DELAY)
    #error "DEPENDENCY ERROR: Missing setting MAX_MEASUREMENT_DELAY."
  #endif
  #if DISABLED(DEFAULT_MEASURED_FILAMENT_DIA)
    #error "DEPENDENCY ERROR: Missing setting DEFAULT_MEASURED_FILAMENT_DIA."
  #endif
#endif

#endif /* _FILAMENT_SANITYCHECK_H_ */
