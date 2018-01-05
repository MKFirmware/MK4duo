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

#ifndef _ADVANCED_PAUSE_SANITYCHECK_H_
#define _ADVANCED_PAUSE_SANITYCHECK_H_

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #if EXTRUDERS == 0
    #error "DEPENDENCY ERROR: ADVANCED_PAUSE_FEATURE currently requires extruders."
  #endif
  #if !HAS_LCD
    #error "DEPENDENCY ERROR: ADVANCED_PAUSE_FEATURE currently requires an LCD controller."
  #elif DISABLED(NOZZLE_PARK_FEATURE)
    #error "DEPENDENCY ERROR: ADVANCED_PAUSE_FEATURE currently requires NOZZLE_PARK_FEATURE."
  #elif ENABLED(EXTRUDER_RUNOUT_PREVENT)
    #error "DEPENDENCY ERROR: EXTRUDER_RUNOUT_PREVENT is incompatible with ADVANCED_PAUSE_FEATURE."
  #endif
  #if DISABLED(PAUSE_PARK_RETRACT_LENGTH)
    #error "DEPENDENCY ERROR: Missing setting PAUSE_PARK_RETRACT_LENGTH."
  #endif
  #if DISABLED(PAUSE_PARK_UNLOAD_LENGTH)
    #error "DEPENDENCY ERROR: Missing setting PAUSE_PARK_UNLOAD_LENGTH."
  #endif
  #if DISABLED(PAUSE_PARK_PRINTER_OFF)
    #error "DEPENDENCY ERROR: Missing setting PAUSE_PARK_PRINTER_OFF."
  #endif
#endif

#endif /* _ADVANCED_PAUSE_SANITYCHECK_H_ */
