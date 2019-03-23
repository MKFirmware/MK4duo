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

/**
 * Filament Runout needs a pin and M600 command
 */
#ifndef _FIL_RUNOUT_SANITYCHECK_H_
#define _FIL_RUNOUT_SANITYCHECK_H_

#if HAS_FILAMENT_SENSOR
  #if !PIN_EXISTS(FIL_RUNOUT_0)
    #error "DEPENDENCY ERROR: You have to set FIL_RUNOUT_0_PIN to a valid pin if you enable FILAMENT_RUNOUT_SENSOR."
  #elif DISABLED(FIL_RUNOUT_0_LOGIC)
    #error "DEPENDENCY ERROR: Missing setting FIL_RUNOUT_0_LOGIC."
  #elif DISABLED(FIL_RUNOUT_0_PULLUP)
    #error "DEPENDENCY ERROR: Missing setting FIL_RUNOUT_0_PULLUP."
  #elif DISABLED(FILAMENT_RUNOUT_SCRIPT)
    #error "DEPENDENCY ERROR: Missing setting FILAMENT_RUNOUT_SCRIPT."
  #elif DISABLED(ADVANCED_PAUSE_FEATURE)
    static_assert(NULL == strstr(FILAMENT_RUNOUT_SCRIPT, "M600"), "DEPENDENCY ERROR: ADVANCED_PAUSE_FEATURE is required to use M600 with FILAMENT_RUNOUT_SENSOR.");
  #endif
#else
  #if ENABLED(EXTRUDER_ENCODER_CONTROL)
    #error "DEPENDENCY ERROR: FILAMENT_RUNOUT_SENSOR is require to use EXTRUDER_ENCODER_CONTROL"
  #elif ENABLED(FILAMENT_RUNOUT_DAV_SYSTEM)
    #error "DEPENDENCY ERROR: FILAMENT_RUNOUT_SENSOR is require to use FILAMENT_RUNOUT_DAV_SYSTEM"
  #endif
#endif


#endif /* _FIL_RUNOUT_SANITYCHECK_H_ */
