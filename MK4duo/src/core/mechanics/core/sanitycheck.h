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

#ifndef _MECH_CORE_SANITYCHECK_H_
#define _MECH_CORE_SANITYCHECK_H_

// Z late enable
#if MECH(COREXZ) && ENABLED(Z_LATE_ENABLE)
  #error CONFLICT ERROR: "Z_LATE_ENABLE can't be used with COREXZ."
#endif

// Core factor
#if IS_CORE
  #if DISABLED(CORE_FACTOR)
    #error DEPENDENCY ERROR: Missing setting CORE_FACTOR
  #endif
#endif

#endif /* _MECH_CORE_SANITYCHECK_H_ */
