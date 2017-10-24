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

#ifndef _MECH_SCARA_SANITYCHECK_H_
#define _MECH_SCARA_SANITYCHECK_H_

// Configuration settings
#if IS_SCARA

  #if DISABLED(SCARA_LINKAGE_1)
    #error DEPENDENCY ERROR: Missing setting SCARA_LINKAGE_1
  #endif
  #if DISABLED(SCARA_LINKAGE_2)
    #error DEPENDENCY ERROR: Missing setting SCARA_LINKAGE_2
  #endif
  #if DISABLED(SCARA_OFFSET_X)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_X
  #endif
  #if DISABLED(SCARA_OFFSET_Y)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_Y
  #endif
  #if DISABLED(THETA_HOMING_OFFSET)
    #error DEPENDENCY ERROR: Missing setting THETA_HOMING_OFFSET
  #endif
  #if DISABLED(PSI_HOMING_OFFSET)
    #error DEPENDENCY ERROR: Missing setting PSI_HOMING_OFFSET
  #endif

  /**
   * Babystepping
   */
  #if ENABLED(BABYSTEPPING)
    #error "BABYSTEPPING is not implemented for SCARA yet."
  #endif

#endif // IS_SCARA

#endif /* _MECH_SCARA_SANITYCHECK_H_ */
