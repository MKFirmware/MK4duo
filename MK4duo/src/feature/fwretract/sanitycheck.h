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

#ifndef _FWRETRACT_SANITYCHECK_H_
#define _FWRETRACT_SANITYCHECK_H_

// Firmware Retract
#if ENABLED(FWRETRACT)
  #if DISABLED(MIN_AUTORETRACT)
    #error "DEPENDENCY ERROR: Missing setting MIN_AUTORETRACT."
  #endif
  #if DISABLED(MAX_AUTORETRACT)
    #error "DEPENDENCY ERROR: Missing setting MAX_AUTORETRACT."
  #endif
  #if DISABLED(RETRACT_LENGTH)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_LENGTH."
  #endif
  #if DISABLED(RETRACT_LENGTH_SWAP)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_LENGTH_SWAP."
  #endif
  #if DISABLED(RETRACT_FEEDRATE)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_FEEDRATE."
  #endif
  #if DISABLED(RETRACT_ZLIFT)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_ZLIFT."
  #endif
  #if DISABLED(RETRACT_RECOVER_LENGTH)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH."
  #endif
  #if DISABLED(RETRACT_RECOVER_LENGTH_SWAP)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH_SWAP."
  #endif
  #if DISABLED(RETRACT_RECOVER_FEEDRATE)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_FEEDRATE."
  #endif
  #if DISABLED(RETRACT_RECOVER_FEEDRATE_SWAP)
    #error "DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_FEEDRATE_SWAP."
  #endif
#endif

#endif /* _FWRETRACT_SANITYCHECK_H_ */
