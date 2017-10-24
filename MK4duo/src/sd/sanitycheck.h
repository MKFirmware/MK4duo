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

#ifndef _SD_CARD_SANITYCHECK_H_
#define _SD_CARD_SANITYCHECK_H_

// SDSUPPORT test
#if ENABLED(SD_SETTINGS) && DISABLED(SDSUPPORT)
  #error DEPENDENCY ERROR: You have to enable SDSUPPORT to use SD_SETTINGS
#endif

/**
 * EEPROM test
 */
#if DISABLED(SDSUPPORT) && ENABLED(EEPROM_SETTINGS) && ENABLED(EEPROM_SD)
  #error DEPENDENCY ERROR: You have to enable SDSUPPORT to use EEPROM_SD
#endif

// addon
#if HAS_SDSUPPORT
  #if DISABLED(SD_FINISHED_STEPPERRELEASE)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_STEPPERRELEASE
  #endif
  #if DISABLED(SD_FINISHED_RELEASECOMMAND)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_RELEASECOMMAND
  #endif
  #if ENABLED(SD_SETTINGS) && DISABLED(SD_CFG_SECONDS)
    #error DEPENDENCY ERROR: Missing setting SD_CFG_SECONDS
  #endif
#endif

#endif /* _SD_CARD_SANITYCHECK_H_ */
