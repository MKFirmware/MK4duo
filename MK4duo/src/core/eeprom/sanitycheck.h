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

#ifndef _EEPROM_SANITYCHECK_H_
#define _EEPROM_SANITYCHECK_H_

#if ENABLED(__AVR__)

  #if ENABLED(EEPROM_SD)
    #error "DEPENDENCY ERROR: EEPROM_SD is not implemented for AVR processor."
  #endif

  #if ENABLED(EEPROM_FLASH)
    #error "DEPENDENCY ERROR: EEPROM_FLASH is not implemented for AVR processor."
  #endif

#else

  #if ENABLED(EEPROM_SETTINGS) && DISABLED(EEPROM_I2C) && DISABLED(EEPROM_SPI) && DISABLED(EEPROM_SD) && DISABLED(EEPROM_FLASH)
    #error "DEPENDENCY ERROR: EEPROM_SETTINGS requires EEPROM_I2C or EEPROM_SPI or EEPROM_SD or EEPROM_FLASH."
  #endif

#endif

#endif /* _EEPROM_SANITYCHECK_H_ */
