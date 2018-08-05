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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define CODE_M500
#define CODE_M501
#define CODE_M502
#define CODE_M503

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500(void) { (void)eeprom.Store_Settings(); }

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501(void) { (void)eeprom.Load_Settings(); }

/**
 * M502: Revert to factory settings
 */
inline void gcode_M502(void) { (void)eeprom.Factory_Settings(); }

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503(void) { (void)eeprom.Print_Settings(); }
