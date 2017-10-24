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

#ifndef _FAN_SANITYCHECK_H_
#define _FAN_SANITYCHECK_H_

// Fan
#if DISABLED(CONTROLLERFAN_SECS)
  #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SECS
#endif
#if DISABLED(CONTROLLERFAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SPEED
#endif
#if DISABLED(CONTROLLERFAN_MIN_SPEED)
  #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_MIN_SPEED
#endif

#if DISABLED(HOTEND_AUTO_FAN_TEMPERATURE)
  #error DEPENDENCY ERROR: Missing setting HOTEND_AUTO_FAN_TEMPERATURE
#endif
#if DISABLED(HOTEND_AUTO_FAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting HOTEND_AUTO_FAN_SPEED
#endif
#if DISABLED(HOTEND_AUTO_FAN_MIN_SPEED)
  #error DEPENDENCY ERROR: Missing setting HOTEND_AUTO_FAN_MIN_SPEED
#endif

#endif /* _FAN_SANITYCHECK_H_ */
