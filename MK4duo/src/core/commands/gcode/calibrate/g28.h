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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define CODE_G28

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 *  O   Home only if position is unknown
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28(void) { 

  #if ENABLED(DEBUG_FEATURE)
    if (printer.debugFeature()) {
      SERIAL_EM(">>> G28");
      mechanics.log_machine_info();
    }
  #endif

  if (printer.isHomedAll() && parser.boolval('O')) { // home only if needed
    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_EM("> homing not needed, skip");
        SERIAL_EM("<<< G28");
      }
    #endif
    return;
  }

  #if IS_KINEMATIC

    (void)mechanics.home();

  #else

    #if ENABLED(FORCE_HOME_XY_BEFORE_Z)
      const bool  homeZ = parser.seen('Z'),
                  homeX = homeZ || parser.seen('X'),
                  homeY = homeZ || parser.seen('Y');
    #else
      const bool  homeX = parser.seen('X'),
                  homeY = parser.seen('Y'),
                  homeZ = parser.seen('Z');
    #endif

    (void)mechanics.home(homeX, homeY, homeZ);

  #endif

}
