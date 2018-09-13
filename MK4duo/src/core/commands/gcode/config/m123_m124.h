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

#define CODE_M123
#define CODE_M124

/**
 * M123: Set Endstop Logic
 *
 *  X<bool> - Endstop X min or max dependent HOME DIR set false or true
 *  Y<bool> - Endstop Y min or max dependent HOME DIR set false or true
 *  Z<bool> - Endstop Z min or max dependent HOME DIR set false or true
 *  I<bool> - Endstop X2 min or max dependent HOME DIR set false or true
 *  J<bool> - Endstop Y2 min or max dependent HOME DIR set false or true
 *  K<bool> - Endstop Z2 min or max dependent HOME DIR set false or true
 *  P<bool> - Endstop Probe set false or true
 *  D<bool> - Endstop Door set false or true
 *  F<bool> - Endstop Fil Runout set false or true
 *  W<bool> - Endstop Power Check set false or true
 *
 */
inline void gcode_M123(void) {

  if (parser.seen('X')) {
    if (mechanics.home_dir[X_AXIS] == -1)
      endstops.setLogic(X_MIN, parser.value_bool());
    else
      endstops.setLogic(X_MAX, parser.value_bool());
  }

  if (parser.seen('Y')) {
    if (mechanics.home_dir[Y_AXIS] == -1)
      endstops.setLogic(Y_MIN, parser.value_bool());
    else
      endstops.setLogic(Y_MAX, parser.value_bool());
  }

  if (parser.seen('Z')) {
    if (mechanics.home_dir[Z_AXIS] == -1)
      endstops.setLogic(Z_MIN, parser.value_bool());
    else
      endstops.setLogic(Z_MAX, parser.value_bool());
  }

  #if HAS_X2_MIN || HAS_X2_MAX
    if (parser.seen('I')) {
      if (mechanics.home_dir[X_AXIS] == -1)
        endstops.setLogic(X2_MIN, parser.value_bool());
      else
        endstops.setLogic(X2_MAX, parser.value_bool());
    }
  #endif

  #if HAS_Y2_MIN || HAS_Y2_MAX
    if (parser.seen('J')) {
      if (mechanics.home_dir[Y_AXIS] == -1)
        endstops.setLogic(Y2_MIN, parser.value_bool());
      else
        endstops.setLogic(Y2_MAX, parser.value_bool());
    }
  #endif

  #if HAS_Z2_MIN || HAS_Z2_MAX
    if (parser.seen('K')) {
      if (mechanics.home_dir[Z_AXIS] == -1)
        endstops.setLogic(Z2_MIN, parser.value_bool());
      else
        endstops.setLogic(Z2_MAX, parser.value_bool());
    }
  #endif

  #if HAS_Z_PROBE_PIN
    if (parser.seen('P')) endstops.setLogic(Z_PROBE, parser.value_bool());
  #endif
  
  #if HAS_FIL_RUNOUT
    if (parser.seen('F')) endstops.setLogic(FIL_RUNOUT, parser.value_bool());
  #endif

  #if HAS_DOOR_OPEN
    if (parser.seen('D')) endstops.setLogic(DOOR_OPEN_SENSOR, parser.value_bool());
  #endif

  #if HAS_POWER_CHECK && HAS_SD_SUPPORT
    if (parser.seen('W')) endstops.setLogic(POWER_CHECK_SENSOR, parser.value_bool());
  #endif

  endstops.report();
}

/**
 * M124: Set Endstop Pullup
 *
 *  X<bool> - Endstop X min or max dependent HOME DIR set false or true
 *  Y<bool> - Endstop Y min or max dependent HOME DIR set false or true
 *  Z<bool> - Endstop Z min or max dependent HOME DIR set false or true
 *  I<bool> - Endstop X2 min or max dependent HOME DIR set false or true
 *  J<bool> - Endstop Y2 min or max dependent HOME DIR set false or true
 *  K<bool> - Endstop Z2 min or max dependent HOME DIR set false or true
 *  P<bool> - Endstop Probe set false or true
 *  D<bool> - Endstop Door set false or true
 *  F<bool> - Endstop Fil Runout set false or true
 *  W<bool> - Endstop Power Check set false or true
 *
 */
inline void gcode_M124(void) {

  if (parser.seen('X')) {
    if (mechanics.home_dir[X_AXIS] == -1)
      endstops.setPullup(X_MIN, parser.value_bool());
    else
      endstops.setPullup(X_MAX, parser.value_bool());
  }

  if (parser.seen('Y')) {
    if (mechanics.home_dir[Y_AXIS] == -1)
      endstops.setPullup(Y_MIN, parser.value_bool());
    else
      endstops.setPullup(Y_MAX, parser.value_bool());
  }

  if (parser.seen('Z')) {
    if (mechanics.home_dir[Z_AXIS] == -1)
      endstops.setPullup(Z_MIN, parser.value_bool());
    else
      endstops.setPullup(Z_MAX, parser.value_bool());
  }

  #if HAS_X2_MIN || HAS_X2_MAX
    if (parser.seen('I')) {
      if (mechanics.home_dir[X_AXIS] == -1)
        endstops.setPullup(X2_MIN, parser.value_bool());
      else
        endstops.setPullup(X2_MAX, parser.value_bool());
    }
  #endif

  #if HAS_Y2_MIN || HAS_Y2_MAX
    if (parser.seen('J')) {
      if (mechanics.home_dir[Y_AXIS] == -1)
        endstops.setPullup(Y2_MIN, parser.value_bool());
      else
        endstops.setPullup(Y2_MAX, parser.value_bool());
    }
  #endif

  #if HAS_Z2_MIN || HAS_Z2_MAX
    if (parser.seen('K')) {
      if (mechanics.home_dir[Z_AXIS] == -1)
        endstops.setPullup(Z2_MIN, parser.value_bool());
      else
        endstops.setPullup(Z2_MAX, parser.value_bool());
    }
  #endif

  #if HAS_Z_PROBE_PIN
    if (parser.seen('P')) endstops.setPullup(Z_PROBE, parser.value_bool());
  #endif
  
  #if HAS_FIL_RUNOUT
    if (parser.seen('F')) endstops.setPullup(FIL_RUNOUT, parser.value_bool());
  #endif

  #if HAS_DOOR_OPEN
    if (parser.seen('D')) endstops.setPullup(DOOR_OPEN_SENSOR, parser.value_bool());
  #endif

  #if HAS_POWER_CHECK && HAS_SD_SUPPORT
    if (parser.seen('W')) endstops.setPullup(POWER_CHECK_SENSOR, parser.value_bool());
  #endif

  endstops.setup_pullup();
  endstops.report();
}
