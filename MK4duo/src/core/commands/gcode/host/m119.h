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

#define CODE_M119

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119(void) {

  SERIAL_EM(MSG_M119_REPORT);

  #if HAS_X_MIN
    SERIAL_EMT(MSG_X_MIN, ((READ(X_MIN_PIN)^endstops.isLogic(X_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X2_MIN
    SERIAL_EMT(MSG_X2_MIN, ((READ(X2_MIN_PIN)^endstops.isLogic(X2_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X_MAX
    SERIAL_EMT(MSG_X_MAX, ((READ(X_MAX_PIN)^endstops.isLogic(X_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X2_MAX
    SERIAL_EMT(MSG_X2_MAX, ((READ(X2_MAX_PIN)^endstops.isLogic(X2_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MIN
    SERIAL_EMT(MSG_Y_MIN, ((READ(Y_MIN_PIN)^endstops.isLogic(Y_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y2_MIN
    SERIAL_EMT(MSG_Y2_MIN, ((READ(Y2_MIN_PIN)^endstops.isLogic(Y2_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MAX
    SERIAL_EMT(MSG_Y_MAX, ((READ(Y_MAX_PIN)^endstops.isLogic(Y_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y2_MAX
    SERIAL_EMT(MSG_Y2_MAX, ((READ(Y2_MAX_PIN)^endstops.isLogic(Y2_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MIN
    SERIAL_EMT(MSG_Z_MIN, ((READ(Z_MIN_PIN)^endstops.isLogic(Z_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MIN
    SERIAL_EMT(MSG_Z2_MIN, ((READ(Z2_MIN_PIN)^endstops.isLogic(Z2_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MAX
    SERIAL_EMT(MSG_Z_MAX, ((READ(Z_MAX_PIN)^endstops.isLogic(Z_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MAX
    SERIAL_EMT(MSG_Z2_MAX, ((READ(Z2_MAX_PIN)^endstops.isLogic(Z2_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_PROBE_PIN
    SERIAL_EMT(MSG_Z_PROBE, ((READ(Z_PROBE_PIN)^endstops.isLogic(Z_PROBE)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_FIL_RUNOUT
    SERIAL_EMT(MSG_FILAMENT_RUNOUT_SENSOR, ((READ(FIL_RUNOUT0_PIN)^endstops.isLogic(FIL_RUNOUT)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_DOOR_OPEN
    SERIAL_EMT(MSG_DOOR_SENSOR, ((READ(DOOR_OPEN_PIN)^endstops.isLogic(DOOR_OPEN_SENSOR)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_POWER_CHECK
    SERIAL_EMT(MSG_POWER_CHECK_SENSOR, ((READ(POWER_CHECK_PIN)^endstops.isLogic(POWER_CHECK_SENSOR)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
}
