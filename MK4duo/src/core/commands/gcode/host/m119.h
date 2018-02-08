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
    SERIAL_EMT(MSG_X_MIN, ((READ(X_MIN_PIN)^endstops.Is_logic(X_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X_MAX
    SERIAL_EMT(MSG_X_MAX, ((READ(X_MAX_PIN)^endstops.Is_logic(X_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MIN
    SERIAL_EMT(MSG_Y_MIN, ((READ(Y_MIN_PIN)^endstops.Is_logic(Y_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MAX
    SERIAL_EMT(MSG_Y_MAX, ((READ(Y_MAX_PIN)^endstops.Is_logic(Y_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MIN
    SERIAL_EMT(MSG_Z_MIN, ((READ(Z_MIN_PIN)^endstops.Is_logic(Z_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MIN
    SERIAL_EMT(MSG_Z2_MIN, ((READ(Z2_MIN_PIN)^endstops.Is_logic(Z2_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z3_MIN
    SERIAL_EMT(MSG_Z3_MIN, ((READ(Z3_MIN_PIN)^endstops.Is_logic(Z3_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z4_MIN
    SERIAL_EMT(MSG_Z4_MIN, ((READ(Z4_MIN_PIN)^endstops.Is_logic(Z4_MIN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MAX
    SERIAL_EMT(MSG_Z_MAX, ((READ(Z_MAX_PIN)^endstops.Is_logic(Z_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MAX
    SERIAL_EMT(MSG_Z2_MAX, ((READ(Z2_MAX_PIN)^endstops.Is_logic(Z2_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z3_MAX
    SERIAL_EMT(MSG_Z3_MAX, ((READ(Z3_MAX_PIN)^endstops.Is_logic(Z3_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z4_MAX
    SERIAL_EMT(MSG_Z4_MAX, ((READ(Z4_MAX_PIN)^endstops.Is_logic(Z4_MAX)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_PROBE_PIN
    SERIAL_EMT(MSG_Z_PROBE, ((READ(Z_PROBE_PIN)^endstops.Is_logic(Z_PROBE)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_FIL_RUNOUT
    SERIAL_EMT(MSG_FILAMENT_RUNOUT_SENSOR, ((READ(FIL_RUNOUT_PIN)^endstops.Is_logic(FIL_RUNOUT)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_DOOR_OPEN
    SERIAL_EMT(MSG_DOOR_SENSOR, ((READ(DOOR_OPEN_PIN)^endstops.Is_logic(DOOR_OPEN)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_POWER_CHECK
    SERIAL_EMT(MSG_POWER_CHECK_SENSOR, ((READ(POWER_CHECK_PIN)^endstops.Is_logic(POWER_CHECK)) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN));
  #endif
}
