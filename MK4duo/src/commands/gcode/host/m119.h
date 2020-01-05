/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#define CODE_M119

static void print_state(const bool is_hit, PGM_P const label=nullptr) {
  if (label) SERIAL_STR(label);
  SERIAL_MSG(": ");
  SERIAL_STR(is_hit ? PSTR(MSG_HOST_ENDSTOP_HIT) : PSTR(MSG_HOST_ENDSTOP_OPEN));
  SERIAL_EOL();
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() {

  SERIAL_EM(MSG_HOST_M119_REPORT);

  #define ES_REPORT(S) print_state(READ(S##_PIN) ^ endstops.isLogic(S), PSTR(MSG_HOST_##S))

  #if HAS_BLTOUCH
    bltouch.cmd_mode_SW();
  #endif

  #if HAS_X_MIN
    ES_REPORT(X_MIN);
  #endif
  #if HAS_X2_MIN
    ES_REPORT(X2_MIN);
  #endif
  #if HAS_X_MAX
    ES_REPORT(X_MAX);
  #endif
  #if HAS_X2_MAX
    ES_REPORT(X2_MAX);
  #endif
  #if HAS_Y_MIN
    ES_REPORT(Y_MIN);
  #endif
  #if HAS_Y2_MIN
    ES_REPORT(Y2_MIN);
  #endif
  #if HAS_Y_MAX
    ES_REPORT(Y_MAX);
  #endif
  #if HAS_Y2_MAX
    ES_REPORT(Y2_MAX);
  #endif
  #if HAS_Z_MIN
    ES_REPORT(Z_MIN);
  #endif
  #if HAS_Z2_MIN
    ES_REPORT(Z2_MIN);
  #endif
  #if HAS_Z3_MIN
    ES_REPORT(Z3_MIN);
  #endif
  #if HAS_Z_MAX
    ES_REPORT(Z_MAX);
  #endif
  #if HAS_Z2_MAX
    ES_REPORT(Z2_MAX);
  #endif
  #if HAS_Z3_MAX
    ES_REPORT(Z3_MAX);
  #endif
  #if HAS_Z_PROBE_PIN
    ES_REPORT(Z_PROBE);
  #endif
  #if HAS_FIL_RUNOUT_0
    print_state(READ(FIL_RUNOUT_0_PIN) ^ filamentrunout.sensor.isLogic(FIL_RUNOUT_0), PSTR(MSG_HOST_FILAMENT_RUNOUT " 0"));
  #endif
  #if HAS_FIL_RUNOUT_1
    print_state(READ(FIL_RUNOUT_1_PIN) ^ filamentrunout.sensor.isLogic(FIL_RUNOUT_1), PSTR(MSG_HOST_FILAMENT_RUNOUT " 1"));
  #endif
  #if HAS_FIL_RUNOUT_2
    print_state(READ(FIL_RUNOUT_2_PIN) ^ filamentrunout.sensor.isLogic(FIL_RUNOUT_2), PSTR(MSG_HOST_FILAMENT_RUNOUT " 2"));
  #endif
  #if HAS_FIL_RUNOUT_3
    print_state(READ(FIL_RUNOUT_3_PIN) ^ filamentrunout.sensor.isLogic(FIL_RUNOUT_3), PSTR(MSG_HOST_FILAMENT_RUNOUT " 3"));
  #endif
  #if HAS_FIL_RUNOUT_4
    print_state(READ(FIL_RUNOUT_4_PIN) ^ filamentrunout.sensor.isLogic(FIL_RUNOUT_4), PSTR(MSG_HOST_FILAMENT_RUNOUT " 4"));
  #endif
  #if HAS_FIL_RUNOUT_5
    print_state(READ(FIL_RUNOUT_5_PIN) ^ filamentrunout.sensor.isLogic(FIL_RUNOUT_5), PSTR(MSG_HOST_FILAMENT_RUNOUT " 5"));
  #endif
  #if HAS_DOOR_OPEN
    ES_REPORT(DOOR_OPEN);
  #endif
  #if HAS_POWER_CHECK
    print_state(READ(POWER_CHECK_PIN) ^ powerManager.isLogic(), PSTR(MSG_HOST_POWER_CHECK));
  #endif

  #if HAS_BLTOUCH
    bltouch.cmd_reset_mode_SW();
  #endif
}
