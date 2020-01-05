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

#if HAS_HEATER

#define CODE_M301

/**
 * M301: Set PID parameters P I D (and optionally C, L)
 *
 *   H[heaters]   0-5 Hotend, -1 BED, -2 CHAMBER, -3 COOLER
 *
 *    T[int]      0-3 For Select Beds, Chambers or Coolers (default 0)
 *
 *    P[float]    Kp term
 *    I[float]    Ki term
 *    D[float]    Kd term
 *
 * With PID_ADD_EXTRUSION_RATE:
 *
 *    C[float]    Kc term
 *    L[int]      LPQ length
 */
inline void gcode_M301() {

  Heater * const act = commands.get_target_heater();

  if (!act) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M301 report.
    if (!parser.seen("PIDCL")) {
      act->print_M301();
      return;
    }
  #endif

  if (parser.seen('P')) act->data.pid.Kp = parser.value_float();
  if (parser.seen('I')) act->data.pid.Ki = parser.value_float();
  if (parser.seen('D')) act->data.pid.Kd = parser.value_float();

  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    if (act->type == IS_HOTEND) {
      if (parser.seen('C')) act->data.pid.Kc = parser.value_float();
      if (parser.seen('L')) tempManager.heater.lpq_len = parser.value_int();
      if (tempManager.heater.lpq_len > LPQ_MAX_LEN) tempManager.heater.lpq_len = LPQ_MAX_LEN;
      if (tempManager.heater.lpq_len < 0) tempManager.heater.lpq_len = 0;
    }
  #endif

  act->setPidTuned(true);
  act->ResetFault();

}

#endif // HAS_HEATER
