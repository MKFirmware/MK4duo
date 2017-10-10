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

#if HAS_PID

  #define CODE_M301

  /**
   * M301: Set PID parameters P I D (and optionally C, L)
   *
   *   H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 COOLER
   *
   *   P[float] Kp term
   *   I[float] Ki term
   *   D[float] Kd term
   *
   * With PID_ADD_EXTRUSION_RATE:
   *
   *   C[float] Kc term
   *   L[float] LPQ length
   */
  inline void gcode_M301(void) {

    // multi-hotend PID patch: M301 updates or prints a single hotend's PID values
    // default behaviour (omitting H parameter) is to update for hotend 0 only
    int8_t h = parser.seen('H') ? parser.value_int() : 0; // hotend being updated

    if (!commands.get_target_heater(h)) return;

    if (parser.seen('P')) heaters[h].Kp = parser.value_float();
    if (parser.seen('I')) heaters[h].Ki = parser.value_float();
    if (parser.seen('D')) heaters[h].Kd = parser.value_float();
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (parser.seen('C')) heaters[h].Kc = parser.value_float();
      if (parser.seen('L')) thermalManager.lpq_len = parser.value_float();
      NOMORE(thermalManager.lpq_len, LPQ_MAX_LEN);
    #endif

    thermalManager.updatePID();

    heaters[h].print_PID(h);

  }

#endif // HAS_PID
