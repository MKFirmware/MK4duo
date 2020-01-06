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

#define CODE_M303

/**
 * M303: PID relay autotune
 *
 *   H[heaters]   0-5 Hotend, -1 BED, -2 CHAMBER, -3 COOLER
 *
 *    T[int]      0-3 For Select Beds, Chambers or Cooler(default 0)
 *
 *    S[temp]     sets the target temperature. (default target temperature = 150C)
 *    C[cycles]   minimum 3 (default 5)
 *    R[method]   0-3 (default 0)
 *    U[bool]     with a non-zero value will apply the result to current settings
 *
 */
inline void gcode_M303() {

  Heater * const act = commands.get_target_heater();

  if (!act) return;

  uint8_t     cycle   = parser.intval('C', 5);
  uint8_t     method  = parser.intval('R', 0);
  const bool  store   = parser.boolval('U');

  const int16_t target = parser.celsiusval('S', act->type == IS_HOTEND ? 200 : 70);

  if (target > act->data.temp.max - 10) {
    SERIAL_EM(MSG_HOST_PID_TEMP_TOO_HIGH);
    return;
  }

  SERIAL_EM(MSG_HOST_PID_AUTOTUNE_START);
  lcdui.reset_alert_level();
  LCD_MESSAGEPGM(MSG_PID_AUTOTUNE_START);

  switch (act->type) {
    case IS_HOTEND:   SERIAL_MV("Hotend:",  act->data.ID);  break;
    case IS_BED:      SERIAL_MV("BED:",     act->data.ID);  break;
    case IS_CHAMBER:  SERIAL_MV("CHAMBER:", act->data.ID);  break;
    case IS_COOLER:   SERIAL_MV("COOLER:",  act->data.ID);  break;
    default: break;
  }

  NOLESS(cycle, 3);
  NOMORE(cycle, 20);

  NOMORE(method, 4);

  SERIAL_MV(" Temp:", target);
  SERIAL_MV(" Cycles:", cycle);
  SERIAL_MV(" Method:", method);
  if (store) SERIAL_MSG(" Apply into EEPROM");
  SERIAL_EOL();

  act->PID_autotune(target, cycle, method, store);

}

#endif // HAS_HEATER
