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

#define CODE_M303

/**
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default target temperature = 150C)
 *       H<hotend> (-1 for the bed, -2 for chamber, -3 for cooler) (default 0)
 *       C<cycles> minimum 3 (default 5)
 *       R<method> (0 - 3)
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303(void) {

  int8_t      h       = parser.intval('H');
  uint8_t     cycle   = parser.intval('C', 5);
  uint8_t     method  = parser.intval('R', 0);
  const bool  store   = parser.boolval('U');

  const int16_t target = parser.celsiusval('S', h < 0 ? 70 : 200);

  if (!commands.get_target_heater(h)) return;

  if (target > heaters[h].data.maxtemp - 15) {
    SERIAL_EM(MSG_PID_TEMP_TOO_HIGH);
    return;
  }

  SERIAL_EM(MSG_PID_AUTOTUNE_START);
  lcdui.reset_alert_level();
  LCD_MESSAGEPGM(MSG_PID_AUTOTUNE_START);

  if (heaters[h].data.type == IS_HOTEND)
    SERIAL_MV("Hotend:", h);
  #if HAS_TEMP_BED
    else if (heaters[h].data.type == IS_BED)
      SERIAL_MSG("BED");
  #endif
  #if HAS_TEMP_CHAMBER
    else if(heaters[h].data.type == IS_CHAMBER)
      SERIAL_MSG("CHAMBER");
  #endif
  #if HAS_TEMP_COOLER
    else if(heaters[h].data.type == IS_COOLER)
      SERIAL_MSG("COOLER");
  #endif

  NOLESS(cycle, 3);
  NOMORE(cycle, 20);

  NOMORE(method, 4);

  SERIAL_MV(" Temp:", target);
  SERIAL_MV(" Cycles:", cycle);
  SERIAL_MV(" Method:", method);
  if (store) SERIAL_MSG(" Apply result");
  SERIAL_EOL();

  thermalManager.PID_autotune(&heaters[h], target, cycle, method, store);

}
