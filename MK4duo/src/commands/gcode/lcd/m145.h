/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_LCD_MENU && HEATER_COUNT > 0

#define CODE_M145

/**
 * M145: Set the heatup state for a material in the LCD menu
 *   S<material> (0=PLA, 1=ABS, 2=GUM)
 *   H<hotend temp>
 *   B<bed temp>
 *   C<chamber temp>
 *   F<fan speed>
 */
inline void gcode_M145(void) {
  uint8_t material = (uint8_t)parser.intval('S');
  if (material >= COUNT(lcdui.preheat_hotend_temp)) {
    SERIAL_LM(ER, MSG_ERR_MATERIAL_INDEX);
  }
  else {
    int v;
    #if HOTENDS > 0
      if (parser.seenval('H')) {
        v = parser.value_int();
        lcdui.preheat_hotend_temp[material] = constrain(v, thermalManager.hotend_mintemp_all(), thermalManager.hotend_maxtemp_all());
      }
    #endif
    #if BEDS > 0
      if (parser.seenval('B')) {
        v = parser.value_int();
        lcdui.preheat_bed_temp[material] = constrain(v, thermalManager.bed_mintemp_all(), thermalManager.bed_maxtemp_all());
      }
    #endif
    #if CHAMBER > 0
      if (parser.seenval('C')) {
        v = parser.value_int();
        lcdui.preheat_chamber_temp[material] = constrain(v, thermalManager.chamber_mintemp_all(), thermalManager.chamber_maxtemp_all());
      }
    #endif
    #if FAN_COUNT > 0
      if (parser.seenval('F')) {
        v = parser.value_int();
        lcdui.preheat_fan_speed[material] = constrain(v, 0, 255);
      }
    #endif
  }
}

#endif
