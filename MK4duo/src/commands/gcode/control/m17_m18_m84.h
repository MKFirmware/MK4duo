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

#define CODE_M17
#define CODE_M18
#define CODE_M84

/**
 * M17: Enable stepper motors
 */
inline void gcode_M17() {
  if (parser.seen_any()) {
    LCD_MESSAGEPGM(MSG_NO_MOVE);
    stepper.enable_all();
  }
  else {
    if (parser.seen('X')) stepper.enable_X();
    if (parser.seen('Y')) stepper.enable_Y();
    if (parser.seen('Z')) stepper.enable_Z();
    if (parser.seen('E')) stepper.enable_E();
  }
}

/**
 * M18, M84: Disable stepper motors
 */
inline void gcode_M18_M84() {
  if (parser.seenval('S')) {
    printer.move_time = parser.value_ushort();
  }
  else {
    if (parser.seen("XYZE")) {
      planner.synchronize();
      if (parser.seen('X')) stepper.disable_X();
      if (parser.seen('Y')) stepper.disable_Y();
      if (parser.seen('Z')) stepper.disable_Z();
      if (parser.seen('E')) stepper.disable_E();
    }
    else
      planner.finish_and_disable();

    #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(ULTIPANEL)
      if (ubl.lcd_map_control) {
        ubl.lcd_map_control = false;
        lcdui.defer_status_screen(false);
      }
    #endif
  }
}
