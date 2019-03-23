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

//
// Delta Calibrate Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && MECH(DELTA)

void _lcd_calibrate_homing() {
  lcd_draw_homing();
  if (mechanics.isHomedAll()) lcdui.goto_previous_screen();
}

void _lcd_delta_calibrate_home() {
  commands.enqueue_and_echo_P(PSTR("G28"));
  lcdui.goto_screen(_lcd_calibrate_homing);
}

void _goto_tower_x() { _man_probe_pt(COS(RADIANS(210)) * mechanics.data.probe_radius, SIN(RADIANS(210)) * mechanics.data.probe_radius); }
void _goto_tower_y() { _man_probe_pt(COS(RADIANS(330)) * mechanics.data.probe_radius, SIN(RADIANS(330)) * mechanics.data.probe_radius); }
void _goto_tower_z() { _man_probe_pt(COS(RADIANS( 90)) * mechanics.data.probe_radius, SIN(RADIANS( 90)) * mechanics.data.probe_radius); }
void _goto_center()  { _man_probe_pt(0, 0); }

void lcd_delta_settings() {
  START_MENU();
  MENU_BACK(MSG_DELTA_CALIBRATE);
  MENU_ITEM_EDIT(float52, MSG_DELTA_HEIGHT, &mechanics.data.height, mechanics.data.height - 10, mechanics.data.height + 10);
  MENU_ITEM_EDIT(float43, "Ex", &mechanics.data.endstop_adj[A_AXIS], -5, 0);
  MENU_ITEM_EDIT(float43, "Ey", &mechanics.data.endstop_adj[B_AXIS], -5, 0);
  MENU_ITEM_EDIT(float43, "Ez", &mechanics.data.endstop_adj[C_AXIS], -5, 0);
  MENU_ITEM_EDIT(float52, MSG_DELTA_DIAG_ROD, &mechanics.data.diagonal_rod, mechanics.data.diagonal_rod - 10, mechanics.data.diagonal_rod + 10);
  MENU_ITEM_EDIT(float52, MSG_DELTA_RADIUS, &mechanics.data.radius, mechanics.data.radius - 10, mechanics.data.radius + 10);
  MENU_ITEM_EDIT(float43, "Tx (deg)", &mechanics.data.tower_angle_adj[A_AXIS], -5, 5);
  MENU_ITEM_EDIT(float43, "Ty (deg)", &mechanics.data.tower_angle_adj[B_AXIS], -5, 5);
  MENU_ITEM_EDIT(float43, "Tz (deg)", &mechanics.data.tower_angle_adj[C_AXIS], -5, 5);
  MENU_ITEM_EDIT(float43, "Tx (radius)", &mechanics.data.tower_radius_adj[A_AXIS], -5, 5);
  MENU_ITEM_EDIT(float43, "Ty (radius)", &mechanics.data.tower_radius_adj[B_AXIS], -5, 5);
  MENU_ITEM_EDIT(float43, "Tz (radius)", &mechanics.data.tower_radius_adj[C_AXIS], -5, 5);
  END_MENU();
}

void menu_delta_calibrate() {
  START_MENU();
  MENU_BACK(MSG_MAIN);
  MENU_ITEM(submenu, MSG_DELTA_SETTINGS, lcd_delta_settings);
  #if ENABLED(DELTA_AUTO_CALIBRATION_1) || ENABLED(DELTA_AUTO_CALIBRATION_2)
    MENU_ITEM(gcode, MSG_DELTA_AUTO_CALIBRATE, PSTR("G33"));
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
      MENU_ITEM(function, MSG_LOAD_EEPROM, lcd_load_settings);
    #endif
  #endif
  MENU_ITEM(submenu, MSG_AUTO_HOME, _lcd_delta_calibrate_home);
  if (mechanics.home_flag.ZHomed) {
    MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_X, _goto_tower_x);
    MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_Y, _goto_tower_y);
    MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_Z, _goto_tower_z);
    MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_CENTER, _goto_center);
  }
  END_MENU();
}

#endif // HAS_LCD_MENU && MECH(DELTA)
