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
  commands.inject_P(G28_CMD);
  lcdui.goto_screen(_lcd_calibrate_homing);
}

void _goto_tower_angle(const float &a) {
  xy_pos_t tower_vector = { cos(RADIANS(a)), sin(RADIANS(a)) };
  _man_probe_pt(tower_vector * mechanics.data.probe_radius);
}

void _goto_tower_x()  { _goto_tower_angle(210); }
void _goto_tower_y()  { _goto_tower_angle(330); }
void _goto_tower_z()  { _goto_tower_angle( 90); }
void _goto_center()   { xy_pos_t center{0}; _man_probe_pt(center); }

void lcd_delta_settings() {

  auto _recalc_delta_settings = []() {
    #if HAS_LEVELING
      bedlevel.reset();
    #endif
    mechanics.recalc_delta_settings();
  };

  START_MENU();
  BACK_ITEM(MSG_DELTA_CALIBRATE);
  EDIT_ITEM(float52sign, MSG_DELTA_HEIGHT, &mechanics.data.height, mechanics.data.height - 100, mechanics.data.height + 100, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Ex"), &mechanics.data.endstop_adj[A_AXIS], -5, 0, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Ey"), &mechanics.data.endstop_adj[B_AXIS], -5, 0, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Ez"), &mechanics.data.endstop_adj[C_AXIS], -5, 0, _recalc_delta_settings);
  EDIT_ITEM(float52sign, MSG_DELTA_DIAG_ROD, &mechanics.data.diagonal_rod, mechanics.data.diagonal_rod - 10, mechanics.data.diagonal_rod + 10, _recalc_delta_settings);
  EDIT_ITEM(float52sign, MSG_DELTA_RADIUS, &mechanics.data.radius, mechanics.data.radius - 10, mechanics.data.radius + 10, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Tx (deg)"), &mechanics.data.tower_angle_adj[A_AXIS], -5, 5, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Ty (deg)"), &mechanics.data.tower_angle_adj[B_AXIS], -5, 5, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Tz (deg)"), &mechanics.data.tower_angle_adj[C_AXIS], -5, 5, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Tx (radius)"), &mechanics.data.tower_radius_adj[A_AXIS], -5, 5, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Ty (radius)"), &mechanics.data.tower_radius_adj[B_AXIS], -5, 5, _recalc_delta_settings);
  EDIT_ITEM_P(float43, PSTR("Tz (radius)"), &mechanics.data.tower_radius_adj[C_AXIS], -5, 5, _recalc_delta_settings);
  END_MENU();
}

void menu_delta_calibrate() {
  START_MENU();
  BACK_ITEM(MSG_MAIN);

  SUBMENU(MSG_DELTA_SETTINGS, lcd_delta_settings);

  #if ENABLED(DELTA_AUTO_CALIBRATION_1) || ENABLED(DELTA_AUTO_CALIBRATION_2)
    GCODES_ITEM(MSG_DELTA_AUTO_CALIBRATE, PSTR("G33"));
    #if ENABLED(EEPROM_SETTINGS)
      ACTION_ITEM(MSG_STORE_EEPROM, []{ eeprom.store(); });
      ACTION_ITEM(MSG_LOAD_EEPROM, []{ eeprom.load(); });
    #endif
  #endif

  SUBMENU(MSG_AUTO_HOME, _lcd_delta_calibrate_home);

  if (mechanics.home_flag.ZHomed) {
    SUBMENU(MSG_DELTA_CALIBRATE_X, _goto_tower_x);
    SUBMENU(MSG_DELTA_CALIBRATE_Y, _goto_tower_y);
    SUBMENU(MSG_DELTA_CALIBRATE_Z, _goto_tower_z);
    SUBMENU(MSG_DELTA_CALIBRATE_CENTER, _goto_center);
  }

  END_MENU();

}

#endif // HAS_LCD_MENU && MECH(DELTA)
