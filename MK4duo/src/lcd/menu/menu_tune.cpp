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

//
// Tune Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

// Refresh the E factor after changing flow
inline void _lcd_refresh_e_factor_0() { tools.refresh_e_factor(0); }
#if EXTRUDERS > 1
  inline void _lcd_refresh_e_factor() { tools.refresh_e_factor(tools.active_extruder); }
  inline void _lcd_refresh_e_factor_1() { tools.refresh_e_factor(1); }
  #if EXTRUDERS > 2
    inline void _lcd_refresh_e_factor_2() { tools.refresh_e_factor(2); }
    #if EXTRUDERS > 3
      inline void _lcd_refresh_e_factor_3() { tools.refresh_e_factor(3); }
      #if EXTRUDERS > 4
        inline void _lcd_refresh_e_factor_4() { tools.refresh_e_factor(4); }
        #if EXTRUDERS > 5
          inline void _lcd_refresh_e_factor_5() { tools.refresh_e_factor(5); }
        #endif // EXTRUDERS > 5
      #endif // EXTRUDERS > 4
    #endif // EXTRUDERS > 3
  #endif // EXTRUDERS > 2
#endif // EXTRUDERS > 1

#if ENABLED(BABYSTEPPING)

  long babysteps_done = 0;

  void _lcd_babystep(const AxisEnum axis, PGM_P msg) {
    if (lcdui.use_click()) return lcdui.goto_previous_screen_no_defer();
    lcdui.encoder_direction_normal();
    if (lcdui.encoderPosition) {
      const int16_t babystep_increment = (int32_t)lcdui.encoderPosition * (BABYSTEP_MULTIPLICATOR);
      lcdui.encoderPosition = 0;
      lcdui.refresh(LCDVIEW_REDRAW_NOW);
      mechanics.babystep_axis(axis, babystep_increment);
      babysteps_done += babystep_increment;
    }
    if (lcdui.should_draw())
      draw_edit_screen(msg, ftostr43sign(mechanics.steps_to_mm[axis] * babysteps_done));
  }

  #if ENABLED(BABYSTEP_XY)
    void _lcd_babystep_x() { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEP_X)); }
    void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEP_Y)); }
    void lcd_babystep_x() { lcdui.goto_screen(_lcd_babystep_x); babysteps_done = 0; lcdui.defer_status_screen(true); }
    void lcd_babystep_y() { lcdui.goto_screen(_lcd_babystep_y); babysteps_done = 0; lcdui.defer_status_screen(true); }
  #endif

  #if DISABLED(BABYSTEP_ZPROBE_OFFSET)
    void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEP_Z)); }
    void lcd_babystep_z() { lcdui.goto_screen(_lcd_babystep_z); babysteps_done = 0; lcdui.defer_status_screen(true); }
  #endif

#endif // BABYSTEPPING

void lcd_tune_fixstep() {
  #if MECH(DELTA)
    commands.enqueue_and_echo_P(PSTR("G28 B"));
  #else
    commands.enqueue_and_echo_P(PSTR("G28 X Y B"));
  #endif
}

void menu_tune() {
  START_MENU();
  MENU_BACK(MSG_MAIN);

  //
  // Speed:
  //
  MENU_ITEM_EDIT(int3, MSG_SPEED, &mechanics.feedrate_percentage, 10, 999);

  //
  // Manual bed leveling, Bed Z:
  //
  #if ENABLED(MESH_BED_LEVELING) && ENABLED(LCD_BED_LEVELING)
    MENU_ITEM_EDIT(float43, MSG_BED_Z, &mbl.z_offset, -1, 1);
  #endif

  //
  // Nozzle:
  // Nozzle [1-4]:
  //
  #if HOTENDS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &heaters[0].target_temperature, 0, heaters[0].data.maxtemp - 15, watch_temp_callback_E0);
  #elif HOTENDS > 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &heaters[0].target_temperature, 0, heaters[0].data.maxtemp - 15, watch_temp_callback_E0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &heaters[1].target_temperature, 0, heaters[1].data.maxtemp - 15, watch_temp_callback_E1);
    #if HOTENDS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &heaters[2].target_temperature, 0, heaters[2].data.maxtemp - 15, watch_temp_callback_E2);
      #if HOTENDS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &heaters[3].target_temperature, 0, heaters[3].data.maxtemp - 15, watch_temp_callback_E3);
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1

  //
  // Bed:
  //
  #if HAS_TEMP_BED
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED, &heaters[BED_INDEX].target_temperature, 0, heaters[BED_INDEX].data.maxtemp - 15, watch_temp_callback_bed);
  #endif

  //
  // Chamber:
  //
  #if HAS_TEMP_CHAMBER
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER, &heaters[CHAMBER_INDEX].target_temperature, 0, heaters[CHAMBER_INDEX].data.maxtemp - 15, watch_temp_callback_chamber);
  #endif

  //
  // Cooler:
  //
  #if HAS_TEMP_COOLER
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_COOLER, &heaters[COOLER_INDEX].target_temperature, 0, heaters[COOLER_INDEX].data.maxtemp - 15, watch_temp_callback_cooler);
  #endif

  //
  // Fan Speed:
  //
  #if FAN_COUNT > 0
    #if HAS_FAN0
      #if FAN_COUNT > 1
        #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED " 0"
      #else
        #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED
      #endif
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_1ST_FAN_SPEED, &fans[0].Speed, 0, 255);
    #endif
    #if HAS_FAN1
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 1", &fans[1].Speed, 0, 255);
    #endif
    #if HAS_FAN2
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 2", &fans[2].Speed, 0, 255);
    #endif
    #if HAS_FAN3
      MENU_MULTIPLIER_ITEM_EDIT(uint8, MSG_FAN_SPEED " 3", &fans[3].Speed, 0, 255);
    #endif
  #endif // FAN_COUNT > 0

  //
  // Flow:
  // Flow [1-6]:
  //
  #if EXTRUDERS == 1
    MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW, &tools.flow_percentage[0], 10, 999, _lcd_refresh_e_factor_0);
  #else // EXTRUDERS > 1
    MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW, &tools.flow_percentage[tools.active_extruder], 10, 999, _lcd_refresh_e_factor);
    MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N1, &tools.flow_percentage[0], 10, 999, _lcd_refresh_e_factor_0);
    MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N2, &tools.flow_percentage[1], 10, 999, _lcd_refresh_e_factor_1);
    #if EXTRUDERS > 2
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N3, &tools.flow_percentage[2], 10, 999, _lcd_refresh_e_factor_2);
      #if EXTRUDERS > 3
        MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N4, &tools.flow_percentage[3], 10, 999, _lcd_refresh_e_factor_3);
        #if EXTRUDERS > 4
          MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N5, &tools.flow_percentage[4], 10, 999, _lcd_refresh_e_factor_4);
          #if EXTRUDERS > 5
            MENU_ITEM_EDIT_CALLBACK(int3, MSG_FLOW MSG_N6, &tools.flow_percentage[5], 10, 999, _lcd_refresh_e_factor_5);
          #endif // EXTRUDERS > 5
        #endif // EXTRUDERS > 4
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  //
  // Babystep X:
  // Babystep Y:
  // Babystep Z:
  //
  #if ENABLED(BABYSTEPPING)
    #if ENABLED(BABYSTEP_XY)
      MENU_ITEM(submenu, MSG_BABYSTEP_X, lcd_babystep_x);
      MENU_ITEM(submenu, MSG_BABYSTEP_Y, lcd_babystep_y);
    #endif
    #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
      MENU_ITEM(submenu, MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
    #else
      MENU_ITEM(submenu, MSG_BABYSTEP_Z, lcd_babystep_z);
    #endif
  #endif

  MENU_ITEM(function, MSG_FIX_LOSE_STEPS, lcd_tune_fixstep);

  END_MENU();
}

#endif // HAS_LCD_MENU
