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
      const int16_t steps = int16_t(lcdui.encoderPosition) * (BABYSTEP_MULTIPLICATOR);
      lcdui.encoderPosition = 0;
      lcdui.refresh(LCDVIEW_REDRAW_NOW);
      babystep.add_steps(axis, steps);
    }
    if (lcdui.should_draw()) {
      const float spm = mechanics.steps_to_mm[axis];
      draw_edit_screen(msg, ftostr54sign(spm * babystep.accum));
      #if ENABLED(BABYSTEP_DISPLAY_TOTAL)
        const bool in_view = (true
          #if HAS_GRAPHICAL_LCD
            && PAGE_CONTAINS(LCD_PIXEL_HEIGHT - MENU_FONT_HEIGHT, LCD_PIXEL_HEIGHT - 1)
          #endif
        );
        if (in_view) {
          #if HAS_GRAPHICAL_LCD
            lcdui.set_font(FONT_MENU);
            lcd_moveto(0, LCD_PIXEL_HEIGHT - MENU_FONT_DESCENT);
          #else
            lcd_moveto(0, LCD_HEIGHT - 1);
          #endif
          lcd_put_u8str_P(PSTR(MSG_BABYSTEP_TOTAL ":"));
          lcd_put_u8str(ftostr54sign(spm * babystep.axis_total[BS_TOTAL_AXIS(axis)]));
        }
      #endif
    }
  }

  inline void _lcd_babystep_go(const screenFunc_t screen) {
    lcdui.goto_screen(screen);
    lcdui.defer_status_screen();
    babystep.accum = 0;
  }

  #if ENABLED(BABYSTEP_XY)
    void _lcd_babystep_x()  { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEP_X)); }
    void _lcd_babystep_y()  { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEP_Y)); }
    void lcd_babystep_x()   { _lcd_babystep_go(_lcd_babystep_x); }
    void lcd_babystep_y()   { _lcd_babystep_go(_lcd_babystep_y); }
  #endif

  #if DISABLED(BABYSTEP_ZPROBE_OFFSET)
    void _lcd_babystep_z()  { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEP_Z)); }
    void lcd_babystep_z()   { _lcd_babystep_go(_lcd_babystep_z); }
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
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &hotends[0].target_temperature, 0, hotends[0].data.maxtemp - 10, watch_temp_callback_H0);
  #elif HOTENDS > 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N0, &hotends[0].target_temperature, 0, hotends[0].data.maxtemp - 10, watch_temp_callback_H0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &hotends[1].target_temperature, 0, hotends[1].data.maxtemp - 10, watch_temp_callback_H1);
    #if HOTENDS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &hotends[2].target_temperature, 0, hotends[2].data.maxtemp - 10, watch_temp_callback_H2);
      #if HOTENDS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &hotends[3].target_temperature, 0, hotends[3].data.maxtemp - 10, watch_temp_callback_H3);
        #if HOTENDS > 4
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &hotends[4].target_temperature, 0, hotends[4].data.maxtemp - 10, watch_temp_callback_H4);
          #if HOTENDS > 5
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N5, &hotends[5].target_temperature, 0, hotends[5].data.maxtemp - 10, watch_temp_callback_H5);
          #endif // HOTENDS > 5
        #endif // HOTENDS > 4
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1

  //
  // Bed:
  //
  #if BEDS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED, &beds[0].target_temperature, 0, beds[0].data.maxtemp - 10, watch_temp_callback_bed0);
  #elif BEDS > 0
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N0, &beds[0].target_temperature, 0, beds[0].data.maxtemp - 10, watch_temp_callback_bed0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N1, &beds[1].target_temperature, 0, beds[1].data.maxtemp - 10, watch_temp_callback_bed1);
    #if BEDS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N2, &beds[2].target_temperature, 0, beds[2].data.maxtemp - 10, watch_temp_callback_bed2);
      #if BEDS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED MSG_N2, &beds[3].target_temperature, 0, beds[3].data.maxtemp - 10, watch_temp_callback_bed3);
      #endif // BEDS > 3
    #endif // BEDS > 2
  #endif // BEDS > 1

  //
  // Chamber:
  //
  #if CHAMBERS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER, &chambers[0].target_temperature, 0, chambers[0].data.maxtemp - 10, watch_temp_callback_chamber0);
  #elif CHAMBERS > 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N0, &chambers[0].target_temperature, 0, chambers[0].data.maxtemp - 10, watch_temp_callback_chamber0);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N1, &chambers[1].target_temperature, 0, chambers[1].data.maxtemp - 10, watch_temp_callback_chamber1);
    #if CHAMBERS > 2
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N2, &chambers[2].target_temperature, 0, chambers[2].data.maxtemp - 10, watch_temp_callback_chamber2);
      #if CHAMBERS > 3
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER MSG_N3, &chambers[3].target_temperature, 0, chambers[3].data.maxtemp - 10, watch_temp_callback_chamber3);
      #endif // CHAMBERS > 3
    #endif // CHAMBERS > 2
  #endif // CHAMBERS > 1

  //
  // Cooler:
  //
  #if COOLERS == 1
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_COOLER, &coolers[0].target_temperature, 0, coolers[0].data.maxtemp - 10, watch_temp_callback_cooler0);
  #endif

  //
  // Fan Speed:
  //
  #if FAN_COUNT > 0
    #if HAS_FAN0
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 0", &fans[0].speed, 0, 255);
    #endif
    #if HAS_FAN1
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 1", &fans[1].speed, 0, 255);
    #endif
    #if HAS_FAN2
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 2", &fans[2].speed, 0, 255);
    #endif
    #if HAS_FAN3
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 3", &fans[3].speed, 0, 255);
    #endif
    #if HAS_FAN4
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 4", &fans[4].speed, 0, 255);
    #endif
    #if HAS_FAN5
      MENU_MULTIPLIER_ITEM_EDIT(percent, MSG_FAN_SPEED " 5", &fans[5].speed, 0, 255);
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
