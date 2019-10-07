/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
inline void _lcd_refresh_e_factor() {
  LOOP_EXTRUDER()
    tools.refresh_e_factor(e);
}

#if ENABLED(BABYSTEPPING)

  long babysteps_done = 0;

  void _lcd_babystep(const AxisEnum axis, PGM_P msg) {
    if (lcdui.use_click()) return lcdui.goto_previous_screen_no_defer();
    if (lcdui.encoderPosition) {
      const int16_t steps = int16_t(lcdui.encoderPosition) * (BABYSTEP_MULTIPLICATOR);
      lcdui.encoderPosition = 0;
      lcdui.refresh(LCDVIEW_REDRAW_NOW);
      babystep.add_steps(axis, steps);
    }
    if (lcdui.should_draw()) {
      const float spm = mechanics.steps_to_mm[axis];
      draw_edit_screen(msg, NULL, ftostr54sign(spm * babystep.accum));
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
    commands.inject_P(PSTR("G28 B"));
  #else
    commands.inject_P(PSTR("G28 X Y B"));
  #endif
}

void menu_tune() {
  START_MENU();
  BACK_ITEM(MSG_MAIN);

  //
  // Speed:
  //
  EDIT_ITEM(int3, MSG_SPEED, &mechanics.feedrate_percentage, 10, 999);

  //
  // Manual bed leveling, Bed Z:
  //
  #if ENABLED(MESH_BED_LEVELING) && ENABLED(LCD_BED_LEVELING)
    EDIT_ITEM(float43, MSG_BED_Z, &mbl.data.z_offset, -1, 1);
  #endif

  //
  // Nozzle:
  //
  #if MAX_HOTEND > 0
    LOOP_HOTEND()
      EDIT_ITEM_FAST_INDEX(int3, MSG_NOZZLE, h, &hotends[h]->target_temperature, 0, hotends[h]->data.temp.max - 10, watch_temp_callback_hotend);
  #endif

  //
  // Bed:
  //
  #if MAX_BED > 0
    LOOP_BED()
      EDIT_ITEM_FAST_INDEX(int3, MSG_BED, h, &beds[h]->target_temperature, 0, beds[h]->data.temp.max - 10, watch_temp_callback_bed);
  #endif

  //
  // Chamber:
  //
  #if MAX_CHAMBER > 0
    LOOP_CHAMBER()
      EDIT_ITEM_FAST_INDEX(int3, MSG_CHAMBER, h, &chambers[h]->target_temperature, 0, chambers[h]->data.temp.max - 10, watch_temp_callback_chamber);
  #endif

  //
  // Cooler:
  //
  #if MAX_COOLER > 0
    EDIT_ITEM_FAST_INDEX(int3, MSG_COOLER, 0xFF, &coolers[0]->target_temperature, 0, coolers[0]->data.temp.max - 10, watch_temp_callback_cooler);
  #endif

  //
  // Fan Speed:
  //
  #if MAX_FAN > 0
    LOOP_FAN()
      EDIT_ITEM_FAST_INDEX(percent, MSG_FAN_SPEED, f, &fans[f]->speed, 0, 255);
  #endif

  //
  // Flow:
  // Flow [1-6]:
  //
  #if MAX_EXTRUDER > 0
    LOOP_EXTRUDER()
      EDIT_ITEM_FAST_INDEX(int3, MSG_FLOW, e, &tools.flow_percentage[e], 10, 999, _lcd_refresh_e_factor);
  #endif

  //
  // Babystep X:
  // Babystep Y:
  // Babystep Z:
  //
  #if ENABLED(BABYSTEPPING)
    #if ENABLED(BABYSTEP_XY)
      SUBMENU(MSG_BABYSTEP_X, lcd_babystep_x);
      SUBMENU(MSG_BABYSTEP_Y, lcd_babystep_y);
    #endif
    #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
      SUBMENU(MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
    #else
      SUBMENU(MSG_BABYSTEP_Z, lcd_babystep_z);
    #endif
  #endif

  ACTION_ITEM(MSG_FIX_LOSE_STEPS, lcd_tune_fixstep);

  END_MENU();
}

#endif // HAS_LCD_MENU
