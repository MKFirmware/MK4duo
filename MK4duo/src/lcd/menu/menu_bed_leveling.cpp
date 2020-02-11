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
// Bed Leveling Menus
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && (HAS_PROBE_MANUALLY || MECH(DELTA))

  void _man_probe_pt(const xy_pos_t &xy) {
    if (!lcdui.wait_for_move) {
      lcdui.wait_for_move = true;
      mechanics.do_blocking_move_to_xy_z(xy, MANUAL_PROBE_HEIGHT);
      lcdui.wait_for_move = false;
      lcdui.synchronize();
      move_menu_scale = LCD_Z_STEP;
      lcdui.goto_screen(lcd_move_z);
    }
  }

  float lcd_probe_pt(const xy_pos_t &xy) {
    _man_probe_pt(xy);
    PRINTER_KEEPALIVE(PausedforUser);
    lcdui.defer_status_screen();
    printer.setWaitForUser(true);
    host_action.prompt_do(PROMPT_USER_CONTINUE, PSTR("Delta Calibration in progress"), PSTR("Continue"));
    while (printer.isWaitForUser()) printer.idle();
    lcdui.goto_previous_screen_no_defer();
    return mechanics.position.z;
  }

#endif

#if HAS_LCD_MENU && ENABLED(LCD_BED_LEVELING)

#if HAS_PROBE_MANUALLY || ENABLED(MESH_BED_LEVELING)

  //
  // Motion > Level Bed handlers
  //

  static uint8_t manual_probe_index;

  // LCD probed points are from defaults
  constexpr uint8_t total_probe_points = (
    #if ENABLED(AUTO_BED_LEVELING_3POINT)
      3
    #elif ABL_GRID || ENABLED(MESH_BED_LEVELING)
      GRID_MAX_POINTS
    #endif
  );

  //
  // Bed leveling is done. Wait for G29 to complete.
  // A flag is used so that this can release control
  // and allow the command queue to be processed.
  //
  // When G29 finishes the last move:
  // - Raise Z to the "manual probe height"
  // - Don't return until done.
  //
  // ** This blocks the command queue! **
  //
  void _lcd_level_bed_done() {
    if (!lcdui.wait_for_move) {
      #if MANUAL_PROBE_HEIGHT > 0 && DISABLED(MESH_BED_LEVELING)
        // Display "Done" screen and wait for moves to complete
        mechanics.do_blocking_move_to_z(MANUAL_PROBE_HEIGHT, manual_feedrate_mm_s.z);
        lcdui.synchronize(GET_TEXT(MSG_LEVEL_BED_DONE));
      #endif
      lcdui.goto_previous_screen_no_defer();
      sound.feedback();
    }
    if (lcdui.should_draw()) MenuItem_static::draw(LCD_HEIGHT >= 4 ? 1 : 0, GET_TEXT(MSG_LEVEL_BED_DONE));
    lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  }

  void _lcd_level_goto_next_point();

  //
  // Step 7: Get the Z coordinate, click goes to the next point or exits
  //
  void _lcd_level_bed_get_z() {

    if (lcdui.use_click()) {

      //
      // Save the current Z position and move
      //

      // If done...
      if (++manual_probe_index >= total_probe_points) {
        //
        // The last G29 records the point and enables bed leveling
        //
        lcdui.wait_for_move = true;
        lcdui.goto_screen(_lcd_level_bed_done);
        #if ENABLED(MESH_BED_LEVELING)
          commands.inject_P(PSTR("G29 S2"));
        #elif HAS_PROBE_MANUALLY
          commands.inject_P(PSTR("G29 V1"));
        #endif
      }
      else
        _lcd_level_goto_next_point();

      return;
    }

    //
    // Encoder knob or keypad buttons adjust the Z position
    //
    if (lcdui.encoderPosition) {
      const float z = mechanics.position.z + float(int32_t(lcdui.encoderPosition)) * (LCD_Z_STEP);
      lcd_line_to_z(constrain(z, -(LCD_PROBE_Z_RANGE) * 0.5f, (LCD_PROBE_Z_RANGE) * 0.5f));
      lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
      lcdui.encoderPosition = 0;
    }

    //
    // Draw on first display, then only on Z change
    //
    if (lcdui.should_draw()) {
      const float v = mechanics.position.z;
      MenuItemBase::itemIndex = NO_INDEX;
      MenuEditItemBase::draw_edit_screen(GET_TEXT(MSG_MOVE_Z), ftostr43sign(v + (v < 0 ? -0.0001f : 0.0001f), '+'));
    }
  }

  //
  // Step 6: Display "Next point: 1 / 9" while waiting for move to finish
  //
  void _lcd_level_bed_moving() {
    if (lcdui.should_draw()) {
      char msg[10];
      sprintf_P(msg, PSTR("%i / %u"), int(manual_probe_index + 1), total_probe_points);
      MenuItemBase::itemIndex = NO_INDEX;
      MenuEditItemBase::draw_edit_screen(GET_TEXT(MSG_LEVEL_BED_NEXT_POINT), msg);
    }
    lcdui.refresh(LCDVIEW_CALL_NO_REDRAW);
    if (!lcdui.wait_for_move) lcdui.goto_screen(_lcd_level_bed_get_z);
  }

  //
  // Step 5: Initiate a move to the next point
  //
  void _lcd_level_goto_next_point() {
    lcdui.goto_screen(_lcd_level_bed_moving);

    // G29 Records Z, moves, and signals when it pauses
    lcdui.wait_for_move = true;
    #if ENABLED(MESH_BED_LEVELING)
      commands.inject_P(manual_probe_index ? PSTR("G29 S2") : PSTR("G29 S1"));
    #elif HAS_PROBE_MANUALLY
      commands.inject_P(PSTR("G29 V1"));
    #endif
  }

  //
  // Step 4: Display "Click to Begin", wait for click
  //         Move to the first probe position
  //
  void _lcd_level_bed_homing_done() {
    if (lcdui.should_draw()) MenuItem_static::draw(1, GET_TEXT(MSG_LEVEL_BED_WAITING));
    if (lcdui.use_click()) {
      manual_probe_index = 0;
      _lcd_level_goto_next_point();
    }
  }

  //
  // Step 3: Display "Homing XYZ" - Wait for homing to finish
  //
  void _lcd_level_bed_homing() {
    lcd_draw_homing();
    if (mechanics.isHomedAll()) lcdui.goto_screen(_lcd_level_bed_homing_done);
  }

  //
  // Step 2: Continue Bed Leveling...
  //
  void _lcd_level_bed_continue() {
    lcdui.defer_status_screen();
    mechanics.unsetHomedAll();
    lcdui.goto_screen(_lcd_level_bed_homing);
    commands.inject_P(G28_CMD);
  }

#endif // PROBE_MANUALLY || MESH_BED_LEVELING

#if ENABLED(MESH_EDIT_MENU)

  inline void refresh_planner() {
    mechanics.set_position_from_steppers_for_axis(ALL_AXES);
    mechanics.sync_plan_position();
  }

  void menu_edit_mesh() {
    static uint8_t xind, yind; // =0
    START_MENU();
    BACK_ITEM(MSG_BED_LEVELING);
    EDIT_ITEM(int8, MSG_MESH_X, &xind, 0, GRID_MAX_POINTS_X - 1);
    EDIT_ITEM(int8, MSG_MESH_Y, &yind, 0, GRID_MAX_POINTS_Y - 1);
    EDIT_ITEM_FAST(float43, MSG_MESH_EDIT_Z, &Z_VALUES(xind, yind), -(LCD_PROBE_Z_RANGE) * 0.5, (LCD_PROBE_Z_RANGE) * 0.5, refresh_planner);
    END_MENU();
  }

#endif // MESH_EDIT_MENU

/**
 * Step 1: Bed Level entry-point
 *
 * << Motion
 *    Auto Home           (if homing needed)
 *    Leveling On/Off     (if data exists, and homed)
 *    Fade Height: ---    (Req: ENABLE_LEVELING_FADE_HEIGHT)
 *    Mesh Z Offset: ---  (Req: MESH_BED_LEVELING)
 *    Z Probe Offset: --- (Req: HAS_BED_PROBE, Opt: BABYSTEP_ZPROBE_OFFSET)
 *    Level Bed >
 *    Level Corners >     (if homed)
 *    Load Settings       (Req: EEPROM_SETTINGS)
 *    Save Settings       (Req: EEPROM_SETTINGS)
 */
void menu_bed_leveling() {
  START_MENU();
  BACK_ITEM(MSG_MOTION);

  const bool is_homed = mechanics.isHomedAll();

  // Auto Home if not using manual probing
  #if DISABLED(PROBE_MANUALLY) && DISABLED(MESH_BED_LEVELING)
    if (!is_homed) GCODES_ITEM(MSG_AUTO_HOME, G28_CMD);
  #endif

  // Level Bed
  #if HAS_PROBE_MANUALLY || ENABLED(MESH_BED_LEVELING)
    // Manual leveling uses a guided procedure
    SUBMENU(MSG_LEVEL_BED, _lcd_level_bed_continue);
  #else
    // Automatic leveling can just run the G-code
    GCODES_ITEM(MSG_LEVEL_BED, is_homed ? PSTR("G29") : PSTR("G28\nG29"));
  #endif

  #if ENABLED(MESH_EDIT_MENU)
    if (bedlevel.leveling_is_valid())
      SUBMENU(MSG_EDIT_MESH, menu_edit_mesh);
  #endif

  // Homed and leveling is valid? Then leveling can be toggled.
  if (is_homed && bedlevel.leveling_is_valid()) {
    bool new_level_state = bedlevel.flag.leveling_active;
    EDIT_ITEM(bool, MSG_BED_LEVELING, &new_level_state, []{ bedlevel.set_bed_leveling_enabled(!bedlevel.flag.leveling_active); });
  }

  // Z Fade Height
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    editable.decimal = bedlevel.z_fade_height;
    EDIT_ITEM_FAST(float3, MSG_Z_FADE_HEIGHT, &editable.decimal, 0, 100, []{ bedlevel.set_z_fade_height(editable.decimal); });
  #endif

  //
  // Mesh Bed Leveling Z-Offset
  //
  #if ENABLED(MESH_BED_LEVELING)
    EDIT_ITEM(float43, MSG_BED_Z, &mbl.data.z_offset, -1, 1);
  #endif

  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    SUBMENU(MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
  #elif HAS_BED_PROBE
    EDIT_ITEM(LCD_Z_OFFSET_TYPE, MSG_ZPROBE_ZOFFSET, &probe.data.offset.z, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
  #endif

  #if ENABLED(LEVEL_BED_CORNERS)
    SUBMENU(MSG_LEVEL_CORNERS, lcd_level_bed_corners);
  #endif

  #if ENABLED(EEPROM_SETTINGS)
    ACTION_ITEM(MSG_LOAD_EEPROM, []{ eeprom.load(); });
    ACTION_ITEM(MSG_STORE_EEPROM, []{ eeprom.store(); });
  #endif

  END_MENU();

}

#endif // HAS_LCD_MENU && LCD_BED_LEVELING
