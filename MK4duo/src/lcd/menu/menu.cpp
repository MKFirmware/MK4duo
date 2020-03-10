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

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

////////////////////////////////////////////
///////////// Global Variables /////////////
////////////////////////////////////////////

// Manual Movement
float move_menu_scale;

// Menu Navigation
int8_t encoderTopLine, encoderLine, screen_items;

typedef struct {
  screenFunc_t menu_function;
  uint32_t  encoder_position;
  int8_t    top_line, items;
} menuPosition;
menuPosition screen_history[6];
uint8_t screen_history_depth = 0;

uint8_t MenuItemBase::itemIndex;  // Index number for draw and action
editable_t editable;              // Value Editing

// Menu Edit Items
PGM_P         MenuEditItemBase::editLabel;
void*         MenuEditItemBase::editValue;
int32_t       MenuEditItemBase::minEditValue,
              MenuEditItemBase::maxEditValue;
screenFunc_t  MenuEditItemBase::callbackFunc;
bool          MenuEditItemBase::liveEdit;

// Prevent recursion into screen handlers
bool no_reentry = false;

////////////////////////////////////////////
//////// Menu Navigation & History /////////
////////////////////////////////////////////

void LcdUI::return_to_status() { goto_screen(status_screen); }

void LcdUI::save_previous_screen() {
  if (screen_history_depth < COUNT(screen_history))
    screen_history[screen_history_depth++] = { currentScreen, encoderPosition, encoderTopLine, screen_items };
}

void LcdUI::goto_previous_screen() {
  if (screen_history_depth > 0) {
    menuPosition &sh = screen_history[--screen_history_depth];
    goto_screen(sh.menu_function, sh.encoder_position, sh.top_line, sh.items);
  }
  else
    return_to_status();
}

////////////////////////////////////////////
/////////// Common Menu Actions ////////////
////////////////////////////////////////////

void MenuItem_gcode::action(PGM_P const, PGM_P const pgcode) { commands.inject_P(pgcode); }

////////////////////////////////////////////
/////////// Menu Editing Actions ///////////
////////////////////////////////////////////

/**
 * Functions for editing single values
 *
 * The "DEFINE_MENU_EDIT_ITEM" macro generates the classes needed to edit a numerical value.
 *
 * The prerequisite is that in the header the type was already declared:
 *
 *   DEFINE_MENU_EDIT_ITEM_TYPE(int16_t, int3, i16tostr3, 1)
 *
 * For example, DEFINE_MENU_EDIT_ITEM(int3) expands into:
 *
 *   template class TMenuEditItem<MenuEditItemInfo_int3>
 *
 * You can then use one of the menu macros to present the edit interface:
 *   EDIT_ITEM(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
 *
 * This expands into a more primitive menu item:
 *  _MENU_ITEM_P(int3, false, GET_TEXT(MSG_SPEED), &feedrate_percentage, 10, 999)
 *
 * ...which calls:
 *       MenuItem_int3::action(plabel, &feedrate_percentage, 10, 999)
 *       MenuItem_int3::draw(encoderLine == _thisItemNr, _lcdLineNr, plabel, &feedrate_percentage, 10, 999)
 */
void MenuEditItemBase::edit_screen(strfunc_t strfunc, loadfunc_t loadfunc) {
  if (int32_t(lcdui.encoderPosition) < 0) lcdui.encoderPosition = 0;
  if (int32_t(lcdui.encoderPosition) > maxEditValue) lcdui.encoderPosition = maxEditValue;
  if (lcdui.should_draw())
    draw_edit_screen(editLabel, strfunc(lcdui.encoderPosition + minEditValue));
  if (lcdui.lcd_clicked || (liveEdit && lcdui.should_draw())) {
    if (editValue != nullptr) loadfunc(editValue, lcdui.encoderPosition + minEditValue);
    if (callbackFunc && (liveEdit || lcdui.lcd_clicked)) (*callbackFunc)();
    if (lcdui.use_click()) lcdui.goto_previous_screen();
  }
}

void MenuEditItemBase::goto_edit_screen(PGM_P const el, void * const ev, const int32_t minv, const int32_t maxv, const uint16_t ep, const screenFunc_t cs, const screenFunc_t cb, const bool le) {
  lcdui.screen_changed = true;
  lcdui.save_previous_screen();
  lcdui.refresh();
  editLabel = el;
  editValue = ev;
  minEditValue = minv;
  maxEditValue = maxv;
  lcdui.encoderPosition = ep;
  lcdui.currentScreen = cs;
  callbackFunc = cb;
  liveEdit = le;
}

void MenuItem_bool::action(PGM_P const, bool * const ptr, screenFunc_t callback) {
  *ptr ^= true; lcdui.refresh();
  if (callback) (*callback)();
}

////////////////////////////////////////////
///////////////// Menu Tree ////////////////
////////////////////////////////////////////

/**
 * General function to go directly to a screen
 */
void LcdUI::goto_screen(screenFunc_t screen, const uint16_t encoder/*=0*/, const uint8_t top/*=0*/, const uint8_t items/*=0*/) {
  if (currentScreen != screen) {

    #if ENABLED(DOUBLECLICK_FOR_Z_BABYSTEPPING) && ENABLED(BABYSTEPPING)
      static short_timer_t doubleclick_expire_timer(millis());
      // Going to menu_main from status screen? Remember first click time.
      // Going back to status screen within a very short time? Go to Z babystepping.
      if (screen == menu_main) {
        if (on_status_screen())
          doubleclick_expire_timer.start();
      }
      else if (screen == status_screen && currentScreen == menu_main && doubleclick_expire_timer.pending(DOUBLECLICK_MAX_INTERVAL)) {
        screen =
          #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
            lcd_babystep_zoffset
          #else
            lcd_babystep_z
          #endif
        ;
      }
    #endif

    currentScreen = screen;
    encoderPosition = encoder;
    encoderTopLine = top;
    screen_items = items;
    if (screen == status_screen) {
      defer_status_screen(false);
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        ubl.lcd_map_control = false;
      #endif
      screen_history_depth = 0;
    }

    clear_lcd();

    // Re-initialize custom characters that may be re-used
    #if HAS_CHARACTER_LCD
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        if (!ubl.lcd_map_control)
      #endif
          set_custom_characters(screen == status_screen ? CHARSET_INFO : CHARSET_MENU);
    #endif

    refresh(LCDVIEW_CALL_REDRAW_NEXT);
    screen_changed = true;
    #if HAS_GRAPHICAL_LCD
      drawing_screen = false;
    #endif

    #if HAS_LCD_MENU
      encoder_direction_normal();
    #endif

    set_selection(false);
  }
}

////////////////////////////////////////////
///////////// Manual Movement //////////////
////////////////////////////////////////////

//
// Display the synchronize screen until moves are
// finished, and don't return to the caller until
// done. ** This blocks the command queue! **
//
static PGM_P sync_message;

void LcdUI::_synchronize() {
  if (should_draw()) MenuItem_static::draw(LCD_HEIGHT >= 4, sync_message);
  if (no_reentry) return;
  // Make this the current handler till all moves are done
  const screenFunc_t old_screen = currentScreen;
  goto_screen(_synchronize);
  no_reentry = true;
  planner.synchronize(); // idle() is called until moves complete
  no_reentry = false;
  goto_screen(old_screen);
}

// Display the synchronize screen with a custom message
// ** This blocks the command queue! **
void LcdUI::synchronize(PGM_P const msg/*=nullptr*/) {
  sync_message = msg ?: GET_TEXT(MSG_MOVING);
  _synchronize();
}

/**
 * Scrolling for menus and other line-based screens
 *
 *   encoderLine is the position based on the encoder
 *   encoderTopLine is the top menu line to display
 *   _lcdLineNr is the index of the LCD line (e.g., 0-3)
 *   _menuLineNr is the menu item to draw and process
 *   _thisItemNr is the index of each MENU_ITEM or STATIC_ITEM
 *   screen_items is the total number of items in the menu (after one call)
 */
void scroll_screen(const uint8_t limit, const bool is_menu) {
  lcdui.encoder_direction_menus();
  ENCODER_RATE_MULTIPLY(false);
  if (int32_t(lcdui.encoderPosition) < 0) lcdui.encoderPosition = 0;
  if (lcdui.first_page) {
    encoderLine = lcdui.encoderPosition / (ENCODER_STEPS_PER_MENU_ITEM);
    lcdui.screen_changed = false;
  }
  if (screen_items > 0 && encoderLine >= screen_items - limit) {
    encoderLine = MAX(0, screen_items - limit);
    lcdui.encoderPosition = encoderLine * (ENCODER_STEPS_PER_MENU_ITEM);
  }
  if (is_menu) {
    NOMORE(encoderTopLine, encoderLine);
    if (encoderLine >= encoderTopLine + LCD_HEIGHT)
      encoderTopLine = encoderLine - LCD_HEIGHT + 1;
  }
  else
    encoderTopLine = encoderLine;
}

void lcd_line_to_z(const float &z) {
  mechanics.position.z = z;
  mechanics.line_to_position(manual_feedrate_mm_s.z);
}

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)

  void lcd_babystep_zoffset() {
    if (lcdui.use_click()) return lcdui.goto_previous_screen_no_defer();
    lcdui.defer_status_screen();
    #if ENABLED(BABYSTEP_HOTEND_Z_OFFSET)
      const bool do_probe = (toolManager.extruder.active == 0);
    #else
      constexpr bool do_probe = true;
    #endif
    if (lcdui.encoderPosition) {
      const int16_t babystep_increment = (int16_t)lcdui.encoderPosition * (
        #if ENABLED(BABYSTEP_XY)
          axis != Z_AXIS ? BABYSTEP_MULTIPLICATOR_XY :
        #endif
        BABYSTEP_MULTIPLICATOR_Z
      );
      lcdui.encoderPosition = 0;

      const float diff = mechanics.steps_to_mm.z * babystep_increment,
                  new_probe_offset = probe.data.offset.z + diff,
                  new_offs =
                    #if ENABLED(BABYSTEP_HOTEND_Z_OFFSET)
                      do_probe ? new_probe_offset : nozzle.data.hotend_offset[toolManager.active_hotend()].z - diff
                    #else
                      new_probe_offset
                    #endif
                  ;
      if (WITHIN(new_offs, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX)) {

        babystep.add_steps(Z_AXIS, babystep_increment);

        if (do_probe) probe.data.offset.z = new_offs;
        #if ENABLED(BABYSTEP_HOTEND_Z_OFFSET)
          else nozzle.data.hotend_offset[toolManager.active_hotend()].z = new_offs;
        #endif

        lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
      }
    }
    if (lcdui.should_draw()) {
      #if ENABLED(BABYSTEP_HOTEND_Z_OFFSET)
        if (!do_probe)
          MenuEditItemBase::draw_edit_screen(GET_TEXT(MSG_DXC_Z_OFFSET), LCD_Z_OFFSET_FUNC(nozzle.data.hotend_offset[toolManager.active_hotend()].z));
        else
      #endif
          MenuEditItemBase::draw_edit_screen(GET_TEXT(MSG_ZPROBE_ZOFFSET), LCD_Z_OFFSET_FUNC(probe.data.offset.z));

      #if ENABLED(BABYSTEP_ZPROBE_GFX_OVERLAY)
        if (do_probe) _lcd_zoffset_overlay_gfx(probe.data.offset.z);
      #endif
    }
  }

#endif // BABYSTEP_ZPROBE_OFFSET

/**
 * Watch temperature callbacks
 */
#if HAS_HOTENDS
  void watch_temp_callback_hotend() {
    hotends[MenuItemBase::itemIndex]->set_target_temp(hotends[MenuItemBase::itemIndex]->deg_target());
    hotends[MenuItemBase::itemIndex]->start_watching();
  }
#endif
#if HAS_BEDS
  void watch_temp_callback_bed() {
    beds[MenuItemBase::itemIndex]->set_target_temp(beds[MenuItemBase::itemIndex]->deg_target());
    beds[MenuItemBase::itemIndex]->start_watching();
  }
#endif
#if HAS_CHAMBERS
  void watch_temp_callback_chamber() {
    chambers[MenuItemBase::itemIndex]->set_target_temp(chambers[MenuItemBase::itemIndex]->deg_target());
    chambers[MenuItemBase::itemIndex]->start_watching();
  }
#endif
#if HAS_COOLERS
  void watch_temp_callback_cooler() {
    coolers[0]->set_target_temp(coolers[0]->deg_target());
    coolers[0]->start_watching();
  }
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(PID_AUTOTUNE_MENU) || ENABLED(ADVANCED_PAUSE_FEATURE)

  void lcd_enqueue_one_now(const char * const cmd) {
    no_reentry = true;
    commands.enqueue_one_now(cmd);
    no_reentry = false;
  }

  void lcd_enqueue_one_now_P(PGM_P const cmd) {
    no_reentry = true;
    commands.enqueue_now_P(cmd);
    no_reentry = false;
  }

#endif

void lcd_draw_homing() {
  constexpr uint8_t line = (LCD_HEIGHT - 1) / 2;
  if (lcdui.should_draw()) MenuItem_static::draw(line, GET_TEXT(MSG_LEVEL_BED_HOMING));
}

//
// Selection screen presents a prompt and two options
//
bool LcdUI::selection; // = false
bool LcdUI::update_selection() {
  encoder_direction_select();
  if (encoderPosition) {
    selection = int16_t(encoderPosition) > 0;
    encoderPosition = 0;
  }
  return selection;
}

void MenuItem_confirm::select_screen(PGM_P const yes, PGM_P const no, selectFunc_t yesFunc, selectFunc_t noFunc, PGM_P const pref, const char * const string/*=nullptr*/, PGM_P const suff/*=nullptr*/) {
  const bool ui_selection = lcdui.update_selection(), got_click = lcdui.use_click();
  if (got_click || lcdui.should_draw()) {
    draw_select_screen(yes, no, ui_selection, pref, string, suff);
    if (got_click) { ui_selection ? yesFunc() : noFunc(); }
  }
}

#endif // HAS_LCD_MENU
