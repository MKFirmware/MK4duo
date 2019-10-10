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
// Filament Change Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && ENABLED(ADVANCED_PAUSE_FEATURE)

//
// Change Filament > Change/Unload/Load Filament
//
static PauseModeEnum _change_filament_temp_mode;  // =PAUSE_MODE_PAUSE_PRINT
static int8_t _change_filament_temp_extruder;     // =0

static char cmd[11];

inline PGM_P _change_filament_temp_command() {
  switch (_change_filament_temp_mode) {
    case PAUSE_MODE_LOAD_FILAMENT:
      return PSTR("M701 T%d");
    case PAUSE_MODE_UNLOAD_FILAMENT:
      return _change_filament_temp_extruder >= 0 ? PSTR("M702 T%d") : PSTR("M702 ;%d");
    case PAUSE_MODE_CHANGE_FILAMENT:
    case PAUSE_MODE_PAUSE_PRINT:
    default:
      return PSTR("M600 B0 T%d");
  }
  return PSTR(MSG_FILAMENTCHANGE);
}

static void _change_filament_temp(const uint16_t temperature) {
  sprintf_P(cmd, _change_filament_temp_command(), _change_filament_temp_extruder);
  hotends[_change_filament_temp_extruder]->set_target_temp(temperature);
  lcd_enqueue_one_now(cmd);
}

static PGM_P change_filament_header(const PauseModeEnum mode) {
  switch (mode) {
    case PAUSE_MODE_LOAD_FILAMENT:
      return PSTR(MSG_FILAMENTLOAD);
    case PAUSE_MODE_UNLOAD_FILAMENT:
      return PSTR(MSG_FILAMENTUNLOAD);
    default: break;
  }
  return PSTR(MSG_FILAMENTCHANGE);
}

void _menu_temp_filament_op(const PauseModeEnum mode, const int8_t extruder) {
  _change_filament_temp_mode = mode;
  _change_filament_temp_extruder = extruder;
  START_MENU();
  if (LCD_HEIGHT >= 4) STATIC_ITEM_P(change_filament_header(mode), SS_CENTER|SS_INVERT);
  BACK_ITEM(MSG_BACK);
  ACTION_ITEM(MSG_PREHEAT_1, [](){ _change_filament_temp(lcdui.preheat_hotend_temp[0]); });
  ACTION_ITEM(MSG_PREHEAT_2, [](){ _change_filament_temp(lcdui.preheat_hotend_temp[1]); });
  ACTION_ITEM(MSG_PREHEAT_3, [](){ _change_filament_temp(lcdui.preheat_hotend_temp[2]); });
  EDIT_ITEM_FAST(int3, MSG_PREHEAT_CUSTOM, &hotends[_change_filament_temp_extruder]->target_temperature, hotends[extruder]->data.temp.min, hotends[extruder]->data.temp.max - 10, [](){
    _change_filament_temp(hotends[_change_filament_temp_extruder]->target_temperature);
  });
  END_MENU();
}

/**
 *
 * "Change Filament" submenu
 *
 */
#if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)

  void menu_change_filament() {

    START_MENU();
    BACK_ITEM(MSG_MAIN);

    editable.int8 = printer.isPaused() ? PAUSE_MODE_PAUSE_PRINT : PAUSE_MODE_CHANGE_FILAMENT;

    // Change filament
    LOOP_EXTRUDER() {
      PGM_P msg = PSTR(MSG_FILAMENTCHANGE);
      if (thermalManager.targetTooColdToExtrude(e))
        MENU_ITEM_P(submenu, msg, e, [](){ _menu_temp_filament_op(PauseModeEnum(editable.int8), menu_edit_index); });
      else
        MENU_ITEM_P(submenu, msg, e, [](){
          sprintf_P(cmd, PSTR("M600 B0 T%d"), menu_edit_index);
          lcd_enqueue_one_now(cmd);
        });
    }

    if (printer.isPaused()) {

      // Load filament
      LOOP_EXTRUDER() {
        PGM_P msg = PSTR(MSG_FILAMENTLOAD);
        if (thermalManager.targetTooColdToExtrude(e))
          MENU_ITEM_P(submenu, msg, e, [](){ _menu_temp_filament_op(PAUSE_MODE_LOAD_FILAMENT, menu_edit_index); });
        else
          MENU_ITEM_P(submenu, msg, e, [](){
            sprintf_P(cmd, PSTR("M701 T%d"), menu_edit_index);
            lcd_enqueue_one_now(cmd);
          });
      }

      // Unload filament
      LOOP_EXTRUDER() {
        PGM_P msg = PSTR(MSG_FILAMENTUNLOAD);
        if (thermalManager.targetTooColdToExtrude(e))
          MENU_ITEM_P(submenu, msg, e, [](){ _menu_temp_filament_op(PAUSE_MODE_UNLOAD_FILAMENT, menu_edit_index); });
        else
          MENU_ITEM_P(submenu, msg, e, [](){
            sprintf_P(cmd, PSTR("M702 T%d"), menu_edit_index);
            lcd_enqueue_one_now(cmd);
          });
      }

    }

    END_MENU();

  }

#endif // ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)

static uint8_t hotend_status_extruder = 0;

static PGM_P pause_header() {
  switch (advancedpause.mode) {
    case PAUSE_MODE_CHANGE_FILAMENT:
      return PSTR(MSG_FILAMENT_CHANGE_HEADER);
    case PAUSE_MODE_LOAD_FILAMENT:
      return PSTR(MSG_FILAMENT_CHANGE_HEADER_LOAD);
    case PAUSE_MODE_UNLOAD_FILAMENT:
      return PSTR(MSG_FILAMENT_CHANGE_HEADER_UNLOAD);
    case PAUSE_MODE_PAUSE_PRINT:
    default: break;
  }
  return PSTR(MSG_FILAMENT_CHANGE_HEADER_PAUSE);
}

#if HAS_NEXTION_LCD
  // Portions from STATIC_ITEM...
  #define HOTEND_STATUS_ITEM() do { \
    if (_menuLineNr == _thisItemNr) { \
      lcdui.draw_hotend_status(_lcdLineNr, hotend_status_extruder); \
      if (_skipStatic && encoderLine <= _thisItemNr) { \
        lcdui.encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
        ++encoderLine; \
      } \
    } \
    ++_thisItemNr; \
  }while(0)
#else  
  // Portions from STATIC_ITEM...
  #define HOTEND_STATUS_ITEM() do { \
    if (_menuLineNr == _thisItemNr) { \
      if (lcdui.should_draw()) { \
        draw_menu_item_static(_lcdLineNr, PSTR(MSG_FILAMENT_CHANGE_NOZZLE), NO_INDEX, SS_INVERT); \
        lcdui.draw_hotend_status(_lcdLineNr, hotend_status_extruder); \
      } \
      if (_skipStatic && encoderLine <= _thisItemNr) { \
        lcdui.encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
        ++encoderLine; \
      } \
      lcdui.refresh(LCDVIEW_CALL_REDRAW_NEXT); \
    } \
    ++_thisItemNr; \
  }while(0)
#endif

void menu_pause_option() {
  START_MENU();
  #if LCD_HEIGHT > 2
    STATIC_ITEM(MSG_FILAMENT_CHANGE_OPTION_HEADER);
  #endif
  ACTION_ITEM(MSG_FILAMENT_CHANGE_OPTION_PURGE, [](){ advancedpause.menu_response = PAUSE_RESPONSE_EXTRUDE_MORE; });
  #if HAS_FILAMENT_SENSOR
    editable.state = filamentrunout.sensor.isEnabled();
    if (filamentrunout.sensor.isFilamentOut())
      EDIT_ITEM(bool, MSG_RUNOUT_SENSOR, &editable.state, [](){
        filamentrunout.sensor.setEnabled(editable.state);
        filamentrunout.reset();
      });
    else
  #endif
    ACTION_ITEM(MSG_FILAMENT_CHANGE_OPTION_RESUME, [](){ advancedpause.menu_response = PAUSE_RESPONSE_RESUME_PRINT; });
  END_MENU();
}

//
// ADVANCED_PAUSE_FEATURE message screens
//

void _lcd_pause_message(PGM_P const msg1, PGM_P const msg2=nullptr, PGM_P const msg3=nullptr) {
  START_SCREEN();
  STATIC_ITEM_P(pause_header(), NO_INDEX, SS_CENTER|SS_INVERT);
  STATIC_ITEM_P(msg1);
  if (msg2) STATIC_ITEM_P(msg2);
  if (msg3 && (LCD_HEIGHT) >= 5) STATIC_ITEM_P(msg3);
  if ((!!msg2) + (!!msg3) + 2 < (LCD_HEIGHT) - 1) STATIC_ITEM(" ");
  HOTEND_STATUS_ITEM();
  END_SCREEN();
}

void lcd_pause_pausing_message() {
  _lcd_pause_message(PSTR(MSG_PAUSE_PRINT_INIT_1)
    #ifdef MSG_PAUSE_PRINT_INIT_2
      , PSTR(MSG_PAUSE_PRINT_INIT_2)
      #ifdef MSG_PAUSE_PRINT_INIT_3
        , PSTR(MSG_PAUSE_PRINT_INIT_3)
      #endif
    #endif
  );
}

void lcd_pause_changing_message() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_INIT_1)
    #ifdef MSG_FILAMENT_CHANGE_INIT_2
      , PSTR(MSG_FILAMENT_CHANGE_INIT_2)
      #ifdef MSG_FILAMENT_CHANGE_INIT_3
        , PSTR(MSG_FILAMENT_CHANGE_INIT_3)
      #endif
    #endif
  );
}

void lcd_pause_unload_message() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_UNLOAD_1)
    #ifdef MSG_FILAMENT_CHANGE_UNLOAD_2
      , PSTR(MSG_FILAMENT_CHANGE_UNLOAD_2)
      #ifdef MSG_FILAMENT_CHANGE_UNLOAD_3
        , PSTR(MSG_FILAMENT_CHANGE_UNLOAD_3)
      #endif
    #endif
  );
}

void lcd_pause_heating_message() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_HEATING_1)
    #ifdef MSG_FILAMENT_CHANGE_HEATING_2
      , PSTR(MSG_FILAMENT_CHANGE_HEATING_2)
      #ifdef MSG_FILAMENT_CHANGE_HEATING_3
        , PSTR(MSG_FILAMENT_CHANGE_HEATING_3)
      #endif
    #endif
  );
}

void lcd_pause_heat_message() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_HEAT_1)
    #ifdef MSG_FILAMENT_CHANGE_HEAT_2
      , PSTR(MSG_FILAMENT_CHANGE_HEAT_2)
      #ifdef MSG_FILAMENT_CHANGE_HEAT_3
        , PSTR(MSG_FILAMENT_CHANGE_HEAT_3)
      #endif
    #endif
  );
}

void lcd_pause_insert_message() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_INSERT_1)
    #ifdef MSG_FILAMENT_CHANGE_INSERT_2
      , PSTR(MSG_FILAMENT_CHANGE_INSERT_2)
      #ifdef MSG_FILAMENT_CHANGE_INSERT_3
        , PSTR(MSG_FILAMENT_CHANGE_INSERT_3)
      #endif
    #endif
  );
}

void lcd_pause_printer_off() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_ZZZ_1)
    #ifdef MSG_FILAMENT_CHANGE_ZZZ_2
      , PSTR(MSG_FILAMENT_CHANGE_ZZZ_2)
      #ifdef MSG_FILAMENT_CHANGE_ZZZ_3
        , PSTR(MSG_FILAMENT_CHANGE_ZZZ_3)
      #endif
    #endif
  );
}

void lcd_pause_load_message() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_LOAD_1)
    #ifdef MSG_FILAMENT_CHANGE_LOAD_2
      , PSTR(MSG_FILAMENT_CHANGE_LOAD_2)
      #ifdef MSG_FILAMENT_CHANGE_LOAD_3
        , PSTR(MSG_FILAMENT_CHANGE_LOAD_3)
      #endif
    #endif
  );
}

void lcd_pause_waiting_message() {
  _lcd_pause_message(PSTR(MSG_ADVANCED_PAUSE_WAITING_1)
    #ifdef MSG_ADVANCED_PAUSE_WAITING_2
      , PSTR(MSG_ADVANCED_PAUSE_WAITING_2)
      #ifdef MSG_ADVANCED_PAUSE_WAITING_3
        , PSTR(MSG_ADVANCED_PAUSE_WAITING_3)
      #endif
    #endif
  );
}

void lcd_pause_resume_message() {
  _lcd_pause_message(PSTR(MSG_FILAMENT_CHANGE_RESUME_1)
    #ifdef MSG_FILAMENT_CHANGE_RESUME_2
      , PSTR(MSG_FILAMENT_CHANGE_RESUME_2)
      #ifdef MSG_FILAMENT_CHANGE_RESUME_3
        , PSTR(MSG_FILAMENT_CHANGE_RESUME_3)
      #endif
    #endif
  );
}

void lcd_pause_purge_message() {
  _lcd_pause_message(
    #if ENABLED(ADVANCED_PAUSE_CONTINUOUS_PURGE)
      PSTR(MSG_FILAMENT_CHANGE_CONT_PURGE_1)
      #ifdef MSG_FILAMENT_CHANGE_CONT_PURGE_2
        , PSTR(MSG_FILAMENT_CHANGE_CONT_PURGE_2)
        #ifdef MSG_FILAMENT_CHANGE_CONT_PURGE_3
          , PSTR(MSG_FILAMENT_CHANGE_CONT_PURGE_3)
        #endif
      #endif
    #else
      PSTR(MSG_FILAMENT_CHANGE_PURGE_1)
      #ifdef MSG_FILAMENT_CHANGE_PURGE_2
        , PSTR(MSG_FILAMENT_CHANGE_PURGE_2)
        #ifdef MSG_FILAMENT_CHANGE_PURGE_3
          , PSTR(MSG_FILAMENT_CHANGE_PURGE_3)
        #endif
      #endif
    #endif
  );
}

FORCE_INLINE screenFunc_t ap_message_screen(const PauseMessageEnum message) {
  switch (message) {
    case PAUSE_MESSAGE_PAUSING:     return lcd_pause_pausing_message;
    case PAUSE_MESSAGE_CHANGING:    return lcd_pause_changing_message;
    case PAUSE_MESSAGE_UNLOAD:      return lcd_pause_unload_message;
    case PAUSE_MESSAGE_WAITING:     return lcd_pause_waiting_message;
    case PAUSE_MESSAGE_INSERT:      return lcd_pause_insert_message;
    case PAUSE_MESSAGE_LOAD:        return lcd_pause_load_message;
    case PAUSE_MESSAGE_PURGE:       return lcd_pause_purge_message;
    case PAUSE_MESSAGE_RESUME:      return lcd_pause_resume_message;
    case PAUSE_MESSAGE_HEAT:        return lcd_pause_heat_message;
    case PAUSE_MESSAGE_PRINTER_OFF: return lcd_pause_printer_off;
    case PAUSE_MESSAGE_HEATING:     return lcd_pause_heating_message;
    case PAUSE_MESSAGE_OPTION:      advancedpause.menu_response = PAUSE_RESPONSE_WAIT_FOR;
                                    return menu_pause_option;
    case PAUSE_MESSAGE_STATUS:
    default: break;
  }
  return nullptr;
}

void lcd_pause_show_message(
  const PauseMessageEnum message,
  const PauseModeEnum mode/*=PAUSE_MODE_SAME*/,
  const uint8_t hotend/*=tools.target_hotend()*/
) {
  if (mode != PAUSE_MODE_SAME) advancedpause.mode = mode;
  hotend_status_extruder = hotend;
  const screenFunc_t next_screen = ap_message_screen(message);
  if (next_screen) {
    lcdui.defer_status_screen();
    lcdui.goto_screen(next_screen);
  }
  else
    lcdui.return_to_status();
}

#endif // HAS_LCD_MENU && ADVANCED_PAUSE_FEATURE
