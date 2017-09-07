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

#include "../../base.h"

#if ENABLED(ULTRA_LCD)

int16_t lcd_preheat_hotend_temp[3], lcd_preheat_bed_temp[3], lcd_preheat_fan_speed[3];

#if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
  millis_t previous_lcd_status_ms = 0;
#endif

#if ENABLED(BABYSTEPPING)
  long babysteps_done = 0;
  static void lcd_babystep_z();
#endif

#if HAS_LCD_POWER_SENSOR
  millis_t print_millis = 0;
#endif

uint8_t lcd_status_message_level;
char lcd_status_message[3 * (LCD_WIDTH) + 1] = WELCOME_MSG; // worst case is kana with up to 3*LCD_WIDTH+1

#if ENABLED(PROBE_MANUALLY) && ENABLED(LCD_BED_LEVELING)
  bool lcd_wait_for_move;
#endif

#if ENABLED(STATUS_MESSAGE_SCROLLING)
  uint8_t status_scroll_pos = 0;
#endif

#if ENABLED(DOGLCD)
  #include "ultralcd_impl_DOGM.h"
#else
  #include "ultralcd_impl_HD44780.h"
#endif

#if ENABLED(LASER)
  void lcd_laser_focus_menu();
  void lcd_laser_menu();
  void lcd_laser_test_fire_menu();
  void laser_test_fire(uint8_t power, int dwell);
  void laser_set_focus(float f_length);
  void action_laser_focus_custom();
  void action_laser_focus_1mm();
  void action_laser_focus_2mm();
  void action_laser_focus_3mm();
  void action_laser_focus_4mm();
  void action_laser_focus_5mm();
  void action_laser_focus_6mm();
  void action_laser_focus_7mm();
  void action_laser_test_weak();
  void action_laser_test_20_50ms();
  void action_laser_test_20_100ms();
  void action_laser_test_100_50ms();
  void action_laser_test_100_100ms();
  void action_laser_test_warm();
  void action_laser_acc_on();
  void action_laser_acc_off();
#endif

// The main status screen
void lcd_status_screen();

millis_t next_lcd_update_ms;

uint8_t lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; // Set when the LCD needs to draw, decrements after every draw. Set to 2 in LCD routines so the LCD gets at least 1 full redraw (first redraw is partial)
uint16_t max_display_update_time = 0;

#if ENABLED(DOGLCD)
  bool drawing_screen = false;
#endif

#if ENABLED(ULTIPANEL)

  #ifndef TALL_FONT_CORRECTION
    #define TALL_FONT_CORRECTION 0
  #endif

  // Function pointer to menu functions.
  typedef void (*screenFunc_t)();

  ////////////////////////////////////////////
  ///////////////// Menu Tree ////////////////
  ////////////////////////////////////////////

  void lcd_main_menu();
  void lcd_tune_menu();
  void lcd_prepare_menu();
  void lcd_move_menu();
  void lcd_control_menu();
  void lcd_control_temperature_menu();
  void lcd_control_temperature_preheat_material1_settings_menu();
  void lcd_control_temperature_preheat_material2_settings_menu();
  void lcd_control_temperature_preheat_material3_settings_menu();
  void lcd_control_motion_menu();
  void lcd_control_filament_menu();

  #if ENABLED(LCD_INFO_MENU)
    void lcd_info_stats_menu();
    void lcd_info_firmware_menu();
    void lcd_info_thermistors_menu();
    void lcd_info_board_menu();
    void lcd_info_menu();
  #endif // LCD_INFO_MENU

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void lcd_advanced_pause_toocold_menu();
    void lcd_advanced_pause_option_menu();
    void lcd_advanced_pause_init_message();
    void lcd_advanced_pause_cool_message();
    void lcd_advanced_pause_unload_message();
    void lcd_advanced_pause_insert_message();
    void lcd_advanced_pause_load_message();
    void lcd_advanced_pause_heat_nozzle();
    void lcd_advanced_pause_printer_off();
    void lcd_advanced_pause_extrude_message();
    void lcd_advanced_pause_resume_message();
  #endif

  #if ENABLED(FWRETRACT)
    void lcd_control_retract_menu();
  #endif

  #if MECH(DELTA)
    void lcd_delta_calibrate_menu();
  #endif

  ////////////////////////////////////////////
  //////////// Menu System Actions ///////////
  ////////////////////////////////////////////

  #define menu_action_back(dummy) _menu_action_back()
  void _menu_action_back();
  void menu_action_submenu(screenFunc_t data);
  void menu_action_gcode(const char* pgcode);
  void menu_action_function(screenFunc_t data);

  #define DECLARE_MENU_EDIT_TYPE(_type, _name) \
    bool _menu_edit_ ## _name(); \
    void menu_edit_ ## _name(); \
    void menu_edit_callback_ ## _name(); \
    void _menu_action_setting_edit_ ## _name(const char * const pstr, _type* const ptr, const _type minValue, const _type maxValue); \
    void menu_action_setting_edit_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue); \
    void menu_action_setting_edit_callback_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue, const screenFunc_t callback, const bool live=false); \
    typedef void _name##_void

  DECLARE_MENU_EDIT_TYPE(int16_t, int3);
  DECLARE_MENU_EDIT_TYPE(uint16_t, uint3);
  DECLARE_MENU_EDIT_TYPE(uint8_t, int8);
  DECLARE_MENU_EDIT_TYPE(float, float3);
  DECLARE_MENU_EDIT_TYPE(float, float32);
  DECLARE_MENU_EDIT_TYPE(float, float43);
  DECLARE_MENU_EDIT_TYPE(float, float5);
  DECLARE_MENU_EDIT_TYPE(float, float51);
  DECLARE_MENU_EDIT_TYPE(float, float52);
  DECLARE_MENU_EDIT_TYPE(float, float62);
  DECLARE_MENU_EDIT_TYPE(uint32_t, long5);

  void menu_action_setting_edit_bool(const char* pstr, bool* ptr);
  void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, screenFunc_t callbackFunc);

  #if HAS_SDSUPPORT
    void lcd_sdcard_menu();
    void menu_action_sdfile(const char* longFilename);
    void menu_action_sddirectory(const char* longFilename);
  #endif

  ////////////////////////////////////////////
  //////////// Menu System Macros ////////////
  ////////////////////////////////////////////

  #ifndef ENCODER_FEEDRATE_DEADZONE
    #define ENCODER_FEEDRATE_DEADZONE 10
  #endif
  #ifndef ENCODER_STEPS_PER_MENU_ITEM
    #define ENCODER_STEPS_PER_MENU_ITEM 5
  #endif
  #ifndef ENCODER_PULSES_PER_STEP
    #define ENCODER_PULSES_PER_STEP 1
  #endif

  /**
   * MENU_ITEM generates draw & handler code for a menu item, potentially calling:
   *
   *   lcd_implementation_drawmenu_[type](sel, row, label, arg3...)
   *   menu_action_[type](arg3...)
   *
   * Examples:
   *   MENU_ITEM(back, MSG_WATCH, 0 [dummy parameter] )
   *   or
   *   MENU_BACK(MSG_WATCH)
   *     lcd_implementation_drawmenu_back(sel, row, PSTR(MSG_WATCH))
   *     menu_action_back()
   *
   *   MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause)
   *     lcd_implementation_drawmenu_function(sel, row, PSTR(MSG_PAUSE_PRINT), lcd_sdcard_pause)
   *     menu_action_function(lcd_sdcard_pause)
   *
   *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
   *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *     lcd_implementation_drawmenu_setting_edit_int3(sel, row, PSTR(MSG_SPEED), PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *     menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *
   */
  #define _MENU_ITEM_PART_1(TYPE, ...) \
    if (_menuLineNr == _thisItemNr) { \
      if (lcd_clicked && encoderLine == _thisItemNr) {

  #define _MENU_ITEM_PART_2(TYPE, LABEL, ...) \
        menu_action_ ## TYPE(__VA_ARGS__); \
        if (screen_changed) return; \
      } \
      if (lcdDrawUpdate) \
        lcd_implementation_drawmenu_ ## TYPE(encoderLine == _thisItemNr, _lcdLineNr, PSTR(LABEL), ## __VA_ARGS__); \
    } \
    ++_thisItemNr

  #define MENU_ITEM(TYPE, LABEL, ...) do { \
      _skipStatic = false; \
      _MENU_ITEM_PART_1(TYPE, ## __VA_ARGS__); \
      _MENU_ITEM_PART_2(TYPE, LABEL, ## __VA_ARGS__); \
    } while(0)

  #define MENU_BACK(LABEL) MENU_ITEM(back, LABEL, 0)

  // Used to print static text with no visible cursor.
  // Parameters: label [, bool center [, bool invert [, char *value] ] ]
  #define STATIC_ITEM(LABEL, ...) \
    if (_menuLineNr == _thisItemNr) { \
      if (_skipStatic && encoderLine <= _thisItemNr) { \
        encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
        ++encoderLine; \
      } \
      if (lcdDrawUpdate) \
        lcd_implementation_drawmenu_static(_lcdLineNr, PSTR(LABEL), ## __VA_ARGS__); \
    } \
    ++_thisItemNr

  #if ENABLED(ENCODER_RATE_MULTIPLIER)

    bool encoderRateMultiplierEnabled;
    #define ENCODER_RATE_MULTIPLY(F) (encoderRateMultiplierEnabled = F)

    //#define ENCODER_RATE_MULTIPLIER_DEBUG  // If defined, output the encoder steps per second value

    /**
     * MENU_MULTIPLIER_ITEM generates drawing and handling code for a multiplier menu item
     */
    #define MENU_MULTIPLIER_ITEM(type, label, ...) do { \
        _MENU_ITEM_PART_1(type, ## __VA_ARGS__); \
        encoderRateMultiplierEnabled = true; \
        lastEncoderMovementMillis = 0; \
        _MENU_ITEM_PART_2(type, label, ## __VA_ARGS__); \
      } while(0)

  #else // !ENCODER_RATE_MULTIPLIER
    #define ENCODER_RATE_MULTIPLY(F) NOOP
  #endif // !ENCODER_RATE_MULTIPLIER

  #define MENU_ITEM_DUMMY() do { _thisItemNr++; } while(0)
  #define MENU_ITEM_EDIT(type, label, ...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #define MENU_ITEM_EDIT_CALLBACK(type, label, ...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, ...) MENU_MULTIPLIER_ITEM(setting_edit_ ## type, label, PSTR(label), ## __VA_ARGS__)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, ...) MENU_MULTIPLIER_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #else // !ENCODER_RATE_MULTIPLIER
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, ...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## __VA_ARGS__)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, ...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## __VA_ARGS__)
  #endif // !ENCODER_RATE_MULTIPLIER

  /**
   * START_SCREEN_OR_MENU generates init code for a screen or menu
   *
   *   encoderLine is the position based on the encoder
   *   encoderTopLine is the top menu line to display
   *   _lcdLineNr is the index of the LCD line (e.g., 0-3)
   *   _menuLineNr is the menu item to draw and process
   *   _thisItemNr is the index of each MENU_ITEM or STATIC_ITEM
   *   _countedItems is the total number of items in the menu (after one call)
   */
  #define START_SCREEN_OR_MENU(LIMIT) \
    ENCODER_DIRECTION_MENUS(); \
    ENCODER_RATE_MULTIPLY(false); \
    if (encoderPosition > 0x8000) encoderPosition = 0; \
    static int8_t _countedItems = 0; \
    int8_t encoderLine = encoderPosition / (ENCODER_STEPS_PER_MENU_ITEM); \
    if (_countedItems > 0 && encoderLine >= _countedItems - (LIMIT)) { \
      encoderLine = max(0, _countedItems - (LIMIT)); \
      encoderPosition = encoderLine * (ENCODER_STEPS_PER_MENU_ITEM); \
    }

  #define SCREEN_OR_MENU_LOOP() \
    int8_t _menuLineNr = encoderTopLine, _thisItemNr; \
    for (int8_t _lcdLineNr = 0; _lcdLineNr < LCD_HEIGHT - (TALL_FONT_CORRECTION); _lcdLineNr++, _menuLineNr++) { \
      _thisItemNr = 0

  /**
   * START_SCREEN  Opening code for a screen having only static items.
   *               Do simplified scrolling of the entire screen.
   *
   * START_MENU    Opening code for a screen with menu items.
   *               Scroll as-needed to keep the selected line in view.
   */
  #define START_SCREEN() \
    START_SCREEN_OR_MENU(LCD_HEIGHT - (TALL_FONT_CORRECTION)); \
    encoderTopLine = encoderLine; \
    bool _skipStatic = false; \
    SCREEN_OR_MENU_LOOP()

  #define START_MENU() \
    START_SCREEN_OR_MENU(1); \
    screen_changed = false; \
    NOMORE(encoderTopLine, encoderLine); \
    if (encoderLine >= encoderTopLine + LCD_HEIGHT - (TALL_FONT_CORRECTION)) { \
      encoderTopLine = encoderLine - (LCD_HEIGHT - (TALL_FONT_CORRECTION) - 1); \
    } \
    bool _skipStatic = true; \
    SCREEN_OR_MENU_LOOP()

  #define END_SCREEN() \
    } \
    _countedItems = _thisItemNr

  #define END_MENU() \
    } \
    _countedItems = _thisItemNr; \
    UNUSED(_skipStatic)

  ////////////////////////////////////////////
  ///////////// Global Variables /////////////
  ////////////////////////////////////////////

  /**
   * REVERSE_MENU_DIRECTION
   *
   * To reverse the menu direction we need a general way to reverse
   * the direction of the encoder everywhere. So encoderDirection is
   * added to allow the encoder to go the other way.
   *
   * This behavior is limited to scrolling Menus and SD card listings,
   * and is disabled in other contexts.
   */
  #if ENABLED(REVERSE_MENU_DIRECTION)
    int8_t encoderDirection = 1;
    #define ENCODER_DIRECTION_NORMAL() (encoderDirection = 1)
    #define ENCODER_DIRECTION_MENUS() (encoderDirection = -1)
  #else
    #define ENCODER_DIRECTION_NORMAL() ;
    #define ENCODER_DIRECTION_MENUS() ;
  #endif

  // Encoder Movement
  volatile int8_t encoderDiff; // Updated in lcd_buttons_update, added to encoderPosition every LCD update
  uint32_t encoderPosition;
  millis_t lastEncoderMovementMillis = 0;

  // Button States
  bool lcd_clicked, wait_for_unclick;
  volatile uint8_t buttons;
  millis_t next_button_update_ms;
  #if ENABLED(REPRAPWORLD_KEYPAD) || ENABLED(ADC_KEYPAD)
    volatile uint8_t buttons_reprapworld_keypad;
  #endif
  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    volatile uint8_t slow_buttons;
  #endif

  // Menu System Navigation
  screenFunc_t currentScreen = lcd_status_screen;
  int8_t encoderTopLine;
  typedef struct {
    screenFunc_t menu_function;
    uint32_t encoder_position;
  } menuPosition;
  menuPosition screen_history[6];
  uint8_t screen_history_depth = 0;
  bool screen_changed, defer_return_to_status;

  // Value Editing
  const char *editLabel;
  void *editValue;
  int32_t minEditValue, maxEditValue;
  screenFunc_t callbackFunc;
  bool liveEdit;

  // Manual Moves
  const float manual_feedrate_mm_m[] = MANUAL_FEEDRATE;
  millis_t manual_move_start_time = 0;
  int8_t manual_move_axis = (int8_t)NO_AXIS;
  #if EXTRUDERS > 1
    int8_t manual_move_e_index = 0;
  #else
    #define manual_move_e_index 0
  #endif

  #if PIN_EXISTS(SD_DETECT)
    uint8_t lcd_sd_status;
  #endif

  #if (PIDTEMP)
    float raw_Ki, raw_Kd; // place-holders for Ki and Kd edits
  #endif

  /**
   * General function to go directly to a screen
   */
  void lcd_goto_screen(screenFunc_t screen, const uint32_t encoder = 0) {
    if (currentScreen != screen) {

      #if ENABLED(DOUBLECLICK_FOR_Z_BABYSTEPPING) && ENABLED(BABYSTEPPING)
        static millis_t doubleclick_expire_ms = 0;
        // Going to lcd_main_menu from status screen? Remember first click time.
        // Going back to status screen within a very short time? Go to Z babystepping.
        if (screen == lcd_main_menu) {
          if (currentScreen == lcd_status_screen)
            doubleclick_expire_ms = millis() + DOUBLECLICK_MAX_INTERVAL;
        }
        else if (screen == lcd_status_screen && currentScreen == lcd_main_menu && PENDING(millis(), doubleclick_expire_ms))
          screen = lcd_babystep_z;
      #endif

      currentScreen = screen;
      encoderPosition = encoder;
      if (screen == lcd_status_screen) {
        defer_return_to_status = false;
        screen_history_depth = 0;
      }
      lcd_implementation_clear();
      #if ENABLED(LCD_PROGRESS_BAR)
        // For LCD_PROGRESS_BAR re-initialize custom characters
        lcd_set_custom_characters(screen == lcd_status_screen);
      #endif
      lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
      screen_changed = true;
      #if ENABLED(DOGLCD)
        drawing_screen = false;
      #endif
    }
  }

  /**
   * Show "Moving..." till moves are done, then revert to previous display.
   */
  static const char moving[] PROGMEM = MSG_MOVING;
  static const char *sync_message = moving;

  //
  // Display the synchronize screen until moves are
  // finished, and don't return to the caller until
  // done. ** This blocks the command queue! **
  //
  void _lcd_synchronize() {
    static bool no_reentry = false;
    if (lcdDrawUpdate) lcd_implementation_drawmenu_static(LCD_HEIGHT >= 4 ? 1 : 0, sync_message);
    if (no_reentry) return;

    // Make this the current handler till all moves are done
    no_reentry = true;
    screenFunc_t old_screen = currentScreen;
    lcd_goto_screen(_lcd_synchronize);
    stepper.synchronize();
    no_reentry = false;
    lcd_goto_screen(old_screen);
  }

  // Display the synchronize screen with a custom message
  // ** This blocks the command queue! **
  void lcd_synchronize(const char * const msg=NULL) {
    sync_message = msg ? msg : moving;
    _lcd_synchronize();
  }

  void lcd_return_to_status() { lcd_goto_screen(lcd_status_screen); }

  void lcd_save_previous_screen() {
    if (screen_history_depth < COUNT(screen_history)) {
      screen_history[screen_history_depth].menu_function = currentScreen;
      screen_history[screen_history_depth].encoder_position = encoderPosition;
      ++screen_history_depth;
    }
  }

  void lcd_goto_previous_menu() {
    if (screen_history_depth > 0) {
      --screen_history_depth;
      lcd_goto_screen(
        screen_history[screen_history_depth].menu_function,
        screen_history[screen_history_depth].encoder_position
      );
    }
    else
      lcd_return_to_status();
  }

  void lcd_goto_previous_menu_no_defer() {
    defer_return_to_status = false;
    lcd_goto_previous_menu();
  }

#endif // ULTIPANEL

/**
 *
 * "Info Screen"
 *
 * This is very display-dependent, so the lcd implementation draws this.
 */

void lcd_status_screen() {

  #if ENABLED(ULTIPANEL)
    ENCODER_DIRECTION_NORMAL();
    ENCODER_RATE_MULTIPLY(false);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    millis_t ms = millis();
    #if DISABLED(PROGRESS_MSG_ONCE)
      if (ELAPSED(ms, progress_bar_ms + PROGRESS_BAR_MSG_TIME + PROGRESS_BAR_BAR_TIME)) {
        progress_bar_ms = ms;
      }
    #endif
    #if PROGRESS_MSG_EXPIRE > 0
      // Handle message expire
      if (expire_status_ms > 0) {
        #if HAS_SDSUPPORT
          if (card.isFileOpen()) {
            // Expire the message when printing is active
            if (IS_SD_PRINTING) {
              if (ELAPSED(ms, expire_status_ms)) {
                lcd_status_message[0] = '\0';
                expire_status_ms = 0;
              }
            }
            else {
              expire_status_ms += LCD_UPDATE_INTERVAL;
            }
          }
          else {
            expire_status_ms = 0;
          }
        #else
          expire_status_ms = 0;
        #endif // SDSUPPORT
      }
    #endif
  #endif // LCD_PROGRESS_BAR

  #if ENABLED(ULTIPANEL)

    if (lcd_clicked) {
      #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
        previous_lcd_status_ms = millis();  // get status message to show up for a while
      #endif
      lcd_implementation_init( // to maybe revive the LCD if static electricity killed it.
        #if ENABLED(LCD_PROGRESS_BAR)
          false
        #endif
      );
      lcd_goto_screen(lcd_main_menu);
      return;
    }

    #if ENABLED(ULTIPANEL_FEEDMULTIPLY)
      const int16_t new_frm = mechanics.feedrate_percentage + (int32_t)encoderPosition;
      // Dead zone at 100% feedrate
      if ((mechanics.feedrate_percentage < 100 && new_frm > 100) || (mechanics.feedrate_percentage > 100 && new_frm < 100)) {
        mechanics.feedrate_percentage = 100;
        encoderPosition = 0;
      }
      else if (mechanics.feedrate_percentage == 100) {
        if ((int32_t)encoderPosition > ENCODER_FEEDRATE_DEADZONE) {
          mechanics.feedrate_percentage += (int32_t)encoderPosition - (ENCODER_FEEDRATE_DEADZONE);
          encoderPosition = 0;
        }
        else if ((int32_t)encoderPosition < -(ENCODER_FEEDRATE_DEADZONE)) {
          mechanics.feedrate_percentage += (int32_t)encoderPosition + ENCODER_FEEDRATE_DEADZONE;
          encoderPosition = 0;
        }
      }
      else {
        mechanics.feedrate_percentage = new_frm;
        encoderPosition = 0;
      }
    #endif // ULTIPANEL_FEEDMULTIPLY

    mechanics.feedrate_percentage = constrain(mechanics.feedrate_percentage, 10, 999);

  #endif // ULTIPANEL

  lcd_implementation_status_screen();
}

void lcd_reset_status() { lcd_setstatusPGM(PSTR(""), -1); }

/**
 *
 * draw the kill screen
 *
 */
void kill_screen(const char* lcd_msg) {
  lcd_init();
  lcd_setalertstatusPGM(lcd_msg);
  #if ENABLED(DOGLCD)
    u8g.firstPage();
    do {
      lcd_kill_screen();
    } while (u8g.nextPage());
  #else
    lcd_kill_screen();
  #endif
}

#if ENABLED(ULTIPANEL)

  /**
   *
   * Audio feedback for controller clicks
   *
   */

  void lcd_quick_feedback() {
    lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    buttons = 0;
    next_button_update_ms = millis() + 500;

    // Buzz and wait. The delay is needed for buttons to settle!
    BUZZ(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
    #if ENABLED(LCD_USE_I2C_BUZZER)
      HAL::delayMilliseconds(10);
    #endif
  }

  void lcd_completion_feedback(const bool good/*=true*/) {
    if (good) {
      BUZZ(100, 659);
      BUZZ(100, 698);
    }
    else BUZZ(20, 440);
  }

  inline void line_to_current_z() {
    planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(manual_feedrate_mm_m[Z_AXIS]), tools.active_extruder);
  }

  inline void line_to_z(const float &z) {
    mechanics.current_position[Z_AXIS] = z;
    line_to_current_z();
  }

  #if HAS_SDSUPPORT

    void lcd_sdcard_pause() {
      card.pauseSDPrint();
      printer.print_job_counter.pause();
      #if ENABLED(PARK_HEAD_ON_PAUSE)
        commands.enqueue_and_echo_commands_P(PSTR("M125"));
      #endif
      lcd_setstatusPGM(PSTR(MSG_PRINT_PAUSED), -1);
    }

    void lcd_sdcard_resume() {
      #if ENABLED(PARK_HEAD_ON_PAUSE)
        commands.enqueue_and_echo_commands_P(PSTR("M24"));
      #else
        card.startFileprint();
        printer.print_job_counter.start();
      #endif
      lcd_reset_status();
    }

    void lcd_sdcard_stop() {
      printer.stopSDPrint(false);
      lcd_return_to_status();
    }

    void lcd_sdcard_stop_save() {
      printer.stopSDPrint(true);
      lcd_return_to_status();
    }

  #endif // SDSUPPORT

  #if HAS_CASE_LIGHT

    void case_light_menu() {
      START_MENU();
      //
      // ^ Main
      //
      MENU_BACK(MSG_MAIN);
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_CASE_LIGHT_BRIGHTNESS, &printer.case_light_brightness, 0, 255, printer.update_case_light, true);
      MENU_ITEM_EDIT_CALLBACK(bool, MSG_CASE_LIGHT, (bool*)&printer.case_light_on, printer.update_case_light);
      END_MENU();
    }
  #endif // HAS_CASE_LIGHT

  #if ENABLED(BLTOUCH)

    /**
     *
     * "BLTouch" submenu
     *
     */
    static void bltouch_menu() {
      START_MENU();
      //
      // ^ Main
      //
      MENU_BACK(MSG_MAIN);
      MENU_ITEM(gcode, MSG_BLTOUCH_RESET, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_RESET)));
      MENU_ITEM(gcode, MSG_BLTOUCH_SELFTEST, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_SELFTEST)));
      MENU_ITEM(gcode, MSG_BLTOUCH_DEPLOY, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_DEPLOY)));
      MENU_ITEM(gcode, MSG_BLTOUCH_STOW, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_STOW)));
      END_MENU();
    }

  #endif // BLTOUCH

  #if ENABLED(LCD_PROGRESS_BAR_TEST)

    static void progress_bar_test() {
      static int8_t bar_percent = 0;
      if (lcd_clicked) {
        lcd_goto_previous_menu();
        lcd_set_custom_characters(false);
        return;
      }
      bar_percent += (int8_t)encoderPosition;
      bar_percent = constrain(bar_percent, 0, 100);
      encoderPosition = 0;
      lcd_implementation_drawmenu_static(0, PSTR(MSG_PROGRESS_BAR_TEST), true, true);
      lcd.setCursor((LCD_WIDTH) / 2 - 2, LCD_HEIGHT - 2);
      lcd.print(itostr3(bar_percent)); lcd.write('%');
      lcd.setCursor(0, LCD_HEIGHT - 1); lcd_draw_progress_bar(bar_percent);
    }

    void _progress_bar_test() {
      lcd_goto_screen(progress_bar_test);
      lcd_set_custom_characters();
    }

  #endif // LCD_PROGRESS_BAR_TEST

  #if HAS_DEBUG_MENU

    void lcd_debug_menu() {
      START_MENU();

      MENU_BACK(MSG_MAIN); // ^ Main

      #if ENABLED(LCD_PROGRESS_BAR_TEST)
        MENU_ITEM(submenu, MSG_PROGRESS_BAR_TEST, _progress_bar_test);
      #endif

      END_MENU();
    }

  #endif // HAS_DEBUG_MENU

  #if ENABLED(CUSTOM_USER_MENUS)

    #ifdef USER_SCRIPT_DONE
      #define _DONE_SCRIPT "\n" USER_SCRIPT_DONE
    #else
      #define _DONE_SCRIPT ""
    #endif

    void _lcd_user_gcode(const char * const cmd) {
      commands.enqueue_and_echo_commands_P(cmd);
      lcd_completion_feedback();
    }

    #if ENABLED(USER_DESC_1) && ENABLED(USER_GCODE_1)
      void lcd_user_gcode_1() { _lcd_user_gcode(PSTR(USER_GCODE_1 _DONE_SCRIPT)); }
    #endif
    #if ENABLED(USER_DESC_2) && ENABLED(USER_GCODE_2)
      void lcd_user_gcode_2() { _lcd_user_gcode(PSTR(USER_GCODE_2 _DONE_SCRIPT)); }
    #endif
    #if ENABLED(USER_DESC_3) && ENABLED(USER_GCODE_3)
      void lcd_user_gcode_3() { _lcd_user_gcode(PSTR(USER_GCODE_3 _DONE_SCRIPT)); }
    #endif
    #if ENABLED(USER_DESC_4) && ENABLED(USER_GCODE_4)
      void lcd_user_gcode_4() { _lcd_user_gcode(PSTR(USER_GCODE_4 _DONE_SCRIPT)); }
    #endif
    #if ENABLED(USER_DESC_5) && ENABLED(USER_GCODE_5)
      void lcd_user_gcode_5() { _lcd_user_gcode(PSTR(USER_GCODE_5 _DONE_SCRIPT)); }
    #endif

    void _lcd_user_menu() {
      START_MENU();
      MENU_BACK(MSG_MAIN);
      #if ENABLED(USER_DESC_1) && ENABLED(USER_GCODE_1)
        MENU_ITEM(function, USER_DESC_1, lcd_user_gcode_1);
      #endif
      #if ENABLED(USER_DESC_2) && ENABLED(USER_GCODE_2)
        MENU_ITEM(function, USER_DESC_2, lcd_user_gcode_2);
      #endif
      #if ENABLED(USER_DESC_3) && ENABLED(USER_GCODE_3)
        MENU_ITEM(function, USER_DESC_3, lcd_user_gcode_3);
      #endif
      #if ENABLED(USER_DESC_4) && ENABLED(USER_GCODE_4)
        MENU_ITEM(function, USER_DESC_4, lcd_user_gcode_4);
      #endif
      #if ENABLED(USER_DESC_5) && ENABLED(USER_GCODE_5)
        MENU_ITEM(function, USER_DESC_5, lcd_user_gcode_5);
      #endif
      END_MENU();
    }

  #endif

  /**
   *
   * "Main" menu
   *
   */

  void lcd_main_menu() {
    START_MENU();
    MENU_BACK(MSG_WATCH);

    #if ENABLED(CUSTOM_USER_MENUS)
      MENU_ITEM(submenu, MSG_USER_MENU, _lcd_user_menu);
    #endif

    //
    // Debug Menu when certain options are enabled
    //
    #if HAS_DEBUG_MENU
      MENU_ITEM(submenu, MSG_DEBUG_MENU, lcd_debug_menu);
    #endif

    #if ENABLED(LASER)
      if ((!(planner.movesplanned() || IS_SD_PRINTING)) && printer.mode == PRINTER_MODE_LASER) {
        MENU_ITEM(submenu, "Laser Functions", lcd_laser_menu);
      }
    #endif

    //
    // Set Case light on/off/brightness
    //
    #if HAS_CASE_LIGHT
      MENU_ITEM(submenu, MSG_CASE_LIGHT, case_light_menu);
    #endif

    if (planner.movesplanned() || IS_SD_PRINTING) {
      MENU_ITEM(submenu, MSG_TUNE, lcd_tune_menu);
    }
    else {
      MENU_ITEM(submenu, MSG_PREPARE, lcd_prepare_menu);
      #if MECH(DELTA)
        MENU_ITEM(submenu, MSG_DELTA_CALIBRATE, lcd_delta_calibrate_menu);
      #endif
    }
    MENU_ITEM(submenu, MSG_CONTROL, lcd_control_menu);

    #if HAS_SDSUPPORT
      if (card.cardOK) {
        if (card.isFileOpen()) {
          if (card.sdprinting)
            MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause);
          else
            MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_resume);
          MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_stop);
          MENU_ITEM(function, MSG_STOP_SAVE_PRINT, lcd_sdcard_stop_save);
        }
        else {
          MENU_ITEM(submenu, MSG_CARD_MENU, lcd_sdcard_menu);
          #if !PIN_EXISTS(SD_DETECT)
            MENU_ITEM(gcode, MSG_CNG_SDCARD, PSTR("M21"));  // SD-card changed by user
          #endif
        }
      }
      else {
        MENU_ITEM(submenu, MSG_NO_CARD, lcd_sdcard_menu);
        #if !PIN_EXISTS(SD_DETECT)
          MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
        #endif
      }
    #endif // SDSUPPORT

    #if ENABLED(LCD_INFO_MENU)
      MENU_ITEM(submenu, MSG_INFO_MENU, lcd_info_menu);
    #endif

    END_MENU();
  }

  /**
   *
   * "Tune" submenu items
   *
   */

  #if HAS_M206_M408_COMMAND
    /**
     * Set the home offset based on the current_position
     */
    void lcd_set_home_offsets() {
      // M428 Command
      commands.enqueue_and_echo_commands_P(PSTR("M428"));
      lcd_return_to_status();
    }
  #endif

  #if ENABLED(BABYSTEPPING)

    void _lcd_babystep(const AxisEnum axis, const char* msg) {
      if (lcd_clicked) { return lcd_goto_previous_menu_no_defer(); }
      ENCODER_DIRECTION_NORMAL();
      if (encoderPosition) {
        const int16_t babystep_increment = (int32_t)encoderPosition * (BABYSTEP_MULTIPLICATOR);
        encoderPosition = 0;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        thermalManager.babystep_axis(axis, babystep_increment);
        babysteps_done += babystep_increment;
      }
      if (lcdDrawUpdate)
        lcd_implementation_drawedit(msg, ftostr43sign(mechanics.steps_to_mm[axis] * babysteps_done));
    }

    #if ENABLED(BABYSTEP_XY)
      void _lcd_babystep_x() { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEPPING_X)); }
      void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEPPING_Y)); }
      void lcd_babystep_x() { lcd_goto_screen(_lcd_babystep_x); babysteps_done = 0; defer_return_to_status = true; }
      void lcd_babystep_y() { lcd_goto_screen(_lcd_babystep_y); babysteps_done = 0; defer_return_to_status = true; }
    #endif
    void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEPPING_Z)); }
    void lcd_babystep_z() { lcd_goto_screen(_lcd_babystep_z); babysteps_done = 0; defer_return_to_status = true; }

  #endif // BABYSTEPPING

  void lcd_tune_fixstep() {
    #if MECH(DELTA)
      commands.enqueue_and_echo_commands_P(PSTR("G28 B"));
    #else
      commands.enqueue_and_echo_commands_P(PSTR("G28 X Y B"));
    #endif
  }

  /**
   * Watch temperature callbacks
   */
  #if HAS_TEMP_HOTEND
    #if WATCH_THE_HOTEND
      #define _WATCH_FUNC(N) thermalManager.start_watching(&heaters[N])
    #else
      #define _WATCH_FUNC(N) NOOP
    #endif
    void watch_temp_callback_E0() { _WATCH_FUNC(0); }
    #if HOTENDS > 1
      void watch_temp_callback_E1() { _WATCH_FUNC(1); }
      #if HOTENDS > 2
        void watch_temp_callback_E2() { _WATCH_FUNC(2); }
        #if HOTENDS > 3
          void watch_temp_callback_E3() { _WATCH_FUNC(3); }
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1
  #endif

  void watch_temp_callback_bed() {
    #if WATCH_THE_BED
      thermalManager.start_watching(&heaters[BED_INDEX]);
    #endif
  }

  void watch_temp_callback_chamber() {
    #if WATCH_THE_CHAMBER
      thermalManager.start_watching(&heaters[CHAMBER_INDEX]);
    #endif
  }

  void watch_temp_callback_cooler() {
    #if WATCH_THE_COOLER
      thermalManager.start_watching(&heaters[COOLER_INDEX]);
    #endif
  }

  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    void lcd_enqueue_filament_change() {

      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (!DEBUGGING(DRYRUN) && thermalManager.tooColdToExtrude(tools.active_extruder)) {
          lcd_save_previous_screen();
          lcd_goto_screen(lcd_advanced_pause_toocold_menu);
          return;
        }
      #endif

      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT);
      commands.enqueue_and_echo_commands_P(PSTR("M600 B0"));
    }

  #endif

  /**
   *
   * "Tune" submenu
   *
   */
  void lcd_tune_menu() {
    START_MENU();

    //
    // ^ Main
    //
    MENU_BACK(MSG_MAIN);

    //
    // Speed:
    //
    MENU_ITEM_EDIT(int3, MSG_SPEED, &mechanics.feedrate_percentage, 10, 999);

    // Manual bed leveling, Bed Z:
    #if ENABLED(MESH_BED_LEVELING) && ENABLED(LCD_BED_LEVELING)
      MENU_ITEM_EDIT(float43, MSG_BED_Z, &mbl.zprobe_zoffset, -1, 1);
    #endif

    //
    // Nozzle:
    // Nozzle [1-4]:
    //
    #if HOTENDS == 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &heaters[0].target_temperature, 0, heaters[0].maxtemp - 15, watch_temp_callback_E0);
    #elif HOTENDS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &heaters[0].target_temperature, 0, heaters[0].maxtemp - 15, watch_temp_callback_E0);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &heaters[1].target_temperature, 0, heaters[1].maxtemp - 15, watch_temp_callback_E1);
      #if HOTENDS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &heaters[2].target_temperature, 0, heaters[2].maxtemp - 15, watch_temp_callback_E2);
        #if HOTENDS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &heaters[3].target_temperature, 0, heaters[3].maxtemp - 15, watch_temp_callback_E3);
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1

    //
    // Bed:
    //
    #if HAS_TEMP_BED
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_BED, &heaters[BED_INDEX].target_temperature, 0, BED_MAXTEMP - 15, watch_temp_callback_bed);
    #endif

    //
    // Chamber:
    //
    #if HAS_TEMP_CHAMBER
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CHAMBER, &heaters[CHAMBER_INDEX].target_temperature, 0, CHAMBER_MAXTEMP - 15, watch_temp_callback_chamber);
    #endif

    //
    // Cooler:
    //
    #if HAS_TEMP_COOLER
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_COOLER, &heaters[COOLER_INDEX].target_temperature, 0, COOLER_MAXTEMP - 15, watch_temp_callback_cooler);
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
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_1ST_FAN_SPEED, &fans[0].Speed, 0, 255);
      #endif
      #if HAS_FAN1
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_FAN_SPEED " 1", &fans[1].Speed, 0, 255);
      #endif
      #if HAS_FAN2
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_FAN_SPEED " 2", &fans[2].Speed, 0, 255);
      #endif
      #if HAS_FAN3
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_FAN_SPEED " 3", &fans[3].Speed, 0, 255);
      #endif
    #endif // FAN_COUNT > 0

    //
    // Flow:
    // Flow 1:
    // Flow 2:
    // Flow 3:
    // Flow 4:
    //
    #if EXTRUDERS == 1
      MENU_ITEM_EDIT(int3, MSG_FLOW, &tools.flow_percentage[0], 10, 999);
    #else // EXTRUDERS > 1
      MENU_ITEM_EDIT(int3, MSG_FLOW, &tools.flow_percentage[tools.active_extruder], 10, 999);
      MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N1, &tools.flow_percentage[0], 10, 999);
      MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N2, &tools.flow_percentage[1], 10, 999);
      #if EXTRUDERS > 2
        MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N3, &tools.flow_percentage[2], 10, 999);
        #if EXTRUDERS > 3
          MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N4, &tools.flow_percentage[3], 10, 999);
          #if EXTRUDERS > 4
            MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N5, &tools.flow_percentage[4], 10, 999);
            #if EXTRUDERS > 5
              MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N6, &tools.flow_percentage[5], 10, 999);
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
      #endif // BABYSTEP_XY
      MENU_ITEM(submenu, MSG_BABYSTEP_Z, lcd_babystep_z);
    #endif

    MENU_ITEM(function, MSG_FIX_LOSE_STEPS, lcd_tune_fixstep);

    //
    // Change filament
    //
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      if (!thermalManager.tooColdToExtrude(tools.active_extruder) && !IS_SD_FILE_OPEN)
        MENU_ITEM(function, MSG_FILAMENTCHANGE, lcd_enqueue_filament_change);
    #endif

    END_MENU();
  }

  #if ENABLED(EASY_LOAD)
    void lcd_extrude(float length, float feedrate) {
      mechanics.current_position[E_AXIS] += length;
      #if MECH(DELTA)
        mechanics.Transform(mechanics.current_position);
        planner.buffer_line(mechanics.delta[X_AXIS], mechanics.delta[Y_AXIS], mechanics.delta[Z_AXIS], mechanics.current_position[E_AXIS], feedrate, tools.active_extruder);
      #else
        planner.buffer_line(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS], mechanics.current_position[E_AXIS], feedrate, tools.active_extruder);
      #endif
    }
    void lcd_purge() { lcd_extrude(LCD_PURGE_LENGTH, LCD_PURGE_FEEDRATE); }
    void lcd_retract() { lcd_extrude(-LCD_RETRACT_LENGTH, LCD_RETRACT_FEEDRATE); }
    void lcd_easy_load() {
      printer.allow_lengthy_extrude_once = true;
      lcd_extrude(BOWDEN_LENGTH, LCD_LOAD_FEEDRATE);
      lcd_return_to_status();
    }
    void lcd_easy_unload() {
      printer.allow_lengthy_extrude_once = true;
      lcd_extrude(-BOWDEN_LENGTH, LCD_UNLOAD_FEEDRATE);
      lcd_return_to_status();
    }
  #endif // EASY_LOAD

  /**
   *
   * "Prepare" submenu items
   *
   */
  void _lcd_preheat(const int16_t endnum, const int16_t temph, const int16_t tempb, const int16_t fan) {
    #if HAS_TEMP_HOTEND
      if (temph > 0) heaters[endnum].setTarget(min(heaters[endnum].maxtemp, temph));
    #endif
    #if HAS_TEMP_BED
      if (tempb >= 0) heaters[BED_INDEX].setTarget(tempb);
    #else
      UNUSED(tempb);
    #endif
    #if FAN_COUNT > 0
      #if FAN_COUNT > 1
        fans[tools.active_extruder < FAN_COUNT ? tools.active_extruder : 0].Speed = fan;
      #else
        fans[0].Speed = fan;
      #endif
    #else
      UNUSED(fan);
    #endif
    lcd_return_to_status();
  }

  #if HAS_TEMP_0
    void lcd_preheat_m1_h0_only() { _lcd_preheat(0, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
    void lcd_preheat_m2_h0_only() { _lcd_preheat(0, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
    void lcd_preheat_m3_h0_only() { _lcd_preheat(0, lcd_preheat_hotend_temp[2], -1, lcd_preheat_fan_speed[2]); }
    #if HAS_TEMP_BED
      void lcd_preheat_m1_h0() { _lcd_preheat(0, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
      void lcd_preheat_m2_h0() { _lcd_preheat(0, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
      void lcd_preheat_m3_h0() { _lcd_preheat(0, lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2]); }
    #endif
  #endif

  #if HOTENDS > 1
    void lcd_preheat_m1_h1_only() { _lcd_preheat(1, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
    void lcd_preheat_m2_h1_only() { _lcd_preheat(1, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
    void lcd_preheat_m3_h1_only() { _lcd_preheat(1, lcd_preheat_hotend_temp[2], -1, lcd_preheat_fan_speed[2]); }
    #if HAS_TEMP_BED
      void lcd_preheat_m1_h1() { _lcd_preheat(1, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
      void lcd_preheat_m2_h1() { _lcd_preheat(1, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
      void lcd_preheat_m3_h1() { _lcd_preheat(1, lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2]); }
    #endif
    #if HOTENDS > 2
      void lcd_preheat_m1_h2_only() { _lcd_preheat(2, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
      void lcd_preheat_m2_h2_only() { _lcd_preheat(2, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
      void lcd_preheat_m3_h2_only() { _lcd_preheat(2, lcd_preheat_hotend_temp[2], -1, lcd_preheat_fan_speed[2]); }
      #if HAS_TEMP_BED
        void lcd_preheat_m1_h2() { _lcd_preheat(2, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
        void lcd_preheat_m2_h2() { _lcd_preheat(2, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
        void lcd_preheat_m3_h2() { _lcd_preheat(2, lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2]); }
      #endif
      #if HOTENDS > 3
        void lcd_preheat_m1_h3_only() { _lcd_preheat(3, lcd_preheat_hotend_temp[0], -1, lcd_preheat_fan_speed[0]); }
        void lcd_preheat_m2_h3_only() { _lcd_preheat(3, lcd_preheat_hotend_temp[1], -1, lcd_preheat_fan_speed[1]); }
        void lcd_preheat_m3_h3_only() { _lcd_preheat(3, lcd_preheat_hotend_temp[2], -1, lcd_preheat_fan_speed[2]); }
        #if HAS_TEMP_BED
          void lcd_preheat_m1_h3() { _lcd_preheat(3, lcd_preheat_hotend_temp[0], lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
          void lcd_preheat_m2_h3() { _lcd_preheat(3, lcd_preheat_hotend_temp[1], lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
          void lcd_preheat_m3_h3() { _lcd_preheat(3, lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2]); }
        #endif
      #endif
    #endif

    void lcd_preheat_m1_all() {
      LOOP_HOTEND() heaters[h].setTarget(lcd_preheat_hotend_temp[0]);
      #if HAS_TEMP_BED
        lcd_preheat_m1_h0();
      #else
        lcd_preheat_m1_h0_only();
      #endif
    }
    void lcd_preheat_m2_all() {
      LOOP_HOTEND() heaters[h].setTarget(lcd_preheat_hotend_temp[1]);
      #if HAS_TEMP_BED
        lcd_preheat_m2_h0();
      #else
        lcd_preheat_m2_h0_only();
      #endif
    }
    void lcd_preheat_m3_all() {
      LOOP_HOTEND() heaters[h].setTarget(lcd_preheat_hotend_temp[2]);
      #if HAS_TEMP_BED
        lcd_preheat_m3_h0();
      #else
        lcd_preheat_m3_h0_only();
      #endif
    }

  #endif // HOTENDS > 1

  #if HAS_TEMP_BED
    void lcd_preheat_m1_bedonly() { _lcd_preheat(0, 0, lcd_preheat_bed_temp[0], lcd_preheat_fan_speed[0]); }
    void lcd_preheat_m2_bedonly() { _lcd_preheat(0, 0, lcd_preheat_bed_temp[1], lcd_preheat_fan_speed[1]); }
    void lcd_preheat_m3_bedonly() { _lcd_preheat(0, 0, lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2]); }
  #endif

  #if HAS_TEMP_0 && (HAS_TEMP_1 || HAS_TEMP_2 || HAS_TEMP_3 || HAS_TEMP_BED)

    void lcd_preheat_m1_menu() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);
      #if HOTENDS == 1
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0);
          MENU_ITEM(function, MSG_PREHEAT_1_END, lcd_preheat_m1_h0_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
        #endif
      #elif HOTENDS > 1
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0);
          MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H1, lcd_preheat_m1_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1);
          MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H2, lcd_preheat_m1_h1_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H1, lcd_preheat_m1_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H2, lcd_preheat_m1_h1_only);
        #endif
        #if HOTENDS > 2
          #if HAS_TEMP_BED
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2);
            MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H3, lcd_preheat_m1_h2_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H3, lcd_preheat_m1_h2_only);
          #endif
          #if HOTENDS > 3
            #if HAS_TEMP_BED
              MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3);
              MENU_ITEM(function, MSG_PREHEAT_1_END " " MSG_H4, lcd_preheat_m1_h3_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_1_N MSG_H4, lcd_preheat_m1_h3_only);
            #endif
          #endif
        #endif
        MENU_ITEM(function, MSG_PREHEAT_1_ALL, lcd_preheat_m1_all);
      #endif
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_1_BEDONLY, lcd_preheat_m1_bedonly);
      #endif
      END_MENU();
    }

    void lcd_preheat_m2_menu() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);
      #if HOTENDS == 1
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0);
          MENU_ITEM(function, MSG_PREHEAT_2_END, lcd_preheat_m2_h0_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
        #endif
      #elif HOTENDS > 1
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0);
          MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H1, lcd_preheat_m2_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1);
          MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H2, lcd_preheat_m2_h1_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H1, lcd_preheat_m2_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H2, lcd_preheat_m2_h1_only);
        #endif
        #if HOTENDS > 2
          #if HAS_TEMP_BED
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2);
            MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H3, lcd_preheat_m2_h2_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H3, lcd_preheat_m2_h2_only);
          #endif
          #if HOTENDS > 3
            #if HAS_TEMP_BED
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3);
              MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m2_h3_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m2_h3_only);
            #endif
          #endif
        #endif
        MENU_ITEM(function, MSG_PREHEAT_2_ALL, lcd_preheat_m2_all);
      #endif
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_2_BEDONLY, lcd_preheat_m2_bedonly);
      #endif
      END_MENU();
    }

    void lcd_preheat_m3_menu() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);
      #if HOTENDS == 1
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0);
          MENU_ITEM(function, MSG_PREHEAT_3_END, lcd_preheat_m3_h0_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
        #endif
      #elif HOTENDS > 1
        #if HAS_TEMP_BED
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0);
          MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H1, lcd_preheat_m3_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1);
          MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H2, lcd_preheat_m3_h1_only);
        #else
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H1, lcd_preheat_m3_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H2, lcd_preheat_m3_h1_only);
        #endif
        #if HOTENDS > 2
          #if HAS_TEMP_BED
            MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2);
            MENU_ITEM(function, MSG_PREHEAT_3_END " " MSG_H3, lcd_preheat_m3_h2_only);
          #else
            MENU_ITEM(function, MSG_PREHEAT_3_N MSG_H3, lcd_preheat_m3_h2_only);
          #endif
          #if HOTENDS > 3
            #if HAS_TEMP_BED
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3);
              MENU_ITEM(function, MSG_PREHEAT_2_END " " MSG_H4, lcd_preheat_m3_h3_only);
            #else
              MENU_ITEM(function, MSG_PREHEAT_2_N MSG_H4, lcd_preheat_m3_h3_only);
            #endif
          #endif
        #endif
        MENU_ITEM(function, MSG_PREHEAT_3_ALL, lcd_preheat_m3_all);
      #endif
      #if HAS_TEMP_BED
        MENU_ITEM(function, MSG_PREHEAT_3_BEDONLY, lcd_preheat_m3_bedonly);
      #endif
      END_MENU();
    }

  #endif // TEMP_SENSOR_0 && (TEMP_SENSOR_1 || TEMP_SENSOR_2 || TEMP_SENSOR_3 || TEMP_SENSOR_BED)

  void lcd_cooldown() {
    #if FAN_COUNT > 0
      LOOP_FAN() fans[f].Speed = 0;
    #endif
    thermalManager.disable_all_heaters();
    lcd_return_to_status();
  }

  #if HAS_SDSUPPORT && ENABLED(MENU_ADDAUTOSTART)

    void lcd_autostart_sd() {
      card.checkautostart(true);
    }

  #endif

  #if ENABLED(EEPROM_SETTINGS)
    static void lcd_store_settings()   { lcd_completion_feedback(eeprom.Store_Settings()); }
    static void lcd_load_settings()    { lcd_completion_feedback(eeprom.Load_Settings()); }
  #endif

  #if HAS_BED_PROBE
    static void lcd_refresh_zprobe_zoffset() { probe.refresh_offset(); }
  #endif

  #if ENABLED(LEVEL_BED_CORNERS)

    /**
     * Level corners, starting in the front-left corner.
     */
    static int8_t bed_corner;
    void _lcd_goto_next_corner() {
      line_to_z(LOGICAL_Z_POSITION(4.0));
      switch (bed_corner) {
        case 0:
          mechanics.current_position[X_AXIS] = X_MIN_POS + 10;
          mechanics.current_position[Y_AXIS] = Y_MIN_POS + 10;
          break;
        case 1:
          mechanics.current_position[X_AXIS] = X_MAX_POS - 10;
          mechanics.current_position[Y_AXIS] = Y_MIN_POS + 10;
          break;
        case 2:
          mechanics.current_position[X_AXIS] = X_MAX_POS - 10;
          mechanics.current_position[Y_AXIS] = Y_MAX_POS - 10;
          break;
        case 3:
          mechanics.current_position[X_AXIS] = X_MIN_POS + 10;
          mechanics.current_position[Y_AXIS] = Y_MAX_POS - 10;
          break;
      }
      planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(manual_feedrate_mm_m[X_AXIS]), tools.active_extruder);
      line_to_z(LOGICAL_Z_POSITION(0.0));
      if (++bed_corner > 3) bed_corner = 0;
    }

    void _lcd_corner_submenu() {
      START_MENU();
      MENU_ITEM(function, MSG_NEXT_CORNER, _lcd_goto_next_corner);
      MENU_ITEM(function, MSG_BACK, lcd_goto_previous_menu_no_defer);
      END_MENU();
    }

    void _lcd_level_bed_corners() {
      defer_return_to_status = true;
      lcd_goto_screen(_lcd_corner_submenu);
      bed_corner = 0;
      _lcd_goto_next_corner();
    }

  #endif // LEVEL_BED_CORNERS

  #if ENABLED(LCD_BED_LEVELING)

    /**
     *
     * "Prepare" > "Level Bed" handlers
     *
     */

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
    // Raise Z to the "manual probe height"
    // Don't return until done.
    // ** This blocks the command queue! **
    //
    void _lcd_after_probing() {
      #if MANUAL_PROBE_HEIGHT > 0
        line_to_z(LOGICAL_Z_POSITION(Z_MIN_POS) + MANUAL_PROBE_HEIGHT);
      #endif
      // Display "Done" screen and wait for moves to complete
      #if MANUAL_PROBE_HEIGHT > 0 || ENABLED(MESH_BED_LEVELING)
        lcd_synchronize(PSTR(MSG_LEVEL_BED_DONE));
      #endif
      lcd_goto_previous_menu();
      lcd_completion_feedback();
      defer_return_to_status = false;
      //LCD_MESSAGEPGM(MSG_LEVEL_BED_DONE);
    }

    #if ENABLED(MESH_BED_LEVELING)

      // Utility to go to the next mesh point
      inline void _manual_probe_goto_xy(float x, float y) {
        #if MANUAL_PROBE_HEIGHT > 0
          line_to_z(LOGICAL_Z_POSITION(Z_MIN_POS) + MANUAL_PROBE_HEIGHT);
        #endif
        mechanics.current_position[X_AXIS] = LOGICAL_X_POSITION(x);
        mechanics.current_position[Y_AXIS] = LOGICAL_Y_POSITION(y);
        planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(XY_PROBE_SPEED), tools.active_extruder);
        #if MANUAL_PROBE_HEIGHT > 0
          line_to_z(LOGICAL_Z_POSITION(Z_MIN_POS));
        #endif
        lcd_synchronize();
      }

    #elif ENABLED(PROBE_MANUALLY)

      //
      // Bed leveling is done. Wait for G29 to complete.
      // A flag is used so that this can release control
      // and allow the command queue to be processed.
      //
      void _lcd_level_bed_done() {
        if (!lcd_wait_for_move) _lcd_after_probing();
        if (lcdDrawUpdate) lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_DONE));
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
      }

    #endif

    void _lcd_level_goto_next_point();

    /**
     * Step 7: Get the Z coordinate, click goes to the next point or exits
     */
    void _lcd_level_bed_get_z() {
      ENCODER_DIRECTION_NORMAL();

      if (lcd_clicked) {

        //
        // Save the current Z position
        //

        #if ENABLED(MESH_BED_LEVELING)

          //
          // MBL records the position but doesn't move to the next one
          //

          mbl.set_zigzag_z(manual_probe_index, mechanics.current_position[Z_AXIS]);

        #endif

        // If done...
        if (++manual_probe_index >= total_probe_points) {

          #if ENABLED(PROBE_MANUALLY)

            //
            // The last G29 will record and enable but not move.
            //
            lcd_wait_for_move = true;
            commands.enqueue_and_echo_commands_P(PSTR("G29 V1"));
            lcd_goto_screen(_lcd_level_bed_done);

          #elif ENABLED(MESH_BED_LEVELING)

            _lcd_after_probing();

            mbl.set_has_mesh(true);
            bedlevel.mesh_probing_done();

          #endif

        }
        else {
          // MESH_BED_LEVELING: Z already stored, just move
          //    PROBE_MANUALLY: Send G29 to record Z, then move
          _lcd_level_goto_next_point();
        }

        return;
      }

      //
      // Encoder knob or keypad buttons adjust the Z position
      //
      if (encoderPosition) {
        commands.refresh_cmd_timeout();
        const float z = mechanics.current_position[Z_AXIS] + float((int32_t)encoderPosition) * (LCD_Z_STEP);
        line_to_z(constrain(z, -(LCD_PROBE_Z_RANGE) * 0.5, (LCD_PROBE_Z_RANGE) * 0.5));
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
        encoderPosition = 0;
      }

      //
      // Draw on first display, then only on Z change
      //
      if (lcdDrawUpdate) {
        const float v = mechanics.current_position[Z_AXIS];
        lcd_implementation_drawedit(PSTR(MSG_MOVE_Z), ftostr43sign(v + (v < 0 ? -0.0001 : 0.0001), '+'));
      }
    }

    /**
     * Step 6: Display "Next point: 1 / 9" while waiting for move to finish
     */

    void _lcd_level_bed_moving() {
      if (lcdDrawUpdate) {
        char msg[10];
        sprintf_P(msg, PSTR("%i / %u"), (int)(manual_probe_index + 1), total_probe_points);
        lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_NEXT_POINT), msg);
      }
      lcdDrawUpdate = LCDVIEW_CALL_NO_REDRAW;
      #if ENABLED(PROBE_MANUALLY)
        if (!lcd_wait_for_move) lcd_goto_screen(_lcd_level_bed_get_z);
      #endif
    }

    /**
     * Step 5: Initiate a move to the next point
     */
    void _lcd_level_goto_next_point() {

      // Set the menu to display ahead of blocking call
      lcd_goto_screen(_lcd_level_bed_moving);

      #if ENABLED(MESH_BED_LEVELING)

        int8_t px, py;
        mbl.zigzag(manual_probe_index, px, py);

        // Controls the loop until the move is done
        _manual_probe_goto_xy(
          LOGICAL_X_POSITION(mbl.index_to_xpos[px]),
          LOGICAL_Y_POSITION(mbl.index_to_ypos[py])
        );

        // After the blocking function returns, change menus
        lcd_goto_screen(_lcd_level_bed_get_z);

      #elif ENABLED(PROBE_MANUALLY)

        // G29 Records Z, moves, and signals when it pauses
        lcd_wait_for_move = true;
        commands.enqueue_and_echo_commands_P(PSTR("G29 V1"));

      #endif
    }

    /**
     * Step 4: Display "Click to Begin", wait for click
     *         Move to the first probe position
     */
    void _lcd_level_bed_homing_done() {
      if (lcdDrawUpdate) lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_WAITING));
      if (lcd_clicked) {
        manual_probe_index = 0;
        _lcd_level_goto_next_point();
      }
    }

    /**
     * Step 3: Display "Homing XYZ" - Wait for homing to finish
     */
    void _lcd_level_bed_homing() {
      if (lcdDrawUpdate) lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_HOMING), NULL);
      lcdDrawUpdate = LCDVIEW_CALL_NO_REDRAW;
      if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
        lcd_goto_screen(_lcd_level_bed_homing_done);
    }

    /**
     * Step 2: Continue Bed Leveling...
     */
    void _lcd_level_bed_continue() {
      defer_return_to_status = true;
      mechanics.axis_homed[X_AXIS] = mechanics.axis_homed[Y_AXIS] = mechanics.axis_homed[Z_AXIS] = false;
      lcd_goto_screen(_lcd_level_bed_homing);
      commands.enqueue_and_echo_commands_P(PSTR("G28"));
    }

    static bool _level_state;
    void _lcd_toggle_bed_leveling() { bedlevel.set_bed_leveling_enabled(_level_state); }

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      void _lcd_set_z_fade_height() { bedlevel.set_z_fade_height(bedlevel.z_fade_height); }
    #endif

    /**
     * Step 1: Bed Level entry-point
     *
     * << Prepare
     *    Auto Home           (if homing needed)
     *    Leveling On/Off     (if data exists, and homed)
     *    Fade Height: ---    (Req: ENABLE_LEVELING_FADE_HEIGHT)
     *    Mesh Z Offset: ---  (Req: MESH_BED_LEVELING)
     *    Z Probe Offset: --- (Req: HAS_BED_PROBE)
     *    Level Bed >
     *    Level Corners >     (if homed)
     *    Load Settings       (Req: EEPROM_SETTINGS)
     *    Save Settings       (Req: EEPROM_SETTINGS)
     */
    void lcd_bed_leveling() {
      START_MENU();
      MENU_BACK(MSG_PREPARE);

      if (!(mechanics.axis_known_position[X_AXIS] && mechanics.axis_known_position[Y_AXIS] && mechanics.axis_known_position[Z_AXIS]))
        MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
      else if (bedlevel.leveling_is_valid()) {
        _level_state = bedlevel.leveling_is_active();
        MENU_ITEM_EDIT_CALLBACK(bool, MSG_BED_LEVELING, &_level_state, _lcd_toggle_bed_leveling);
      }

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        bedlevel.set_z_fade_height(bedlevel.z_fade_height);
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_Z_FADE_HEIGHT, &bedlevel.z_fade_height, 0.0, 100.0, _lcd_set_z_fade_height);
      #endif

      //
      // MBL Z Offset
      //
      #if ENABLED(MESH_BED_LEVELING)
        MENU_ITEM_EDIT(float43, MSG_BED_Z, &mbl.zprobe_zoffset, -1, 1);
      #endif

      #if HAS_BED_PROBE
        MENU_ITEM_EDIT_CALLBACK(float32, MSG_PROBE_OFFSET, &probe.offset[Z_AXIS], Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX, lcd_refresh_zprobe_zoffset);
      #endif

      MENU_ITEM(submenu, MSG_LEVEL_BED, _lcd_level_bed_continue);

      #if ENABLED(LEVEL_BED_CORNERS)
        // Move to the next corner for leveling
        if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
          MENU_ITEM(function, MSG_LEVEL_CORNERS, _lcd_level_bed_corners);
      #endif

      #if ENABLED(EEPROM_SETTINGS)
        MENU_ITEM(function, MSG_LOAD_EEPROM, lcd_load_settings);
        MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
      #endif
      END_MENU();
    }

  #endif  // LCD_BED_LEVELING

  /**
   *
   * "Prepare" submenu
   *
   */

  void lcd_prepare_menu() {
    START_MENU();

    //
    // ^ Main
    //
    MENU_BACK(MSG_MAIN);

    //
    // Move Axis
    //
    #if MECH(DELTA)
      if (mechanics.axis_homed[Z_AXIS])
    #endif
        MENU_ITEM(submenu, MSG_MOVE_AXIS, lcd_move_menu);

    //
    // Auto Home
    //
    if (printer.mode == PRINTER_MODE_LASER)
      MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28 X Y F2000"));
    else {
      MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
      #if NOMECH(DELTA)
        MENU_ITEM(gcode, MSG_AUTO_HOME_X, PSTR("G28 X"));
        MENU_ITEM(gcode, MSG_AUTO_HOME_Y, PSTR("G28 Y"));
        MENU_ITEM(gcode, MSG_AUTO_HOME_Z, PSTR("G28 Z"));
      #endif
    }

    //
    // Level Bed
    //
    #if ENABLED(LCD_BED_LEVELING)
      #if ENABLED(PROBE_MANUALLY)
        if (!bedlevel.g29_in_progress)
      #endif
      MENU_ITEM(submenu, MSG_BED_LEVELING, lcd_bed_leveling);
    #else
      #if HAS_LEVELING
        MENU_ITEM(gcode, MSG_BED_LEVELING, PSTR("G28\nG29"));
      #endif
      #if ENABLED(LEVEL_BED_CORNERS)
        if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
          MENU_ITEM(function, MSG_LEVEL_CORNERS, _lcd_level_bed_corners);
      #endif
    #endif

    #if HAS_M206_M408_COMMAND
      //
      // Set Home Offsets
      //
      MENU_ITEM(function, MSG_SET_HOME_OFFSETS, lcd_set_home_offsets);
      //MENU_ITEM(gcode, MSG_SET_ORIGIN, PSTR("G92 X0 Y0 Z0"));
    #endif

    //
    // Disable Steppers
    //
    MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));

    if (printer.mode == PRINTER_MODE_FFF) {

      //
      // Change filament
      //
      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        if (!thermalManager.tooColdToExtrude(tools.active_extruder) && !IS_SD_PRINTING)
          MENU_ITEM(function, MSG_FILAMENTCHANGE, lcd_enqueue_filament_change);
      #endif

      #if HAS_TEMP_0

        //
        // Cooldown
        //
        bool has_heat = false;
        #if HEATER_COUNT > 0
          LOOP_HEATER() if (heaters[h].target_temperature) { has_heat = true; break; }
        #endif
        if (has_heat) MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);

        //
        // Preheat for Material 1, 2 and 3
        //
        #if HAS_TEMP_1 || HAS_TEMP_2 || HAS_TEMP_3 || HAS_TEMP_BED
          MENU_ITEM(submenu, MSG_PREHEAT_1, lcd_preheat_m1_menu);
          MENU_ITEM(submenu, MSG_PREHEAT_2, lcd_preheat_m2_menu);
          MENU_ITEM(submenu, MSG_PREHEAT_3, lcd_preheat_m3_menu);
        #else
          MENU_ITEM(function, MSG_PREHEAT_1, lcd_preheat_m1_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_2, lcd_preheat_m2_h0_only);
          MENU_ITEM(function, MSG_PREHEAT_3, lcd_preheat_m3_h0_only);
        #endif

      #endif // HAS_TEMP_0

      //
      // Easy Load
      //
      #if ENABLED(EASY_LOAD)
        MENU_ITEM(function, MSG_E_BOWDEN_LENGTH, lcd_easy_load);
        MENU_ITEM(function, MSG_R_BOWDEN_LENGTH, lcd_easy_unload);
        MENU_ITEM(function, MSG_PURGE_XMM, lcd_purge);
        MENU_ITEM(function, MSG_RETRACT_XMM, lcd_retract);
      #endif // EASY_LOAD

    } // printer mode FFF

    //
    // BLTouch Self-Test and Reset
    //
    #if ENABLED(BLTOUCH)
      MENU_ITEM(gcode, MSG_BLTOUCH_SELFTEST, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_SELFTEST)));
      if (!probe.enabled && TEST_BLTOUCH())
        MENU_ITEM(gcode, MSG_BLTOUCH_RESET, PSTR("M280 P" STRINGIFY(Z_ENDSTOP_SERVO_NR) " S" STRINGIFY(BLTOUCH_RESET)));
    #endif

    //
    // Switch power on/off
    //
    #if HAS_POWER_SWITCH
      if (powerManager.powersupply_on)
        MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
      else
        MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
    #endif

    //
    // Autostart
    //
    #if HAS_SDSUPPORT && ENABLED(MENU_ADDAUTOSTART)
      MENU_ITEM(function, MSG_AUTOSTART, lcd_autostart_sd);
    #endif

    END_MENU();
  }

  float move_menu_scale;

  #if MECH(DELTA)

    void lcd_move_z();
    void lcd_delta_calibrate_menu();

    void _lcd_calibrate_homing() {
      if (lcdDrawUpdate) lcd_implementation_drawmenu_static(LCD_HEIGHT >= 4 ? 1 : 0, PSTR(MSG_LEVEL_BED_HOMING));
      lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT;
      if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
        lcd_goto_previous_menu();
    }

    void _lcd_delta_calibrate_home() {
      #if HAS_LEVELING
        bedlevel.reset_bed_level(); // After calibration bed-level data is no longer valid
      #endif

      commands.enqueue_and_echo_commands_P(PSTR("G28"));
      lcd_goto_screen(_lcd_calibrate_homing);
    }

    void _goto_tower_x() { mechanics.manual_goto_xy(COS(RADIANS(210)) * mechanics.delta_probe_radius, SIN(RADIANS(210)) * mechanics.delta_probe_radius); }
    void _goto_tower_y() { mechanics.manual_goto_xy(COS(RADIANS(330)) * mechanics.delta_probe_radius, SIN(RADIANS(330)) * mechanics.delta_probe_radius); }
    void _goto_tower_z() { mechanics.manual_goto_xy(COS(RADIANS( 90)) * mechanics.delta_probe_radius, SIN(RADIANS( 90)) * mechanics.delta_probe_radius); }
    void _goto_center()  { mechanics.manual_goto_xy(0,0); }

    void lcd_delta_settings() {
      START_MENU();
      MENU_BACK(MSG_DELTA_CALIBRATE);
      MENU_ITEM_EDIT(float52, MSG_DELTA_HEIGHT, &mechanics.delta_height, DELTA_HEIGHT - 10.0, DELTA_HEIGHT + 10.0);
      MENU_ITEM_EDIT(float43, "Ex", &mechanics.delta_endstop_adj[A_AXIS], -5.0, 0.0);
      MENU_ITEM_EDIT(float43, "Ey", &mechanics.delta_endstop_adj[B_AXIS], -5.0, 0.0);
      MENU_ITEM_EDIT(float43, "Ez", &mechanics.delta_endstop_adj[C_AXIS], -5.0, 0.0);
      MENU_ITEM_EDIT(float52, MSG_DELTA_DIAG_ROG, &mechanics.delta_diagonal_rod, DELTA_DIAGONAL_ROD - 5.0, DELTA_DIAGONAL_ROD + 5.0);
      MENU_ITEM_EDIT(float52, MSG_DELTA_RADIUS, &mechanics.delta_radius, DELTA_RADIUS - 5.0, DELTA_RADIUS + 5.0);
      MENU_ITEM_EDIT(float43, "Tx", &mechanics.delta_tower_radius_adj[A_AXIS], -5.0, 5.0);
      MENU_ITEM_EDIT(float43, "Ty", &mechanics.delta_tower_radius_adj[B_AXIS], -5.0, 5.0);
      MENU_ITEM_EDIT(float43, "Tz", &mechanics.delta_tower_radius_adj[C_AXIS], -5.0, 5.0);
      END_MENU();
    }

    void lcd_delta_calibrate_menu() {
      START_MENU();
      MENU_BACK(MSG_MAIN);
      MENU_ITEM(submenu, MSG_DELTA_SETTINGS, lcd_delta_settings);
      #if ENABLED(DELTA_AUTO_CALIBRATION_1)
        MENU_ITEM(gcode, MSG_DELTA_AUTO_CALIBRATE, PSTR("G33"));
      #elif ENABLED(DELTA_AUTO_CALIBRATION_2)
        MENU_ITEM(gcode, MSG_DELTA_AUTO_CALIBRATE, PSTR("G33"));
        MENU_ITEM(gcode, MSG_DELTA_HEIGHT_CALIBRATE, PSTR("G33 P1"));
      #endif
      MENU_ITEM(submenu, MSG_AUTO_HOME, _lcd_delta_calibrate_home);
      if (mechanics.axis_homed[Z_AXIS]) {
        MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_X, _goto_tower_x);
        MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_Y, _goto_tower_y);
        MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_Z, _goto_tower_z);
        MENU_ITEM(submenu, MSG_DELTA_CALIBRATE_CENTER, _goto_center);
      }
      END_MENU();
    }

  #endif // DELTA

  /**
   * If the most recent manual move hasn't been fed to the planner yet,
   * and the planner can accept one, send immediately
   */
  inline void manage_manual_move() {
    if (manual_move_axis != (int8_t)NO_AXIS && ELAPSED(millis(), manual_move_start_time) && !planner.is_full()) {
      planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]), tools.active_extruder);
      manual_move_axis = (int8_t)NO_AXIS;
    }
  }

  /**
   * Set a flag that lcd_update() should start a move
   * to "current_position" after a short delay.
   */
  inline void manual_move_to_current(AxisEnum axis) {
    manual_move_start_time = millis() + (move_menu_scale < 0.99 ? 0UL : 250UL); // delay for bigger moves
    manual_move_axis = (int8_t)axis;
  }

  /**
   *
   * "Prepare" > "Move Axis" submenu
   *
   */

  void _lcd_move_xyz(const char* name, AxisEnum axis) {
    if (lcd_clicked) { return lcd_goto_previous_menu(); }
    ENCODER_DIRECTION_NORMAL();
    if (encoderPosition) {
      commands.refresh_cmd_timeout();

      float min = mechanics.current_position[axis] - 1000,
            max = mechanics.current_position[axis] + 1000;

      #if HAS_SOFTWARE_ENDSTOPS
        // Limit to software endstops, if enabled
        if (endstops.soft_endstops_enabled) {
          #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
            min = endstops.soft_endstop_min[axis];
          #endif
          #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
            max = endstops.soft_endstop_max[axis];
          #endif
        }
      #endif

      // Get the new position
      mechanics.current_position[axis] += float((int32_t)encoderPosition) * move_menu_scale;

      // Delta limits XY based on the current offset from center
      // This assumes the center is 0,0
      #if MECH(DELTA)
        if (axis != Z_AXIS) {
          max = SQRT(sq((float)(DELTA_PRINTABLE_RADIUS)) - sq(mechanics.current_position[Y_AXIS - axis]));
          min = -max;
        }
      #endif

      // Limit only when trying to move towards the limit
      if ((int32_t)encoderPosition < 0) NOLESS(mechanics.current_position[axis], min);
      if ((int32_t)encoderPosition > 0) NOMORE(mechanics.current_position[axis], max);

      manual_move_to_current(axis);

      encoderPosition = 0;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }
    if (lcdDrawUpdate) lcd_implementation_drawedit(name, ftostr41sign(mechanics.current_position[axis]));
  }
  void lcd_move_x() { _lcd_move_xyz(PSTR(MSG_MOVE_X), X_AXIS); }
  void lcd_move_y() { _lcd_move_xyz(PSTR(MSG_MOVE_Y), Y_AXIS); }
  void lcd_move_z() { _lcd_move_xyz(PSTR(MSG_MOVE_Z), Z_AXIS); }
  void _lcd_move_e(
    #if EXTRUDERS > 1
      int8_t eindex=-1
    #endif
  ) {
    #if EXTRUDERS > 1
      static uint8_t old_extruder = 0;
      old_extruder = tools.active_extruder;
      printer.tool_change(eindex, 0.0, true);
    #endif
    if (lcd_clicked) {
      #if EXTRUDERS > 1
        printer.tool_change(old_extruder, 0.0, true);
      #endif
      return lcd_goto_previous_menu();
    }
    ENCODER_DIRECTION_NORMAL();
    if (encoderPosition) {
      mechanics.current_position[E_AXIS] += float((int32_t)encoderPosition) * move_menu_scale;
      encoderPosition = 0;
      manual_move_to_current(E_AXIS);
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }
    if (lcdDrawUpdate) {
      PGM_P pos_label;
      #if EXTRUDERS == 1
        pos_label = PSTR(MSG_MOVE_E);
      #elif EXTRUDERS > 1
        switch (eindex) {
          default: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E1); break;
          case 1: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E2); break;
          #if EXTRUDERS > 2
            case 2: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E3); break;
            #if EXTRUDERS > 3
              case 3: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E4); break;
              #if EXTRUDERS > 4
                case 4: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E5); break;
                #if EXTRUDERS > 5
                  case 5: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E6); break;
                #endif
              #endif
            #endif
          #endif
        }
      #endif
      lcd_implementation_drawedit(pos_label, ftostr41sign(mechanics.current_position[E_AXIS]));
    }
  }

  void lcd_move_e() { _lcd_move_e(); }
  #if EXTRUDERS > 1
    void lcd_move_e0() { _lcd_move_e(0); }
    void lcd_move_e1() { _lcd_move_e(1); }
    #if EXTRUDERS > 2
      void lcd_move_e2() { _lcd_move_e(2); }
      #if EXTRUDERS > 3
        void lcd_move_e3() { _lcd_move_e(3); }
        #if EXTRUDERS > 4
          void lcd_move_e4() { _lcd_move_e(4); }
          #if EXTRUDERS > 5
            void lcd_move_e5() { _lcd_move_e(5); }
          #endif
        #endif
      #endif
    #endif
  #endif

  /**
   *
   * "Prepare" > "Move Xmm" > "Move XYZ" submenu
   *
   */

  screenFunc_t _manual_move_func_ptr;

  void _goto_manual_move(const float scale) {
    defer_return_to_status = true;
    move_menu_scale = scale;
    lcd_goto_screen(_manual_move_func_ptr);
  }
  void lcd_move_menu_10mm() {  _goto_manual_move(10.0); }
  void lcd_move_menu_1mm()  {  _goto_manual_move( 1.0); }
  void lcd_move_menu_01mm() {  _goto_manual_move( 0.1); }
  void lcd_move_z_probe()   { move_menu_scale = LCD_Z_STEP ; lcd_goto_screen(lcd_move_z); }

  void _lcd_move_distance_menu(const AxisEnum axis, const screenFunc_t func) {
    _manual_move_func_ptr = func;
    START_MENU();
    if (LCD_HEIGHT >= 4) {
      switch(axis) {
        case X_AXIS:
          STATIC_ITEM(MSG_MOVE_X, true, true); break;
        case Y_AXIS:
          STATIC_ITEM(MSG_MOVE_Y, true, true); break;
        case Z_AXIS:
          STATIC_ITEM(MSG_MOVE_Z, true, true); break;
        default:
          STATIC_ITEM(MSG_MOVE_E, true, true); break;
      }
    }
    MENU_BACK(MSG_MOVE_AXIS);
    MENU_ITEM(submenu, MSG_MOVE_10MM, lcd_move_menu_10mm);
    MENU_ITEM(submenu, MSG_MOVE_1MM, lcd_move_menu_1mm);
    MENU_ITEM(submenu, MSG_MOVE_01MM, lcd_move_menu_01mm);
    END_MENU();
  }
  void lcd_move_get_x_amount()            { _lcd_move_distance_menu(X_AXIS, lcd_move_x); }
  void lcd_move_get_y_amount()            { _lcd_move_distance_menu(Y_AXIS, lcd_move_y); }
  void lcd_move_get_z_amount()            { _lcd_move_distance_menu(Z_AXIS, lcd_move_z); }
  void lcd_move_get_e_amount()            { _lcd_move_distance_menu(E_AXIS, lcd_move_e); }
  #if EXTRUDERS > 1
    void lcd_move_get_e0_amount()         { _lcd_move_distance_menu(E_AXIS, lcd_move_e0); }
    void lcd_move_get_e1_amount()         { _lcd_move_distance_menu(E_AXIS, lcd_move_e1); }
    #if EXTRUDERS > 2
      void lcd_move_get_e2_amount()       { _lcd_move_distance_menu(E_AXIS, lcd_move_e2); }
      #if EXTRUDERS > 3
        void lcd_move_get_e3_amount()     { _lcd_move_distance_menu(E_AXIS, lcd_move_e3); }
        #if EXTRUDERS > 4
          void lcd_move_get_e4_amount()   { _lcd_move_distance_menu(E_AXIS, lcd_move_e4); }
          #if EXTRUDERS > 5
            void lcd_move_get_e5_amount() { _lcd_move_distance_menu(E_AXIS, lcd_move_e5); }
          #endif
        #endif
      #endif
    #endif
  #endif

  /**
   *
   * "Prepare" > "Move Axis" submenu
   *
   */

  #if IS_KINEMATIC
    #define _MOVE_XYZ_ALLOWED (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
    #if MECH(DELTA)
      #define _MOVE_XY_ALLOWED (mechanics.current_position[Z_AXIS] <= mechanics.delta_clip_start_height)
      void lcd_lower_z_to_clip_height() {
        line_to_z(mechanics.delta_clip_start_height);
        lcd_synchronize();
      }
    #else
      #define _MOVE_XY_ALLOWED true
    #endif
  #else
    #define _MOVE_XYZ_ALLOWED true
    #define _MOVE_XY_ALLOWED true
  #endif

  void lcd_move_menu() {
    START_MENU();
    MENU_BACK(MSG_PREPARE);

    if (_MOVE_XYZ_ALLOWED) {
      if (_MOVE_XY_ALLOWED) {
        MENU_ITEM(submenu, MSG_MOVE_X, lcd_move_get_x_amount);
        MENU_ITEM(submenu, MSG_MOVE_Y, lcd_move_get_y_amount);
      }
      #if MECH(DELTA)
        else
          MENU_ITEM(function, MSG_FREE_XY, lcd_lower_z_to_clip_height);
      #endif

      MENU_ITEM(submenu, MSG_MOVE_Z, lcd_move_get_z_amount);
    }
    else
      MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));

    #if ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)
      if (tools.active_extruder)
        MENU_ITEM(gcode, MSG_SELECT MSG_E1, PSTR("T0"));
      else
        MENU_ITEM(gcode, MSG_SELECT MSG_E2, PSTR("T1"));
    #endif

    MENU_ITEM(submenu, MSG_MOVE_E, lcd_move_get_e_amount);
    #if EXTRUDERS > 1
      MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E1, lcd_move_get_e0_amount);
      MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E2, lcd_move_get_e1_amount);
      #if EXTRUDERS > 2
        MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E3, lcd_move_get_e2_amount);
        #if EXTRUDERS > 3
          MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E4, lcd_move_get_e3_amount);
          #if EXTRUDERS > 4
            MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E5, lcd_move_get_e4_amount);
            #if EXTRUDERS > 5
              MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E6, lcd_move_get_e5_amount);
            #endif
          #endif
        #endif
      #endif
    #endif

    END_MENU();
  }

  /**
   *
   * "Control" submenu
   *
   */

  #if HAS_LCD_CONTRAST
    void lcd_callback_set_contrast() { set_lcd_contrast(lcd_contrast); }
  #endif

  static void lcd_factory_settings() {
    eeprom.Factory_Settings();
    lcd_completion_feedback();
  }

  void lcd_control_menu() {
    START_MENU();
    MENU_BACK(MSG_MAIN);
    if (printer.mode == PRINTER_MODE_FFF)
      MENU_ITEM(submenu, MSG_TEMPERATURE, lcd_control_temperature_menu);
    MENU_ITEM(submenu, MSG_MOTION, lcd_control_motion_menu);
    if (printer.mode == PRINTER_MODE_FFF)
      MENU_ITEM(submenu, MSG_FILAMENT, lcd_control_filament_menu);

    #if HAS_LCD_CONTRAST
      MENU_ITEM_EDIT_CALLBACK(int3, MSG_CONTRAST, (int*)&lcd_contrast, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX, lcd_callback_set_contrast, true);
    #endif
    #if ENABLED(FWRETRACT)
      MENU_ITEM(submenu, MSG_RETRACT, lcd_control_retract_menu);
    #endif
    #if ENABLED(BLTOUCH)
      MENU_ITEM(submenu, MSG_BLTOUCH, bltouch_menu);
    #endif
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
      MENU_ITEM(function, MSG_LOAD_EEPROM, lcd_load_settings);
    #endif
    MENU_ITEM(function, MSG_RESTORE_FAILSAFE, lcd_factory_settings);

    END_MENU();
  }

  /**
   *
   * "Temperature" submenu
   *
   */

  #if ENABLED(PID_AUTOTUNE_MENU)

    #if (PIDTEMP)
      int16_t autotune_temp[HOTENDS] = ARRAY_BY_HOTENDS(200);
    #endif

    #if (PIDTEMPBED)
      int16_t autotune_temp_bed = 60;
    #endif

    void _lcd_autotune(int16_t h) {
      char cmd[30];
      sprintf_P(cmd, PSTR("M303 U1 H%i S%i"), h,
        #if HAS_PID_FOR_BOTH
          h < 0 ? autotune_temp_bed : autotune_temp[h]
        #elif (PIDTEMPBED)
          autotune_temp_bed
        #else
          autotune_temp[h]
        #endif
      );
      commands.enqueue_and_echo_command(cmd);
    }

  #endif //PID_AUTOTUNE_MENU

  #if (PIDTEMP)

    // Helpers for editing PID Ki & Kd values
    // grab the PID value out of the temp variable; scale it; then update the PID driver
    void copy_PID_i(int16_t h) {
      heaters[h].Ki = raw_Ki;
      thermalManager.updatePID();
    }
    void copy_PID_d(int16_t h) {
      heaters[h].Kd = raw_Kd;
      thermalManager.updatePID();
    }
    #define _DEFINE_PIDTEMP_BASE_FUNCS(N) \
      void copy_PID_i_H ## N() { copy_PID_i(N); } \
      void copy_PID_d_H ## N() { copy_PID_d(N); }

    #if ENABLED(PID_AUTOTUNE_MENU)
      #define DEFINE_PIDTEMP_FUNCS(N) \
        _DEFINE_PIDTEMP_BASE_FUNCS(N); \
        void lcd_autotune_callback_H ## N() { _lcd_autotune(N); } typedef void _pid_##N##_void
    #else
      #define DEFINE_PIDTEMP_FUNCS(N) _DEFINE_PIDTEMP_BASE_FUNCS(N) typedef void _pid_##N##_void
    #endif

    DEFINE_PIDTEMP_FUNCS(0);
    #if HOTENDS > 1
      DEFINE_PIDTEMP_FUNCS(1);
      #if HOTENDS > 2
        DEFINE_PIDTEMP_FUNCS(2);
        #if HOTENDS > 3
          DEFINE_PIDTEMP_FUNCS(3);
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1

  #endif // PIDTEMP

  /**
   *
   * "Control" > "Temperature" submenu
   *
   */
  void lcd_control_temperature_menu() {
    START_MENU();

    //
    // ^ Control
    //
    MENU_BACK(MSG_CONTROL);

    //
    // Nozzle:
    // Nozzle [1-4]:
    //
    #if HOTENDS == 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &heaters[0].target_temperature, 0, heaters[0].maxtemp - 15, watch_temp_callback_E0);
    #elif HOTENDS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &heaters[0].target_temperature, 0, heaters[0].maxtemp - 15, watch_temp_callback_E0);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &heaters[1].target_temperature, 0, heaters[1].maxtemp - 15, watch_temp_callback_E1);
      #if HOTENDS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &heaters[2].target_temperature, 0, heaters[2].maxtemp - 15, watch_temp_callback_E2);
        #if HOTENDS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &heaters[3].target_temperature, 0, heaters[3].maxtemp - 15, watch_temp_callback_E3);
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1

    //
    // Bed:
    //
    #if HAS_TEMP_BED
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_BED, &heaters[BED_INDEX].target_temperature, 0, BED_MAXTEMP - 15);
    #endif

    //
    // Chamber:
    //
    #if HAS_TEMP_CHAMBER
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_CHAMBER, &heaters[CHAMBER_INDEX].target_temperature, 0, CHAMBER_MAXTEMP - 15);
    #endif

    //
    // Cooler:
    //
    #if HAS_TEMP_COOLER
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_COOLER, &heaters[COOLER_INDEX].target_temperature, 0, COOLER_MAXTEMP - 15);
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
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_1ST_FAN_SPEED, &fans[0].Speed, 0, 255);
      #endif
      #if HAS_FAN1
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_FAN_SPEED " 1", &fans[1].Speed, 0, 255);
      #endif
      #if HAS_FAN2
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_FAN_SPEED " 2", &fans[2].Speed, 0, 255);
      #endif
      #if HAS_FAN3
        MENU_MULTIPLIER_ITEM_EDIT(uint3, MSG_FAN_SPEED " 3", &fans[3].Speed, 0, 255);
      #endif
    #endif // FAN_COUNT > 0

    //
    // Autotemp, Min, Max, Fact
    //
    #if ENABLED(AUTOTEMP) && (HAS_TEMP_0)
      MENU_ITEM_EDIT(bool, MSG_AUTOTEMP, &planner.autotemp_enabled);
      MENU_ITEM_EDIT(float3, MSG_MIN, &planner.autotemp_min, 0, heaters[0].maxtemp - 15);
      MENU_ITEM_EDIT(float3, MSG_MAX, &planner.autotemp_max, 0, heaters[0].maxtemp - 15);
      MENU_ITEM_EDIT(float32, MSG_FACTOR, &planner.autotemp_factor, 0.0, 1.0);
    #endif

    //
    // PID-P, PID-I, PID-D, PID-C, PID Autotune
    // PID-P H1, PID-I H1, PID-D H1, PID-C H1, PID Autotune H1
    // PID-P H2, PID-I H2, PID-D H2, PID-C H2, PID Autotune H2
    // PID-P H3, PID-I H3, PID-D H3, PID-C H3, PID Autotune H3
    // PID-P H4, PID-I H4, PID-D H4, PID-C H4, PID Autotune H4
    //
    #if (PIDTEMP)
      #define _PID_BASE_MENU_ITEMS(HLABEL, hindex) \
        raw_Ki = heaters[hindex].Ki; \
        raw_Kd = heaters[hindex].Kd; \
        MENU_ITEM_EDIT(float52, MSG_PID_P HLABEL, &heaters[hindex].Kp, 1, 9990); \
        MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I HLABEL, &raw_Ki, 0.01, 9990, copy_PID_i_H ## hindex); \
        MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D HLABEL, &raw_Kd, 1, 9990, copy_PID_d_H ## hindex)

      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        #define _PID_MENU_ITEMS(HLABEL, hindex) \
          _PID_BASE_MENU_ITEMS(HLABEL, hindex); \
          MENU_ITEM_EDIT(float3, MSG_PID_C HLABEL, &heaters[hindex].Kc, 1, 9990)
      #else
        #define _PID_MENU_ITEMS(HLABEL, hindex) _PID_BASE_MENU_ITEMS(HLABEL, hindex)
      #endif

      #if ENABLED(PID_AUTOTUNE_MENU)
        #define PID_MENU_ITEMS(HLABEL, hindex) \
          _PID_MENU_ITEMS(HLABEL, hindex); \
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE HLABEL, &autotune_temp[hindex], 150, heaters[hindex].maxtemp - 15, lcd_autotune_callback_H ## hindex)
      #else
        #define PID_MENU_ITEMS(HLABEL, hindex) _PID_MENU_ITEMS(HLABEL, hindex)
      #endif

      PID_MENU_ITEMS("", 0);
      #if HOTENDS > 1
        PID_MENU_ITEMS(MSG_H1, 1);
        #if HOTENDS > 2
          PID_MENU_ITEMS(MSG_H2, 2);
          #if HOTENDS > 3
            PID_MENU_ITEMS(MSG_H3, 3);
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1
    #endif // PIDTEMP

    //
    // Idle oozing
    //
    #if ENABLED(IDLE_OOZING_PREVENT)
      MENU_ITEM_EDIT(bool, MSG_IDLEOOZING, &printer.IDLE_OOZING_enabled);
    #endif

    //
    // Preheat Material 1 conf
    //
    MENU_ITEM(submenu, MSG_PREHEAT_1_SETTINGS, lcd_control_temperature_preheat_material1_settings_menu);

    //
    // Preheat Material 2 conf
    //
    MENU_ITEM(submenu, MSG_PREHEAT_2_SETTINGS, lcd_control_temperature_preheat_material2_settings_menu);

    //
    // Preheat Material 3 conf
    //
    MENU_ITEM(submenu, MSG_PREHEAT_3_SETTINGS, lcd_control_temperature_preheat_material3_settings_menu);
    END_MENU();
  }

  void _lcd_control_temperature_preheat_settings_menu(uint8_t material) {
    #if HOTENDS > 3
      #define MINTEMP_ALL MIN4(HEATER_0_MINTEMP, HEATER_1_MINTEMP, HEATER_2_MINTEMP, HEATER_3_MINTEMP)
      #define MAXTEMP_ALL MAX4(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP)
    #elif HOTENDS > 2
      #define MINTEMP_ALL MIN3(HEATER_0_MINTEMP, HEATER_1_MINTEMP, HEATER_2_MINTEMP)
      #define MAXTEMP_ALL MAX3(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP)
    #elif HOTENDS > 1
      #define MINTEMP_ALL min(HEATER_0_MINTEMP, HEATER_1_MINTEMP)
      #define MAXTEMP_ALL max(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP)
    #elif HOTENDS > 0
      #define MINTEMP_ALL HEATER_0_MINTEMP
      #define MAXTEMP_ALL HEATER_0_MAXTEMP
    #endif
    START_MENU();
    MENU_BACK(MSG_TEMPERATURE);
    MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &lcd_preheat_fan_speed[material], 0, 255);
    #if HAS_TEMP_0
      MENU_ITEM_EDIT(int3, MSG_NOZZLE, &lcd_preheat_hotend_temp[material], MINTEMP_ALL, MAXTEMP_ALL - 15);
    #endif
    #if HAS_TEMP_BED
      MENU_ITEM_EDIT(int3, MSG_BED, &lcd_preheat_bed_temp[material], BED_MINTEMP, BED_MAXTEMP - 15);
    #endif
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(function, MSG_STORE_EEPROM, lcd_store_settings);
    #endif
    END_MENU();
  }

  /**
   *
   * "Temperature" > "Preheat Material 1 conf" submenu
   *
   */
  void lcd_control_temperature_preheat_material1_settings_menu() { _lcd_control_temperature_preheat_settings_menu(0); }

  /**
   *
   * "Temperature" > "Preheat Material 2 conf" submenu
   *
   */
  void lcd_control_temperature_preheat_material2_settings_menu() { _lcd_control_temperature_preheat_settings_menu(1); }

  /**
   *
   * "Temperature" > "Preheat Material 3 conf" submenu
   *
   */
  void lcd_control_temperature_preheat_material3_settings_menu() { _lcd_control_temperature_preheat_settings_menu(2); }

  /**
   *
   * "Control" > "Motion" submenu
   *
   */
  void _reset_acceleration_rates() { mechanics.reset_acceleration_rates(); }
  #if EXTRUDERS > 1
    void _reset_e_acceleration_rate(const uint8_t e) { if (e == tools.active_extruder) _reset_acceleration_rates(); }
    void _reset_e0_acceleration_rate() { _reset_e_acceleration_rate(0); }
    void _reset_e1_acceleration_rate() { _reset_e_acceleration_rate(1); }
    #if EXTRUDERS > 2
      void _reset_e2_acceleration_rate() { _reset_e_acceleration_rate(2); }
      #if EXTRUDERS > 3
        void _reset_e3_acceleration_rate() { _reset_e_acceleration_rate(3); }
        #if EXTRUDERS > 4
          void _reset_e4_acceleration_rate() { _reset_e_acceleration_rate(4); }
          #if EXTRUDERS > 5
            void _reset_e5_acceleration_rate() { _reset_e_acceleration_rate(5); }
          #endif // EXTRUDERS > 5
        #endif // EXTRUDERS > 4
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  void _planner_refresh_positioning() { mechanics.refresh_positioning(); }
  #if EXTRUDERS > 1
    void _planner_refresh_e_positioning(const uint8_t e) {
      if (e == tools.active_extruder)
        _planner_refresh_positioning();
      else
        mechanics.steps_to_mm[E_AXIS + e] = 1.0 / mechanics.axis_steps_per_mm[E_AXIS + e];
    }
    void _planner_refresh_e0_positioning() { _planner_refresh_e_positioning(0); }
    void _planner_refresh_e1_positioning() { _planner_refresh_e_positioning(1); }
    #if EXTRUDERS > 2
      void _planner_refresh_e2_positioning() { _planner_refresh_e_positioning(2); }
      #if EXTRUDERS > 3
        void _planner_refresh_e3_positioning() { _planner_refresh_e_positioning(3); }
        #if EXTRUDERS > 4
          void _planner_refresh_e4_positioning() { _planner_refresh_e_positioning(4); }
          #if EXTRUDERS > 5
            void _planner_refresh_e5_positioning() { _planner_refresh_e_positioning(5); }
          #endif // EXTRUDERS > 5
        #endif // EXTRUDERS > 4
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  // M203 / M205 Velocity options
  void lcd_control_motion_velocity_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    // M203 Max Feedrate
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_X, &mechanics.max_feedrate_mm_s[X_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Y, &mechanics.max_feedrate_mm_s[Y_AXIS], 1, 999);
    MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Z, &mechanics.max_feedrate_mm_s[Z_AXIS], 1, 999);

    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &mechanics.max_feedrate_mm_s[E_AXIS + tools.active_extruder], 1, 999);
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E1, &mechanics.max_feedrate_mm_s[E_AXIS], 1, 999);
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E2, &mechanics.max_feedrate_mm_s[E_AXIS + 1], 1, 999);
      #if EXTRUDERS > 2
        MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E3, &mechanics.max_feedrate_mm_s[E_AXIS + 2], 1, 999);
        #if EXTRUDERS > 3
          MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E4, &mechanics.max_feedrate_mm_s[E_AXIS + 3], 1, 999);
          #if EXTRUDERS > 4
            MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E5, &mechanics.max_feedrate_mm_s[E_AXIS + 4], 1, 999);
            #if EXTRUDERS > 5
              MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E6, &mechanics.max_feedrate_mm_s[E_AXIS + 5], 1, 999);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &mechanics.max_feedrate_mm_s[E_AXIS], 1, 999);
    #endif

    // M205 S Min Feedrate
    MENU_ITEM_EDIT(float3, MSG_VMIN, &mechanics.min_feedrate_mm_s, 0, 999);

    // M205 T Min Travel Feedrate
    MENU_ITEM_EDIT(float3, MSG_VTRAV_MIN, &mechanics.min_travel_feedrate_mm_s, 0, 999);

    END_MENU();
  }

  // M201 / M204 Accelerations
  void lcd_control_motion_acceleration_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    // M204 P Acceleration
    MENU_ITEM_EDIT(float5, MSG_ACC, &mechanics.acceleration, 10, 99000);

    // M204 R Retract Acceleration
    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E, &Tools::retract_acceleration[Tools::active_extruder], 100, 99000);
      MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E1, &Tools::retract_acceleration[0], 100, 99000);
      MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E2, &Tools::retract_acceleration[1], 100, 99000);
      #if EXTRUDERS > 2
        MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E3, &Tools::retract_acceleration[2], 100, 99000);
        #if EXTRUDERS > 3
          MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E4, &Tools::retract_acceleration[3], 100, 99000);
          #if EXTRUDERS > 4
            MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E5, &Tools::retract_acceleration[4], 100, 99000);
            #if EXTRUDERS > 5
              MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E6, &Tools::retract_acceleration[5], 100, 99000);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #elif EXTRUDERS == 1
      MENU_ITEM_EDIT(float5, MSG_A_RETRACT MSG_E, &Tools::retract_acceleration[0], 100, 99000);
    #endif

    // M204 T Travel Acceleration
    MENU_ITEM_EDIT(float5, MSG_A_TRAVEL, &mechanics.travel_acceleration, 100, 99000);

    // M201 settings
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_X, &mechanics.max_acceleration_mm_per_s2[X_AXIS], 100, 99000, _reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Y, &mechanics.max_acceleration_mm_per_s2[Y_AXIS], 100, 99000, _reset_acceleration_rates);
    MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Z, &mechanics.max_acceleration_mm_per_s2[Z_AXIS], 10, 99000, _reset_acceleration_rates);

    #if EXTRUDERS > 1
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &mechanics.max_acceleration_mm_per_s2[E_AXIS + tools.active_extruder], 100, 99000, _reset_acceleration_rates);
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E1, &mechanics.max_acceleration_mm_per_s2[E_AXIS], 100, 99000, _reset_e0_acceleration_rate);
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E2, &mechanics.max_acceleration_mm_per_s2[E_AXIS + 1], 100, 99000, _reset_e1_acceleration_rate);
      #if EXTRUDERS > 2
        MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E3, &mechanics.max_acceleration_mm_per_s2[E_AXIS + 2], 100, 99000, _reset_e2_acceleration_rate);
        #if EXTRUDERS > 3
          MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E4, &mechanics.max_acceleration_mm_per_s2[E_AXIS + 3], 100, 99000, _reset_e3_acceleration_rate);
          #if EXTRUDERS > 4
            MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E5, &mechanics.max_acceleration_mm_per_s2[E_AXIS + 4], 100, 99000, _reset_e4_acceleration_rate);
            #if EXTRUDERS > 5
              MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E6, &mechanics.max_acceleration_mm_per_s2[E_AXIS + 5], 100, 99000, _reset_e5_acceleration_rate);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &mechanics.max_acceleration_mm_per_s2[E_AXIS], 100, 99000, _reset_acceleration_rates);
    #endif

    END_MENU();
  }

  // M205 Jerk
  void lcd_control_motion_jerk_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    MENU_ITEM_EDIT(float3, MSG_VX_JERK, &mechanics.max_jerk[X_AXIS], 1, 990);
    MENU_ITEM_EDIT(float3, MSG_VY_JERK, &mechanics.max_jerk[Y_AXIS], 1, 990);
    #if IS_DELTA
      MENU_ITEM_EDIT(float3, MSG_VZ_JERK, &mechanics.max_jerk[Z_AXIS], 1, 990);
    #else
      MENU_ITEM_EDIT(float52, MSG_VZ_JERK, &mechanics.max_jerk[Z_AXIS], 0.1, 990);
    #endif

    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float3, MSG_VE_JERK MSG_E, &mechanics.max_jerk[E_AXIS + tools.active_extruder], 1, 990);
      MENU_ITEM_EDIT(float3, MSG_VE_JERK MSG_E1, &mechanics.max_jerk[E_AXIS], 1, 990);
      MENU_ITEM_EDIT(float3, MSG_VE_JERK MSG_E2, &mechanics.max_jerk[E_AXIS + 1], 1, 990);
      #if EXTRUDERS > 2
        MENU_ITEM_EDIT(float3, MSG_VE_JERK MSG_E3, &mechanics.max_jerk[E_AXIS + 2], 1, 990);
        #if EXTRUDERS > 3
          MENU_ITEM_EDIT(float3, MSG_VE_JERK MSG_E4, &mechanics.max_jerk[E_AXIS + 3], 1, 990);
          #if EXTRUDERS > 4
            MENU_ITEM_EDIT(float3, MSG_VE_JERK MSG_E5, &mechanics.max_jerk[E_AXIS + 4], 1, 990);
            #if EXTRUDERS > 5
              MENU_ITEM_EDIT(float3, MSG_VE_JERK MSG_E6, &mechanics.max_jerk[E_AXIS + 5], 1, 990);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_ITEM_EDIT(float3, MSG_VE_JERK, &mechanics.max_jerk[E_AXIS], 1, 990);
    #endif

    END_MENU();
  }

  // M92 Steps-per-mm
  void lcd_control_motion_steps_per_mm_menu() {
    START_MENU();
    MENU_BACK(MSG_MOTION);

    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_XSTEPS, &mechanics.axis_steps_per_mm[X_AXIS], 5, 9999, _planner_refresh_positioning);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_YSTEPS, &mechanics.axis_steps_per_mm[Y_AXIS], 5, 9999, _planner_refresh_positioning);
    MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ZSTEPS, &mechanics.axis_steps_per_mm[Z_AXIS], 5, 9999, _planner_refresh_positioning);

    #if EXTRUDERS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ESTEPS, &mechanics.axis_steps_per_mm[E_AXIS + tools.active_extruder], 5, 9999, _planner_refresh_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E1STEPS, &mechanics.axis_steps_per_mm[E_AXIS], 5, 9999, _planner_refresh_e0_positioning);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E2STEPS, &mechanics.axis_steps_per_mm[E_AXIS + 1], 5, 9999, _planner_refresh_e1_positioning);
      #if EXTRUDERS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E3STEPS, &mechanics.axis_steps_per_mm[E_AXIS + 2], 5, 9999, _planner_refresh_e2_positioning);
        #if EXTRUDERS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E4STEPS, &mechanics.axis_steps_per_mm[E_AXIS + 3], 5, 9999, _planner_refresh_e3_positioning);
          #if EXTRUDERS > 4
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E5STEPS, &mechanics.axis_steps_per_mm[E_AXIS + 4], 5, 9999, _planner_refresh_e4_positioning);
            #if EXTRUDERS > 5
              MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_E6STEPS, &mechanics.axis_steps_per_mm[E_AXIS + 5], 5, 9999, _planner_refresh_e4_positioning);
            #endif // EXTRUDERS > 5
          #endif // EXTRUDERS > 4
        #endif // EXTRUDERS > 3
      #endif // EXTRUDERS > 2
    #else
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float62, MSG_ESTEPS, &mechanics.axis_steps_per_mm[E_AXIS], 5, 9999, _planner_refresh_positioning);
    #endif

    END_MENU();
  }

  void lcd_control_motion_menu() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);

    #if HAS_BED_PROBE
      MENU_ITEM_EDIT(float32, MSG_PROBE_OFFSET, &probe.offset[Z_AXIS], Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
    #endif

    // M203 / M205 - Feedrate items
    MENU_ITEM(submenu, MSG_VELOCITY, lcd_control_motion_velocity_menu);

    // M201 - Acceleration items
    MENU_ITEM(submenu, MSG_ACCELERATION, lcd_control_motion_acceleration_menu);

    // M205 - Max Jerk
    MENU_ITEM(submenu, MSG_JERK, lcd_control_motion_jerk_menu);

    // M92 - Steps Per mm
    MENU_ITEM(submenu, MSG_STEPS_PER_MM, lcd_control_motion_steps_per_mm_menu);

    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
      MENU_ITEM_EDIT(bool, MSG_ENDSTOP_ABORT, &stepper.abort_on_endstop_hit);
    #endif

    END_MENU();
  }

  /**
   *
   * "Control" > "Filament" submenu
   *
   */
  void lcd_control_filament_menu() {
    START_MENU();
    MENU_BACK(MSG_CONTROL);

    #if ENABLED(LIN_ADVANCE)
      MENU_ITEM_EDIT(float3, MSG_ADVANCE_K, &planner.extruder_advance_k, 0, 999);
    #endif

    MENU_ITEM_EDIT_CALLBACK(bool, MSG_VOLUMETRIC_ENABLED, &tools.volumetric_enabled, printer.calculate_volumetric_multipliers);

    if (tools.volumetric_enabled) {
      #if EXTRUDERS == 1
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM, &tools.filament_size[0], DEFAULT_NOMINAL_FILAMENT_DIA - .5, DEFAULT_NOMINAL_FILAMENT_DIA + .5, printer.calculate_volumetric_multipliers);
      #else // EXTRUDERS > 1
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E1, &tools.filament_size[0], DEFAULT_NOMINAL_FILAMENT_DIA - .5, DEFAULT_NOMINAL_FILAMENT_DIA + .5, printer.calculate_volumetric_multipliers);
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E2, &tools.filament_size[1], DEFAULT_NOMINAL_FILAMENT_DIA - .5, DEFAULT_NOMINAL_FILAMENT_DIA + .5, printer.calculate_volumetric_multipliers);
        #if EXTRUDERS > 2
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E3, &tools.filament_size[2], DEFAULT_NOMINAL_FILAMENT_DIA - .5, DEFAULT_NOMINAL_FILAMENT_DIA + .5, printer.calculate_volumetric_multipliers);
          #if EXTRUDERS > 3
            MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E4, &tools.filament_size[3], DEFAULT_NOMINAL_FILAMENT_DIA - .5, DEFAULT_NOMINAL_FILAMENT_DIA + .5, printer.calculate_volumetric_multipliers);
            #if EXTRUDERS > 4
              MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E5, &tools.filament_size[4], DEFAULT_NOMINAL_FILAMENT_DIA - .5, DEFAULT_NOMINAL_FILAMENT_DIA + .5, printer.calculate_volumetric_multipliers);
              #if EXTRUDERS > 5
                MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E6, &tools.filament_size[5], DEFAULT_NOMINAL_FILAMENT_DIA - .5, DEFAULT_NOMINAL_FILAMENT_DIA + .5, printer.calculate_volumetric_multipliers);
              #endif // EXTRUDERS > 5
            #endif // EXTRUDERS > 4
          #endif // EXTRUDERS > 3
        #endif // EXTRUDERS > 2
      #endif // EXTRUDERS > 1
    }

    END_MENU();
  }

  /**
   *
   * "Control" > "Retract" submenu
   *
   */
  #if ENABLED(FWRETRACT)

    void lcd_control_retract_menu() {
      START_MENU();
      MENU_BACK(MSG_CONTROL);
      MENU_ITEM_EDIT(bool, MSG_AUTORETRACT, &tools.autoretract_enabled);
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT, &retract_length, 0, 100);
      #if EXTRUDERS > 1
        MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_SWAP, &retract_length_swap, 0, 100);
      #endif
      MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACTF, &retract_feedrate_mm_s, 1, 999);
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_ZLIFT, &retract_zlift, 0, 999);
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER, &retract_recover_length, -100, 100);
      #if EXTRUDERS > 1
        MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER_SWAP, &retract_recover_length_swap, -100, 100);
      #endif
      MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVERF, &retract_recover_feedrate_mm_s, 1, 999);
      END_MENU();
    }

  #endif // FWRETRACT

  #if ENABLED(LASER)

    void lcd_laser_menu() {
      START_MENU();
      MENU_BACK(MSG_MAIN);
      MENU_ITEM(submenu, "Set Focus", lcd_laser_focus_menu);
      MENU_ITEM(submenu, "Test Fire", lcd_laser_test_fire_menu);
      #if ENABLED(LASER_PERIPHERALS)
        if (laser.peripherals_ok()) {
          MENU_ITEM(function, "Turn On Pumps/Fans", action_laser_acc_on);
        }
        else if (!(planner.movesplanned() || IS_SD_PRINTING)) {
          MENU_ITEM(function, "Turn Off Pumps/Fans", action_laser_acc_off);
        }
      #endif // LASER_PERIPHERALS
      END_MENU();
    }

    void lcd_laser_test_fire_menu() {
      START_MENU();
       MENU_BACK("Laser Functions");
       MENU_ITEM(function, "Weak ON", action_laser_test_weak);
       MENU_ITEM(function, " 20%  50ms", action_laser_test_20_50ms);
       MENU_ITEM(function, " 20% 100ms", action_laser_test_20_100ms);
       MENU_ITEM(function, "100%  50ms", action_laser_test_100_50ms);
       MENU_ITEM(function, "100% 100ms", action_laser_test_100_100ms);
       MENU_ITEM(function, "Warm-up Laser 2sec", action_laser_test_warm);
       END_MENU();
    }

    void action_laser_acc_on() { commands.enqueue_and_echo_commands_P(PSTR("M80")); }
    void action_laser_acc_off() { commands.enqueue_and_echo_commands_P(PSTR("M81")); }
    void action_laser_test_weak() { laser.fire(0.3); }
    void action_laser_test_20_50ms() { laser_test_fire(20, 50); }
    void action_laser_test_20_100ms() { laser_test_fire(20, 100); }
    void action_laser_test_100_50ms() { laser_test_fire(100, 50); }
    void action_laser_test_100_100ms() { laser_test_fire(100, 100); }
    void action_laser_test_warm() { laser_test_fire(15, 2000); }

    void laser_test_fire(uint8_t power, int dwell) {
      commands.enqueue_and_echo_commands_P(PSTR("M80"));  // Enable laser accessories since we don't know if its been done (and there's no penalty for doing it again).
      laser.fire(power);
      delay(dwell);
      laser.extinguish();
    }

    float focalLength = 0;
    void lcd_laser_focus_menu() {
      START_MENU();
      MENU_BACK("Laser Functions");
      MENU_ITEM(function, "1mm", action_laser_focus_1mm);
      MENU_ITEM(function, "2mm", action_laser_focus_2mm);
      MENU_ITEM(function, "3mm - 1/8in", action_laser_focus_3mm);
      MENU_ITEM(function, "4mm", action_laser_focus_4mm);
      MENU_ITEM(function, "5mm", action_laser_focus_5mm);
      MENU_ITEM(function, "6mm - 1/4in", action_laser_focus_6mm);
      MENU_ITEM(function, "7mm", action_laser_focus_7mm);
      MENU_ITEM_EDIT_CALLBACK(float32, "Custom", &focalLength, 0, LASER_FOCAL_HEIGHT, action_laser_focus_custom);
      END_MENU();
    }

    void action_laser_focus_custom() { laser_set_focus(focalLength); }
    void action_laser_focus_1mm() { laser_set_focus(1); }
    void action_laser_focus_2mm() { laser_set_focus(2); }
    void action_laser_focus_3mm() { laser_set_focus(3); }
    void action_laser_focus_4mm() { laser_set_focus(4); }
    void action_laser_focus_5mm() { laser_set_focus(5); }
    void action_laser_focus_6mm() { laser_set_focus(6); }
    void action_laser_focus_7mm() { laser_set_focus(7); }

    void laser_set_focus(float f_length) {
      if (!mechanics.axis_homed[Z_AXIS]) {
        commands.enqueue_and_echo_commands_P(PSTR("G28 Z F150"));
      }
      focalLength = f_length;
      float focus = LASER_FOCAL_HEIGHT - f_length;
      char cmd[20];

      sprintf_P(cmd, PSTR("G0 Z%s F150"), ftostr52sign(focus));
      commands.enqueue_and_echo_commands_P(cmd);
    }

  #endif // LASER

  #if HAS_SDSUPPORT

    #if !PIN_EXISTS(SD_DETECT)
      void lcd_sd_refresh() {
        card.mount();
        encoderTopLine = 0;
      }
    #endif

    void lcd_sd_updir() {
      card.updir();
      encoderTopLine = 0;
      screen_changed = true;
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    }

    /**
     *
     * "Print from SD" submenu
     *
     */
    void lcd_sdcard_menu() {
      ENCODER_DIRECTION_MENUS();
      if (!lcdDrawUpdate && !lcd_clicked) return; // nothing to do (so don't thrash the SD card)
      const uint16_t fileCnt = card.getnrfilenames();
      START_MENU();
      MENU_BACK(MSG_MAIN);
      card.getWorkDirName();
      if (card.fileName[0] == '/') {
        #if !PIN_EXISTS(SD_DETECT)
          MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
        #endif
      }
      else {
        MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);
      }

      for (uint16_t i = 0; i < fileCnt; i++) {
        if (_menuLineNr == _thisItemNr) {
          card.getfilename(
            #if ENABLED(SDCARD_RATHERRECENTFIRST)
              fileCnt-1 -
            #endif
            i
          );

          if (card.filenameIsDir)
            MENU_ITEM(sddirectory, MSG_CARD_MENU, card.fileName);
          else
            MENU_ITEM(sdfile, MSG_CARD_MENU, card.fileName);
        }
        else {
          MENU_ITEM_DUMMY();
        }
      }
      END_MENU();
    }

  #endif // SDSUPPORT

  #if ENABLED(LCD_INFO_MENU)
    /**
     *
     * About Printer > Statistics submenu
     *
     */

    void lcd_info_stats_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }

      uint16_t day, hours, minutes, kmeter, meter, centimeter;
      millis_t t;
      char lifeTime[20];
      char printTime[20];
      char Filamentlung[20];

      t       = printer.print_job_counter.data.printer_usage / 60;
      day     = t / 60 / 24;
      hours   = (t / 60) % 24;
      minutes = t % 60;
      sprintf_P(lifeTime, PSTR("%ud %uh %um"), day, hours, minutes);

      t       = printer.print_job_counter.data.printTime / 60;
      day     = t / 60 / 24;
      hours   = (t / 60) % 24;
      minutes = t % 60;
      sprintf_P(printTime, PSTR("%ud %uh %um"), day, hours, minutes);

      kmeter      = (long)printer.print_job_counter.data.filamentUsed / 1000 / 1000;
      meter       = ((long)printer.print_job_counter.data.filamentUsed / 1000) % 1000;
      centimeter  = ((long)printer.print_job_counter.data.filamentUsed / 10) % 100;
      sprintf_P(Filamentlung, PSTR("%uKm %um %ucm"), kmeter, meter, centimeter);

      #if HAS_POWER_CONSUMPTION_SENSOR
        char Power[10];
        sprintf_P(Power, PSTR("%uWh"), consumption_hour);
      #endif

      START_SCREEN();
      STATIC_ITEM(MSG_INFO_TOTAL_PRINTS ": ", false, false, itostr3left(printer.print_job_counter.data.totalPrints));
      STATIC_ITEM(MSG_INFO_FINISHED_PRINTS ": ",  false, false, itostr3left(printer.print_job_counter.data.finishedPrints));
      STATIC_ITEM(MSG_INFO_ON_TIME ": ",  false, false, lifeTime);
      STATIC_ITEM(MSG_INFO_PRINT_TIME ": ",  false, false, printTime);
      STATIC_ITEM(MSG_INFO_FILAMENT_USAGE ": ",  false, false, Filamentlung);
      #if HAS_POWER_CONSUMPTION_SENSOR
        STATIC_ITEM(MSG_INFO_PWRCONSUMED ": ",  false, false, Power);
      #endif
      END_SCREEN();
    }

    /**
     *
     * About Printer > Thermistors
     *
     */
    void lcd_info_thermistors_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }
      START_SCREEN();

      #if HAS_TEMP_0
        #define THERMISTOR_ID TEMP_SENSOR_0
        #include "../temperature/thermistornames.h"
        STATIC_ITEM("T0: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(heaters[0].mintemp), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(heaters[0].maxtemp), false);
      #endif

      #if HAS_TEMP_1
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_1
        #include "../temperature/thermistornames.h"
        STATIC_ITEM("T1: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(heaters[1].mintemp), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(heaters[1].maxtemp), false);
      #endif

      #if HAS_TEMP_2
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_2
        #include "../temperature/thermistornames.h"
        STATIC_ITEM("T2: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(heaters[2].mintemp), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(heaters[2].maxtemp), false);
      #endif

      #if HAS_TEMP_3
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_3
        #include "../temperature/thermistornames.h"
        STATIC_ITEM("T3: " THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(heaters[3].mintemp), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(heaters[3].maxtemp), false);
      #endif

      #if HAS_TEMP_BED
        #undef THERMISTOR_ID
        #define THERMISTOR_ID TEMP_SENSOR_BED
        #include "../temperature/thermistornames.h"
        STATIC_ITEM("TBed:" THERMISTOR_NAME, false, true);
        STATIC_ITEM(MSG_INFO_MIN_TEMP ": " STRINGIFY(BED_MINTEMP), false);
        STATIC_ITEM(MSG_INFO_MAX_TEMP ": " STRINGIFY(BED_MAXTEMP), false);
      #endif
      END_SCREEN();
    }

    /**
     *
     * About Printer > Board Info
     *
     */
    void lcd_info_board_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }
      START_SCREEN();
      STATIC_ITEM(BOARD_NAME, true, true);                           // MyPrinterController
      STATIC_ITEM(MSG_INFO_BAUDRATE ": " STRINGIFY(BAUDRATE), true); // Baud: 250000
      STATIC_ITEM(MSG_INFO_PROTOCOL ": " PROTOCOL_VERSION, true);    // Protocol: 1.0
      #if (POWER_SUPPLY == 0)
        STATIC_ITEM(MSG_INFO_PSU ": Normal", true); // Power Supply: Normal
      #elif (POWER_SUPPLY == 1)
        STATIC_ITEM(MSG_INFO_PSU ": ATX", true);    // Power Supply: ATX
      #elif (POWER_SUPPLY == 2)
        STATIC_ITEM(MSG_INFO_PSU ": XBox", true);   // Power Supply: XBox
      #endif
      END_SCREEN();
    }

    /**
     *
     * About Printer > Firmware Info
     *
     */
    void lcd_info_firmware_menu() {
      if (lcd_clicked) { return lcd_goto_previous_menu(); }
      START_SCREEN();
      STATIC_ITEM(FIRMWARE_NAME, true, true);
      STATIC_ITEM(SHORT_BUILD_VERSION, true);
      STATIC_ITEM(STRING_DISTRIBUTION_DATE, true);
      STATIC_ITEM(MACHINE_NAME, true);
      STATIC_ITEM(FIRMWARE_URL, true);
      STATIC_ITEM(MSG_INFO_EXTRUDERS ": " STRINGIFY(EXTRUDERS), true);
      STATIC_ITEM(MSG_INFO_HOTENDS ": " STRINGIFY(HOTENDS), true);
      END_SCREEN();
    }

    /**
     *
     * "About Printer" submenu
     *
     */
    void lcd_info_menu() {
      START_MENU();
      MENU_BACK(MSG_MAIN);
      MENU_ITEM(submenu, MSG_INFO_FIRMWARE_MENU, lcd_info_firmware_menu);       // Printer Info >
      MENU_ITEM(submenu, MSG_INFO_BOARD_MENU, lcd_info_board_menu);             // Board Info >
      MENU_ITEM(submenu, MSG_INFO_THERMISTOR_MENU, lcd_info_thermistors_menu);  // Thermistors >
      MENU_ITEM(submenu, MSG_INFO_STATS_MENU, lcd_info_stats_menu);             // Printer Statistics >
      END_MENU();
    }
  #endif // LCD_INFO_MENU

  /**
   *
   * Filament Change Feature Screens
   *
   */
  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    // Portions from STATIC_ITEM...
    #define HOTEND_STATUS_ITEM() do { \
      if (_menuLineNr == _thisItemNr) { \
        if (lcdDrawUpdate) { \
          lcd_implementation_drawmenu_static(_lcdLineNr, PSTR(MSG_FILAMENT_CHANGE_NOZZLE), false, true); \
          lcd_implementation_hotend_status(_lcdLineNr); \
        } \
        if (_skipStatic && encoderLine <= _thisItemNr) { \
          encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
          ++encoderLine; \
        } \
        lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT; \
      } \
      ++_thisItemNr; \
    } while(0)

    void lcd_advanced_pause_toocold_menu() {
      START_MENU();
      STATIC_ITEM(MSG_HEATING_FAILED_LCD, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_MINTEMP STRINGIFY(EXTRUDE_MINTEMP) ".", false, false);
      MENU_BACK(MSG_BACK);
      #if LCD_HEIGHT > 4
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_MENU();
    }

    void lcd_filament_change_resume_print() {
      printer.advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
    }

    void lcd_filament_change_extrude_more() {
      printer.advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
    }

    void lcd_advanced_pause_option_menu() {
      START_MENU();
      #if LCD_HEIGHT > 2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_OPTION_HEADER, true, false);
      #endif
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_RESUME, lcd_filament_change_resume_print);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_EXTRUDE, lcd_filament_change_extrude_more);
      END_MENU();
    }

    void lcd_advanced_pause_init_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_1);
      #ifdef MSG_FILAMENT_CHANGE_INIT_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_2);
        #define __FC_LINES_A 3
      #else
        #define __FC_LINES_A 2
      #endif
      #ifdef MSG_FILAMENT_CHANGE_INIT_3
        STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_3);
        #define _FC_LINES_A (__FC_LINES_A + 1)
      #else
        #define _FC_LINES_A __FC_LINES_A
      #endif
      #if LCD_HEIGHT > _FC_LINES_A + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_cool_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_COOL_1);
      #ifdef MSG_FILAMENT_CHANGE_COOL_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_COOL_2);
        #define __FC_LINES_A 3
      #else
        #define __FC_LINES_A 2
      #endif
      #ifdef MSG_FILAMENT_CHANGE_COOL_3
        STATIC_ITEM(MSG_FILAMENT_CHANGE_COOL_3);
        #define _FC_LINES_A (__FC_LINES_A + 1)
      #else
        #define _FC_LINES_A __FC_LINES_A
      #endif
      #if LCD_HEIGHT > _FC_LINES_A + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_unload_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_1);
      #ifdef MSG_FILAMENT_CHANGE_UNLOAD_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_2);
        #define __FC_LINES_B 3
      #else
        #define __FC_LINES_B 2
      #endif
      #ifdef MSG_FILAMENT_CHANGE_UNLOAD_3
        STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_3);
        #define _FC_LINES_B (__FC_LINES_B + 1)
      #else
        #define _FC_LINES_B __FC_LINES_B
      #endif
      #if LCD_HEIGHT > _FC_LINES_B + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_wait_for_nozzles_to_heat() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_1);
      #ifdef MSG_FILAMENT_CHANGE_HEATING_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_2);
        #define _FC_LINES_C 3
      #else
        #define _FC_LINES_C 2
      #endif
      #if LCD_HEIGHT > _FC_LINES_C + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_heat_nozzle() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEAT_1);
      #ifdef MSG_FILAMENT_CHANGE_HEAT_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_HEAT_2);
        #define _FC_LINES_D 3
      #else
        #define _FC_LINES_D 2
      #endif
      #if LCD_HEIGHT > _FC_LINES_D + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_printer_off() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_ZZZ_1);
      #ifdef MSG_FILAMENT_CHANGE_ZZZ_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_ZZZ_2);
        #define _FC_LINES_D 3
      #else
        #define _FC_LINES_D 2
      #endif
      #if LCD_HEIGHT > _FC_LINES_D + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_insert_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_1);
      #ifdef MSG_FILAMENT_CHANGE_INSERT_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_2);
        #define __FC_LINES_E 3
      #else
        #define __FC_LINES_E 2
      #endif
      #ifdef MSG_FILAMENT_CHANGE_INSERT_3
        STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_3);
        #define _FC_LINES_E (__FC_LINES_E + 1)
      #else
        #define _FC_LINES_E __FC_LINES_E
      #endif
      #if LCD_HEIGHT > _FC_LINES_E + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_load_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_1);
      #ifdef MSG_FILAMENT_CHANGE_LOAD_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_2);
        #define __FC_LINES_F 3
      #else
        #define __FC_LINES_F 2
      #endif
      #ifdef MSG_FILAMENT_CHANGE_LOAD_3
        STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_3);
        #define _FC_LINES_F (__FC_LINES_F + 1)
      #else
        #define _FC_LINES_F __FC_LINES_F
      #endif
      #if LCD_HEIGHT > _FC_LINES_F + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_extrude_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_1);
      #ifdef MSG_FILAMENT_CHANGE_EXTRUDE_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_2);
        #define __FC_LINES_G 3
      #else
        #define __FC_LINES_G 2
      #endif
      #ifdef MSG_FILAMENT_CHANGE_EXTRUDE_3
        STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_3);
        #define _FC_LINES_G (__FC_LINES_G + 1)
      #else
        #define _FC_LINES_G __FC_LINES_G
      #endif
      #if LCD_HEIGHT > _FC_LINES_G + 1
        STATIC_ITEM(" ");
      #endif
      HOTEND_STATUS_ITEM();
      END_SCREEN();
    }

    void lcd_advanced_pause_resume_message() {
      START_SCREEN();
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEADER, true, true);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_1);
      #ifdef MSG_FILAMENT_CHANGE_RESUME_2
        STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_2);
      #endif
      #ifdef MSG_FILAMENT_CHANGE_RESUME_3
        STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_3);
      #endif
      END_SCREEN();
    }

    void lcd_advanced_pause_show_message(const AdvancedPauseMessage message) {
      switch (message) {
        case ADVANCED_PAUSE_MESSAGE_INIT:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_init_message);
          break;
        case ADVANCED_PAUSE_MESSAGE_COOLDOWN:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_cool_message);
          break;
        case ADVANCED_PAUSE_MESSAGE_UNLOAD:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_unload_message);
          break;
        case ADVANCED_PAUSE_MESSAGE_INSERT:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_insert_message);
          break;
        case ADVANCED_PAUSE_MESSAGE_LOAD:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_load_message);
          break;
        case ADVANCED_PAUSE_MESSAGE_EXTRUDE:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_extrude_message);
          break;
        case ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_heat_nozzle);
          break;
        case ADVANCED_PAUSE_MESSAGE_PRINTER_OFF:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_printer_off);
          break;
        case ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_wait_for_nozzles_to_heat);
          break;
        case ADVANCED_PAUSE_MESSAGE_OPTION:
          defer_return_to_status = true;
          printer.advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
          lcd_goto_screen(lcd_advanced_pause_option_menu);
          break;
        case ADVANCED_PAUSE_MESSAGE_RESUME:
          defer_return_to_status = true;
          lcd_goto_screen(lcd_advanced_pause_resume_message);
          break;
        case ADVANCED_PAUSE_MESSAGE_STATUS:
          lcd_return_to_status();
          break;
      }
    }

  #endif // ADVANCED_PAUSE_FEATURE

  /**
   *
   * Functions for editing single values
   *
   * The "DEFINE_MENU_EDIT_TYPE" macro generates the functions needed to edit a numerical value.
   *
   * For example, DEFINE_MENU_EDIT_TYPE(int16_t, int3, itostr3, 1) expands into these functions:
   *
   *   bool _menu_edit_int3();
   *   void menu_edit_int3(); // edit int16_t (interactively)
   *   void menu_edit_callback_int3(); // edit int16_t (interactively) with callback on completion
   *   void _menu_action_setting_edit_int3(const char * const pstr, int16_t * const ptr, const int16_t minValue, const int16_t maxValue);
   *   void menu_action_setting_edit_int3(const char * const pstr, int16_t * const ptr, const int16_t minValue, const int16_t maxValue);
   *   void menu_action_setting_edit_callback_int3(const char * const pstr, int16_t * const ptr, const int16_t minValue, const int16_t maxValue, const screenFunc_t callback, const bool live); // edit int16_t with callback
   *
   * You can then use one of the menu macros to present the edit interface:
   *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
   *
   * This expands into a more primitive menu item:
   *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   *
   * ...which calls:
   *       menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_percentage, 10, 999)
   */
  #define DEFINE_MENU_EDIT_TYPE(_type, _name, _strFunc, _scale) \
    bool _menu_edit_ ## _name() { \
      ENCODER_DIRECTION_NORMAL(); \
      if ((int32_t)encoderPosition < 0) encoderPosition = 0; \
      if ((int32_t)encoderPosition > maxEditValue) encoderPosition = maxEditValue; \
      if (lcdDrawUpdate) \
        lcd_implementation_drawedit(editLabel, _strFunc(((_type)((int32_t)encoderPosition + minEditValue)) * (1.0 / _scale))); \
      if (lcd_clicked || (liveEdit && lcdDrawUpdate)) { \
        _type value = ((_type)((int32_t)encoderPosition + minEditValue)) * (1.0 / _scale); \
        if (editValue != NULL) *((_type*)editValue) = value; \
        if (liveEdit) (*callbackFunc)(); \
        if (lcd_clicked) lcd_goto_previous_menu(); \
      } \
      return lcd_clicked; \
    } \
    void menu_edit_ ## _name() { _menu_edit_ ## _name(); } \
    void menu_edit_callback_ ## _name() { if (_menu_edit_ ## _name()) (*callbackFunc)(); } \
    void _menu_action_setting_edit_ ## _name(const char * const pstr, _type* const ptr, const _type minValue, const _type maxValue) { \
      lcd_save_previous_screen(); \
      \
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; \
      \
      editLabel = pstr; \
      editValue = ptr; \
      minEditValue = minValue * _scale; \
      maxEditValue = maxValue * _scale - minEditValue; \
      encoderPosition = (*ptr) * _scale - minEditValue; \
    } \
    void menu_action_setting_edit_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue) { \
      _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
      currentScreen = menu_edit_ ## _name; \
    } \
    void menu_action_setting_edit_callback_ ## _name(const char * const pstr, _type * const ptr, const _type minValue, const _type maxValue, const screenFunc_t callback, const bool live) { \
      _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
      currentScreen = menu_edit_callback_ ## _name; \
      callbackFunc = callback; \
      liveEdit = live; \
    } \
    typedef void _name

  DEFINE_MENU_EDIT_TYPE(int16_t, int3, itostr3, 1);
  DEFINE_MENU_EDIT_TYPE(uint16_t, uint3, itostr3, 1);
  DEFINE_MENU_EDIT_TYPE(uint8_t, int8, i8tostr3, 1);
  DEFINE_MENU_EDIT_TYPE(float, float3, ftostr3, 1.0);
  DEFINE_MENU_EDIT_TYPE(float, float32, ftostr32, 100.0);
  DEFINE_MENU_EDIT_TYPE(float, float43, ftostr43sign, 1000.0);
  DEFINE_MENU_EDIT_TYPE(float, float5, ftostr5rj, 0.01);
  DEFINE_MENU_EDIT_TYPE(float, float51, ftostr51sign, 10.0);
  DEFINE_MENU_EDIT_TYPE(float, float52, ftostr52sign, 100.0);
  DEFINE_MENU_EDIT_TYPE(float, float62, ftostr62rj, 100.0);
  DEFINE_MENU_EDIT_TYPE(uint32_t, long5, ftostr5rj, 0.01);

  /**
   *
   * Handlers for RepRap World Keypad input
   *
   */
  #if ENABLED(REPRAPWORLD_KEYPAD)

    void _reprapworld_keypad_move(const AxisEnum axis, const int16_t dir) {
      move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
      encoderPosition = dir;
      switch (axis) {
        case X_AXIS: lcd_move_x(); break;
        case Y_AXIS: lcd_move_y(); break;
        case Z_AXIS: lcd_move_z();
        default: break;
      }
    }
    void reprapworld_keypad_move_z_up()    { _reprapworld_keypad_move(Z_AXIS,  1); }
    void reprapworld_keypad_move_z_down()  { _reprapworld_keypad_move(Z_AXIS, -1); }
    void reprapworld_keypad_move_x_left()  { _reprapworld_keypad_move(X_AXIS, -1); }
    void reprapworld_keypad_move_x_right() { _reprapworld_keypad_move(X_AXIS,  1); }
    void reprapworld_keypad_move_y_up()    { _reprapworld_keypad_move(Y_AXIS, -1); }
    void reprapworld_keypad_move_y_down()  { _reprapworld_keypad_move(Y_AXIS,  1); }
    void reprapworld_keypad_move_home()    { commands.enqueue_and_echo_commands_P(PSTR("G28")); } // move all axes home and wait
    void reprapworld_keypad_move_menu()    { lcd_goto_screen(lcd_move_menu); }

    inline void handle_reprapworld_keypad() {

      static uint8_t keypad_debounce = 0;

      if (!REPRAPWORLD_KEYPAD_PRESSED) {
        if (keypad_debounce > 0) keypad_debounce--;
      }
      else if (!keypad_debounce) {
        keypad_debounce = 2;

        if (REPRAPWORLD_KEYPAD_MOVE_MENU)       reprapworld_keypad_move_menu();

        #if NOMECH(DELTA) && Z_HOME_DIR == -1
          if (REPRAPWORLD_KEYPAD_MOVE_Z_UP)     reprapworld_keypad_move_z_up();
        #endif

        if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS]) {
          #if MECH(DELTA) || Z_HOME_DIR != -1
            if (REPRAPWORLD_KEYPAD_MOVE_Z_UP)   reprapworld_keypad_move_z_up();
          #endif
          if (REPRAPWORLD_KEYPAD_MOVE_Z_DOWN)   reprapworld_keypad_move_z_down();
          if (REPRAPWORLD_KEYPAD_MOVE_X_LEFT)   reprapworld_keypad_move_x_left();
          if (REPRAPWORLD_KEYPAD_MOVE_X_RIGHT)  reprapworld_keypad_move_x_right();
          if (REPRAPWORLD_KEYPAD_MOVE_Y_DOWN)   reprapworld_keypad_move_y_down();
          if (REPRAPWORLD_KEYPAD_MOVE_Y_UP)     reprapworld_keypad_move_y_up();
        }
        else {
          if (REPRAPWORLD_KEYPAD_MOVE_HOME)     reprapworld_keypad_move_home();
        }
      }
    }

  #elif ENABLED(ADC_KEYPAD) // ADC_KEYPAD

    inline bool handle_adc_keypad() {
      static uint8_t adc_steps = 0;
      if (buttons_reprapworld_keypad) {
        if (adc_steps < 20) ++adc_steps;
        lcd_quick_feedback();
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        if (encoderDirection == -1) { // side effect which signals we are inside a menu
          if      (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_DOWN)  encoderPosition -= ENCODER_STEPS_PER_MENU_ITEM;
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_UP)    encoderPosition += ENCODER_STEPS_PER_MENU_ITEM;
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_LEFT)  menu_action_back();
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_RIGHT) lcd_return_to_status();
        }
        else {
          const int8_t step = adc_steps > 19 ? 100 : adc_steps > 10 ? 10 : 1;
               if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_DOWN)  encoderPosition += ENCODER_PULSES_PER_STEP * step;
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_UP)    encoderPosition -= ENCODER_PULSES_PER_STEP * step;
          else if (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_RIGHT) encoderPosition = 0;
        }
        #if ENABLED(ADC_KEYPAD_DEBUG)
          SERIAL_EMV("buttons_reprapworld_keypad = ", (uint32_t)buttons_reprapworld_keypad);
          SERIAL_EMV("encoderPosition = ", (uint32_t)encoderPosition);
        #endif
        return true;
      }
      else if (!HAL::AnalogInputValues[ADC_KEYPAD_ANALOG_INDEX])
        adc_steps = 0; // reset stepping acceleration

      return false;
    }

  #endif

  /**
   *
   * Menu actions
   *
   */
  void _menu_action_back() { lcd_goto_previous_menu(); }
  void menu_action_submenu(screenFunc_t func) { lcd_save_previous_screen(); lcd_goto_screen(func); }
  void menu_action_gcode(const char* pgcode) { commands.enqueue_and_echo_commands_P(pgcode); }
  void menu_action_function(screenFunc_t func) { (*func)(); }

  #if HAS_SDSUPPORT

    void menu_action_sdfile(const char* longFilename) {
      card.openAndPrintFile(longFilename);
      lcd_return_to_status();
    }

    void menu_action_sddirectory(const char* longFilename) {
      card.chdir(longFilename);
      encoderPosition = 0;
      screen_changed = true;
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    }

  #endif // SDSUPPORT

  void menu_action_setting_edit_bool(const char* pstr, bool* ptr) { UNUSED(pstr); *ptr ^= true; lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; }
  void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, screenFunc_t callback) {
    menu_action_setting_edit_bool(pstr, ptr);
    (*callback)();
  }

#endif // ULTIPANEL

void lcd_init() {

  lcd_implementation_init(
    #if ENABLED(LCD_PROGRESS_BAR)
      true
    #endif
  );

  #if ENABLED(NEWPANEL)
    #if BUTTON_EXISTS(EN1)
      SET_INPUT_PULLUP(BTN_EN1);
    #endif

    #if BUTTON_EXISTS(EN2)
      SET_INPUT_PULLUP(BTN_EN2);
    #endif

    #if BUTTON_EXISTS(ENC)
      SET_INPUT_PULLUP(BTN_ENC);
    #endif

    #if ENABLED(REPRAPWORLD_KEYPAD)
      SET_OUTPUT(SHIFT_CLK);
      OUT_WRITE(SHIFT_LD, HIGH);
      SET_INPUT_PULLUP(SHIFT_OUT);
    #endif

    #if BUTTON_EXISTS(UP)
      SET_INPUT(BTN_UP);
    #endif
    #if BUTTON_EXISTS(DWN)
      SET_INPUT(BTN_DWN);
    #endif
    #if BUTTON_EXISTS(LFT)
      SET_INPUT(BTN_LFT);
    #endif
    #if BUTTON_EXISTS(RT)
      SET_INPUT(BTN_RT);
    #endif

  #else  // Not NEWPANEL

    #if ENABLED(SR_LCD_2W_NL) // Non latching 2 wire shift register
      SET_OUTPUT(SR_DATA_PIN);
      SET_OUTPUT(SR_CLK_PIN);
    #elif ENABLED(SHIFT_CLK)
      SET_OUTPUT(SHIFT_CLK);
      OUT_WRITE(SHIFT_LD, HIGH);
      OUT_WRITE(SHIFT_EN, LOW);
      SET_INPUT_PULLUP(SHIFT_OUT);
    #endif // SR_LCD_2W_NL

  #endif // !NEWPANEL

  #if HAS_SDSUPPORT && PIN_EXISTS(SD_DETECT)
    SET_INPUT_PULLUP(SD_DETECT_PIN);
    lcd_sd_status = 2; // UNKNOWN
  #endif

  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    slow_buttons = 0;
  #endif

  lcd_buttons_update();

  #if ENABLED(ULTIPANEL)
    encoderDiff = 0;
  #endif
}

int16_t lcd_strlen(const char* s) {
  int16_t i = 0, j = 0;
  while (s[i]) {
    if (PRINTABLE(s[i])) j++;
    i++;
  }
  return j;
}

int16_t lcd_strlen_P(const char* s) {
  int16_t j = 0;
  while (pgm_read_byte(s)) {
    if (PRINTABLE(pgm_read_byte(s))) j++;
    s++;
  }
  return j;
}

bool lcd_blink() {
  static uint8_t blink = 0;
  static millis_t next_blink_ms = 0;
  millis_t ms = millis();
  if (ELAPSED(ms, next_blink_ms)) {
    blink ^= 0xFF;
    next_blink_ms = ms + 1000 - LCD_UPDATE_INTERVAL / 2;
  }
  return blink != 0;
}

/**
 * Update the LCD, read encoder buttons, etc.
 *   - Read button states
 *   - Check the SD Card slot state
 *   - Act on RepRap World keypad input
 *   - Update the encoder position
 *   - Apply acceleration to the encoder position
 *   - Set lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NOW on controller events
 *   - Reset the Info Screen timeout if there's any input
 *   - Update status indicators, if any
 *
 *   Run the current LCD menu handler callback function:
 *   - Call the handler only if lcdDrawUpdate != LCDVIEW_NONE
 *   - Before calling the handler, LCDVIEW_CALL_NO_REDRAW => LCDVIEW_NONE
 *   - Call the menu handler. Menu handlers should do the following:
 *     - If a value changes, set lcdDrawUpdate to LCDVIEW_REDRAW_NOW and draw the value
 *       (Encoder events automatically set lcdDrawUpdate for you.)
 *     - if (lcdDrawUpdate) { redraw }
 *     - Before exiting the handler set lcdDrawUpdate to:
 *       - LCDVIEW_CLEAR_CALL_REDRAW to clear screen and set LCDVIEW_CALL_REDRAW_NEXT.
 *       - LCDVIEW_REDRAW_NOW to draw now (including remaining stripes).
 *       - LCDVIEW_CALL_REDRAW_NEXT to draw now and get LCDVIEW_REDRAW_NOW on the next loop.
 *       - LCDVIEW_CALL_NO_REDRAW to draw now and get LCDVIEW_NONE on the next loop.
 *     - NOTE: For graphical displays menu handlers may be called 2 or more times per loop,
 *             so don't change lcdDrawUpdate without considering this.
 *
 *   After the menu handler callback runs (or not):
 *   - Clear the LCD if lcdDrawUpdate == LCDVIEW_CLEAR_CALL_REDRAW
 *   - Update lcdDrawUpdate for the next loop (i.e., move one state down, usually)
 *
 * No worries. This function is only called from the main thread.
 */
void lcd_update() {

  #if ENABLED(ULTIPANEL)
    static millis_t return_to_status_ms = 0;
    manage_manual_move();

    lcd_buttons_update();

    // If the action button is pressed...
    if (LCD_CLICKED) {
      if (!wait_for_unclick) {           // If not waiting for a debounce release:
        wait_for_unclick = true;         //  Set debounce flag to ignore continous clicks
        lcd_clicked = !printer.wait_for_user;    //  Keep the click if not waiting for a user-click
        printer.wait_for_user = false;           //  Any click clears wait for user
        lcd_quick_feedback();            //  Always make a click sound
      }
    }
    else wait_for_unclick = false;
  #endif

  #if HAS_SDSUPPORT && PIN_EXISTS(SD_DETECT)

    const bool sd_status = IS_SD_INSERTED;
    if (sd_status != lcd_sd_status && lcd_detected()) {

      if (sd_status) {
        card.mount();
        if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_INSERTED);
      }
      else {
        card.unmount();
        if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_REMOVED);
      }

      lcd_sd_status = sd_status;
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
      lcd_implementation_init( // to maybe revive the LCD if static electricity killed it.
        #if ENABLED(LCD_PROGRESS_BAR)
          currentScreen == lcd_status_screen
        #endif
      );
    }

  #endif // SDSUPPORT && SD_DETECT_PIN

  const millis_t ms = millis();
  if (ELAPSED(ms, next_lcd_update_ms)
    #if ENABLED(DOGLCD)
      || drawing_screen
    #endif
    ) {

    next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;

    #if ENABLED(LCD_HAS_STATUS_INDICATORS)
      lcd_implementation_update_indicators();
    #endif

    #if ENABLED(ULTIPANEL)

      #if ENABLED(LCD_HAS_SLOW_BUTTONS)
        slow_buttons = lcd_implementation_read_slow_buttons(); // buttons which take too long to read in interrupt context
      #endif

      #if ENABLED(REPRAPWORLD_KEYPAD)

        handle_reprapworld_keypad();

      #elif ENABLED(ADC_KEYPAD)

        if (handle_adc_keypad())
          return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;

      #endif

      bool encoderPastThreshold = (abs(encoderDiff) >= ENCODER_PULSES_PER_STEP);
      if (encoderPastThreshold || lcd_clicked) {
        if (encoderPastThreshold) {
          int32_t encoderMultiplier = 1;

          #if ENABLED(ENCODER_RATE_MULTIPLIER)

            if (encoderRateMultiplierEnabled) {
              int32_t encoderMovementSteps = abs(encoderDiff) / ENCODER_PULSES_PER_STEP;

              if (lastEncoderMovementMillis != 0) {
                // Note that the rate is always calculated between to passes through the
                // loop and that the abs of the encoderDiff value is tracked.
                float encoderStepRate = (float)(encoderMovementSteps) / ((float)(ms - lastEncoderMovementMillis)) * 1000.0;

                if (encoderStepRate >= ENCODER_100X_STEPS_PER_SEC)     encoderMultiplier = 100;
                else if (encoderStepRate >= ENCODER_10X_STEPS_PER_SEC) encoderMultiplier = 10;

                #if ENABLED(ENCODER_RATE_MULTIPLIER_DEBUG)
                  SERIAL_SMV(DEB, "Enc Step Rate: ", encoderStepRate);
                  SERIAL_MV("  Multiplier: ", encoderMultiplier);
                  SERIAL_MV("  ENCODER_10X_STEPS_PER_SEC: ", ENCODER_10X_STEPS_PER_SEC);
                  SERIAL_EMV("  ENCODER_100X_STEPS_PER_SEC: ", ENCODER_100X_STEPS_PER_SEC);
                #endif
              }

              lastEncoderMovementMillis = ms;
            } // encoderRateMultiplierEnabled
          #endif // ENCODER_RATE_MULTIPLIER

          encoderPosition += (encoderDiff * encoderMultiplier) / ENCODER_PULSES_PER_STEP;
          encoderDiff = 0;
        }
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }
    #endif // ULTIPANEL

    // We arrive here every ~100ms when idling often enough.
    // Instead of tracking the changes simply redraw the Info Screen ~1 time a second.
    static int8_t lcd_status_update_delay = 1; // first update one loop delayed
    if (
      #if ENABLED(ULTIPANEL)
        currentScreen == lcd_status_screen &&
      #endif
      !lcd_status_update_delay--
    ) {
      lcd_status_update_delay = 9
        #if ENABLED(DOGLCD)
          + 3
        #endif
      ;
      max_display_update_time--;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }

    // then we want to use 1/2 of the time only.
    uint16_t bbr2 = planner.block_buffer_runtime() >> 1;

    #if ENABLED(DOGLCD)
      if ((lcdDrawUpdate || drawing_screen) && (!bbr2 || (bbr2 > max_display_update_time)))
    #else
      if (lcdDrawUpdate && (!bbr2 || (bbr2 > max_display_update_time)))
    #endif
    {
      #if ENABLED(DOGLCD)
        if (!drawing_screen)
      #endif
        {
          switch (lcdDrawUpdate) {
            case LCDVIEW_CALL_NO_REDRAW:
              lcdDrawUpdate = LCDVIEW_NONE;
              break;
            case LCDVIEW_CLEAR_CALL_REDRAW: // set by handlers, then altered after (rarely occurs here)
            case LCDVIEW_CALL_REDRAW_NEXT:  // set by handlers, then altered after (never occurs here?)
              lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
            case LCDVIEW_REDRAW_NOW:        // set above, or by a handler through LCDVIEW_CALL_REDRAW_NEXT
            case LCDVIEW_NONE:
              break;
          } // switch
        }

      #if ENABLED(ADC_KEYPAD)
        buttons_reprapworld_keypad = 0;
      #endif

      #if ENABLED(ULTIPANEL)
        #define CURRENTSCREEN() (*currentScreen)(), lcd_clicked = false
      #else
        #define CURRENTSCREEN() lcd_status_screen()
      #endif

      #if ENABLED(DOGLCD)  // Changes due to different driver architecture of the DOGM display
        if (!drawing_screen) {
          u8g.firstPage();
          drawing_screen = 1;
        }
        lcd_setFont(FONT_MENU);
        u8g.setColorIndex(1);
        CURRENTSCREEN();
        if (drawing_screen && (drawing_screen = u8g.nextPage())) {
          NOLESS(max_display_update_time, millis() - ms);
          return;
        }
      #else
        CURRENTSCREEN();
      #endif
      NOLESS(max_display_update_time, millis() - ms);
    }

    #if ENABLED(ULTIPANEL)

      // Return to Status Screen after a timeout
      if (currentScreen == lcd_status_screen || defer_return_to_status)
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
      else if (ELAPSED(ms, return_to_status_ms))
        lcd_return_to_status();

    #endif // ULTIPANEL

    #if ENABLED(DOGLCD)
      if (!drawing_screen)
    #endif
      {
        switch (lcdDrawUpdate) {
          case LCDVIEW_CLEAR_CALL_REDRAW:
            lcd_implementation_clear();
          case LCDVIEW_CALL_REDRAW_NEXT:
            lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
            break;
          case LCDVIEW_REDRAW_NOW:
            lcdDrawUpdate = LCDVIEW_NONE;
            break;
          case LCDVIEW_NONE:
            break;
        } // switch
      }
  } // ELAPSED(ms, next_lcd_update_ms)
}

void pad_message_string() {
  uint8_t i = 0, j = 0;
  char c;
  while ((c = lcd_status_message[i]) && j < LCD_WIDTH) {
    if (PRINTABLE(c)) j++;
    i++;
  }
  if (true
    #if ENABLED(STATUS_MESSAGE_SCROLLING)
      && j < LCD_WIDTH
    #endif
  ) {
    // pad with spaces to fill up the line
    while (j++ < LCD_WIDTH) lcd_status_message[i++] = ' ';
    // chop off at the edge
    lcd_status_message[i] = '\0';
  }
}

void lcd_finishstatus(const bool persist=false) {

  pad_message_string();

  #if !(ENABLED(LCD_PROGRESS_BAR) && (PROGRESS_MSG_EXPIRE > 0))
    UNUSED(persist);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    progress_bar_ms = millis();
    #if PROGRESS_MSG_EXPIRE > 0
      expire_status_ms = persist ? 0 : progress_bar_ms + PROGRESS_MSG_EXPIRE;
    #endif
  #endif
  lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

  #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
    previous_lcd_status_ms = millis();  // get status message to show up for a while
  #endif

  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    status_scroll_pos = 0;
  #endif
}

#if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
  void dontExpireStatus() { expire_status_ms = 0; }
#endif

bool lcd_hasstatus() { return (lcd_status_message[0] != '\0'); }

void lcd_setstatus(const char * const message, const bool persist) {
  if (lcd_status_message_level > 0) return;
  strncpy(lcd_status_message, message, 3 * (LCD_WIDTH));
  lcd_finishstatus(persist);
}

void lcd_setstatusPGM(const char * const message, int8_t level) {
  if (level < 0) level = lcd_status_message_level = 0;
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;
  strncpy_P(lcd_status_message, message, 3 * (LCD_WIDTH));
  lcd_finishstatus(level > 0);
}

void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...) {
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(lcd_status_message, 3 * (LCD_WIDTH), fmt, args);
  va_end(args);
  lcd_finishstatus(level > 0);
}

void lcd_setalertstatusPGM(const char * const message) {
  lcd_setstatusPGM(message, 1);
  #if ENABLED(ULTIPANEL)
    lcd_return_to_status();
  #endif
}

void lcd_reset_alert_level() { lcd_status_message_level = 0; }

#if HAS_LCD_CONTRAST
  void set_lcd_contrast(const uint16_t value) {
    lcd_contrast = constrain(value, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX);
    u8g.setContrast(lcd_contrast);
  }
#endif

#if ENABLED(ULTIPANEL)

  /**
   * Setup Rotary Encoder Bit Values (for two pin encoders to indicate movement)
   * These values are independent of which pins are used for EN_A and EN_B indications
   * The rotary encoder part is also independent to the chipset used for the LCD
   */
  #if ENABLED(EN_A) && ENABLED(EN_B)
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #endif

  #define GET_BUTTON_STATES(DST) \
    uint8_t new_##DST = 0; \
    WRITE(SHIFT_LD, LOW); \
    WRITE(SHIFT_LD, HIGH); \
    for (int8_t i = 0; i < 8; i++) { \
      new_##DST >>= 1; \
      if (READ(SHIFT_OUT)) SBI(new_##DST, 7); \
      WRITE(SHIFT_CLK, HIGH); \
      WRITE(SHIFT_CLK, LOW); \
    } \
    DST = ~new_##DST; //invert it, because a pressed switch produces a logical 0


  /**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
  void lcd_buttons_update() {
    static uint8_t lastEncoderBits;
    millis_t now = millis();
    if (ELAPSED(now, next_button_update_ms)) {

      #if ENABLED(NEWPANEL)
        uint8_t newbutton = 0;

        #if BUTTON_EXISTS(EN1)
          if (BUTTON_PRESSED(EN1)) newbutton |= EN_A;
        #endif

        #if BUTTON_EXISTS(EN2)
          if (BUTTON_PRESSED(EN2)) newbutton |= EN_B;
        #endif

        #if BUTTON_EXISTS(ENC)
          if (BUTTON_PRESSED(ENC)) newbutton |= EN_C;
        #endif

        #if LCD_HAS_DIRECTIONAL_BUTTONS

          // Manage directional buttons
          #if ENABLED(REVERSE_MENU_DIRECTION)
            #define _ENCODER_UD_STEPS (ENCODER_STEPS_PER_MENU_ITEM * encoderDirection)
          #else
            #define _ENCODER_UD_STEPS ENCODER_STEPS_PER_MENU_ITEM
          #endif
          #if ENABLED(REVERSE_ENCODER_DIRECTION)
            #define ENCODER_UD_STEPS _ENCODER_UD_STEPS
            #define ENCODER_LR_PULSES ENCODER_PULSES_PER_STEP
          #else
            #define ENCODER_UD_STEPS -(_ENCODER_UD_STEPS)
            #define ENCODER_LR_PULSES -(ENCODER_PULSES_PER_STEP)
          #endif

          if (false) {
            // for the else-ifs below
          }
          #if BUTTON_EXISTS(UP)
            else if (BUTTON_PRESSED(UP)) {
              encoderDiff = -(ENCODER_UD_STEPS);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(DWN)
            else if (BUTTON_PRESSED(DWN)) {
              encoderDiff = ENCODER_UD_STEPS;
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(LFT)
            else if (BUTTON_PRESSED(LFT)) {
              encoderDiff = -(ENCODER_LR_PULSES);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(RT)
            else if (BUTTON_PRESSED(RT)) {
              encoderDiff = ENCODER_LR_PULSES;
              next_button_update_ms = now + 300;
            }
          #endif

        #endif // LCD_HAS_DIRECTIONAL_BUTTONS

        buttons = newbutton;
        #if ENABLED(LCD_HAS_SLOW_BUTTONS)
          buttons |= slow_buttons;
        #endif

        #if ENABLED(ADC_KEYPAD)
          
          uint8_t newbutton_reprapworld_keypad = 0;
          buttons = 0;
          if (buttons_reprapworld_keypad == 0) {
            newbutton_reprapworld_keypad = get_ADC_keyValue();
            if (WITHIN(newbutton_reprapworld_keypad, 1, 8))
              buttons_reprapworld_keypad = _BV(newbutton_reprapworld_keypad - 1);
          }

        #elif ENABLED(REPRAPWORLD_KEYPAD)

          GET_BUTTON_STATES(buttons_reprapworld_keypad);

        #endif

      #else
        GET_BUTTON_STATES(buttons);
      #endif // !NEWPANEL

    } // next_button_update_ms

    // Manage encoder rotation
    #if ENABLED(REVERSE_MENU_DIRECTION) && ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff -= encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff += encoderDirection)
    #elif ENABLED(REVERSE_MENU_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff += encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff -= encoderDirection)
    #elif ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff--)
      #define ENCODER_DIFF_CCW (encoderDiff++)
    #else
      #define ENCODER_DIFF_CW  (encoderDiff++)
      #define ENCODER_DIFF_CCW (encoderDiff--)
    #endif
    #define ENCODER_SPIN(_E1, _E2) switch (lastEncoderBits) { case _E1: ENCODER_DIFF_CW; break; case _E2: ENCODER_DIFF_CCW; }

    uint8_t enc = 0;
    if (buttons & EN_A) enc |= B01;
    if (buttons & EN_B) enc |= B10;
    if (enc != lastEncoderBits) {
      switch (enc) {
        case encrot0: ENCODER_SPIN(encrot3, encrot1); break;
        case encrot1: ENCODER_SPIN(encrot0, encrot2); break;
        case encrot2: ENCODER_SPIN(encrot1, encrot3); break;
        case encrot3: ENCODER_SPIN(encrot2, encrot0); break;
      }
      lastEncoderBits = enc;
    }
  }

  #if (ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)) && ENABLED(DETECT_DEVICE)
    bool lcd_detected() { return lcd.LcdDetected() == 1; }
  #else
    bool lcd_detected() { return true; }
  #endif

#endif // ULTIPANEL

#endif // ULTRA_LCD

#if ENABLED(ADC_KEYPAD)

  typedef struct {
    uint16_t ADCKeyValueMin, ADCKeyValueMax;
    uint8_t  ADCKeyNo;
  } _stADCKeypadTable_;

  static const _stADCKeypadTable_ stADCKeyTable[] PROGMEM = {
    // VALUE_MIN, VALUE_MAX , KEY
    { 250, 256, BLEN_REPRAPWORLD_KEYPAD_F1 + 1 },     // F1
    { 250, 256, BLEN_REPRAPWORLD_KEYPAD_F2 + 1 },     // F2
    { 250, 256, BLEN_REPRAPWORLD_KEYPAD_F3 + 1 },     // F3
    {  18,  32, BLEN_REPRAPWORLD_KEYPAD_LEFT + 1 },   // LEFT
    { 118, 138, BLEN_REPRAPWORLD_KEYPAD_RIGHT + 1 },  // RIGHT
    {  34,  54, BLEN_REPRAPWORLD_KEYPAD_UP + 1 },     // UP
    { 166, 180, BLEN_REPRAPWORLD_KEYPAD_DOWN + 1 },   // DOWN
    {  70,  90, BLEN_REPRAPWORLD_KEYPAD_MIDDLE + 1 }, // ENTER
  };

  uint8_t get_ADC_keyValue(void) {
    const uint16_t currentkpADCValue = (HAL::AnalogInputValues[ADC_KEYPAD_ANALOG_INDEX] >> 2);
    #if ENABLED(ADC_KEYPAD_DEBUG)
      SERIAL_EV(currentkpADCValue);
    #endif
    if (currentkpADCValue < 250) {
      for (uint8_t i = 0; i < ADC_KEY_NUM; i++) {
        const uint16_t lo = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMin),
                       hi = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMax);
        if (WITHIN(currentkpADCValue, lo, hi)) return pgm_read_byte(&stADCKeyTable[i].ADCKeyNo);
      }
    }
    return 0;
  }

#endif

#if HAS_SDSUPPORT
  void set_sd_dot() {
    #if ENABLED(DOGLCD)
      u8g.firstPage();
      do {
        u8g.setColorIndex(1);
        u8g.drawPixel(0, 0); // draw sd dot
        u8g.setColorIndex(1); // black on white
        (*currentScreen)();
      } while( u8g.nextPage() );
    #endif
  }
  void unset_sd_dot() {
    #if ENABLED(DOGLCD)
      u8g.firstPage();
      do {
        u8g.setColorIndex(0);
        u8g.drawPixel(0, 0); // draw sd dot
        u8g.setColorIndex(1); // black on white
        (*currentScreen)();
      } while( u8g.nextPage() );
    #endif
  }
#endif
