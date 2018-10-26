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

#ifndef ULTRALCD_H
#define ULTRALCD_H

#if ENABLED(ULTRA_LCD) || ENABLED(NEXTION)
  void lcd_init();
  void lcd_update();
  void lcd_reset_alert_level();
  void lcd_setstatusPGM(PGM_P message, const int8_t level=0);
  void lcd_setalertstatusPGM(PGM_P message);
  void lcd_setstatus(PGM_P message, const bool persist=false);
  void lcd_status_printf_P(const uint8_t level, PGM_P const fmt, ...);
  bool lcd_detected();
#else
  inline void lcd_init() {}
  inline void lcd_update() {}
  inline void lcd_reset_alert_level() {}
  inline void lcd_setstatusPGM(PGM_P const message, const int8_t level=0) { UNUSED(message); UNUSED(level); }
  inline void lcd_setalertstatusPGM(PGM_P message) { UNUSED(message); }
  inline void lcd_setstatus(PGM_P const message, const bool persist=false) { UNUSED(message); UNUSED(persist); }
  inline void lcd_status_printf_P(const uint8_t level, PGM_P const fmt, ...) { UNUSED(level); UNUSED(fmt); }
  inline bool lcd_detected() { return true; }
#endif

#if ENABLED(ULTRA_LCD)

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    #include "../feature/advanced_pause/advanced_pause.h"
  #endif

  enum LCDViewAction : uint8_t {
    LCDVIEW_NONE,
    LCDVIEW_REDRAW_NOW,
    LCDVIEW_CALL_REDRAW_NEXT,
    LCDVIEW_CLEAR_CALL_REDRAW,
    LCDVIEW_CALL_NO_REDRAW
  };

  bool lcd_hasstatus();

  void lcd_reset_status();
  
  void lcd_kill_screen();
  void kill_screen(PGM_P lcd_msg);

  extern LCDViewAction lcdDrawUpdate;
  inline void lcd_refresh() { lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; }

  extern void lcd_move_z_probe();

  void lcd_quick_feedback(const bool clear_buttons); // Audible feedback for a button click - could also be visual

  #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
    void dontExpireStatus();
  #endif

  #if ENABLED(ADC_KEYPAD)
    uint8_t get_ADC_keyValue();
  #endif

  #if HAS_LCD_CONTRAST
    extern uint8_t lcd_contrast;
    void set_lcd_contrast(const uint8_t value);
  #endif

  #if ENABLED(DOGLCD)
    #define SETCURSOR(col, row) lcd_moveto(col * (DOG_CHAR_WIDTH), (row + 1) * row_height)
    #define SETCURSOR_RJ(len, row) lcd_moveto(LCD_PIXEL_WIDTH - len * (DOG_CHAR_WIDTH), (row + 1) * row_height)
  #else
    #define SETCURSOR(col, row) lcd_moveto(col, row)
    #define SETCURSOR_RJ(len, row) lcd_moveto(LCD_WIDTH - len, row)
  #endif

  #if ENABLED(SHOW_BOOTSCREEN)
    void lcd_bootscreen();
  #endif

  #define LCD_UPDATE_INTERVAL 100
  #define BUTTON_EXISTS(BN) (ENABLED(BTN_## BN) && BTN_## BN >= 0)
  #define BUTTON_PRESSED(BN) !READ(BTN_## BN)

  #if ENABLED(ULTIPANEL) // LCD with a click-wheel input

    extern bool defer_return_to_status;

    // Function pointer to menu functions.
    using screenFunc_t = void(*)();
    using menuAction_t = void(*)();

    extern int16_t lcd_preheat_hotend_temp[3], lcd_preheat_bed_temp[3], lcd_preheat_fan_speed[3];

    #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
      extern bool lcd_external_control;
    #else
      constexpr bool lcd_external_control = false;
    #endif

    #if ENABLED(LCD_BED_LEVELING)
      extern bool lcd_wait_for_move;
    #else
      constexpr bool lcd_wait_for_move = false;
    #endif

    void lcd_goto_screen(screenFunc_t screen, const uint32_t encoder=0);

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      void lcd_advanced_pause_show_message(const AdvancedPauseMessage message,
                                           const AdvancedPauseMode mode=ADVANCED_PAUSE_MODE_PAUSE_PRINT,
                                           const uint8_t hotend=TARGET_HOTEND);
    #endif // ADVANCED_PAUSE_FEATURE

    #if ENABLED(G26_MESH_VALIDATION)
      void lcd_chirp();
    #endif

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      void lcd_mesh_edit_setup(const float &initial);
      float lcd_mesh_edit();
      void lcd_z_offset_edit_setup(const float &initial);
      float lcd_z_offset_edit();
    #endif

    #if ENABLED(PROBE_MANUALLY)
      float lcd_probe_pt(const float &rx, const float &ry);
    #endif

  #endif

  #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
    extern millis_t previous_lcd_status_ms;
  #endif

  bool lcd_blink();

  #if ENABLED(REPRAPWORLD_KEYPAD)     // is also ULTIPANEL and NEWPANEL or ANET display

    #define REPRAPWORLD_BTN_OFFSET 0  // bit offset into buttons for shift register values

    #define BLEN_REPRAPWORLD_KEYPAD_F3     0
    #define BLEN_REPRAPWORLD_KEYPAD_F2     1
    #define BLEN_REPRAPWORLD_KEYPAD_F1     2
    #define BLEN_REPRAPWORLD_KEYPAD_DOWN   3
    #define BLEN_REPRAPWORLD_KEYPAD_RIGHT  4
    #define BLEN_REPRAPWORLD_KEYPAD_MIDDLE 5
    #define BLEN_REPRAPWORLD_KEYPAD_UP     6
    #define BLEN_REPRAPWORLD_KEYPAD_LEFT   7

    #define EN_REPRAPWORLD_KEYPAD_F3      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F3))
    #define EN_REPRAPWORLD_KEYPAD_F2      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F2))
    #define EN_REPRAPWORLD_KEYPAD_F1      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F1))
    #define EN_REPRAPWORLD_KEYPAD_DOWN    (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_DOWN))
    #define EN_REPRAPWORLD_KEYPAD_RIGHT   (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_RIGHT))
    #define EN_REPRAPWORLD_KEYPAD_MIDDLE  (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_MIDDLE))
    #define EN_REPRAPWORLD_KEYPAD_UP      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_UP))
    #define EN_REPRAPWORLD_KEYPAD_LEFT    (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_LEFT))

    #define REPRAPWORLD_KEYPAD_MOVE_Z_DOWN  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F3)
    #define REPRAPWORLD_KEYPAD_MOVE_Z_UP    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_F2)
    #define REPRAPWORLD_KEYPAD_MOVE_Y_DOWN  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_DOWN)
    #define REPRAPWORLD_KEYPAD_MOVE_X_RIGHT (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_RIGHT)
    #define REPRAPWORLD_KEYPAD_MOVE_Y_UP    (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_UP)
    #define REPRAPWORLD_KEYPAD_MOVE_X_LEFT  (buttons_reprapworld_keypad & EN_REPRAPWORLD_KEYPAD_LEFT)

    #if ENABLED(ADC_KEYPAD)
      #define KEYPAD_HOME EN_REPRAPWORLD_KEYPAD_F1
      #define KEYPAD_EN_C EN_REPRAPWORLD_KEYPAD_MIDDLE
    #else
      #define KEYPAD_HOME EN_REPRAPWORLD_KEYPAD_MIDDLE
      #define KEYPAD_EN_C EN_REPRAPWORLD_KEYPAD_F1
    #endif
    #define REPRAPWORLD_KEYPAD_MOVE_HOME    (buttons_reprapworld_keypad & KEYPAD_HOME)
    #define REPRAPWORLD_KEYPAD_MOVE_MENU    (buttons_reprapworld_keypad & KEYPAD_EN_C)

    #define REPRAPWORLD_KEYPAD_PRESSED      (buttons_reprapworld_keypad & ( \
                                              EN_REPRAPWORLD_KEYPAD_F3 | \
                                              EN_REPRAPWORLD_KEYPAD_F2 | \
                                              EN_REPRAPWORLD_KEYPAD_F1 | \
                                              EN_REPRAPWORLD_KEYPAD_DOWN | \
                                              EN_REPRAPWORLD_KEYPAD_RIGHT | \
                                              EN_REPRAPWORLD_KEYPAD_MIDDLE | \
                                              EN_REPRAPWORLD_KEYPAD_UP | \
                                              EN_REPRAPWORLD_KEYPAD_LEFT) \
                                            )

  #endif

  #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
    bool is_lcd_clicked();
    void wait_for_release();
  #endif

  void lcd_eeprom_allert();

#else // no NEXTION or no LCD

  constexpr bool lcd_wait_for_move = false;

  inline void lcd_refresh() {}
  inline bool lcd_hasstatus() { return false; }
  
  inline void lcd_eeprom_allert() {}

#endif // ULTRA_LCD

#if ENABLED(ULTIPANEL)

  #if ENABLED(NEWPANEL) // Uses digital switches, not a shift register

    // Wheel spin pins where BA is 00, 10, 11, 01 (1 bit always changes)
    #define BLEN_A 0
    #define BLEN_B 1

    #define EN_A (_BV(BLEN_A))
    #define EN_B (_BV(BLEN_B))

    #if BUTTON_EXISTS(ENC)
      #define BLEN_C 2
      #define EN_C (_BV(BLEN_C))
    #endif

    #if BUTTON_EXISTS(BACK)
      #define BLEN_D 3
      #define EN_D (_BV(BLEN_D))
      #if ENABLED(INVERT_BACK_BUTTON)
        #define LCD_BACK_CLICKED !(buttons&EN_D)
      #else
        #define LCD_BACK_CLICKED (buttons&EN_D)
      #endif
    #endif

  #endif // NEWPANEL

  extern volatile uint8_t buttons;  // The last-checked buttons in a bit array.
  void lcd_buttons_update();

#else

  inline void lcd_buttons_update() {}

#endif    

#if ENABLED(REPRAPWORLD_KEYPAD)
  #if BUTTON_EXISTS(ENC)
    #define LCD_CLICKED ((buttons & EN_C) || REPRAPWORLD_KEYPAD_MOVE_MENU)
  #else
    #define LCD_CLICKED REPRAPWORLD_KEYPAD_MOVE_MENU
  #endif
#elif ENABLED(EN_C)
  #if ENABLED(INVERT_CLICK_BUTTON)
    #define LCD_CLICKED !(buttons & EN_C)
  #else
    #define LCD_CLICKED (buttons & EN_C)
  #endif
#else
  #define LCD_CLICKED false
#endif

#define LCD_MESSAGEPGM(x)      lcd_setstatusPGM(PSTR(x))
#define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatusPGM(PSTR(x))

#endif // ULTRALCD_H
