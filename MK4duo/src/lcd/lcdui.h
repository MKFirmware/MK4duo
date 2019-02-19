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
#pragma once

#if HAS_SD_SUPPORT
  #include "../sdcard/sdcard.h"
#endif

#define LCD_MESSAGEPGM(x)       lcdui.set_status_P(PSTR(x))
#define LCD_ALERTMESSAGEPGM(x)  lcdui.set_alert_status_P(PSTR(x))

#if ENABLED(LCD_PROGRESS_BAR) || ENABLED(SHOW_BOOTSCREEN)
  #define LCD_SET_CHARSET(C)    set_custom_characters(C)
#else
  #define LCD_SET_CHARSET(C)    set_custom_characters()
#endif

#if ENABLED(REVERSE_ENCODER_DIRECTION)
  #define ENCODERBASE -1
#else
  #define ENCODERBASE +1
#endif

using screenFunc_t = void(*)();
using menuAction_t = void(*)();

class LcdUI {

  public: /** Constructor */

    LcdUI() {
      #if HAS_LCD_MENU
        currentScreen = status_screen;
      #endif
    }

  public: /** Public Parameters */

    #if HAS_SPI_LCD || HAS_NEXTION_LCD

      static LCDViewActionEnum lcdDrawUpdate;

      #if HAS_GRAPHICAL_LCD 

        static bool drawing_screen,
                    first_page;

      #else

        static constexpr bool drawing_screen = false,
                              first_page = true;

      #endif

      #if HAS_CHARACTER_LCD && ENABLED(LCD_PROGRESS_BAR)
        static millis_t progress_bar_ms;  // Start time for the current progress bar cycle
        #if PROGRESS_MSG_EXPIRE > 0
          static millis_t expire_status_ms; // = 0
        #endif
      #endif

      // Status message
      static char status_message[];
      static uint8_t status_message_level; // Higher levels block lower levels

      #if HAS_SPI_LCD

        #if ENABLED(STATUS_MESSAGE_SCROLLING)
          static uint8_t status_scroll_offset;
        #endif
        static uint8_t status_update_delay;

        #if HAS_LCD_CONTRAST
          static uint8_t contrast;
        #endif

        #if ENABLED(FILAMENT_LCD_DISPLAY) && HAS_SD_SUPPORT
          static millis_t previous_status_ms;
        #endif
        
      #endif

    #endif // HAS_SPI_LCD || HAS_NEXTION_LCD

    #if HAS_LCD_MENU

      static screenFunc_t currentScreen;

      #if ENABLED(ENCODER_RATE_MULTIPLIER)
        static bool encoderRateMultiplierEnabled;
        static millis_t lastEncoderMovementMillis;
      #endif

      #if ENABLED(SCROLL_LONG_FILENAMES)
        static uint8_t  filename_scroll_pos,
                        filename_scroll_max;
      #endif

      #if IS_KINEMATIC
        static bool processing_manual_move;
      #else
        static constexpr bool processing_manual_move = false;
      #endif

      #if E_MANUAL > 1
        static int8_t manual_move_e_index;
      #else
        static constexpr int8_t manual_move_e_index = 0;
      #endif

      static int16_t preheat_hotend_temp[3], preheat_bed_temp[3], preheat_fan_speed[3];

    #elif HAS_SPI_LCD

      static constexpr bool lcd_clicked = false;

    #endif

    #if HAS_LCD_MENU && (ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION))
      static bool external_control;
    #else
      static constexpr bool external_control = false;
    #endif

    #if ENABLED(LCD_BED_LEVELING) && (ENABLED(PROBE_MANUALLY) || ENABLED(MESH_BED_LEVELING))
      static bool wait_for_bl_move;
    #else
      static constexpr bool wait_for_bl_move = false;
    #endif

    #if HAS_ENCODER_ACTION

      static uint32_t encoderPosition;

      static volatile uint8_t buttons;
      #if ENABLED(REPRAPWORLD_KEYPAD)
        static volatile uint8_t keypad_buttons;
      #endif
      #if HAS_SLOW_BUTTONS
        static volatile uint8_t slow_buttons;
      #endif

      #if ENABLED(REVERSE_MENU_DIRECTION)
        static int8_t encoderDirection;
      #else
        static constexpr int8_t encoderDirection = ENCODERBASE;
      #endif

    #endif // HAS_ENCODER_ACTION

  private: /** Private Parameters */

    #if HAS_SPI_LCD || HAS_NEXTION_LCD
      #if HAS_LCD_MENU
        #if LCD_TIMEOUT_TO_STATUS > 0
          static bool defer_return_to_status;
        #else
          static constexpr bool defer_return_to_status = false;
        #endif
      #endif
    #endif

  public: /** Public Function */

    static void clear_lcd();
    
    #if HAS_SPI_LCD || HAS_NEXTION_LCD

      static void init();
      static void update();
      static void set_alert_status_P(PGM_P message);
      static void quick_feedback(const bool clear_buttons=true);

      #if HAS_SPI_LCD

        static void init_lcd();

        #if ENABLED(SHOW_BOOTSCREEN)
          static void show_bootscreen();
        #endif

        #if HAS_GRAPHICAL_LCD

          static void set_font(const MK4duoFontEnum font_nr);

        #elif HAS_CHARACTER_LCD

          static void set_custom_characters(
            #if ENABLED(LCD_PROGRESS_BAR) || ENABLED(SHOW_BOOTSCREEN)
              const HD44780CharSetEnum screen_charset=CHARSET_INFO
            #endif
          );

          #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
            static inline void reset_progress_bar_timeout() { expire_status_ms = 0; }
          #endif

        #endif

        #if HAS_LCD_CONTRAST
          static void set_contrast(const uint8_t value);
          static inline void refresh_contrast() { set_contrast(contrast); }
        #endif

        static void draw_status_message(const bool blink);

      #endif // HAS_SPI_LCD

      static inline bool should_draw() { return bool(lcdDrawUpdate); }
      static inline void refresh(const LCDViewActionEnum type) { lcdDrawUpdate = type; }
      static inline void refresh() { refresh(LCDVIEW_CLEAR_CALL_REDRAW); }

      static bool detected();

      // Status message
      static bool has_status();
      static inline void reset_alert_level() { status_message_level = 0; }

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        static void draw_hotend_status(const uint8_t row, const uint8_t extruder);
      #endif

      static void status_screen();

      static bool get_blink();
      static void kill_screen(PGM_P const lcd_msg);
      static void draw_kill_screen();
      static void set_status(const char* const message, const bool persist=false);
      static void set_status_P(PGM_P const message, int8_t level=0);
      static void status_printf_P(const uint8_t level, PGM_P const fmt, ...);
      static void reset_status();

      static void pause_print();
      static void resume_print();
      static void stop_print();

    #else // NO LCD

      static inline void init() {}
      static inline void update() {}
      static inline void set_alert_status_P(PGM_P message) { UNUSED(message); }
      static inline void refresh() {}
      static inline void set_status(const char* const message, const bool persist=false) { UNUSED(message); UNUSED(persist); }
      static inline void set_status_P(PGM_P const message, const int8_t level=0) { UNUSED(message); UNUSED(level); }
      static inline void status_printf_P(const uint8_t level, PGM_P const fmt, ...) { UNUSED(level); UNUSED(fmt); }
      static inline void return_to_status() {}
      static inline void reset_status() {}
      static inline void reset_alert_level() {}
      static constexpr bool has_status() { return false; }

    #endif

    #if HAS_LCD_MENU

      #if ENABLED(ENCODER_RATE_MULTIPLIER)
        static void enable_encoder_multiplier(const bool onoff);
      #endif

      #if HAS_SD_SUPPORT && HAS_SPI_LCD
        static const char * scrolled_filename(SDCard &theCard, const uint8_t maxlen, uint8_t hash, const bool doScroll);
      #endif

      static void manage_manual_move();

      static bool lcd_clicked;
      static bool use_click();

      static void synchronize(PGM_P const msg=NULL);

      static void goto_screen(const screenFunc_t screen, const uint32_t encoder=0);
      static void save_previous_screen();
      static void goto_previous_screen();
      static void return_to_status();
      static inline bool on_status_screen() { return currentScreen == status_screen; }
      static inline void run_current_screen() { (*currentScreen)(); }

      static inline void defer_status_screen(const bool defer) {
        #if LCD_TIMEOUT_TO_STATUS > 0
          defer_return_to_status = defer;
        #else
          UNUSED(defer);
        #endif
      }

      static inline void goto_previous_screen_no_defer() {
        defer_status_screen(false);
        goto_previous_screen();
      }

      #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
        static void reselect_last_file();
      #endif

      #if ENABLED(G26_MESH_VALIDATION)
        static inline void chirp() { sound.playTone(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ); }
      #endif

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        static void ubl_plot(const uint8_t x, const uint8_t inverted_y);
      #endif

    #elif HAS_SPI_LCD

      static constexpr bool on_status_screen() { return true; }
      static inline void run_current_screen() { status_screen(); }

    #endif

    #if HAS_LCD_MENU && (ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION))
      FORCE_INLINE static void capture() { external_control = true; }
      FORCE_INLINE static void release() { external_control = false; }
    #endif

    #if HAS_ENCODER_ACTION

      #if ENABLED(REPRAPWORLD_KEYPAD)
        static bool handle_keypad();
      #endif
      #if HAS_SLOW_BUTTONS
        static uint8_t read_slow_buttons();
      #endif
      static void update_buttons();
      static bool button_pressed();
      #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
        static void wait_for_release();
      #endif

      #if ENABLED(REVERSE_ENCODER_DIRECTION)
        #define ENCODERBASE -1
      #else
        #define ENCODERBASE +1
      #endif
      #if ENABLED(REVERSE_MENU_DIRECTION)
        static inline void encoder_direction_normal() { encoderDirection = +(ENCODERBASE); }
        static inline void encoder_direction_menus()  { encoderDirection = -(ENCODERBASE); }
      #else
        static inline void encoder_direction_normal() { }
        static inline void encoder_direction_menus()  { }
      #endif

    #else

      static inline void update_buttons() { }

    #endif

  private: /** Public Function */

    #if HAS_LCD_MENU
      static void _synchronize();
    #endif

    #if HAS_SPI_LCD
      static void draw_status_screen();
      static void finishstatus(const bool persist);
    #else
      static inline void finishstatus(const bool persist) { UNUSED(persist); refresh(); }
    #endif

};

extern LcdUI lcdui;
