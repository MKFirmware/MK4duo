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
#pragma once

#if HAS_LCD_MENU

// Manual Movement
constexpr XYZEval<const float> manual_feedrate_mm_m = MANUAL_FEEDRATE;
extern float move_menu_scale;

extern int8_t encoderLine, encoderTopLine, screen_items;
extern bool screen_changed;

void scroll_screen(const uint8_t limit, const bool is_menu);

////////////////////////////////////////////
///////// Menu Item Draw Functions /////////
////////////////////////////////////////////

void draw_select_screen(PGM_P const yes, PGM_P const no, const bool yesno, PGM_P const pref, const char * const string, PGM_P const suff);
void do_select_screen(PGM_P const yes, PGM_P const no, selectFunc_t yesFunc, selectFunc_t noFunc, PGM_P const pref, const char * const string=nullptr, PGM_P const suff=nullptr);
inline void do_select_screen_yn(selectFunc_t yesFunc, selectFunc_t noFunc, PGM_P const pref, const char * const string=nullptr, PGM_P const suff=nullptr) {
  do_select_screen(GET_TEXT(MSG_YES), GET_TEXT(MSG_NO), yesFunc, noFunc, pref, string, suff);
}

#define SS_LEFT     0x00
#define SS_CENTER   0x01
#define SS_INVERT   0x02
#define SS_DEFAULT  SS_CENTER
#define NO_INDEX    0xFF

//
// The Menu Edit shadow value
// Only one edit value is needed at a time
//

typedef union {
  bool     state;
  float    decimal;
  int8_t   int8;
  int16_t  int16;
  int32_t  int32;
  uint8_t  uint8;
  uint16_t uint16;
  uint32_t uint32;
} editable_t;

extern editable_t editable;

#if HAS_GRAPHICAL_LCD && (ENABLED(BABYSTEP_ZPROBE_GFX_OVERLAY) || ENABLED(MESH_EDIT_GFX_OVERLAY))
  void _lcd_zoffset_overlay_gfx(const float zvalue);
#endif

////////////////////////////////////////////
////////// Menu Item Base Classes //////////
////////////////////////////////////////////

class MenuItem_static {
  public:
    static void draw(const uint8_t row, PGM_P const pstr, const uint8_t style=SS_DEFAULT, const char * const valstr=nullptr);
};

class MenuItemBase {
  public:
    // Number to interject in item label or elsewhere
    static uint8_t itemIndex;

    // Store the index of the item ahead of use by indexed items
    static void init(const uint8_t idx) { itemIndex = idx; }

    // Draw an INDEXED item either selected (pre_char) or not (space) with post_char
    // The index should always be itemIndex. Subclass methods taking 'idx' should init(idx)
    // to provide an accessible index for callbacks
    static void _draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, const char pre_char, const char post_char);

    // Draw an item either selected (pre_char) or not (space) with post_char
    static void _draw(const bool sel, const uint8_t row, PGM_P const pstr, const char pre_char, const char post_char);

    // Draw an item either selected ('>') or not (space) with post_char
    static inline void _draw(const bool sel, const uint8_t row, PGM_P const pstr, const char post_char) {
      _draw(sel, row, pstr, '>', post_char);
    }
};

// BACK_ITEM(PLABEL)
class MenuItem_back : public MenuItemBase {
  public:
    // Back Item just draws the label with up-level indicators
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, ...) {
      _draw(sel, row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0]);
    }
    // Back Item action is to go back in the history
    static inline void action(PGM_P const=nullptr) {
      lcdui.goto_previous_screen(
        #if ENABLED(TURBO_BACK_MENU_ITEM)
          true
        #endif
      );
    }
};

// SUBMENU(PLABEL, FUNC)
class MenuItem_submenu : public MenuItemBase {
  public:
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, ...) {
      _draw(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0]);
    }
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, ...) {
      init(idx);
      _draw(sel, row, pstr, idx, '>', LCD_STR_ARROW_RIGHT[0]);
    }
    static inline void action(PGM_P const, const screenFunc_t func) { lcdui.save_previous_screen(); lcdui.goto_screen(func); }
    static inline void action(PGM_P const, const uint8_t idx, const screenFunc_t func) { init(idx); lcdui.save_previous_screen(); lcdui.goto_screen(func); }
};

class MenuItem_button : public MenuItemBase {
  public:
    // IDXEXED Button-y Items are selectable lines with no other indicator
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, ...) {
      init(idx);
      _draw(sel, row, pstr, idx, '>', ' ');
    }
    // Button-y Items are selectable lines with no other indicator
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, ...) {
      _draw(sel, row, pstr, '>', ' ');
    }
};

// GCODES_ITEM(LABEL, GCODES)
// GCODES_ITEM_P(PLABEL, GCODES)
// GCODES_ITEM_N(LABEL, IDX, GCODES)
// GCODES_ITEM_N_P(PLABEL, IDX, GCODES)
class MenuItem_gcode : public MenuItem_button {
  public:
    static void action(PGM_P const, const char * const pgcode);
    static inline void action(PGM_P const pstr, const uint8_t, const char * const pgcode) { action(pstr, pgcode); }
};

// ACTION_ITEM(LABEL, FUNC)
// ACTION_ITEM_P(PLABEL, FUNC)
// ACTION_ITEM_N(LABEL, IDX, FUNC)
// ACTION_ITEM_N_P(PLABEL, IDX, FUNC)
class MenuItem_function : public MenuItem_button {
  public:
    //static inline void action(PGM_P const, const uint8_t, const menuAction_t func) { (*func)(); };
    static inline void action(PGM_P const, const menuAction_t func) { (*func)(); };
    static inline void action(PGM_P const, const uint8_t idx, const screenFunc_t func) { init(idx); (*func)(); };
};

#if HAS_SD_SUPPORT
  class SDCard;
  class MenuItem_sdbase {
    public:
      // Implemented for HD44780 and DOGM
      static void _draw(const bool sel, const uint8_t row, PGM_P const pstr, SDCard &theCard, const bool isDir);
  };
#endif

////////////////////////////////////////////
////////////// Menu Edit Items /////////////
////////////////////////////////////////////

// Base class for Menu Edit Items
class MenuEditItemBase : public MenuItemBase {
  private:
    // These values are statically constructed by init() via action()
    // The action() method acts like the instantiator. The entire lifespan
    // of a menu item is within its declaration, so all these values decompose
    // into behavior and unused items get optimized out.
    static PGM_P editLabel;
    static void *editValue;
    static int32_t minEditValue, maxEditValue;  // Encoder value range
    static screenFunc_t callbackFunc;
    static bool liveEdit;
  protected:
    typedef const char* (*strfunc_t)(const int32_t);
    typedef void (*loadfunc_t)(void *, const int32_t);
    static void init(PGM_P const el, const uint8_t idx, void * const ev, const int32_t minv, const int32_t maxv, const uint16_t ep, const screenFunc_t cs, const screenFunc_t cb, const bool le);
    static void edit(strfunc_t, loadfunc_t);

  public:
    // Implemented for HD44780 and DOGM
    // Low-level method to draw (sel,row,pstr,idx,data,pgm), used by both MenuEditItem_NAME::_draw methods
    static void _draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, const char* const data, const bool pgm);

    // Draw the current item at specified row with edit data
    static void _draw(const bool sel, const uint8_t row, const char* const data, const bool pgm) {
      _draw(sel, row, editLabel, itemIndex, data, pgm);
    }

    // Implemented for HD44780 and DOGM
    // This low-level method is good to draw from anywhere
    static void edit_screen(PGM_P const pstr, const char* const value);

    // This method is for the current menu item
    static inline void edit_screen(const char* const value) { edit_screen(editLabel, value); }
};

// Template for specific Menu Edit Item Types
template<typename NAME>
class TMenuEditItem : MenuEditItemBase {
  private:
    typedef typename NAME::type_t type_t;
    static inline float unscale(const float value)    { return value * (1.0f / NAME::scale);  }
    static inline float scale(const float value)      { return value * NAME::scale;           }
    static void load(void *ptr, const int32_t value)  { *((type_t*)ptr) = unscale(value);     }
    static const char* to_string(const int32_t value) { return NAME::strfunc(unscale(value)); }
  public:
    // These methods call the specific type-plus-formatting class
    // for the given NAME.
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, type_t * const data, ...) { \
      NAME::_draw(sel, row, pstr, idx, data);
    }
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, PGM_P const, type_t (*pget)(), ...) { \
      NAME::_draw(sel, row, pstr, idx, pget);
    }
    static void action(
      PGM_P const pstr,                     // Edit label
      const uint8_t idx,                    // Index to include in the label
      type_t * const ptr,                   // Value pointer
      const type_t minValue,                // Value min
      const type_t maxValue,                // Value max
      const screenFunc_t callback=nullptr,  // Value update callback
      const bool live=false                 // Callback during editing
    ) {
      // Make sure minv and maxv fit within int32_t
      const int32_t minv = MAX(scale(minValue), INT32_MIN),
                    maxv = MIN(scale(maxValue), INT32_MAX);
      init(pstr, idx, ptr, minv, maxv - minv, scale(*ptr) - minv, edit, callback, live);
    }
    static void edit() { MenuEditItemBase::edit(to_string, load); }
};

// Provide a set of Edit Item Types which encompass a primitive
// type, a string function, and a scale factor for edit and display.
// These items call the Edit Item draw method passing the prepared string.
#define DEFINE_MENU_EDIT_ITEM_TYPE(TYPE, NAME, STRFUNC, SCALE) \
  struct MenuEditItemInfo_##NAME { \
    typedef TYPE type_t; \
    static constexpr float scale = SCALE; \
    static inline const char* strfunc(const float value) { return STRFUNC((TYPE)value); } \
    static inline void _draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, TYPE * const data) { \
      MenuEditItemBase::_draw(sel, row, pstr, idx, STRFUNC(*(data)), false); \
    } \
    static inline void _draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, TYPE (*pget)()) { \
      MenuEditItemBase::_draw(sel, row, pstr, idx, STRFUNC(pget()), false); \
    } \
  }; \
  typedef TMenuEditItem<MenuEditItemInfo_##NAME> MenuItem_##NAME

//                      TYPE      NAME         STRFUNC       SCALE
DEFINE_MENU_EDIT_ITEM_TYPE(uint8_t,  percent,     ui8tostr4pct, 100.0/255);   // 100%       right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(int16_t,  int3,        i16tostr3,       1     );   // 123, -12   right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(int16_t,  int4,        i16tostr4sign,   1     );   // 1234, -123 right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(int8_t,   int8,        i8tostr3,        1     );   // 123, -12   right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint8_t,  uint8,       ui8tostr3,       1     );   // 123        right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, microstep,   ui16tostr3,      1/16  );   // 16
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, uint16_3,    ui16tostr3,      1     );   // 123        right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, uint16_4,    ui16tostr4,      0.1   );   // 1234       right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, uint16_5,    ui16tostr5,      0.01  );   // 12345      right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float3,      ftostr3,         1     );   // 123        right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float52,     ftostr42_52,   100     );   // _2.34, 12.34, -2.34 or 123.45, -23.45
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float43,     ftostr43sign, 1000     );   // 1.234
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float5,      ftostr5rj,       1     );   // 12345      right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float5_25,   ftostr5rj,       0.04f );   // 12345      right-justified (25 increment)
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float51,     ftostr51rj,     10     );   // 1234.5     right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float51sign, ftostr51sign,   10     );   // +1234.5
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float52sign, ftostr52sign,  100     );   // +123.45
DEFINE_MENU_EDIT_ITEM_TYPE(uint32_t, long5,       ftostr5rj,       0.01f );   // 12345      right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint32_t, long5_25,    ftostr5rj,       0.04f );   // 12345      right-justified (25 increment)

class MenuItem_bool : MenuEditItemBase {
  public:
    static void action(PGM_P const pstr, const uint8_t idx, bool * const ptr, const screenFunc_t callbackFunc=nullptr);
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, const bool onoff) {
      _draw(sel, row, pstr, idx, onoff ? GET_TEXT(MSG_LCD_ON) : GET_TEXT(MSG_LCD_OFF), true);
    }
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, bool * const data, ...) {
      draw(sel, row, pstr, idx, *data);
    }
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, const uint8_t idx, PGM_P const, bool (*pget)(), ...) {
      draw(sel, row, pstr, idx, pget());
    }
};

////////////////////////////////////////////
//////////// Menu System Macros ////////////
////////////////////////////////////////////

/**
 * SCREEN_OR_MENU_LOOP generates init code for a screen or menu
 *
 *   encoderTopLine is the top menu line to display
 *   _lcdLineNr is the index of the LCD line (e.g., 0-3)
 *   _menuLineNr is the menu item to draw and process
 *   _thisItemNr is the index of each MENU_ITEM or STATIC_ITEM
 */
#define SCREEN_OR_MENU_LOOP() \
  int8_t _menuLineNr = encoderTopLine, _thisItemNr; \
  for (int8_t _lcdLineNr = 0; _lcdLineNr < LCD_HEIGHT; _lcdLineNr++, _menuLineNr++) { \
    _thisItemNr = 0

/**
 * START_SCREEN  Opening code for a screen having only static items.
 *               Do simplified scrolling of the entire screen.
 *
 * START_MENU    Opening code for a screen with menu items.
 *               Scroll as-needed to keep the selected line in view.
 */
#define START_SCREEN() \
  scroll_screen(LCD_HEIGHT, false); \
  bool _skipStatic = false; \
  SCREEN_OR_MENU_LOOP()

#define START_MENU() \
  scroll_screen(1, true); \
  bool _skipStatic = true; \
  SCREEN_OR_MENU_LOOP()

#define END_SCREEN() \
  } \
  screen_items = _thisItemNr

#define END_MENU() \
  } \
  screen_items = _thisItemNr; \
  UNUSED(_skipStatic)

#if ENABLED(ENCODER_RATE_MULTIPLIER)
  #define ENCODER_RATE_MULTIPLY(F) (lcdui.encoderRateMultiplierEnabled = F)
  #define _MENU_ITEM_MULTIPLIER_CHECK(USE_MULTIPLIER) do{ if (USE_MULTIPLIER) lcdui.enable_encoder_multiplier(true); }while(0)
  //#define ENCODER_RATE_MULTIPLIER_DEBUG  // If defined, output the encoder steps per second value
#else
  #define ENCODER_RATE_MULTIPLY(F) NOOP
  #define _MENU_ITEM_MULTIPLIER_CHECK(USE_MULTIPLIER)
#endif

/**
 * MENU_ITEM generates draw & handler code for a menu item, potentially calling:
 *
 *   MenuItem_<type>::draw(sel, row, label, arg3...)
 *   MenuItem_<type>::action(arg3...)
 *
 * Examples:
 *   BACK_ITEM(MSG_WATCH)
 *   MENU_ITEM(back, MSG_WATCH)
 *   MENU_ITEM_P(back, GET_TEXT(MSG_WATCH))
 *   _MENU_ITEM_P(back, PLABEL, false, ...)
 *     MenuItem_back::action(plabel, ...)
 *     MenuItem_back::draw(sel, row, plabel, ...)
 *
 *   ACTION_ITEM(MSG_PAUSE_PRINT, lcd_sdcard_pause)
 *   MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause)
 *   MENU_ITEM_P(function, GET_TEXT(MSG_PAUSE_PRINT))
 *     MenuItem_function::action(plabel, lcd_sdcard_pause)
 *     MenuItem_function::draw(sel, row, plabel, lcd_sdcard_pause)
 *
 *   EDIT_ITEM(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
 *   EDIT_ITEM_N(int3, MSG_SPEED, 0, &feedrate_percentage, 10, 999)
 *   EDIT_ITEM_N_P(int3, GET_TEXT(MSG_SPEED), 0, &feedrate_percentage, 10, 999)
 *   MENU_ITEM_P(int3, PLABEL, 0, &feedrate_percentage, 10, 999)
 *  _MENU_ITEM_P(int3, PLABEL, false, 0, &feedrate_percentage, 10, 999)
 *     MenuItem_int3::action(plabel, 0, &feedrate_percentage, 10, 999)
 *     MenuItem_int3::draw(sel, row, plabel, 0, &feedrate_percentage, 10, 999)
 *
 */
#define _MENU_ITEM_P(TYPE, PLABEL, USE_MULTIPLIER, V...) do {     \
    _skipStatic = false;                                          \
    if (_menuLineNr == _thisItemNr) {                             \
      PGM_P const plabel = PLABEL;                                \
      if (encoderLine == _thisItemNr && lcdui.use_click()) {      \
        _MENU_ITEM_MULTIPLIER_CHECK(USE_MULTIPLIER);              \
        MenuItem_##TYPE ::action(plabel, ##V);                    \
        if (screen_changed) return;                               \
      }                                                           \
      if (lcdui.should_draw())                                    \
        MenuItem_##TYPE::draw                                     \
          (encoderLine == _thisItemNr, _lcdLineNr, plabel, ##V);  \
    }                                                             \
  ++_thisItemNr;                                                  \
}while(0)

#define MENU_ITEM_ADDON_START(X) do{ \
  if (lcdui.should_draw() && _menuLineNr == _thisItemNr - 1) { \
    SETCURSOR(X, _lcdLineNr)

#define MENU_ITEM_ADDON_END() } }while(0)

// Used to print static text with no visible cursor.
// Parameters: label [, bool center [, bool invert [, char *value] ] ]
#define STATIC_ITEM_P(PLABEL, V...) do{                     \
  if (_menuLineNr == _thisItemNr) {                         \
    if (_skipStatic && encoderLine <= _thisItemNr) {        \
      lcdui.encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
      ++encoderLine;                                        \
    }                                                       \
    if (lcdui.should_draw())                                \
      MenuItem_static::draw(_lcdLineNr, PLABEL, ##V);       \
  }                                                         \
  ++_thisItemNr;                                            \
} while(0)

#define STATIC_ITEM(LABEL, V...)             STATIC_ITEM_P(GET_TEXT(LABEL), ##V)

#define SKIP_ITEM() (_thisItemNr++)

#define MENU_ITEM_P(TYPE, PLABEL, V...)       _MENU_ITEM_P(TYPE, PLABEL,          false,  ##V)
#define MENU_ITEM(TYPE, LABEL, V...)           MENU_ITEM_P(TYPE, GET_TEXT(LABEL),         ##V)

#define BACK_ITEM(LABEL)                         MENU_ITEM(back, LABEL)

#define ACTION_ITEM_P(PLABEL, ACTION)          MENU_ITEM_P(function, PLABEL, ACTION)
#define ACTION_ITEM(LABEL, ACTION)           ACTION_ITEM_P(GET_TEXT(LABEL), ACTION)

#define ACTION_ITEM_N_P(PLABEL, IDX, ACTION)   MENU_ITEM_P(function, PLABEL,  IDX, ACTION)
#define ACTION_ITEM_N(LABEL, IDX, ACTION)  ACTION_ITEM_N_P(GET_TEXT(LABEL),   IDX, ACTION)

#define GCODES_ITEM_P(PLABEL, GCODES)          MENU_ITEM_P(gcode, PLABEL,   GCODES)
#define GCODES_ITEM(LABEL, GCODES)           GCODES_ITEM_P(GET_TEXT(LABEL), GCODES)

#define GCODES_ITEM_N_P(PLABEL, IDX, GCODES)   MENU_ITEM_P(gcode,  PLABEL,  IDX, GCODES)
#define GCODES_ITEM_N(LABEL, IDX, GCODES)  GCODES_ITEM_N_P(GET_TEXT(LABEL), IDX, GCODES)

#define SUBMENU_P(PLABEL, DEST)                MENU_ITEM_P(submenu, PLABEL, DEST)
#define SUBMENU(LABEL, DEST)                     SUBMENU_P(GET_TEXT(LABEL), DEST)

#define SUBMENU_N_P(PLABEL, IDX, DEST)         MENU_ITEM_P(submenu, PLABEL, IDX, DEST)
#define SUBMENU_N(LABEL, IDX, DEST)            SUBMENU_N_P(GET_TEXT(LABEL), IDX, DEST)

// Edit Items take an index
#define EDIT_ITEM_N_P(TYPE, PLABEL, N, V...)   MENU_ITEM_P(TYPE,  PLABEL,           N,          ##V)
#define EDIT_ITEM_P(TYPE, PLABEL, V...)      EDIT_ITEM_N_P(TYPE,  PLABEL,           NO_INDEX,   ##V)
#define EDIT_ITEM_N(TYPE, LABEL, N, V...)    EDIT_ITEM_N_P(TYPE,  GET_TEXT(LABEL),  N,          ##V)
#define EDIT_ITEM(TYPE, LABEL, V...)           EDIT_ITEM_N(TYPE,  LABEL,            NO_INDEX,   ##V)

#define EDIT_ITEM_FAST_N_P(TYPE, PLABEL, N, V...)    _MENU_ITEM_P(TYPE, PLABEL,         true, N,        ##V)
#define EDIT_ITEM_FAST_P(TYPE, PLABEL, V...)   EDIT_ITEM_FAST_N_P(TYPE, PLABEL,               NO_INDEX, ##V)
#define EDIT_ITEM_FAST_N(TYPE, LABEL, N, V...) EDIT_ITEM_FAST_N_P(TYPE, GET_TEXT(LABEL),      N,        ##V)
#define EDIT_ITEM_FAST(TYPE, LABEL, V...)        EDIT_ITEM_FAST_N(TYPE, LABEL,                NO_INDEX, ##V)

////////////////////////////////////////////
/////////////// Menu Screens ///////////////
////////////////////////////////////////////

void menu_main();
void menu_move();

void menu_stop_print();

#if HAS_SD_SUPPORT
  void menu_sdcard();
#endif

#if HAS_EEPROM
  void menu_eeprom();
#endif

#if HAS_NEXTION_LCD
  void menu_nextion();
  void menu_m0();
  #if HAS_SD_SUPPORT
    void menu_firmware();
  #endif
#endif

////////////////////////////////////////////
//////// Menu Item Helper Functions ////////
////////////////////////////////////////////

void lcd_move_z();
void lcd_line_to_z(const float &z);
void lcd_draw_homing();

#if MAX_HOTEND > 0
  void watch_temp_callback_hotend();
#endif
#if MAX_BED > 0
  void watch_temp_callback_bed();
#endif
#if MAX_CHAMBER > 0
  void watch_temp_callback_chamber();
#endif
#if MAX_COOLER > 0
  void watch_temp_callback_cooler();
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  void lcd_pause_show_message(const PauseMessageEnum message,
                              const PauseModeEnum mode=PAUSE_MODE_SAME,
                              const uint8_t hotend=tools.active_hotend());
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  void lcd_mesh_edit_setup(const float &initial);
  float lcd_mesh_edit();
#endif

#if HAS_PROBE_MANUALLY || MECH(DELTA)
  void _man_probe_pt(const xy_pos_t &xy);
#endif

#if HAS_PROBE_MANUALLY
  float lcd_probe_pt(const xy_pos_t &xy);
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(PID_AUTOTUNE_MENU) || ENABLED(ADVANCED_PAUSE_FEATURE)
  void lcd_enqueue_one_now(const char * const cmd);
  void lcd_enqueue_one_now_P(PGM_P const cmd);
#endif

#if ENABLED(LEVEL_BED_CORNERS)
  void lcd_level_bed_corners();
#endif

#if HAS_SOFTWARE_ENDSTOPS
  void lcd_toggle_soft_endstops(); 
#endif

#if ENABLED(BABYSTEPPING)
  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    void lcd_babystep_zoffset();
  #else
    void lcd_babystep_z();
  #endif
#endif

#if HAS_SD_RESTART
  void menu_sdcard_restart();
#endif

#if HAS_MMU2
  void menu_mmu2();
  void mmu2_M600();
  uint8_t mmu2_choose_filament();
#endif

#endif // HAS_LCD_MENU
