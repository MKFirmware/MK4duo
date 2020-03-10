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
#pragma once

#if HAS_LCD_MENU

// Manual Movement
constexpr xyze_feedrate_t manual_feedrate_mm_s = MANUAL_FEEDRATE;
extern float move_menu_scale;

extern int8_t encoderLine, encoderTopLine, screen_items;

void scroll_screen(const uint8_t limit, const bool is_menu);

////////////////////////////////////////////
///////// Menu Item Draw Functions /////////
////////////////////////////////////////////

#define SS_LEFT     0x00
#define SS_CENTER   0x01
#define SS_INVERT   0x02
#define SS_DEFAULT  SS_CENTER
#define NO_INDEX    0xFF

#if Z_PROBE_OFFSET_RANGE_MIN >= -9 && Z_PROBE_OFFSET_RANGE_MAX <= 9
  #define LCD_Z_OFFSET_FUNC(N)  ftostr54sign(N)
  #define LCD_Z_OFFSET_TYPE     float43
#else
  #define LCD_Z_OFFSET_FUNC(N)  ftostr52sign(N)
  #define LCD_Z_OFFSET_TYPE     float52
#endif

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
class MenuItemBase {
  public:
    // Number to interject in item label or elsewhere
    static uint8_t itemIndex;

    // Store the index of the item ahead of use by indexed items
    FORCE_INLINE static void init(const uint8_t idx=NO_INDEX) { itemIndex = idx; }

    // Draw an item either selected (pre_char) or not (space) with post_char
    static void _draw(const bool sel, const uint8_t row, PGM_P const pstr, const char pre_char, const char post_char);

    // Draw an item either selected ('>') or not (space) with post_char
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, const char post_char) {
      _draw(sel, row, pstr, '>', post_char);
    }
};

class MenuItem_static : public MenuItemBase {
  public:
    static void draw(const uint8_t row, PGM_P const pstr, const uint8_t style=SS_DEFAULT, const char * const valstr=nullptr);
};

// CONFIRM_ITEM(PLABEL,Y,N,FY,FN,V...)
// YESNO_ITEM(PLABEL,FY,FN,V...)
class MenuItem_confirm : MenuItemBase {
  public:
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, ...) {
      _draw(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0]);
    }
    // Implemented for HD44780 and DOGM
    // Draw the prompt, buttons, and state
    static void draw_select_screen(
      PGM_P const yes,            // Right option label
      PGM_P const no,             // Left option label
      const bool yesno,           // Is "yes" selected?
      PGM_P const pref,           // Prompt prefix
      const char * const string,  // Prompt runtime string
      PGM_P const suff            // Prompt suffix
    );
    static void select_screen(
      PGM_P const yes, PGM_P const no,
      selectFunc_t yesFunc, selectFunc_t noFunc,
      PGM_P const pref, const char * const string=nullptr, PGM_P const suff=nullptr
    );
    static inline void select_screen(
      PGM_P const yes, PGM_P const no,
      selectFunc_t yesFunc, selectFunc_t noFunc,
      PGM_P const pref, const progmem_str string, PGM_P const suff=nullptr
    ) {
      char str[strlen_P((PGM_P)string) + 1];
      strcpy_P(str, (PGM_P)string);
      select_screen(yes, no, yesFunc, noFunc, pref, str, suff);
    }
    // Shortcut for prompt with "NO"/ "YES" labels
    FORCE_INLINE static void confirm_screen(selectFunc_t yesFunc, selectFunc_t noFunc, PGM_P const pref, const char * const string=nullptr, PGM_P const suff=nullptr) {
      select_screen(GET_TEXT(MSG_YES), GET_TEXT(MSG_NO), yesFunc, noFunc, pref, string, suff);
    }
};

// BACK_ITEM(LABEL)
class MenuItem_back : public MenuItemBase {
  public:
    // Back Item just draws the label with up-level indicators
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr) {
      _draw(sel, row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0]);
    }
    // Back Item action is to go back in the history
    FORCE_INLINE static void action(PGM_P const=nullptr) {
      lcdui.goto_previous_screen();
    }
};

// SUBMENU(LABEL, screen_handler)
class MenuItem_submenu : public MenuItemBase {
  public:
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, ...) {
      _draw(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0]);
    }
    static inline void action(PGM_P const, const screenFunc_t func) { lcdui.save_previous_screen(); lcdui.goto_screen(func); }
};

// Any menu item that invokes an immediate action
class MenuItem_button : public MenuItemBase {
  public:
    // Button-y Items are selectable lines with no other indicator
    static inline void draw(const bool sel, const uint8_t row, PGM_P const pstr, ...) {
      _draw(sel, row, pstr, '>', ' ');
    }
};

// GCODES_ITEM(LABEL, GCODES)
class MenuItem_gcode : public MenuItem_button {
  public:
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, ...) {
      _draw(sel, row, pstr, '>', ' ');
    }
    static void action(PGM_P const, const char * const pgcode);
    static inline void action(PGM_P const pstr, const uint8_t, const char * const pgcode) { action(pstr, pgcode); }
};

// ACTION_ITEM(LABEL, FUNC)
class MenuItem_function : public MenuItem_button {
  public:
    static inline void action(PGM_P const, const menuAction_t func) { (*func)(); };
};

#if HAS_SD_SUPPORT
  class SDCard;
  class MenuItem_sdbase {
    public:
      // Implemented for HD44780 and DOGM
      static void draw(const bool sel, const uint8_t row, PGM_P const pstr, SDCard &theCard, const bool isDir);
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
    static void goto_edit_screen(
      PGM_P const el,         // Edit label
      void * const ev,        // Edit value pointer
      const int32_t minv,     // Encoder minimum
      const int32_t maxv,     // Encoder maximum
      const uint16_t ep,      // Initial encoder value
      const screenFunc_t cs,  // MenuItem_type::draw_edit_screen => MenuEditItemBase::edit()
      const screenFunc_t cb,  // Callback after edit
      const bool le           // Flag to call cb() during editing
    );
    static void edit_screen(strfunc_t, loadfunc_t); // Edit value handler
  public:
    // Implemented for HD44780 and DOGM
    // Draw the current item at specified row with edit data
    static void draw(const bool sel, const uint8_t row, PGM_P const pstr,const char* const data, const bool pgm=false);

    // Implemented for HD44780 and DOGM
    // This low-level method is good to draw from anywhere
    static void draw_edit_screen(PGM_P const pstr, const char* const value);

    // This method is for the current menu item
    static inline void draw_edit_screen(const char* const value) { draw_edit_screen(editLabel, value); }
};

// Template for specific Menu Edit Item Types
template<typename NAME>
class TMenuEditItem : MenuEditItemBase {
  private:
    typedef typename NAME::type_t type_t;
    static inline float scale(const float value)      { return NAME::scale(value);            }
    static inline float unscale(const float value)    { return NAME::unscale(value);          }
    static const char* to_string(const int32_t value) { return NAME::strfunc(unscale(value)); }
    static void load(void *ptr, const int32_t value)  { *((type_t*)ptr) = unscale(value);     }
  public:
    // These methods call the specific type-plus-formatting class
    // for the given NAME.
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, type_t * const data, ...) {
      MenuEditItemBase::draw(sel, row, pstr, NAME::strfunc(*(data)));
    }
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, PGM_P const, type_t (*pget)(), ...) {
      MenuEditItemBase::draw(sel, row, pstr, NAME::strfunc(pget()));
    }
    // Edit screen for this type of item
    static void edit_screen() { MenuEditItemBase::edit_screen(to_string, load); }
    static void action(
      PGM_P const pstr,                     // Edit label
      type_t * const ptr,                   // Value pointer
      const type_t minValue,                // Value min
      const type_t maxValue,                // Value max
      const screenFunc_t callback=nullptr,  // Value update callback
      const bool live=false                 // Callback during editing
    ) {
      // Make sure minv and maxv fit within int32_t
      const int32_t minv = MAX(scale(minValue), INT32_MIN),
                    maxv = MIN(scale(maxValue), INT32_MAX);
      goto_edit_screen(pstr, ptr, minv, maxv - minv, scale(*ptr) - minv, edit_screen, callback, live);
    }
    
};

// Provide a set of Edit Item Types which encompass a primitive
// type, a string function, and a scale factor for edit and display.
// These items call the Edit Item draw method passing the prepared string.
#define DEFINE_MENU_EDIT_ITEM_TYPE(TYPE, NAME, FIX, STRFUNC, SCALE, V...) \
  struct MenuEditItemInfo_##NAME {                                        \
    typedef TYPE type_t;                                                  \
    static inline float scale(const float value)   { return value * (SCALE) + (V+0); } \
    static inline float unscale(const float value) { return value / (SCALE) + (V+0); } \
    static inline const char* strfunc(const float value) { return STRFUNC((TYPE)(FIX ? FIXFLOAT(value) : value)); } \
  }; \
  typedef TMenuEditItem<MenuEditItemInfo_##NAME> MenuItem_##NAME

//                          TYPE      NAME      FIX   STRFUNC       SCALE
DEFINE_MENU_EDIT_ITEM_TYPE(uint8_t,  percent,     0,  ui8tostr4pct, 100.0/255, 0.5);  // 100%       right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(int16_t,  int3,        0,  i16tostr3,       1          );  // 123, -12   right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(int16_t,  int4,        0,  i16tostr4sign,   1          );  // 1234, -123 right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(int8_t,   int8,        0,  i8tostr3,        1          );  // 123, -12   right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint8_t,  uint8,       0,  ui8tostr3,       1          );  // 123        right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, microstep,   0,  ui16tostr3,      1/16       );  // 16
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, uint16_3,    0,  ui16tostr3,      1          );  // 123        right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, uint16_4,    0,  ui16tostr4,      0.1        );  // 1234       right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint16_t, uint16_5,    0,  ui16tostr5,      0.01       );  // 12345      right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float3,      1,  ftostr3,         1          );  // 123        right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float52,     1,  ftostr42_52,   100          );  // _2.34, 12.34, -2.34 or 123.45, -23.45
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float43,     1,  ftostr43sign, 1000          );  // 1.234
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float5,      1,  ftostr5rj,       1          );  // 12345      right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float5_25,   1,  ftostr5rj,       0.04f      );  // 12345      right-justified (25 increment)
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float51,     1,  ftostr51rj,     10          );  // 1234.5     right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float41sign, 1,  ftostr41sign,   10          );  // +123.4
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float51sign, 1,  ftostr51sign,   10          );  // +1234.5
DEFINE_MENU_EDIT_ITEM_TYPE(float,    float52sign, 1,  ftostr52sign,  100          );  // +123.45
DEFINE_MENU_EDIT_ITEM_TYPE(uint32_t, long5,       0,  ftostr5rj,       0.01f      );  // 12345      right-justified
DEFINE_MENU_EDIT_ITEM_TYPE(uint32_t, long5_25,    0,  ftostr5rj,       0.04f      );  // 12345      right-justified (25 increment)

class MenuItem_bool : MenuEditItemBase {
  public:
    static void action(PGM_P const pstr, bool * const ptr, const screenFunc_t callbackFunc=nullptr);
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, const bool onoff) {
      MenuEditItemBase::draw(sel, row, pstr, onoff ? GET_TEXT(MSG_LCD_ON) : GET_TEXT(MSG_LCD_OFF), true);
    }
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, bool * const data, ...) {
      draw(sel, row, pstr,*data);
    }
    FORCE_INLINE static void draw(const bool sel, const uint8_t row, PGM_P const pstr, PGM_P const, bool (*pget)(), ...) {
      draw(sel, row, pstr, pget());
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
#define SCREEN_OR_MENU_LOOP(IS_MENU)                                                  \
  scroll_screen(IS_MENU ? 1 : LCD_HEIGHT, IS_MENU);                                   \
  int8_t _menuLineNr = encoderTopLine, _thisItemNr;                                   \
  bool _skipStatic = IS_MENU;                                                         \
  for (int8_t _lcdLineNr = 0; _lcdLineNr < LCD_HEIGHT; _lcdLineNr++, _menuLineNr++) { \
    _thisItemNr = 0

/**
 * START_SCREEN  Opening code for a screen having only static items.
 *               Do simplified scrolling of the entire screen.
 *
 * START_MENU    Opening code for a screen with menu items.
 *               Scroll as-needed to keep the selected line in view.
 */
#define START_SCREEN()  SCREEN_OR_MENU_LOOP(false)
#define START_MENU()    SCREEN_OR_MENU_LOOP(true)
#define NEXT_ITEM()     (++_thisItemNr)
#define SKIP_ITEM()     NEXT_ITEM()
#define END_SCREEN()    } screen_items = _thisItemNr
#define END_MENU()      END_SCREEN(); UNUSED(_skipStatic)

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
 *   BACK_ITEM(MSG_INFO_SCREEN)
 *     MenuItem_back::action(plabel, ...)
 *     MenuItem_back::draw(sel, row, plabel, ...)
 *
 *   ACTION_ITEM(MSG_PAUSE_PRINT, lcd_sdcard_pause)
 *     MenuItem_function::action(plabel, lcd_sdcard_pause)
 *     MenuItem_function::draw(sel, row, plabel, lcd_sdcard_pause)
 *
 *   EDIT_ITEM(int3, MSG_SPEED, &feedrate_percentage, 10, 999)
 *     MenuItem_int3::action(plabel, &feedrate_percentage, 10, 999)
 *     MenuItem_int3::draw(sel, row, plabel, &feedrate_percentage, 10, 999)
 */
#define _MENU_INNER_P(TYPE, USE_MULTIPLIER, PLABEL, V...) do {  \
  PGM_P const plabel = PLABEL;                                  \
  if (encoderLine == _thisItemNr && lcdui.use_click()) {        \
    _MENU_ITEM_MULTIPLIER_CHECK(USE_MULTIPLIER);                \
    MenuItem_##TYPE::action(plabel, ##V);                       \
    if (lcdui.screen_changed) return;                           \
  }                                                             \
  if (lcdui.should_draw())                                      \
    MenuItem_##TYPE::draw                                       \
      (encoderLine == _thisItemNr, _lcdLineNr, plabel, ##V);    \
}while(0)

// No Indexed items set a global index to NO_INDEX
#define _MENU_ITEM_P(TYPE, V...) do { \
  _skipStatic = false;                \
  if (_menuLineNr == _thisItemNr) {   \
    MenuItemBase::init();             \
    _MENU_INNER_P(TYPE, ##V);         \
  }                                   \
  NEXT_ITEM();                        \
}while(0)

// Indexed items set a global index value
#define _MENU_ITEM_N_P(TYPE, N, V...) do{ \
  _skipStatic = false;                    \
  if (_menuLineNr == _thisItemNr) {       \
    MenuItemBase::init(N);                \
    _MENU_INNER_P(TYPE, ##V);             \
  }                                       \
  NEXT_ITEM();                            \
}while(0)

#define MENU_ITEM_ADDON_START(X) do{ \
  if (lcdui.should_draw() && _menuLineNr == _thisItemNr - 1) { \
    SETCURSOR(X, _lcdLineNr)

#define MENU_ITEM_ADDON_END() } }while(0)

// STATIC_ITEM draws a styled string with no highlight.
// Parameters: label [, style [, char *value] ]
#define STATIC_ITEM_INNER_P(PLABEL, V...) do{             \
  if (_skipStatic && encoderLine <= _thisItemNr) {        \
    lcdui.encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
    ++encoderLine;                                        \
  }                                                       \
  if (lcdui.should_draw())                                \
    MenuItem_static::draw(_lcdLineNr, PLABEL, ##V);       \
} while(0)

#define STATIC_ITEM_P(PLABEL, V...) do{ \
  if (_menuLineNr == _thisItemNr) {     \
    MenuItemBase::init();               \
    STATIC_ITEM_INNER_P(PLABEL, ##V);   \
  }                                     \
  NEXT_ITEM();                          \
} while(0)

#define STATIC_ITEM_N_P(PLABEL, N, V...) do{  \
  if (_menuLineNr == _thisItemNr) {           \
    MenuItemBase::init(N);                    \
    STATIC_ITEM_INNER_P(PLABEL, ##V);         \
  }                                           \
  NEXT_ITEM();                                \
}while(0)

#define STATIC_ITEM(LABEL,        V...)       STATIC_ITEM_P(  GET_TEXT(LABEL),  ##V)
#define STATIC_ITEM_N(LABEL, N, V...)         STATIC_ITEM_N_P(GET_TEXT(LABEL),  ##V)

#define MENU_ITEM_P(TYPE, PLABEL, V...)       _MENU_ITEM_P(TYPE,        false,  PLABEL,           ##V)
#define MENU_ITEM(TYPE, LABEL, V...)           MENU_ITEM_P(TYPE,                GET_TEXT(LABEL),  ##V)

#define MENU_ITEM_N_P(TYPE, N, PLABEL, V...)  _MENU_ITEM_N_P(TYPE,  N,  false,  PLABEL,           ##V)
#define MENU_ITEM_N(TYPE, N, LABEL, V...)      MENU_ITEM_N_P(TYPE,  N,          GET_TEXT(LABEL),  ##V)

#define BACK_ITEM(LABEL)                       MENU_ITEM(back, LABEL)

#define ACTION_ITEM_P(PLABEL, ACTION)           MENU_ITEM_P(function,       PLABEL,           ACTION)
#define ACTION_ITEM(LABEL, ACTION)            ACTION_ITEM_P(                GET_TEXT(LABEL),  ACTION)
#define ACTION_ITEM_N_P(N, PLABEL, ACTION)    MENU_ITEM_N_P(function,   N,  PLABEL,           ACTION)
#define ACTION_ITEM_N(N, LABEL, ACTION)       ACTION_ITEM_N_P(          N,  GET_TEXT(LABEL),  ACTION)

#define GCODES_ITEM_P(PLABEL, GCODES)           MENU_ITEM_P(gcode,          PLABEL,           GCODES)
#define GCODES_ITEM(LABEL, GCODES)            GCODES_ITEM_P(                GET_TEXT(LABEL),  GCODES)
#define GCODES_ITEM_N_P(N, PLABEL, GCODES)    MENU_ITEM_N_P(gcode,      N,  PLABEL,           GCODES)
#define GCODES_ITEM_N(N, LABEL, GCODES)       GCODES_ITEM_N_P(          N,  GET_TEXT(LABEL),  GCODES)

#define SUBMENU_P(PLABEL, DEST)                 MENU_ITEM_P(submenu,        PLABEL,           DEST)
#define SUBMENU(LABEL, DEST)                    SUBMENU_P(                  GET_TEXT(LABEL),  DEST)
#define SUBMENU_N_P(N, PLABEL, DEST)          MENU_ITEM_N_P(submenu,    N,  PLABEL,           DEST)
#define SUBMENU_N(N, LABEL, DEST)               SUBMENU_N_P(            N,  GET_TEXT(LABEL),  DEST)

#define EDIT_ITEM_P(TYPE, PLABEL, V...)         MENU_ITEM_P(TYPE,           PLABEL,           ##V)
#define EDIT_ITEM(TYPE, LABEL, V...)            EDIT_ITEM_P(TYPE,           GET_TEXT(LABEL),  ##V)
#define EDIT_ITEM_N_P(TYPE, N, PLABEL, V...)  MENU_ITEM_N_P(TYPE,       N,  PLABEL,           ##V)
#define EDIT_ITEM_N(TYPE, N, LABEL, V...)     EDIT_ITEM_N_P(TYPE,       N,  GET_TEXT(LABEL),  ##V)

#define EDIT_ITEM_FAST_P(TYPE, PLABEL, V...)          _MENU_ITEM_P(TYPE,    true, PLABEL,           ##V)
#define EDIT_ITEM_FAST(TYPE, LABEL, V...)         EDIT_ITEM_FAST_P(TYPE,          GET_TEXT(LABEL),  ##V)
#define EDIT_ITEM_FAST_N_P(TYPE, N, PLABEL, V...)   _MENU_ITEM_N_P(TYPE, N, true, PLABEL,           ##V)
#define EDIT_ITEM_FAST_N(TYPE, N, LABEL, V...)  EDIT_ITEM_FAST_N_P(TYPE, N,       GET_TEXT(LABEL),  ##V)

#define _CONFIRM_ITEM_INNER_P(PLABEL, V...) do {                \
  if (encoderLine == _thisItemNr && lcdui.use_click()) {        \
    lcdui.goto_screen([]{MenuItem_confirm::select_screen(V);}); \
    return;                                                     \
  }                                                             \
  if (lcdui.should_draw()) MenuItem_confirm::draw               \
    (encoderLine == _thisItemNr, _lcdLineNr, PLABEL, ##V);      \
}while(0)

// No Indexed items set a global index to NO_INDEX
#define _CONFIRM_ITEM_P(PLABEL, V...) do {  \
  _skipStatic = false;                      \
  if (_menuLineNr == _thisItemNr) {         \
    MenuItemBase::init();                   \
    _CONFIRM_ITEM_INNER_P(PLABEL, ##V);     \
  }                                         \
  NEXT_ITEM();                              \
}while(0)

// Indexed items set a global index value
#define _CONFIRM_ITEM_N_P(N, V...) do{  \
  _skipStatic = false;                  \
  if (_menuLineNr == _thisItemNr) {     \
    MenuItemBase::init(N);              \
    _CONFIRM_ITEM_INNER_P(TYPE, ##V);   \
  }                                     \
  NEXT_ITEM();                          \
}while(0)

#define CONFIRM_ITEM_P(PLABEL,A,B,V...)       _CONFIRM_ITEM_P(    PLABEL,           GET_TEXT(A),  GET_TEXT(B),  ##V)
#define CONFIRM_ITEM(LABEL, V...)              CONFIRM_ITEM_P(    GET_TEXT(LABEL),                              ##V)

#define YESNO_ITEM_P(PLABEL, V...)            _CONFIRM_ITEM_P(    PLABEL,                                       ##V)
#define YESNO_ITEM(LABEL, V...)               _CONFIRM_ITEM_P(    GET_TEXT(LABEL),                              ##V)

#define CONFIRM_ITEM_N_P(N,PLABEL,A,B,V...) _CONFIRM_ITEM_N_P(N,  PLABEL,           GET_TEXT(A),  GET_TEXT(B),  ##V)
#define CONFIRM_ITEM_N(N,LABEL, V...)        CONFIRM_ITEM_N_P(N,  GET_TEXT(LABEL),                              ##V)

#define YESNO_ITEM_N_P(N,PLABEL, V...)      _CONFIRM_ITEM_N_P(N,  PLABEL,                                       ##V)
#define YESNO_ITEM_N(N,LABEL, V...)         _CONFIRM_ITEM_N_P(N,  GET_TEXT(LABEL),                              ##V)

////////////////////////////////////////////
/////////////// Menu Screens ///////////////
////////////////////////////////////////////

void menu_main();
void menu_move();

#if HAS_SD_SUPPORT
  void menu_sdcard();
#endif

#if HAS_EEPROM
  void lcd_eeprom_allert();
#endif

#if HAS_NEXTION_LCD
  void lcd_nextion_allert();
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

#if HAS_HOTENDS
  void watch_temp_callback_hotend();
#endif
#if HAS_BEDS
  void watch_temp_callback_bed();
#endif
#if HAS_CHAMBERS
  void watch_temp_callback_chamber();
#endif
#if HAS_COOLERS
  void watch_temp_callback_cooler();
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  void lcd_pause_show_message(const PauseMessageEnum message,
                              const PauseModeEnum mode=PAUSE_MODE_SAME,
                              const uint8_t hotend=toolManager.active_hotend());
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
