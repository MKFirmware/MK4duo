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
// Motion Menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU

#if HAS_SPI_LCD
  #include "../ultralcd/lcdprint.h"
#endif

#if HAS_GRAPHICAL_LCD
  #include "../ultralcd/dogm/ultralcd_dogm.h"
#endif

extern short_timer_t manual_move_timer;
extern int8_t manual_move_axis;
#if ENABLED(MANUAL_E_MOVES_RELATIVE)
  float manual_move_e_origin = 0;
#endif
#if IS_KINEMATIC
  extern float manual_move_offset;
#endif

//
// Tell lcdui.update() to start a move to position.x" after a short delay.
//
inline void manual_move_to_current(AxisEnum axis, const int8_t eindex=-1) {
  if (axis == E_AXIS) lcdui.manual_move_e_index = eindex >= 0 ? eindex : toolManager.extruder.active;
  manual_move_timer.start(); // delay for bigger moves
  manual_move_axis = (int8_t)axis;
}

//
// "Motion" > "Move Axis" submenu
//

static void _lcd_move_xyz(PGM_P name, AxisEnum axis) {
  if (lcdui.use_click()) return lcdui.goto_previous_screen_no_defer();
  if (lcdui.encoderPosition && !lcdui.processing_manual_move) {

    // Start with no limits to movement
    float min = mechanics.position[axis] - 1000,
          max = mechanics.position[axis] + 1000;

    // Limit to software endstops, if enabled
    #if HAS_SOFTWARE_ENDSTOPS && NOMECH(DELTA)

      if (endstops.isSoftEndstop()) switch (axis) {
        case X_AXIS:
          #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
            min = endstops.soft_endstop.min.x;
          #endif
          #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
            max = endstops.soft_endstop.max.x;
          #endif
          break;
        case Y_AXIS:
          #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
            min = endstops.soft_endstop.min.y;
          #endif
          #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
            max = endstops.soft_endstop.max.y;
          #endif
          break;
        case Z_AXIS:
          #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
            min = endstops.soft_endstop.min.z;
          #endif
          #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
            max = endstops.soft_endstop.max.z;
          #endif
        default: break;
      }

    #elif MECH(DELTA)

      // Delta limits XY based on the current offset from center
      // This assumes the center is 0,0
      if (axis != Z_AXIS) {
        max = SQRT(sq((float)(mechanics.data.print_radius)) - sq(mechanics.position[Y_AXIS - axis]));
        min = -max;
      }

    #endif

    // Get the new position
    const float diff = float(int32_t(lcdui.encoderPosition)) * move_menu_scale;
    #if IS_KINEMATIC
      manual_move_offset += diff;
      NOLESS(manual_move_offset, min - mechanics.position[axis]);
      NOMORE(manual_move_offset, max - mechanics.position[axis]);
    #else
      mechanics.position[axis] += diff;
      NOLESS(mechanics.position[axis], min);
      NOMORE(mechanics.position[axis], max);
    #endif

    manual_move_to_current(axis);
    lcdui.refresh(LCDVIEW_REDRAW_NOW);
  }
  lcdui.encoderPosition = 0;
  if (lcdui.should_draw()) {
    const float pos = NATIVE_TO_LOGICAL(lcdui.processing_manual_move ? mechanics.destination[axis] : mechanics.position[axis]
      #if IS_KINEMATIC
        + manual_move_offset
      #endif
    , axis);
    MenuEditItemBase::draw_edit_screen(name, move_menu_scale >= 0.1f ? ftostr41sign(pos) : ftostr43sign(pos));
  }
}
void lcd_move_x() { _lcd_move_xyz(GET_TEXT(MSG_MOVE_X), X_AXIS); }
void lcd_move_y() { _lcd_move_xyz(GET_TEXT(MSG_MOVE_Y), Y_AXIS); }
void lcd_move_z() { _lcd_move_xyz(GET_TEXT(MSG_MOVE_Z), Z_AXIS); }

void lcd_move_e(const int8_t eindex=-1) {
  if (lcdui.use_click()) return lcdui.goto_previous_screen_no_defer();
  if (lcdui.encoderPosition) {
    if (!lcdui.processing_manual_move) {
      const float diff = float(int32_t(lcdui.encoderPosition)) * move_menu_scale;
      #if IS_KINEMATIC
        manual_move_offset += diff;
      #else
        mechanics.position.e += diff;
      #endif
      manual_move_to_current(E_AXIS, eindex);
      lcdui.refresh(LCDVIEW_REDRAW_NOW);
    }
    lcdui.encoderPosition = 0;
  }
  if (lcdui.should_draw()) {
    MenuItemBase::init(eindex);
    MenuEditItemBase::draw_edit_screen(GET_TEXT(MSG_MOVE_E), ftostr41sign(mechanics.position.e
      #if IS_KINEMATIC
        + manual_move_offset
      #endif
    ));
  }
}

/**
 *
 * "Motion" > "Move Xmm" > "Move XYZ" submenu
 *
 */

screenFunc_t _manual_move_func_ptr;

void _goto_manual_move(const float scale) {
  lcdui.defer_status_screen();
  move_menu_scale = scale;
  lcdui.goto_screen(_manual_move_func_ptr);
}

void _menu_move_distance(const AxisEnum axis, const screenFunc_t func, const int8_t eindex=-1) {
  _manual_move_func_ptr = func;
  START_MENU();
  if (LCD_HEIGHT >= 4) {
    switch (axis) {
      case X_AXIS: STATIC_ITEM(MSG_MOVE_X, SS_CENTER|SS_INVERT); break;
      case Y_AXIS: STATIC_ITEM(MSG_MOVE_Y, SS_CENTER|SS_INVERT); break;
      case Z_AXIS: STATIC_ITEM(MSG_MOVE_Z, SS_CENTER|SS_INVERT); break;
      default:
        STATIC_ITEM(MSG_MOVE_E, SS_CENTER|SS_INVERT); break;
    }
  }
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    if (axis == E_AXIS && tempManager.tooColdToExtrude(eindex >= 0 ? eindex : toolManager.extruder.active))
      BACK_ITEM(MSG_HOTEND_TOO_COLD);
    else
  #endif
  {
    BACK_ITEM(MSG_MOVE_AXIS);
    SUBMENU(MSG_MOVE_10MM,  []{ _goto_manual_move(10);    });
    SUBMENU(MSG_MOVE_1MM,   []{ _goto_manual_move( 1);    });
    SUBMENU(MSG_MOVE_01MM,  []{ _goto_manual_move( 0.1f); });
    if (axis == Z_AXIS && (SHORT_MANUAL_Z_MOVE) > 0.0f && (SHORT_MANUAL_Z_MOVE) < 0.1f) {
      SUBMENU_P(NULL_STR, []{ _goto_manual_move(float(SHORT_MANUAL_Z_MOVE)); });
      MENU_ITEM_ADDON_START(
        #if HAS_GRAPHICAL_LCD
          0
        #else 
          1
        #endif
      );
        char tmp[20], numstr[10];
        // Determine digits needed right of decimal
        const uint8_t digs = !UNEAR_ZERO((SHORT_MANUAL_Z_MOVE) * 1000 - int((SHORT_MANUAL_Z_MOVE) * 1000)) ? 4 :
                             !UNEAR_ZERO((SHORT_MANUAL_Z_MOVE) *  100 - int((SHORT_MANUAL_Z_MOVE) *  100)) ? 3 : 2;
        sprintf_P(tmp, GET_TEXT(MSG_MOVE_Z_DIST), dtostrf(SHORT_MANUAL_Z_MOVE, 1, digs, numstr));
        LCDPRINT(tmp);
      MENU_ITEM_ADDON_END();
    }
  }
  END_MENU();
}

/**
 *
 * "Motion" > "Move Axis" submenu
 *
 */
void menu_move() {
  START_MENU();
  BACK_ITEM(MSG_MOTION);

  #if HAS_SOFTWARE_ENDSTOPS
    bool new_soft_endstop_state = endstops.isSoftEndstop();
    EDIT_ITEM(bool, MSG_LCD_SOFT_ENDSTOPS, &new_soft_endstop_state, []{ endstops.setSoftEndstop(!endstops.isSoftEndstop()); });
  #endif

  #if IS_KINEMATIC
    const bool do_move_xyz = mechanics.isHomedAll();
  #else
    constexpr bool do_move_xyz = true;
  #endif
  if (true
    #if MECH(DELTA)
      && mechanics.isHomedAll()
    #endif
  ) {
    if (true
      #if MECH(DELTA)
        && mechanics.position.z <= mechanics.delta_clip_start_height
      #endif
    ) {
      SUBMENU(MSG_MOVE_X, []{ _menu_move_distance(X_AXIS, lcd_move_x); });
      SUBMENU(MSG_MOVE_Y, []{ _menu_move_distance(Y_AXIS, lcd_move_y); });
    }
    #if MECH(DELTA)
      else
        ACTION_ITEM(MSG_FREE_XY, []{
          mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height, manual_feedrate_mm_s.z);
          lcdui.synchronize();
        });
    #endif

    SUBMENU(MSG_MOVE_Z, []{ _menu_move_distance(Z_AXIS, lcd_move_z); });
  }
  else
    GCODES_ITEM(MSG_AUTO_HOME, G28_CMD);

  #if ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR) || ENABLED(DUAL_X_CARRIAGE)

    if (toolManager.extruder.active)
      GCODES_ITEM_N(0, MSG_SELECT, PSTR("T0"));
    else
      GCODES_ITEM_N(1, MSG_SELECT, PSTR("T1"));

  #endif

  // The current extruder
  SUBMENU(MSG_MOVE_E, []{ _menu_move_distance(E_AXIS, []{ lcd_move_e(); }, -1); });

  if (toolManager.extruder.total > 1) {
    LOOP_EXTRUDER()
      SUBMENU_N(e, MSG_MOVE_E, []{ _menu_move_distance(E_AXIS, []{ lcd_move_e(MenuItemBase::itemIndex); }, MenuItemBase::itemIndex); });
  }

  END_MENU();
}

#if ENABLED(AUTO_BED_LEVELING_UBL)
  void _lcd_ubl_level_bed();
#elif ENABLED(LCD_BED_LEVELING)
  void menu_bed_leveling();
#endif

void menu_motion() {
  START_MENU();

  //
  // ^ Main
  //
  BACK_ITEM(MSG_MAIN);

  //
  // Move Axis
  //
  #if ENABLED(DELTA)
    if (mechanics.isHomedAll())
  #endif
      SUBMENU(MSG_MOVE_AXIS, menu_move);

  //
  // Auto Home
  //
  if (printer.mode == PRINTER_MODE_LASER)
    GCODES_ITEM(MSG_AUTO_HOME, PSTR("G28 X Y F2000"));
  else {
    GCODES_ITEM(MSG_AUTO_HOME, G28_CMD);
    #if NOMECH(DELTA)
      GCODES_ITEM(MSG_AUTO_HOME_X, PSTR("G28 X"));
      GCODES_ITEM(MSG_AUTO_HOME_Y, PSTR("G28 Y"));
      GCODES_ITEM(MSG_AUTO_HOME_Z, PSTR("G28 Z"));
    #endif
  }

  //
  // Auto Z-Align
  //
  #if ENABLED(Z_STEPPER_AUTO_ALIGN)
    GCODES_ITEM(MSG_AUTO_Z_ALIGN, PSTR("G34"));
  #endif

  //
  // Level Bed
  //
  #if ENABLED(AUTO_BED_LEVELING_UBL)

    SUBMENU(MSG_UBL_LEVEL_BED, _lcd_ubl_level_bed);

  #elif ENABLED(LCD_BED_LEVELING)

    #if HAS_PROBE_MANUALLY
      if (!bedlevel.flag.g29_in_progress)
    #endif
        SUBMENU(MSG_BED_LEVELING, menu_bed_leveling);

  #elif HAS_LEVELING && DISABLED(SLIM_LCD_MENUS)

    #if DISABLED(PROBE_MANUALLY)
      GCODES_ITEM(MSG_LEVEL_BED, PSTR("G28\nG29"));
    #endif
    if (mechanics.isHomedAll() && bedlevel.leveling_is_valid()) {
      bool new_level_state = bedlevel.flag.leveling_active;
      EDIT_ITEM(bool, MSG_BED_LEVELING, &new_level_state, []{ bedlevel.set_bed_leveling_enabled(!bedlevel.flag.leveling_active); });
    }
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      editable.decimal = bedlevel.z_fade_height;
    EDIT_ITEM_FAST(float3, MSG_Z_FADE_HEIGHT, &editable.decimal, 0, 100, []{ bedlevel.set_z_fade_height(editable.decimal); });
    #endif

  #endif

  #if ENABLED(LEVEL_BED_CORNERS) && DISABLED(LCD_BED_LEVELING)
    ACTION_ITEM(MSG_LEVEL_CORNERS, lcd_level_bed_corners);
  #endif

  //
  // Disable Steppers
  //
  GCODES_ITEM(MSG_DISABLE_STEPPERS, PSTR("M84"));

  END_MENU();
}

#endif // HAS_LCD_MENU
