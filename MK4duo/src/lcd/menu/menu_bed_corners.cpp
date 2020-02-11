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
// Level Bed Corners menu
//

#include "../../../MK4duo.h"

#if HAS_LCD_MENU && ENABLED(LEVEL_BED_CORNERS)

/**
 * Level corners, starting in the front-left corner.
 */
static int8_t bed_corner;
void _lcd_goto_next_corner() {

  constexpr float lfrb[4] = LEVEL_CORNERS_INSET_LFRB;
  const xy_pos_t  lf { (X_MIN_BED) + lfrb[0], (Y_MIN_BED) + lfrb[1] },
                  rb { (X_MAX_BED) - lfrb[2], (Y_MAX_BED) - lfrb[3] };

  mechanics.do_blocking_move_to_z(LEVEL_CORNERS_Z_HOP, manual_feedrate_mm_s.z);

  switch (bed_corner) {
    case 0: mechanics.position    = lf;   break;
    case 1: mechanics.position.x  = rb.x; break;
    case 2: mechanics.position.y  = rb.y; break;
    case 3: mechanics.position.x  = lf.x; break;
    #if ENABLED(LEVEL_CENTER_TOO)
      case 4: mechanics.position.set(X_CENTER, Y_CENTER); break;
    #endif
  }
  mechanics.line_to_position(manual_feedrate_mm_s.x);
  mechanics.do_blocking_move_to_z(LEVEL_CORNERS_HEIGHT, manual_feedrate_mm_s.z);
  if (++bed_corner > 3
    #if ENABLED(LEVEL_CENTER_TOO)
      + 1
    #endif
  ) bed_corner = 0;
}

static inline void _lcd_level_bed_corners_homing() {
  lcd_draw_homing();
  if (mechanics.isHomedAll()) {
    bed_corner = 0;
    lcdui.goto_screen([]{
      MenuItem_confirm::select_screen(
        GET_TEXT(MSG_BUTTON_NEXT), GET_TEXT(MSG_BUTTON_DONE),
        _lcd_goto_next_corner,
        []{
          lcdui.goto_previous_screen_no_defer();
        },
        GET_TEXT(
          #if ENABLED(LEVEL_CENTER_TOO)
            MSG_LEVEL_BED_NEXT_POINT
          #else
            MSG_NEXT_CORNER
          #endif
        ), (PGM_P)nullptr, PSTR("?")
      );
    });
    lcdui.set_selection(true);
    _lcd_goto_next_corner();
  }
}

void lcd_level_bed_corners() {
  lcdui.defer_status_screen();
  if (!mechanics.isHomedAll()) commands.inject_P(G28_CMD);
  lcdui.goto_screen(_lcd_level_bed_corners_homing);
}

#endif // HAS_LCD_MENU && LEVEL_BED_CORNERS
