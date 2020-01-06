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

/**
 * Included in Marlin to Thinkyhead
 */

#include "../../../../MK4duo.h"

#if HAS_GAMES

#include "game.h"

int LcdGame::score;
uint8_t LcdGame::game_state;
millis_l LcdGame::next_frame;

game_data_t game_data;

bool LcdGame::game_frame() {
  static int8_t slew;
  if (lcdui.first_page) slew = 2;
  lcdui.refresh(LCDVIEW_CALL_NO_REDRAW); // Refresh as often as possible
  return (game_state && slew-- > 0);
}

void LcdGame::draw_game_over() {
  constexpr int8_t gowide = (MENU_FONT_WIDTH) * 9,
                   gohigh = MENU_FONT_ASCENT - 3,
                       lx = (LCD_PIXEL_WIDTH - gowide) / 2,
                       ly = (LCD_PIXEL_HEIGHT + gohigh) / 2;
  if (PAGE_CONTAINS(ly - gohigh - 1, ly + 1)) {
    u8g.setColorIndex(0);
    u8g.drawBox(lx - 1, ly - gohigh - 1, gowide + 2, gohigh + 2);
    u8g.setColorIndex(1);
    if (lcdui.get_blink()) lcd_put_u8str_P(lx, ly, PSTR("GAME OVER"));
  }
}

void LcdGame::init_game(const uint8_t init_state, const screenFunc_t screen) {
  score = 0;
  game_state = init_state;
  lcdui.goto_screen(screen);
  lcdui.defer_status_screen();
}

void LcdGame::exit_game() {
  lcdui.goto_previous_screen_no_defer();
}

#endif // HAS_GAMES
