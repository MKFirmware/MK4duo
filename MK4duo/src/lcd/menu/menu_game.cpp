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
 
#include "../../../MK4duo.h"

#if HAS_GAME_MENU

#include "menu.h"
#include "game/game.h"

void menu_game() {
  START_MENU();
  BACK_ITEM(MSG_MAIN);
  #if ENABLED(GAME_BRICKOUT)
    SUBMENU(MSG_BRICKOUT, brickout.enter_game);
  #endif
  #if ENABLED(GAME_INVADERS)
    SUBMENU(MSG_INVADERS, invaders.enter_game);
  #endif
  #if ENABLED(GAME_SNAKE)
    SUBMENU(MSG_SNAKE, snake.enter_game);
  #endif
  #if ENABLED(GAME_MAZE)
    SUBMENU(MSG_MAZE, maze.enter_game);
  #endif
  END_MENU();
}

#endif // HAS_GAME_MENU
