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

/**
 * Included in Marlin to Thinkyhead
 */

#include "../menu.h"
#include "../../ultralcd/dogm/ultralcd_dogm.h"
#include "../../ultralcd/lcdprint.h"

#define SCREEN_M ((LCD_PIXEL_WIDTH) / 2)

typedef struct { int8_t x, y; } pos_t;

// Simple 8:8 fixed-point
typedef int16_t fixed_t;
#define FTOP(F) fixed_t((F)*256.0f)
#define PTOF(P) (float(P)*(1.0f/256.0f))
#define BTOF(X) (fixed_t(X)<<8)
#define FTOB(X) int8_t(fixed_t(X)>>8)

#if HAS_GAME_MENU
  void menu_game();
#endif

class LcdGame {

  protected: /** Protected Parameters */

    static int score;
    static uint8_t game_state;
    static millis_l next_frame;

  protected: /** Protected Function */

    static bool game_frame();
    static void draw_game_over();
    static void exit_game();

  public: /** Public Function */

    static void init_game(const uint8_t init_state, const screenFunc_t screen);

};

#if ENABLED(GAME_BRICKOUT)
  #include "brickout.h"
#endif
#if ENABLED(GAME_INVADERS)
  #include "invaders.h"
#endif
#if ENABLED(GAME_MAZE)
  #include "maze.h"
#endif
#if ENABLED(GAME_SNAKE)
  #include "snake.h"
#endif

// Pool game data to save SRAM
union game_data_t {
  #if ENABLED(GAME_BRICKOUT)
    brickout_data_t brickout;
  #endif
  #if ENABLED(GAME_INVADERS)
    invaders_data_t invaders;
  #endif
  #if ENABLED(GAME_MAZE)
    maze_data_t maze;
  #endif
  #if ENABLED(GAME_SNAKE)
    snake_data_t snake;
  #endif
};

extern game_data_t game_data;
