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

#define BRICK_ROWS   4
#define BRICK_COLS  16

typedef struct {
  uint8_t   balls_left, brick_count;
  uint16_t  bricks[BRICK_ROWS];
  int8_t    paddle_x, hit_dir;
  fixed_t   ballx, bally, ballh, ballv;
} brickout_data_t;

class BrickoutGame : LcdGame { public: static void enter_game(), game_screen(); };
extern BrickoutGame brickout;
