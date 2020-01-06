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
 * nextion_gfx.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 * All right reserved.
 * Author: Alberto Cotronei <magokimbra@hotmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#if ENABLED(NEXTION_GFX)

#include "nextionlcd.h"

#define NX_AXIS         0
#define NX_MOVE         3
#define NX_TOOL         4
#define NX_BACKGROUND   5
#define NX_LOW          6
#define NX_HIGH         7
#define NX_MAX          8
#define NX_SCALE        4.0

struct cursor_t {
  xy_uint_t point;
  xyz_pos_t position;
};
    
class GFX {

  public: /** Constructor */

    GFX(const uint16_t px=0, const uint16_t py=0, const uint16_t pwidth=1, const uint16_t pheight=1) {

      left   = px;
      top    = py;
      width  = pwidth;
      height = pheight;

      point_max = 1000.0f;
      for (uint8_t i = 0; i < NX_MAX; i++) color_tool[i] = 65535;

      color_tool[NX_BACKGROUND] = 0;
    }

  private: /** Private Parameters */

    /* Location of visualization in NEXTION LCD*/
    static uint16_t   left, top, width, height,
                      color_tool[NX_MAX];
    
    static float      scale;
    
    static xyz_pos_t  point_max,
                      point_origin;

    static cursor_t   cursor;

  public: /** Public Function */

    static void set_position(const uint16_t px = 0, const uint16_t py = 0, const uint16_t pwidth = 1, const uint16_t pheight = 1) {
      left   = px;
      top    = py;
      width  = pwidth;
      height = pheight;
    }

    static void clear() {
      const xyz_pos_t zero{0.0f};
      const xy_uint_t point_a = { left, top },
                      point_b = { width, height };

      fill(point_a, point_b, color_tool[NX_BACKGROUND]);

      LOOP_XYZ(i) {
        xyz_pos_t pos{0.0f};
        pos[i] = point_max[i];
        cursor_to(zero);
        line_to(NX_AXIS + i, pos, true);
      }
      cursor_to(zero);
    }

    static void set_scale(const float _scale) {
      scale = _scale;
      clear();
    }

    static void clear(const xyz_pos_t &pos) {
      /* Bounding box for the build volume */
      const float scale_y = height / (pos.z + pos.y / NX_SCALE);
      const float scale_x = width  / (pos.x + pos.y / NX_SCALE);

      point_max = pos;

      set_scale(scale_x > scale_y ? scale_y : scale_x);
    }

    static void origin(const xyz_pos_t &pos) {
      point_origin = pos;
    }

    static void color_set(const uint8_t color_index, uint16_t color) {
      color_tool[color_index] = color;
    }

    static void cursor_to(const xyz_pos_t &pos) {
      cursor.position = pos;
      _flatten(cursor.position, cursor.point);
    }

    static void line_to(const uint8_t color_index, const xyz_pos_t &pos, const bool shade=false);

    static inline void line_to(const uint8_t color_index, const float x, const float y, const float z, const bool shade=false) {
      const xyz_pos_t pos = { x, y, z };
      line_to(color_index, pos, shade);
    }

  private: /** Private Function */

    static int ComputeOutCode(const xy_uint_t &point, const int w, const int h);

    static uint16_t r5g6b5(const float *color_a, const float *cinc, const int pixel);

    static void fcolor(float *c, uint16_t r5g6b5, const float y, float max_y);

    static void _flatten(const xyz_pos_t &pos, xy_uint_t &pt) {
      pt.x  = ((pos.x - point_origin.x) +
               (pos.y - point_origin.y) / NX_SCALE) * scale + 1;
      pt.y  = (height - 1) -
              ((pos.z - point_origin.z) +
               (pos.y - point_origin.y) / NX_SCALE) * scale - 1;
    }

    static void nextion_line2d_shade( const float *a_color, xy_uint_t &point_a,
                                      const float *b_color, xy_uint_t &point_b,
                                      const bool shade=false);

    static void fill(const xy_uint_t &point_a, const xy_uint_t &point_b, uint16_t color);

    static void drawLine(const int x0, const int y0, const int x1, const int y1, uint16_t color);
    static void drawPixel(const int x, const int y, uint16_t color);

};

extern GFX gfx;

#endif // NEXTION_GFX
