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
 * nextion_gfx.cpp
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

#include "../../../MK4duo.h"

#if ENABLED(NEXTION_GFX)

#include "nextion_gfx.h"

GFX gfx = GFX(1, 1, 1, 1);

const int INSIDE  = 0;  // 0000
const int LEFT    = 1;  // 0001
const int RIGHT   = 2;  // 0010
const int BOTTOM  = 4;  // 0100
const int TOP     = 8;  // 1000

/** Private Parameters */
uint16_t  GFX::left               = 0,
          GFX::top                = 0,
          GFX::width              = 0,
          GFX::height             = 0,
          GFX::color_tool[NX_MAX] = { 0 };

float     GFX::scale  = 0.0f;

xyz_pos_t GFX::point_max{0},
          GFX::point_origin{0};

cursor_t  GFX::cursor;

/** Private Function */
int GFX::ComputeOutCode(const xy_uint_t &point, const int w, const int h) {
  int code;

  code = INSIDE;        // initialised as being inside of clip window

  if (point.x < 0)      // to the left of clip window
    code |= LEFT;
  else if (point.x > w) // to the right of clip window
    code |= RIGHT;
  if (point.y < 0)      // below the clip window
    code |= BOTTOM;
  else if (point.y > h) // above the clip window
    code |= TOP;

  return code;
}

uint16_t GFX::r5g6b5(const float *color_a, const float *cinc, int pixel) {
  return (((int)((color_a[0] + cinc[0] * pixel) * 31) & 0x1F) << 11) |
         (((int)((color_a[1] + cinc[1] * pixel) * 63) & 0x3F) << 5) |
         (((int)((color_a[2] + cinc[2] * pixel) * 31) & 0x1F) << 0);
}

void GFX::fcolor(float *c, uint16_t r5g6b5, const float y, float max_y) {
  float dim;

  max_y *= 1.5;

  dim = (max_y - y) / max_y;
  c[0] = ((r5g6b5 >> 11) & 0x1F) / 31.0 * dim;
  c[1] = ((r5g6b5 >>  5) & 0x3F) / 63.0 * dim;
  c[2] = ((r5g6b5 >>  0) & 0x1F) / 31.0 * dim;
}

void GFX::nextion_line2d_shade(const float *color_a, xy_uint_t &point_a, const float *color_b, xy_uint_t &point_b, const bool shade/*=false*/) {

  int outcode0 = ComputeOutCode(point_a, width, height);
  int outcode1 = ComputeOutCode(point_b, width, height);
  bool accept = false;

  while (true) {
    if (!(outcode0 | outcode1)) {
      accept = true;
      break;
    }
    else if (outcode0 & outcode1) {
      break;
    }
    else {
      xy_uint_t point_new;
      int outcodeOut = outcode0 ? outcode0 : outcode1;

      if (outcodeOut & TOP)
        point_new.set(point_a.x + (point_b.x - point_a.x) * (height - point_a.y) / (point_b.y - point_a.y), height);
      else if (outcodeOut & BOTTOM)
        point_new.set(point_a.x + (point_b.x - point_a.x) * ( - point_a.y) / (point_b.y - point_a.y), 0);
      else if (outcodeOut & RIGHT)
        point_new.set(width, point_a.y + (point_b.y - point_a.y) * (width - point_a.x) / (point_b.x - point_a.x));
      else if (outcodeOut & LEFT)
        point_new.set(0, point_a.y + (point_b.y - point_a.y) * ( - point_a.x) / (point_b.x - point_a.x));

      if (outcodeOut == outcode0) {
        point_a = point_new;
        outcode0 = ComputeOutCode(point_a, width, height);
      }
      else {
        point_b = point_new;
        outcode1 = ComputeOutCode(point_b, width, height);
      }
    }
  }

  if (!accept) return;

  if (shade) {
    int delta_x(point_b.x - point_a.x);
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = ABS(delta_x) << 1;

    int delta_y(point_b.y - point_a.y);
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = ABS(delta_y) << 1;

    float dist = SQRT((point_b.x - point_a.x) * (point_b.x - point_a.x) + (point_b.y - point_a.y) * (point_b.y - point_a.y));
    float cinc[3];
    int pixel = 0;

    LOOP_XYZ(i)
      cinc[i] = (color_b[i] - color_a[i]) / dist;

    drawPixel(left + point_a.x, top + point_a.y, r5g6b5(color_a, cinc, pixel++));

    if (delta_x >= delta_y) {
      int error(delta_y - (delta_x >> 1));

      while (point_a.x != point_b.x) {
        if ((error >= 0) && (error || (ix > 0))) {
          error -= delta_x;
          point_a.y += iy;
        }

        error += delta_y;
        point_a.x += ix;

        drawPixel(left + point_a.x, top + point_a.y, r5g6b5(color_a, cinc, pixel++));
      }
    }
    else {
      int error(delta_x - (delta_y >> 1));

      while (point_a.y != point_b.y) {
        if ((error >= 0) && (error || (iy > 0))) {
          error -= delta_y;
          point_a.x += ix;
        }

        error += delta_x;
        point_a.y += iy;

        drawPixel(left + point_a.x, top + point_a.y, r5g6b5(color_a, cinc, pixel++));
      }
    }
  }
  else {
    drawLine(left + point_a.x, top + point_a.y, left + point_b.x, top + point_b.y, color_tool[NX_TOOL]);
  }
}

void GFX::line_to(const uint8_t color_index, const xyz_pos_t &pos, const bool shade) {

  xy_uint_t loc;

  _flatten(pos, loc);

  if (color_index < NX_MAX) {
    float color1[3], color2[3];
    fcolor(color1, color_tool[color_index], cursor.position.y, point_max.y);
    fcolor(color2, color_tool[color_index], pos.y, point_max.y);
    nextion_line2d_shade(color1, cursor.point, color2, loc, shade);
  }

  cursor.position = pos;
  cursor.point = loc;

}

void GFX::fill(const xy_uint_t &point_a, const xy_uint_t &point_b, uint16_t color) {
  char buf0[10], buf1[10], buf2[10], buf3[10], buf4[10] = {0};
  String cmd;
  utoa(point_a.x, buf0, 10);
  utoa(point_a.y, buf1, 10);
  utoa(point_b.x, buf2, 10);
  utoa(point_b.y, buf3, 10);
  utoa(color, buf4,10);
  cmd += "fill ";
  cmd += buf0;
  cmd += ",";
  cmd += buf1;
  cmd += ",";
  cmd += buf2;
  cmd += ",";
  cmd += buf3;
  cmd += ",";
  cmd += buf4;
  nexlcd.sendCommand(cmd.c_str());
}

void GFX::drawLine(const int x0, const int y0, const int x1, const int y1, uint16_t color) {
  char bufx0[10], bufy0[10], bufx1[10], bufy1[10], bufc[10] = {0};
  String cmd;
  utoa(x0, bufx0, 10);
  utoa(y0, bufy0, 10);
  utoa(x1, bufx1, 10);
  utoa(y1, bufy1, 10);
  utoa(color, bufc, 10);
  cmd += "line ";
  cmd += bufx0;
  cmd += ",";
  cmd += bufy0;
  cmd += ",";
  cmd += bufx1;
  cmd += ",";
  cmd += bufy1;
  cmd += ",";
  cmd += bufc;
  nexlcd.sendCommand(cmd.c_str());
}

void GFX::drawPixel(const int x, const int y, uint16_t color) {
  char buf0[10], buf1[10], buf2[10] = {0};
  String cmd;
  utoa(x, buf0, 10);
  utoa(y, buf1, 10);
  utoa(color, buf2,10);
  cmd += "line ";
  cmd += buf0;
  cmd += ",";
  cmd += buf1;
  cmd += ",";
  cmd += buf0;
  cmd += ",";
  cmd += buf1;
  cmd += ",";
  cmd += buf2;
  nexlcd.sendCommand(cmd.c_str());
}

#endif // NEXTION_GFX
