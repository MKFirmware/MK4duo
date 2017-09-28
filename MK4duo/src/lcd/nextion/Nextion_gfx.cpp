/*
 * Copyright (C) 2015, MagoKimbra
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

#include "../../../base.h"

#if ENABLED(NEXTION_GFX)

  #include "Nextion_gfx.h"

  const int INSIDE  = 0;  // 0000
  const int LEFT    = 1;  // 0001
  const int RIGHT   = 2;  // 0010
  const int BOTTOM  = 4;  // 0100
  const int TOP     = 8;  // 1000

  int GFX::ComputeOutCode(const int x, const int y, const int w, const int h) {
    int code;

    code = INSIDE;       // initialised as being inside of clip window

    if (x < 0)           // to the left of clip window
      code |= LEFT;
    else if (x > w)      // to the right of clip window
      code |= RIGHT;
    if (y < 0)           // below the clip window
      code |= BOTTOM;
    else if (y > h)      // above the clip window
      code |= TOP;

    return code;
  }

  uint16_t GFX::r5g6b5(const float *color_a, const float *cinc, int pixel) {
    return (((int)((color_a[0] + cinc[0] * pixel) * 31) & 0x1f) << 11) |
           (((int)((color_a[1] + cinc[1] * pixel) * 63) & 0x3f) << 5) |
           (((int)((color_a[2] + cinc[2] * pixel) * 31) & 0x1f) << 0);
  }

  void GFX::fcolor(float *c, uint16_t r5g6b5, float y, float max_y) {
    float dim;

    max_y *= 1.5;

    dim = (max_y - y) / max_y;
    c[0] = ((r5g6b5 >> 11) & 0x1f) / 31.0 * dim;
    c[1] = ((r5g6b5 >>  5) & 0x3f) / 63.0 * dim;
    c[2] = ((r5g6b5 >>  0) & 0x1f) / 31.0 * dim;
  }

  void GFX::nextion_line2d_shade(const float *color_a, const struct Point *a, const float *color_b, const struct Point *b, bool shade) {
    int x0, y0, x1, y1;

    x0 = a->x;
    y0 = a->y;
    x1 = b->x;
    y1 = b->y;

    int outcode0 = ComputeOutCode(x0, y0, _width, _height);
    int outcode1 = ComputeOutCode(x1, y1, _width, _height);
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
        int x = 0, y = 0;
        int outcodeOut = outcode0 ? outcode0 : outcode1;

        if (outcodeOut & TOP) {
          x = x0 + (x1 - x0) * (_height - y0) / (y1 - y0);
          y = _height;
        }
        else if (outcodeOut & BOTTOM) {
          x = x0 + (x1 - x0) * ( - y0) / (y1 - y0);
          y = 0;
        }
        else if (outcodeOut & RIGHT) {
          y = y0 + (y1 - y0) * (_width - x0) / (x1 - x0);
          x = _width;
        }
        else if (outcodeOut & LEFT) {
          y = y0 + (y1 - y0) * ( - x0) / (x1 - x0);
          x = 0;
        }

        if (outcodeOut == outcode0) {
          x0 = x;
          y0 = y;
          outcode0 = ComputeOutCode(x0, y0, _width, _height);
        }
        else {
          x1 = x;
          y1 = y;
          outcode1 = ComputeOutCode(x1, y1, _width, _height);
        }
      }
    }

    if (!accept) return;

    if (shade) {
      int delta_x(x1 - x0);
      signed char const ix((delta_x > 0) - (delta_x < 0));
      delta_x = abs(delta_x) << 1;

      int delta_y(y1 - y0);
      signed char const iy((delta_y > 0) - (delta_y < 0));
      delta_y = abs(delta_y) << 1;

      float dist = SQRT((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
      float cinc[3];
      int pixel = 0;

      for (int i = 0; i < 3; i++)
        cinc[i] = (color_b[i] - color_a[i]) / dist;

      drawPixel(_left + x0, _top + y0, r5g6b5(color_a, cinc, pixel++));

      if (delta_x >= delta_y) {
        int error(delta_y - (delta_x >> 1));

        while (x0 != x1) {
          if ((error >= 0) && (error || (ix > 0))) {
            error -= delta_x;
            y0 += iy;
          }

          error += delta_y;
          x0 += ix;

          drawPixel(_left + x0, _top + y0, r5g6b5(color_a, cinc, pixel++));
        }
      }
      else {
        int error(delta_x - (delta_y >> 1));

        while (y0 != y1) {
          if ((error >= 0) && (error || (iy > 0))) {
            error -= delta_y;
            x0 += ix;
          }

          error += delta_x;
          y0 += iy;

          drawPixel(_left + x0, _top + y0, r5g6b5(color_a, cinc, pixel++));
        }
      }
    }
    else {
      drawLine(_left + x0, _top + y0, _left + x1, _top + y1, color_tool[NX_TOOL]);
    }
  }

  void GFX::line_to(const uint8_t color_index, const float *pos, bool shade) {

    struct Point loc;

    _flatten(pos, &loc);

    if (color_index >= 0 && color_index < NX_MAX) {
      float color1[3], color2[3];
      fcolor(color1, color_tool[color_index], _cursor.position[Y_AXIS], _max[Y_AXIS]);
      fcolor(color2, color_tool[color_index], pos[Y_AXIS], _max[Y_AXIS]);
      nextion_line2d_shade(color1, &_cursor.point, color2, &loc, shade);
    }

    for (int i = 0; i < 3; i++) _cursor.position[i] = pos[i];

    _cursor.point = loc;
  }

#endif // NEXTION_GFX
