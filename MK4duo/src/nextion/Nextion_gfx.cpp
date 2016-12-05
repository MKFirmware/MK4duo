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

#include "../../base.h"

#if ENABLED(NEXTION_GFX)

  #include "Nextion_gfx.h"

  const int INSIDE  = 0;  // 0000
  const int LEFT    = 1;  // 0001
  const int RIGHT   = 2;  // 0010
  const int BOTTOM  = 4;  // 0100
  const int TOP     = 8;  // 1000
 
  int GFX::ComputeOutCode(int x, int y, int w, int h) {
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

  void GFX::_line2d(const int ndx, const struct point* a, const struct point* b) {
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

    drawLine(_left + x0, _top + y0, _left + x1, _top + y1, _color[ndx]);
  }

  void GFX::line_to(const int ndx, const float *pos) {
    struct point loc;

    _flatten(pos, &loc);
    _line2d(ndx, &_cursor.point, &loc);

    for (int i = 0; i < 3; i++)
      _cursor.position[i] = pos[i];
    _cursor.point = loc;
  }

#endif // NEXTION_GFX
