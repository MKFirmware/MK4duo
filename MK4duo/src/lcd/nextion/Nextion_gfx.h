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

#ifndef _NEXTION_GFX_H_
#define _NEXTION_GFX_H_

#if ENABLED(NEXTION_GFX)

  #include "nextion_lib/Nextion.h"

  #define NX_AXIS         0
  #define NX_MOVE         3
  #define NX_TOOL         4
  #define NX_BACKGROUND   5
  #define NX_LOW          6
  #define NX_HIGH         7
  #define NX_MAX          8
  #define NX_SCALE        4.0

  struct Point {
    uint16_t x, y;
  };

  class GFX {

    public: /** Constructor */

      GFX(const uint16_t px=0, const uint16_t py=0, const uint16_t width=1, const uint16_t height=1) {

        _left   = px;
        _top    = py;
        _width  = width;
        _height = height;

        for (uint8_t i = 0; i < 3; i++) _max[i] = 1000.0;
        for (uint8_t i = 0; i < NX_MAX; i++) color_tool[i] = 65535;

        color_tool[NX_BACKGROUND] = 0;
      }

    private: /** Private Parameters */

      /* Location of visualization in NEXTION LCD*/
      uint16_t _top, _left, _width, _height;
      float _scale, _max[3], _origin[3];

      uint16_t color_tool[NX_MAX];

      struct {
        struct Point point;
        float position[3];
      } _cursor;

    public: /** Public Function */

      void set_position(const uint16_t px = 0, const uint16_t py = 0, const uint16_t width = 1, const uint16_t height = 1) {
        _left   = px;
        _top    = py;
        _width  = width;
        _height = height;
      }

      void clear() {
        const float zero[3] = { 0, 0, 0 };
        fill(_left, _top, _width, _height, color_tool[NX_BACKGROUND]);

        for (uint8_t i = 0; i < 3; i++) {
          float pos[3] = { 0, 0, 0 };
          pos[i] = _max[i];
          cursor_to(zero);
          line_to(NX_AXIS + i, pos, true);
        }
        cursor_to(zero);
      }

      void set_scale(const float scale) { _scale = scale; }

      void clear(const float x_mm, const float y_mm, const float z_mm) {
        /* Bounding box for the build volume */
        const float scale_y = _height / (z_mm + y_mm / NX_SCALE);
        const float scale_x = _width  / (x_mm + y_mm / NX_SCALE);

        _max[X_AXIS] = x_mm;
        _max[Y_AXIS] = y_mm;
        _max[Z_AXIS] = z_mm;

        set_scale(scale_x > scale_y ? scale_y : scale_x);
        clear();
      }

      void origin(const float x, const float y, const float z) {
        _origin[X_AXIS] = x;
        _origin[Y_AXIS] = y;
        _origin[Z_AXIS] = z;
      }

      void color_set(const uint8_t color_index, uint16_t color) {
        color_tool[color_index] = color;
      }

      void cursor_to(const float *pos) {
        for (int i = 0; i < 3; i++)
          _cursor.position[i] = pos[i];
        _flatten(_cursor.position, &_cursor.point);
      }

      void cursor_to(float x, float y, float z) {
        const float pos[3] = { x, y, z };
        cursor_to(pos);
      }

      void line_to(const uint8_t color_index, const float *pos, bool shade=false);

      void line_to(const uint8_t color_index, float x, float y, float z, bool shade=false) {
        float pos[3] = { x, y, z };
        line_to(color_index, pos, shade);
      }

    private: /** Private Function */

      int ComputeOutCode(const int x, const int y, const int w, const int h);

      uint16_t r5g6b5(const float *color_a, const float *cinc, const int pixel);

      void fcolor(float *c, uint16_t r5g6b5, float y, float max_y);

      void _flatten(const float *pos, struct Point *pt) {
        pt->x = ((pos[X_AXIS] - _origin[X_AXIS]) +
                 (pos[Y_AXIS] - _origin[Y_AXIS]) / NX_SCALE) * _scale + 1;
        pt->y = (_height - 1) -
                ((pos[Z_AXIS] - _origin[Z_AXIS]) +
                 (pos[Y_AXIS] - _origin[Y_AXIS]) / NX_SCALE) * _scale - 1;
      }

      void nextion_line2d_shade(const float *a_color, const struct Point *a,
                                const float *b_color, const struct Point *b,
                                bool shade=false);

      void fill(const int x0, const int y0, const int x1, const int y1, uint16_t color) {
        char buf0[10], buf1[10], buf2[10], buf3[10], buf4[10] = {0};
        String cmd;
        utoa(x0, buf0, 10);
        utoa(y0, buf1, 10);
        utoa(x1, buf2, 10);
        utoa(y1, buf3, 10);
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
        sendCommand(cmd.c_str());
        recvRetCommandFinished();
      }

      void drawLine(const int x0, const int y0, const int x1, const int y1, uint16_t color) {
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
        sendCommand(cmd.c_str());
        recvRetCommandFinished();
      }

      void drawPixel(const int x, const int y, uint16_t color) {
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
        sendCommand(cmd.c_str());
        recvRetCommandFinished();
      }
  };

#endif // NEXTION_GFX

#endif /* _NEXTION_GFX_H_ */
