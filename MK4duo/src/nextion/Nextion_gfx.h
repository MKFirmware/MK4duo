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

#ifndef NEXTION_GFX_H
#define NEXTION_GFX_H

#if ENABLED(NEXTION_GFX)
  #include "nextion_lib/Nextion.h"

  #define NX_AXIS         0
  #define NX_MOVE         3
  #define NX_TOOL         4
  #define NX_BACKGROUND   5
  #define NX_MAX          6
  #define NX_SCALE        4.0

  struct point {
    uint16_t x, y;
  };

  class GFX {

    private:

      /* Location of visualization in NEXTION LCD*/
      uint16_t _top, _left, _width, _height;
      float _scale, _max[3], _origin[3];

      uint16_t _color[NX_MAX];

      struct {
        struct point point;
        float position[3];
      } _cursor;

    public:

      GFX(const uint16_t x = 0, const uint16_t y = 0, const uint16_t width = 1, const uint16_t height = 1) {

        _top = y;
        _left = x;
        _width = width;
        _height = height;

        for (uint8_t i = 0; i < 3; i++)
          _max[i] = 1000.0;

        for (uint8_t i = 0; i < NX_MAX; i++)
          _color[i] = 65535;
        _color[NX_BACKGROUND] = 0;
      }

      void clear() {
        const float zero[3] = { 0, 0, 0 };
        fill(_left, _top, _width, _height, _color[NX_BACKGROUND]);

        for (uint8_t i = 0; i < 3; i++) {
          float pos[3] = {};
          pos[i] = _max[i];
          cursor_to(zero);
          line_to(NX_AXIS + i, pos);
        }
        cursor_to(zero);
      }

      void clear(const float scale) {
        _scale = scale;
        clear();
      }

      void clear(const float x_mm, const float y_mm, const float z_mm) {
        /* Bounding box for the build volume */
        const float scale_y = _height / (z_mm + y_mm / NX_SCALE);
        const float scale_x = _width  / (x_mm + y_mm / NX_SCALE);

        _max[X_AXIS] = x_mm;
        _max[Y_AXIS] = y_mm;
        _max[Z_AXIS] = z_mm;

        clear(scale_x > scale_y ? scale_y : scale_x);
      }

      void origin(const float x, const float y, const float z) {
        _origin[X_AXIS] = x;
        _origin[Y_AXIS] = y;
        _origin[Z_AXIS] = z;
      }

      void color_set(const int color_ndx, uint16_t color) {
        _color[color_ndx] = color;
      }

      void cursor_to(const float *pos) {
        for (uint8_t i = 0; i < 3; i++)
          _cursor.position[i] = pos[i];

        _flatten(_cursor.position, &_cursor.point);
      }

      void line_to(const int color_ndx, const float *pos);

      void cursor_to(float x, float y, float z) {
        const float pos[3] = { x, y, z };
        cursor_to(pos);
      }
      
      void line_to(const int color_ndx, float x, float y, float z) {
        float pos[3] = { x, y, z };
        line_to(color_ndx, pos);
      }

    private:

      static int ComputeOutCode(int x, int y, int w, int h);

      void _flatten(const float* pos, struct point* pt) {
        pt->x = ((pos[X_AXIS] - _origin[X_AXIS]) +
                 (pos[Y_AXIS] - _origin[Y_AXIS]) / NX_SCALE) * _scale + 1;
        pt->y = (_height - 1) -
                ((pos[Z_AXIS] - _origin[Z_AXIS]) +
                 (pos[Y_AXIS] - _origin[Y_AXIS]) / NX_SCALE) * _scale - 1;
      }

      void _line2d_clipped(const float* a_color, const struct point* a,
                           const float* b_color, const struct point* b);

      void _line2d(const int color_ndx, const struct point* a, const struct point* b);

      bool fill(const int x0, const int y0, const int x1, const int y1, uint16_t color) {
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
        return recvRetCommandFinished();
      }

      bool drawLine(const int x0, const int y0, const int x1, const int y1, uint16_t color) {
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
        return recvRetCommandFinished();
      }
  };

#endif // NEXTION_GFX
#endif // NEXTION_GFX_H
