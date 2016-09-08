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
    #include "Nextion.h"

    #define VC_AXIS         0
    #define VC_MOVE         3
    #define VC_TOOL         4
    #define VC_BACKGROUND   5
    #define VC_MAX          6

    struct point {
      int x, y;
    };

    class GFX {
      private:
        /* Location of visualization in NEXTION LCD*/
        int _top, _left, _width, _height;
        float _scale, _max[3], _origin[3];

        uint16_t _color[VC_MAX];

        struct {
          struct point point;
          float position[3];
        } _cursor;

      public:
        GFX(int width, int height, int x = 0, int y = 0) {

          _top = y + 1;
          _left = x + 1;
          _width = width - 2;
          _height = height - 2;

          for (int i = 0; i < 3; i++)
            _max[i] = 1000.0;

          for (int i = 0; i < VC_MAX; i++)
            _color[i] = 65535;
          _color[VC_BACKGROUND] = 0;
        }

        void clear() {
          float zero[3] = {};
          fill(_left, _top, _width, _height, _color[VC_BACKGROUND]);

          for (int i = 0; i < 3; i++) {
            float pos[3] = {};
            pos[i] = _max[i];
            cursor_to(zero);
            line_to(VC_AXIS + i, pos);
          }
          cursor_to(zero);
        }

        void clear(float scale) {
          _scale = scale;
          clear();
        }

        void clear(float x_mm, float y_mm, float z_mm) {
          /* Bounding box for the build volume */
          float scale_y = _height / (z_mm + y_mm / 4.0);
          float scale_x = _width  / (x_mm + y_mm / 4.0);

          _max[X_AXIS] = x_mm;
          _max[Y_AXIS] = y_mm;
          _max[Z_AXIS] = z_mm;

          clear(scale_x > scale_y ? scale_y : scale_x);
        }

        void origin(float x, float y, float z) {
          _origin[X_AXIS] = x;
          _origin[Y_AXIS] = y;
          _origin[Z_AXIS] = z;

          clear();
        }

        void color_set(int color_ndx, uint16_t color) {
          _color[color_ndx] = color;
        }

        void cursor_to(const float *pos) {
          for (int i = 0; i < 3; i++)
            _cursor.position[i] = pos[i];

          _flatten(_cursor.position, &_cursor.point);
        }

        void line_to(int color_ndx, const float *pos);

        void cursor_to(float x, float y, float z) {
          float pos[3] = { x, y, z };
          cursor_to(pos);
        }
        
        void line_to(int color_ndx, float x, float y, float z) {
          float pos[3] = { x, y, z };
          line_to(color_ndx, pos);
        }

      private:
        void _flatten(const float* pos, struct point* pt) {
          pt->x = ((pos[X_AXIS] - _origin[X_AXIS]) +
                   (pos[Y_AXIS] - _origin[Y_AXIS]) / 4.0) * _scale + 1;
          pt->y = (_height - 1) -
                  ((pos[Z_AXIS] - _origin[Z_AXIS]) +
                   (pos[Y_AXIS] - _origin[Y_AXIS]) / 4) * _scale - 1;
        }

        void _line2d_clipped(const float* a_color, const struct point* a,
                             const float* b_color, const struct point* b);

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

        bool drawPixel(const int x, const int y, uint16_t color) {
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
          return recvRetCommandFinished();
        }
    };

  #endif // NEXTION
#endif /* NEXTION_GFX */
