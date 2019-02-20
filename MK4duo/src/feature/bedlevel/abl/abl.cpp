/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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

#include "../../../../MK4duo.h"

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

  AutoBedLevel abl;

  int   AutoBedLevel::bilinear_grid_spacing[2],
        AutoBedLevel::bilinear_start[2];
  float AutoBedLevel::bilinear_grid_factor[2],
        AutoBedLevel::z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y] = { 0, 0 };

  /**
   * Extrapolate a single point from its neighbors
   */
  void AutoBedLevel::extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {
    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MSG("Extrapolate [");
        if (x < 10) SERIAL_CHR(' ');
        SERIAL_VAL((int)x);
        SERIAL_CHR(xdir ? (xdir > 0 ? '+' : '-') : ' ');
        SERIAL_CHR(' ');
        if (y < 10) SERIAL_CHR(' ');
        SERIAL_VAL((int)y);
        SERIAL_CHR(ydir ? (ydir > 0 ? '+' : '-') : ' ');
        SERIAL_CHR(']');
      }
    #endif
    if (!isnan(z_values[x][y])) {
      #if ENABLED(DEBUG_FEATURE)
        if (printer.debugFeature()) SERIAL_EM(" (done)");
      #endif
      return;  // Don't overwrite good values.
    }
    SERIAL_EOL();

    // Get X neighbors, Y neighbors, and XY neighbors
    const uint8_t x1 = x + xdir, y1 = y + ydir, x2 = x1 + xdir, y2 = y1 + ydir;
    float a1 = z_values[x1][y ], a2 = z_values[x2][y ],
          b1 = z_values[x ][y1], b2 = z_values[x ][y2],
          c1 = z_values[x1][y1], c2 = z_values[x2][y2];

    // Treat far unprobed points as zero, near as equal to far
    if (isnan(a2)) a2 = 0.0; if (isnan(a1)) a1 = a2;
    if (isnan(b2)) b2 = 0.0; if (isnan(b1)) b1 = b2;
    if (isnan(c2)) c2 = 0.0; if (isnan(c1)) c1 = c2;

    const float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;

    // Take the average instead of the median
    z_values[x][y] = (a + b + c) / 3.0;

    // Median is robust (ignores outliers).
    // z_values[x][y] = (a < b) ? ((b < c) ? b : (c < a) ? a : c)
    //                                : ((c < b) ? b : (a < c) ? a : c);
  }

  #if ENABLED(EXTRAPOLATE_FROM_EDGE)
    #if GRID_MAX_POINTS_X < GRID_MAX_POINTS_Y
      #define HALF_IN_X
    #elif GRID_MAX_POINTS_Y < GRID_MAX_POINTS_X
      #define HALF_IN_Y
    #endif
  #endif

  /**
   * Fill in the unprobed points (corners of circular print surface)
   * using linear extrapolation, away from the center.
   */
  void AutoBedLevel::extrapolate_unprobed_bed_level() {
    #if ENABLED(HALF_IN_X)
      constexpr uint8_t ctrx2 = 0, xlen = GRID_MAX_POINTS_X - 1;
    #else
      constexpr uint8_t ctrx1 = (GRID_MAX_POINTS_X - 1) * 0.5, // left-of-center
                        ctrx2 = GRID_MAX_POINTS_X * 0.5,       // right-of-center
                        xlen = ctrx1;
    #endif

    #if ENABLED(HALF_IN_Y)
      constexpr uint8_t ctry2 = 0, ylen = GRID_MAX_POINTS_Y - 1;
    #else
      constexpr uint8_t ctry1 = (GRID_MAX_POINTS_Y - 1) * 0.5, // top-of-center
                        ctry2 = GRID_MAX_POINTS_Y * 0.5,       // bottom-of-center
                        ylen = ctry1;
    #endif

    for (uint8_t xo = 0; xo <= xlen; xo++) {
      for (uint8_t yo = 0; yo <= ylen; yo++) {
        uint8_t x2 = ctrx2 + xo, y2 = ctry2 + yo;
        #if DISABLED(HALF_IN_X)
          const uint8_t x1 = ctrx1 - xo;
        #endif
        #if DISABLED(HALF_IN_Y)
          const uint8_t y1 = ctry1 - yo;
          #if DISABLED(HALF_IN_X)
            extrapolate_one_point(x1, y1, +1, +1);   //  left-below + +
          #endif
          extrapolate_one_point(x2, y1, -1, +1);     // right-below - +
        #endif
        #if DISABLED(HALF_IN_X)
          extrapolate_one_point(x1, y2, +1, -1);     //  left-above + -
        #endif
        extrapolate_one_point(x2, y2, -1, -1);       // right-above - -
      }
    }
  }

  void AutoBedLevel::print_bilinear_leveling_grid() {
    SERIAL_LM(ECHO, "Bilinear Leveling Grid:");
    bedlevel.print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3,
      [](const uint8_t ix, const uint8_t iy){ return z_values[ix][iy]; }
    );
  }

  #if ENABLED(ABL_BILINEAR_SUBDIVISION)

    float AutoBedLevel::bilinear_grid_factor_virt[2] = { 0 },
          AutoBedLevel::z_values_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
    int   AutoBedLevel::bilinear_grid_spacing_virt[2] = { 0 };

    void AutoBedLevel::print_bilinear_leveling_grid_virt() {
      SERIAL_LM(ECHO, "Subdivided with CATMULL ROM Leveling Grid:");
      bedlevel.print_2d_array(ABL_GRID_POINTS_VIRT_X, ABL_GRID_POINTS_VIRT_Y, 5,
        [](const uint8_t ix, const uint8_t iy){ return z_values_virt[ix][iy]; }
      );
    }

    #define LINEAR_EXTRAPOLATION(E, I) ((E) * 2 - (I))

    float AutoBedLevel::bed_level_virt_coord(const uint8_t x, const uint8_t y) {
      uint8_t ep = 0, ip = 1;

      if (!x || x == ABL_TEMP_POINTS_X - 1) {
        if (x) {
          ep = GRID_MAX_POINTS_X - 1;
          ip = GRID_MAX_POINTS_X - 2;
        }
        if (WITHIN(y, 1, ABL_TEMP_POINTS_Y - 2))
          return LINEAR_EXTRAPOLATION(
            z_values[ep][y - 1],
            z_values[ip][y - 1]
          );
        else
          return LINEAR_EXTRAPOLATION(
            bed_level_virt_coord(ep + 1, y),
            bed_level_virt_coord(ip + 1, y)
          );
      }
      if (!y || y == ABL_TEMP_POINTS_Y - 1) {
        if (y) {
          ep = GRID_MAX_POINTS_Y - 1;
          ip = GRID_MAX_POINTS_Y - 2;
        }
        if (WITHIN(x, 1, ABL_TEMP_POINTS_X - 2))
          return LINEAR_EXTRAPOLATION(
            z_values[x - 1][ep],
            z_values[x - 1][ip]
          );
        else
          return LINEAR_EXTRAPOLATION(
            bed_level_virt_coord(x, ep + 1),
            bed_level_virt_coord(x, ip + 1)
          );
      }
      return z_values[x - 1][y - 1];
    }

    float AutoBedLevel::bed_level_virt_cmr(const float p[4], const uint8_t i, const float t) {
      return (
          p[i-1] * -t * sq(1 - t)
        + p[i]   * (2 - 5 * sq(t) + 3 * t * sq(t))
        + p[i+1] * t * (1 + 4 * t - 3 * sq(t))
        - p[i+2] * sq(t) * (1 - t)
      ) * 0.5;
    }

    float AutoBedLevel::bed_level_virt_2cmr(const uint8_t x, const uint8_t y, const float &tx, const float &ty) {
      float row[4], column[4];
      for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
          column[j] = bed_level_virt_coord(i + x - 1, j + y - 1);
        }
        row[i] = bed_level_virt_cmr(column, 1, ty);
      }
      return bed_level_virt_cmr(row, 1, tx);
    }

    void AutoBedLevel::virt_interpolate() {
      bilinear_grid_spacing_virt[X_AXIS] = bilinear_grid_spacing[X_AXIS] / (BILINEAR_SUBDIVISIONS);
      bilinear_grid_spacing_virt[Y_AXIS] = bilinear_grid_spacing[Y_AXIS] / (BILINEAR_SUBDIVISIONS);
      bilinear_grid_factor_virt[X_AXIS] = RECIPROCAL(bilinear_grid_spacing_virt[X_AXIS]);
      bilinear_grid_factor_virt[Y_AXIS] = RECIPROCAL(bilinear_grid_spacing_virt[Y_AXIS]);
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
          for (uint8_t ty = 0; ty < BILINEAR_SUBDIVISIONS; ty++) {
            for (uint8_t tx = 0; tx < BILINEAR_SUBDIVISIONS; tx++) {
              if ((ty && y == GRID_MAX_POINTS_Y - 1) || (tx && x == GRID_MAX_POINTS_X - 1))
                continue;
              z_values_virt[x * (BILINEAR_SUBDIVISIONS) + tx][y * (BILINEAR_SUBDIVISIONS) + ty] =
                bed_level_virt_2cmr(
                  x + 1,
                  y + 1,
                  (float)tx / (BILINEAR_SUBDIVISIONS),
                  (float)ty / (BILINEAR_SUBDIVISIONS)
                );
            }
          }
        }
      }
    }

  #endif // ABL_BILINEAR_SUBDIVISION

  // Refresh after other values have been updated
  void AutoBedLevel::refresh_bed_level() {
    bilinear_grid_factor[X_AXIS] = RECIPROCAL(bilinear_grid_spacing[X_AXIS]);
    bilinear_grid_factor[Y_AXIS] = RECIPROCAL(bilinear_grid_spacing[Y_AXIS]);
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      virt_interpolate();
    #endif
  }

  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
    #define ABL_BG_SPACING(A) bilinear_grid_spacing_virt[A]
    #define ABL_BG_FACTOR(A)  bilinear_grid_factor_virt[A]
    #define ABL_BG_POINTS_X   ABL_GRID_POINTS_VIRT_X
    #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_VIRT_Y
    #define ABL_BG_GRID(X,Y)  z_values_virt[X][Y]
  #else
    #define ABL_BG_SPACING(A) bilinear_grid_spacing[A]
    #define ABL_BG_FACTOR(A)  bilinear_grid_factor[A]
    #define ABL_BG_POINTS_X   GRID_MAX_POINTS_X
    #define ABL_BG_POINTS_Y   GRID_MAX_POINTS_Y
    #define ABL_BG_GRID(X,Y)  z_values[X][Y]
  #endif

  // Get the Z adjustment for non-linear bed leveling
  float AutoBedLevel::bilinear_z_offset(const float raw[XYZ]) {

    static float  z1, d2, z3, d4, L, D, ratio_x, ratio_y,
                  last_x = -999.999, last_y = -999.999;

    // Whole units for the grid line indices. Constrained within bounds.
    static int8_t gridx, gridy, nextx, nexty,
                  last_gridx = -99, last_gridy = -99;

    // XY relative to the probed area
    const float rx = raw[X_AXIS] - bilinear_start[X_AXIS],
                ry = raw[Y_AXIS] - bilinear_start[Y_AXIS];

    if (last_x != rx) {
      last_x = rx;
      ratio_x = rx * ABL_BG_FACTOR(X_AXIS);
      const float gx = constrain(FLOOR(ratio_x), 0, ABL_BG_POINTS_X - 1);
      ratio_x -= gx;      // Subtract whole to get the ratio within the grid box
      NOLESS(ratio_x, 0); // Never < 0.0. (> 1.0 is ok when nextx==gridx.)
      gridx = gx;
      nextx = MIN(gridx + 1, ABL_BG_POINTS_X - 1);
    }

    if (last_y != ry || last_gridx != gridx) {

      if (last_y != ry) {
        last_y = ry;
        ratio_y = ry * ABL_BG_FACTOR(Y_AXIS);
        const float gy = constrain(FLOOR(ratio_y), 0, ABL_BG_POINTS_Y - 1);
        ratio_y -= gy;
        NOLESS(ratio_y, 0);
        gridy = gy;
        nexty = MIN(gridy + 1, ABL_BG_POINTS_Y - 1);
      }

      if (last_gridx != gridx || last_gridy != gridy) {
        last_gridx = gridx;
        last_gridy = gridy;
        // Z at the box corners
        z1 = ABL_BG_GRID(gridx, gridy);       // left-front
        d2 = ABL_BG_GRID(gridx, nexty) - z1;  // left-back (delta)
        z3 = ABL_BG_GRID(nextx, gridy);       // right-front
        d4 = ABL_BG_GRID(nextx, nexty) - z3;  // right-back (delta)
      }

      // Bilinear interpolate. Needed since ry or gridx has changed.
                  L = z1 + d2 * ratio_y;   // Linear interp. LF -> LB
      const float R = z3 + d4 * ratio_y;   // Linear interp. RF -> RB

      D = R - L;
    }

    const float offset = L + ratio_x * D;   // the offset almost always changes

    /*
    static float last_offset = 0;
    if (ABS(last_offset - offset) > 0.2) {
      SERIAL_MSG("Sudden Shift at ");
      SERIAL_MV("x=", rx);
      SERIAL_MV(" / ", ABL_BG_SPACING(X_AXIS));
      SERIAL_EMV(" -> gridx=", gridx);
      SERIAL_MV(" y=", ry);
      SERIAL_MV(" / ", ABL_BG_SPACING(Y_AXIS));
      SERIAL_EMV(" -> gridy=", gridy);
      SERIAL_MV(" ratio_x=", ratio_x);
      SERIAL_EMV(" ratio_y=", ratio_y);
      SERIAL_MV(" z1=", z1);
      SERIAL_MV(" d2=", d2);
      SERIAL_MV(" z3=", z3);
      SERIAL_EMV(" d4=", d4);
      SERIAL_MV(" L=", L);
      SERIAL_MV(" R=", R);
      SERIAL_EMV(" offset=", offset);
    }
    last_offset = offset;
    */

    return offset;
  }

  #if !IS_KINEMATIC

    /**
     * Prepare a bilinear-leveled linear move on Cartesian,
     * splitting the move where it crosses mesh borders.
     */
    void AutoBedLevel::bilinear_line_to_destination(float fr_mm_s, uint16_t x_splits/*= 0xFFFF*/, uint16_t y_splits/*= 0xFFFF*/) {

      int cx1 = (mechanics.current_position[X_AXIS] - bilinear_start[X_AXIS]) * ABL_BG_FACTOR(X_AXIS),
          cy1 = (mechanics.current_position[Y_AXIS] - bilinear_start[Y_AXIS]) * ABL_BG_FACTOR(Y_AXIS),
          cx2 = (mechanics.destination[X_AXIS]      - bilinear_start[X_AXIS]) * ABL_BG_FACTOR(X_AXIS),
          cy2 = (mechanics.destination[Y_AXIS]      - bilinear_start[Y_AXIS]) * ABL_BG_FACTOR(Y_AXIS);

      cx1 = constrain(cx1, 0, ABL_BG_POINTS_X - 2);
      cy1 = constrain(cy1, 0, ABL_BG_POINTS_Y - 2);
      cx2 = constrain(cx2, 0, ABL_BG_POINTS_X - 2);
      cy2 = constrain(cy2, 0, ABL_BG_POINTS_Y - 2);

      if (cx1 == cx2 && cy1 == cy2) {
        // Start and end on same mesh square
        mechanics.line_to_destination(fr_mm_s);
        mechanics.set_current_to_destination();
        return;
      }

      #define LINE_SEGMENT_END(A) (mechanics.current_position[_AXIS(A)] + (mechanics.destination[_AXIS(A)] - mechanics.current_position[_AXIS(A)]) * normalized_dist)

      float normalized_dist, end[XYZE];

      // Split at the left/front border of the right/top square
      int8_t gcx = MAX(cx1, cx2), gcy = MAX(cy1, cy2);
      if (cx2 != cx1 && TEST(x_splits, gcx)) {
        COPY_ARRAY(end, mechanics.destination);
        mechanics.destination[X_AXIS] = bilinear_start[X_AXIS] + ABL_BG_SPACING(X_AXIS) * gcx;
        normalized_dist = (mechanics.destination[X_AXIS] - mechanics.current_position[X_AXIS]) / (end[X_AXIS] - mechanics.current_position[X_AXIS]);
        mechanics.destination[Y_AXIS] = LINE_SEGMENT_END(Y);
        CBI(x_splits, gcx);
      }
      else if (cy2 != cy1 && TEST(y_splits, gcy)) {
        COPY_ARRAY(end, mechanics.destination);
        mechanics.destination[Y_AXIS] = bilinear_start[Y_AXIS] + ABL_BG_SPACING(Y_AXIS) * gcy;
        normalized_dist = (mechanics.destination[Y_AXIS] - mechanics.current_position[Y_AXIS]) / (end[Y_AXIS] - mechanics.current_position[Y_AXIS]);
        mechanics.destination[X_AXIS] = LINE_SEGMENT_END(X);
        CBI(y_splits, gcy);
      }
      else {
        // Already split on a border
        mechanics.line_to_destination(fr_mm_s);
        mechanics.set_current_to_destination();
        return;
      }

      mechanics.destination[Z_AXIS] = LINE_SEGMENT_END(Z);
      mechanics.destination[E_AXIS] = LINE_SEGMENT_END(E);

      // Do the split and look for more borders
      bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);

      // Restore destination from stack
      COPY_ARRAY(mechanics.destination, end);
      bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);
    }

  #endif // !IS_KINEMATIC

#endif // AUTO_BED_LEVELING_BILINEAR
