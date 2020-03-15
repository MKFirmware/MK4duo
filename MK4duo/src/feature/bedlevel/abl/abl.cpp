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

#include "../../../../MK4duo.h"

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

AutoBedLevel abl;

/** Public Parameters */
abl_data_t AutoBedLevel::data;

/** Private Parameters */
xy_float_t  AutoBedLevel::bilinear_grid_factor;

/** Public Function */
/**
 * Extrapolate a single point from its neighbors
 */
void AutoBedLevel::extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {

  if (!isnan(data.z_values[x][y])) return;

  if (printer.debugFeature()) {
    DEBUG_MSG("Extrapolate [");
    if (x < 10) DEBUG_CHR(' ');
    DEBUG_VAL((int)x);
    DEBUG_CHR(xdir ? (xdir > 0 ? '+' : '-') : ' ');
    DEBUG_CHR(' ');
    if (y < 10) DEBUG_CHR(' ');
    DEBUG_VAL((int)y);
    DEBUG_CHR(ydir ? (ydir > 0 ? '+' : '-') : ' ');
    DEBUG_CHR(']');
    SERIAL_EOL();
  }

  // Get X neighbors, Y neighbors, and XY neighbors
  const uint8_t x1 = x + xdir, y1 = y + ydir, x2 = x1 + xdir, y2 = y1 + ydir;
  float a1 = data.z_values[x1][y ], a2 = data.z_values[x2][y ],
        b1 = data.z_values[x ][y1], b2 = data.z_values[x ][y2],
        c1 = data.z_values[x1][y1], c2 = data.z_values[x2][y2];

  // Treat far unprobed points as zero, near as equal to far
  if (isnan(a2)) a2 = 0.0; if (isnan(a1)) a1 = a2;
  if (isnan(b2)) b2 = 0.0; if (isnan(b1)) b1 = b2;
  if (isnan(c2)) c2 = 0.0; if (isnan(c1)) c1 = c2;

  const float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;

  // Take the average instead of the median
  data.z_values[x][y] = (a + b + c) / 3.0;

  // Median is robust (ignores outliers).
  // data.z_values[x][y] = (a < b) ? ((b < c) ? b : (c < a) ? a : c)
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
    [](const uint8_t ix, const uint8_t iy){ return data.z_values[ix][iy]; }
  );
}

#if ENABLED(ABL_BILINEAR_SUBDIVISION)

  float       AutoBedLevel::z_values_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
  xy_float_t  AutoBedLevel::bilinear_grid_factor_virt;
  xy_pos_t    AutoBedLevel::bilinear_grid_spacing_virt;

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
          data.z_values[ep][y - 1],
          data.z_values[ip][y - 1]
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
          data.z_values[x - 1][ep],
          data.z_values[x - 1][ip]
        );
      else
        return LINEAR_EXTRAPOLATION(
          bed_level_virt_coord(x, ep + 1),
          bed_level_virt_coord(x, ip + 1)
        );
    }
    return data.z_values[x - 1][y - 1];
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
    bilinear_grid_spacing_virt.x = data.bilinear_grid_spacing.x / (BILINEAR_SUBDIVISIONS);
    bilinear_grid_spacing_virt.y = data.bilinear_grid_spacing.y / (BILINEAR_SUBDIVISIONS);
    bilinear_grid_factor_virt.x = RECIPROCAL(bilinear_grid_spacing_virt.x);
    bilinear_grid_factor_virt.y = RECIPROCAL(bilinear_grid_spacing_virt.y);
    for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
      for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
        for (uint8_t ty = 0; ty < BILINEAR_SUBDIVISIONS; ty++) {
          for (uint8_t tx = 0; tx < BILINEAR_SUBDIVISIONS; tx++) {
            if ((ty && y == (GRID_MAX_POINTS_Y) - 1) || (tx && x == (GRID_MAX_POINTS_X) - 1))
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
  bilinear_grid_factor.x = RECIPROCAL(data.bilinear_grid_spacing.x);
  bilinear_grid_factor.y = RECIPROCAL(data.bilinear_grid_spacing.y);
  #if ENABLED(ABL_BILINEAR_SUBDIVISION)
    virt_interpolate();
  #endif
}

#if ENABLED(ABL_BILINEAR_SUBDIVISION)
  #define ABL_BG_SPACING(A) bilinear_grid_spacing_virt.A
  #define ABL_BG_FACTOR(A)  bilinear_grid_factor_virt.A
  #define ABL_BG_POINTS_X   ABL_GRID_POINTS_VIRT_X
  #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_VIRT_Y
  #define ABL_BG_GRID(X,Y)  z_values_virt[X][Y]
#else
  #define ABL_BG_SPACING(A) data.bilinear_grid_spacing.A
  #define ABL_BG_FACTOR(A)  bilinear_grid_factor.A
  #define ABL_BG_POINTS_X   GRID_MAX_POINTS_X
  #define ABL_BG_POINTS_Y   GRID_MAX_POINTS_Y
  #define ABL_BG_GRID(X,Y)  data.z_values[X][Y]
#endif

// Get the Z adjustment for non-linear bed leveling
float AutoBedLevel::bilinear_z_offset(const xy_pos_t &raw) {

  static float  z1, d2, z3, d4, L, D;

  static xy_pos_t prev { -999.999, -999.999 }, ratio;

  // Whole units for the grid line indices. Constrained within bounds.
  static xy_int8_t thisg, nextg, lastg { -99, -99 };

  // XY relative to the probed area
  xy_pos_t rel = raw - data.bilinear_start.asFloat();

  if (prev.x != rel.x) {
    prev.x = rel.x;
    ratio.x = rel.x * ABL_BG_FACTOR(x);
    const float gx = constrain(FLOOR(ratio.x), 0, ABL_BG_POINTS_X - 1);
    ratio.x -= gx;      // Subtract whole to get the ratio within the grid box
    NOLESS(ratio.x, 0); // Never < 0.0. (> 1.0 is ok when nextg.x==thisg.x.)
    thisg.x = gx;
    nextg.x = MIN(thisg.x + 1, ABL_BG_POINTS_X - 1);
  }

  if (prev.y != rel.y || lastg.x != thisg.x) {

    if (prev.y != rel.y) {
      prev.y = rel.y;
      ratio.y = rel.y * ABL_BG_FACTOR(y);
      const float gy = constrain(FLOOR(ratio.y), 0, ABL_BG_POINTS_Y - 1);
      ratio.y -= gy;
      NOLESS(ratio.y, 0);
      thisg.y = gy;
      nextg.y = MIN(thisg.y + 1, ABL_BG_POINTS_Y - 1);
    }

    if (lastg != thisg) {
      lastg = thisg;
      // Z at the box corners
      z1 = ABL_BG_GRID(thisg.x, thisg.y);       // left-front
      d2 = ABL_BG_GRID(thisg.x, nextg.y) - z1;  // left-back (delta)
      z3 = ABL_BG_GRID(nextg.x, thisg.y);       // right-front
      d4 = ABL_BG_GRID(nextg.x, nextg.y) - z3;  // right-back (delta)
    }

    // Bilinear interpolate. Needed since ry or thisg.x has changed.
                L = z1 + d2 * ratio.y;      // Linear interp. LF -> LB
    const float R = z3 + d4 * ratio.y;      // Linear interp. RF -> RB

    D = R - L;
  }

  const float offset = L + ratio.x * D;     // the offset almost always changes

  /*
  static float last_offset = 0;
  if (ABS(last_offset - offset) > 0.2) {
    SERIAL_MSG("Sudden Shift at ");
    SERIAL_MV("x=", rx);
    SERIAL_MV(" / ", ABL_BG_SPACING(X_AXIS));
    SERIAL_EMV(" -> thisg.x=", thisg.x);
    SERIAL_MV(" y=", ry);
    SERIAL_MV(" / ", ABL_BG_SPACING(Y_AXIS));
    SERIAL_EMV(" -> thisg.y=", thisg.y);
    SERIAL_MV(" ratio.x=", ratio.x);
    SERIAL_EMV(" ratio.y=", ratio.y);
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

  #define CELL_INDEX(A,V) ((V - data.bilinear_start.A) * ABL_BG_FACTOR(A))

  /**
   * Prepare a bilinear-leveled linear move on Cartesian,
   * splitting the move where it crosses mesh borders.
   */
  void AutoBedLevel::line_to_destination(const feedrate_t scaled_fr_mm_s, uint16_t x_splits/*= 0xFFFF*/, uint16_t y_splits/*= 0xFFFF*/) {

    // Get current and destination cells for this line
    xy_int_t  c1 { CELL_INDEX(x, mechanics.position.x),     CELL_INDEX(y, mechanics.position.y) },
              c2 { CELL_INDEX(x, mechanics.destination.x),  CELL_INDEX(y, mechanics.destination.y) };

    LIMIT(c1.x, 0, ABL_BG_POINTS_X - 2);
    LIMIT(c1.y, 0, ABL_BG_POINTS_Y - 2);
    LIMIT(c2.x, 0, ABL_BG_POINTS_X - 2);
    LIMIT(c2.y, 0, ABL_BG_POINTS_Y - 2);

    // Start and end on same mesh square
    if (c1 == c2) {
      mechanics.line_to_destination(scaled_fr_mm_s);
      mechanics.position = mechanics.destination;
      return;
    }

    #define LINE_SEGMENT_END(A) (mechanics.position.A + (mechanics.destination.A - mechanics.position.A) * normalized_dist)

    xyze_pos_t end;
    float normalized_dist;
    const xy_int8_t gc { MAX(c1.x, c2.x), MAX(c1.y, c2.y) };

    // Crosses on the X and not already split on this X?
    // The x_splits flags are insurance against rounding errors.
    if (c2.x != c1.x && TEST(x_splits, gc.x)) {
      // Split on the X grid line
      CBI(x_splits, gc.x);
      end = mechanics.destination;
      mechanics.destination.x = data.bilinear_start.x + ABL_BG_SPACING(x) * gc.x;
      normalized_dist = (mechanics.destination.x - mechanics.position.x) / (end.x - mechanics.position.x);
      mechanics.destination.y = LINE_SEGMENT_END(y);
    }
    // Crosses on the Y and not already split on this Y?
    else if (c2.y != c1.y && TEST(y_splits, gc.y)) {
      CBI(y_splits, gc.y);
      COPY_ARRAY(end, mechanics.destination);
      mechanics.destination.y = data.bilinear_start.y + ABL_BG_SPACING(y) * gc.y;
      normalized_dist = (mechanics.destination.y - mechanics.position.y) / (end.y - mechanics.position.y);
      mechanics.destination.x = LINE_SEGMENT_END(x);
    }
    else {
      // Must already have been split on these border(s)
      // This should be a rare case.
      mechanics.line_to_destination(scaled_fr_mm_s);
      mechanics.position = mechanics.destination;
      return;
    }

    mechanics.destination.z = LINE_SEGMENT_END(z);
    mechanics.destination.e = LINE_SEGMENT_END(e);

    // Do the split and look for more borders
    line_to_destination(scaled_fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    mechanics.destination = end;
    line_to_destination(scaled_fr_mm_s, x_splits, y_splits);
  }

#endif // !IS_KINEMATIC

#endif // AUTO_BED_LEVELING_BILINEAR
