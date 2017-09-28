/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
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
 * bedlevel.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../base.h"

#if HAS_LEVELING

  Bedlevel bedlevel;

  #if HAS_ABL
    bool  Bedlevel::abl_enabled = false; // Flag that auto bed leveling is enabled
    int   Bedlevel::xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
  #endif

  #if ABL_PLANAR
    matrix_3x3 Bedlevel::matrix; // Transform to compensate for bed level
  #endif

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    float Bedlevel::z_fade_height,
          Bedlevel::inverse_z_fade_height;
  #endif

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    int   Bedlevel::bilinear_grid_spacing[2], Bedlevel::bilinear_start[2];
    float Bedlevel::bilinear_grid_factor[2],
          Bedlevel::z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      float Bedlevel::bilinear_grid_factor_virt[2] = { 0 },
            Bedlevel::z_values_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
      int   Bedlevel::bilinear_grid_spacing_virt[2] = { 0 };
    #endif
  #endif

  #if ENABLED(PROBE_MANUALLY)
    bool Bedlevel::g29_in_progress = false;
  #else
    const bool Bedlevel::g29_in_progress = false;
  #endif

  #if PLANNER_LEVELING

    /**
     * lx, ly, lz - Logical (cartesian, not delta) positions in mm
     */
    void Bedlevel::apply_leveling(float &lx, float &ly, float &lz) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        if (!ubl.state.active) return;
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          // if z_fade_height enabled (nonzero) and raw_z above it, no leveling required
          if (z_fade_height && z_fade_height <= RAW_Z_POSITION(lz)) return;
          lz += ubl.state.z_offset + ubl.get_z_correction(lx, ly) * ubl.fade_scaling_factor_for_z(lz);
        #else // no fade
          lz += ubl.state.z_offset + ubl.get_z_correction(lx, ly);
        #endif // FADE
      #endif // UBL

      #if OLDSCHOOL_ABL
        if (!abl_enabled) return;
      #endif

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT) && DISABLED(AUTO_BED_LEVELING_UBL)
        static float z_fade_factor = 1.0, last_raw_lz = -999.0;
        if (z_fade_height) {
          const float raw_lz = RAW_Z_POSITION(lz);
          if (raw_lz >= z_fade_height) return;
          if (last_raw_lz != raw_lz) {
            last_raw_lz = raw_lz;
            z_fade_factor = 1.0 - raw_lz * inverse_z_fade_height;
          }
        }
        else
          z_fade_factor = 1.0;
      #endif

      #if ENABLED(MESH_BED_LEVELING)

        if (mbl.active())
          lz += mbl.get_z(RAW_X_POSITION(lx), RAW_Y_POSITION(ly)
            #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
              , z_fade_factor
            #endif
            );

      #elif ABL_PLANAR

        float dx = RAW_X_POSITION(lx) - (X_TILT_FULCRUM),
              dy = RAW_Y_POSITION(ly) - (Y_TILT_FULCRUM),
              dz = RAW_Z_POSITION(lz);

        apply_rotation_xyz(matrix, dx, dy, dz);

        lx = LOGICAL_X_POSITION(dx + X_TILT_FULCRUM);
        ly = LOGICAL_Y_POSITION(dy + Y_TILT_FULCRUM);
        lz = LOGICAL_Z_POSITION(dz);

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        float tmp[XYZ] = { lx, ly, 0 };
        lz += bilinear_z_offset(tmp)
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            * z_fade_factor
          #endif
        ;

      #endif
    }

    void Bedlevel::unapply_leveling(float logical[XYZ]) {

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        if (ubl.state.active) {

          const float z_physical = RAW_Z_POSITION(logical[Z_AXIS]),
                      z_correct = ubl.get_z_correction(logical[X_AXIS], logical[Y_AXIS]),
                      z_virtual = z_physical - ubl.state.z_offset - z_correct;
                float z_logical = LOGICAL_Z_POSITION(z_virtual);

          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

            // for P=physical_z, L=logical_z, M=mesh_z, O=z_offset, H=fade_height,
            // Given P=L+O+M(1-L/H) (faded mesh correction formula for L<H)
            //  then L=P-O-M(1-L/H)
            //    so L=P-O-M+ML/H
            //    so L-ML/H=P-O-M
            //    so L(1-M/H)=P-O-M
            //    so L=(P-O-M)/(1-M/H) for L<H

            if (planner.z_fade_height) {
              if (z_logical >= z_fade_height)
                z_logical = LOGICAL_Z_POSITION(z_physical - ubl.state.z_offset);
              else
                z_logical /= 1.0 - z_correct * inverse_z_fade_height;
            }

          #endif // ENABLE_LEVELING_FADE_HEIGHT

          logical[Z_AXIS] = z_logical;
        }

        return; // don't fall thru to other ENABLE_LEVELING_FADE_HEIGHT logic

      #endif

      #if OLDSCHOOL_ABL
        if (!abl_enabled) return;
      #endif

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (z_fade_height && RAW_Z_POSITION(logical[Z_AXIS]) >= z_fade_height) return;
      #endif

      #if ENABLED(MESH_BED_LEVELING)

        if (mbl.active()) {
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            const float c = mbl.get_z(RAW_X_POSITION(logical[X_AXIS]), RAW_Y_POSITION(logical[Y_AXIS]), 1.0);
            logical[Z_AXIS] = (z_fade_height * (RAW_Z_POSITION(logical[Z_AXIS]) - c)) / (z_fade_height - c);
          #else
            logical[Z_AXIS] -= mbl.get_z(RAW_X_POSITION(logical[X_AXIS]), RAW_Y_POSITION(logical[Y_AXIS]));
          #endif
        }

      #elif ABL_PLANAR

        matrix_3x3 inverse = matrix_3x3::transpose(matrix);

        float dx = RAW_X_POSITION(logical[X_AXIS]) - (X_TILT_FULCRUM),
              dy = RAW_Y_POSITION(logical[Y_AXIS]) - (Y_TILT_FULCRUM),
              dz = RAW_Z_POSITION(logical[Z_AXIS]);

        apply_rotation_xyz(inverse, dx, dy, dz);

        logical[X_AXIS] = LOGICAL_X_POSITION(dx + X_TILT_FULCRUM);
        logical[Y_AXIS] = LOGICAL_Y_POSITION(dy + Y_TILT_FULCRUM);
        logical[Z_AXIS] = LOGICAL_Z_POSITION(dz);

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          const float c = bilinear_z_offset(logical);
          logical[Z_AXIS] = (z_fade_height * (RAW_Z_POSITION(logical[Z_AXIS]) - c)) / (z_fade_height - c);
        #else
          logical[Z_AXIS] -= bilinear_z_offset(logical);
        #endif

      #endif
    }

  #endif // PLANNER_LEVELING

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

    // Get the Z adjustment for non-linear bed leveling
    float Bedlevel::bilinear_z_offset(const float logical[XYZ]) {

      static float  z1, d2, z3, d4, L, D, ratio_x, ratio_y,
                    last_x = -999.999, last_y = -999.999;

      // Whole units for the grid line indices. Constrained within bounds.
      static int8_t gridx, gridy, nextx, nexty,
                    last_gridx = -99, last_gridy = -99;

      // XY relative to the probed area
      const float x = RAW_X_POSITION(logical[X_AXIS]) - bilinear_start[X_AXIS],
                  y = RAW_Y_POSITION(logical[Y_AXIS]) - bilinear_start[Y_AXIS];

      if (last_x != x) {
        last_x = x;
        ratio_x = x * ABL_BG_FACTOR(X_AXIS);
        const float gx = constrain(floor(ratio_x), 0, ABL_BG_POINTS_X - 1);
        ratio_x -= gx;      // Subtract whole to get the ratio within the grid box
        NOLESS(ratio_x, 0); // Never < 0.0. (> 1.0 is ok when nextx==gridx.)
        gridx = gx;
        nextx = min(gridx + 1, ABL_BG_POINTS_X - 1);
      }

      if (last_y != y || last_gridx != gridx) {

        if (last_y != y) {
          last_y = y;
          ratio_y = y * ABL_BG_FACTOR(Y_AXIS);
          const float gy = constrain(floor(ratio_y), 0, ABL_BG_POINTS_Y - 1);
          ratio_y -= gy;
          NOLESS(ratio_y, 0);
          gridy = gy;
          nexty = min(gridy + 1, ABL_BG_POINTS_Y - 1);
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

        // Bilinear interpolate. Needed since y or gridx has changed.
                    L = z1 + d2 * ratio_y;   // Linear interp. LF -> LB
        const float R = z3 + d4 * ratio_y;   // Linear interp. RF -> RB

        D = R - L;
      }

      const float offset = L + ratio_x * D;   // the offset almost always changes

      /*
      static float last_offset = 0;
      if (FABS(last_offset - offset) > 0.2) {
        SERIAL_MSG("Sudden Shift at ");
        SERIAL_MV("x=", x);
        SERIAL_MV(" / ", ABL_BG_SPACING(X_AXIS));
        SERIAL_EMV(" -> gridx=", gridx);
        SERIAL_MV(" y=", y);
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

    // Refresh after other values have been updated
    void Bedlevel::refresh_bed_level() {
      bilinear_grid_factor[X_AXIS] = RECIPROCAL(bilinear_grid_spacing[X_AXIS]);
      bilinear_grid_factor[Y_AXIS] = RECIPROCAL(bilinear_grid_spacing[Y_AXIS]);
      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        virt_interpolate();
      #endif
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
    void Bedlevel::extrapolate_unprobed_bed_level() {
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

    void Bedlevel::print_bilinear_leveling_grid() {
      SERIAL_LM(ECHO, "Bilinear Leveling Grid:");
      print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3,
        [](const uint8_t ix, const uint8_t iy){ return z_values[ix][iy]; }
      );
    }

    /**
     * Extrapolate a single point from its neighbors
     */
    void Bedlevel::extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
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
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EM(" (done)");
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

    #if ENABLED(ABL_BILINEAR_SUBDIVISION)

      void Bedlevel::print_bilinear_leveling_grid_virt() {
        SERIAL_LM(ECHO, "Subdivided with CATMULL ROM Leveling Grid:");
        print_2d_array(ABL_GRID_POINTS_VIRT_X, ABL_GRID_POINTS_VIRT_Y, 5,
          [](const uint8_t ix, const uint8_t iy){ return z_values_virt[ix][iy]; }
        );
      }

      #define LINEAR_EXTRAPOLATION(E, I) ((E) * 2 - (I))

      float Bedlevel::bed_level_virt_coord(const uint8_t x, const uint8_t y) {
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

      float Bedlevel::bed_level_virt_cmr(const float p[4], const uint8_t i, const float t) {
        return (
            p[i-1] * -t * sq(1 - t)
          + p[i]   * (2 - 5 * sq(t) + 3 * t * sq(t))
          + p[i+1] * t * (1 + 4 * t - 3 * sq(t))
          - p[i+2] * sq(t) * (1 - t)
        ) * 0.5;
      }

      float Bedlevel::bed_level_virt_2cmr(const uint8_t x, const uint8_t y, const float &tx, const float &ty) {
        float row[4], column[4];
        for (uint8_t i = 0; i < 4; i++) {
          for (uint8_t j = 0; j < 4; j++) {
            column[j] = bed_level_virt_coord(i + x - 1, j + y - 1);
          }
          row[i] = bed_level_virt_cmr(column, 1, ty);
        }
        return bed_level_virt_cmr(row, 1, tx);
      }

      void Bedlevel::virt_interpolate() {
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
  #endif // AUTO_BED_LEVELING_BILINEAR

  bool Bedlevel::leveling_is_valid() {
    #if ENABLED(MESH_BED_LEVELING)
      return mbl.has_mesh();
    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
      return !!bilinear_grid_spacing[X_AXIS];
    #elif ENABLED(AUTO_BED_LEVELING_UBL)
      return true;
    #else // 3POINT, LINEAR
      return true;
    #endif
  }

  bool Bedlevel::leveling_is_active() {
    #if ENABLED(MESH_BED_LEVELING)
      return mbl.active();
    #elif ENABLED(AUTO_BED_LEVELING_UBL)
      return ubl.state.active;
    #else // OLDSCHOOL_ABL
      return abl_enabled;
    #endif
  }

  /**
   * Turn bed leveling on or off, fixing the current
   * position as-needed.
   *
   * Disable: Current position = physical position
   *  Enable: Current position = "unleveled" physical position
   */
  void Bedlevel::set_bed_leveling_enabled(const bool enable/*=true*/) {

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      const bool can_change = (!enable || leveling_is_valid());
    #else
      constexpr bool can_change = true;
    #endif

    if (can_change && enable != leveling_is_active()) {

      #if ENABLED(MESH_BED_LEVELING)

        if (!enable)
          apply_leveling(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS]);

        const bool enabling = enable && leveling_is_valid();
        mbl.set_active(enabling);
        if (enabling) unapply_leveling(mechanics.current_position);

      #elif ENABLED(AUTO_BED_LEVELING_UBL)

        #if PLANNER_LEVELING
          if (ubl.state.active) {                       // leveling from on to off
            // change unleveled current_position to physical current_position without moving steppers.
            apply_leveling(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS]);
            ubl.state.active = false;                   // disable only AFTER calling apply_leveling
          }
          else {                                        // leveling from off to on
            ubl.state.active = true;                    // enable BEFORE calling unapply_leveling, otherwise ignored
            // change physical current_position to unleveled current_position without moving steppers.
            unapply_leveling(mechanics.current_position);
          }
        #else
          ubl.state.active = enable;                    // just flip the bit, current_position will be wrong until next move.
        #endif

      #else // OLDSCHOOL_ABL

        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          // Force bilinear_z_offset to re-calculate next time
          const float reset[XYZ] = { -9999.999, -9999.999, 0 };
          (void)bilinear_z_offset(reset);
        #endif

        // Enable or disable leveling compensation in the planner
        abl_enabled = enable;

        if (!enable)
          // When disabling just get the current position from the steppers.
          // This will yield the smallest error when first converted back to steps.
          mechanics.set_current_from_steppers_for_axis(
            #if ABL_PLANAR
              ALL_AXES
            #else
              Z_AXIS
            #endif
          );
        else
          // When enabling, remove compensation from the current position,
          // so compensation will give the right stepper counts.
          unapply_leveling(mechanics.current_position);

      #endif // ABL
    }
  }

  /**
   * Reset calibration results to zero.
   */
  void Bedlevel::reset() {
    set_bed_leveling_enabled(false);
    #if ENABLED(MESH_BED_LEVELING)
      if (leveling_is_valid()) {
        mbl.reset();
        mbl.set_has_mesh(false);
      }
    #else
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("Reset Bed Level");
      #endif
      #if ABL_PLANAR
        matrix.set_to_identity();
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        bilinear_start[X_AXIS] = bilinear_start[Y_AXIS] =
        bilinear_grid_spacing[X_AXIS] = bilinear_grid_spacing[Y_AXIS] = 0;
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
          for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
            z_values[x][y] = NAN;
      #endif
    #endif
  }

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

    void Bedlevel::set_z_fade_height(const float zfh) {

      const bool level_active = leveling_is_active();

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        if (level_active)
          set_bed_leveling_enabled(false);  // turn off before changing fade height for proper apply/unapply leveling to maintain current_position
        z_fade_height = zfh;
        inverse_z_fade_height = RECIPROCAL(zfh);
        if (level_active)
          set_bed_leveling_enabled(true);  // turn back on after changing fade height

      #else

        z_fade_height = zfh;
        inverse_z_fade_height = RECIPROCAL(zfh);

        if (level_active) {
          mechanics.set_current_from_steppers_for_axis(
            #if ABL_PLANAR
              ALL_AXES
            #else
              Z_AXIS
            #endif
          );
        }
      #endif
    }

  #endif // LEVELING_FADE_HEIGHT

  #if ENABLED(MESH_BED_LEVELING)

    void Bedlevel::mbl_mesh_report() {
      SERIAL_EM("Num X,Y: " STRINGIFY(GRID_MAX_POINTS_X) "," STRINGIFY(GRID_MAX_POINTS_Y));
      SERIAL_EMV("Z offset: ", mbl.zprobe_zoffset, 5);
      SERIAL_EM("Measured points:");
      print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 5,
        [](const uint8_t ix, const uint8_t iy) { return mbl.z_values[ix][iy]; }
      );
    }

    void Bedlevel::mesh_probing_done() {
      mbl.set_has_mesh(true);
      mechanics.Home(true);
      set_bed_leveling_enabled(true);
      #if ENABLED(MESH_G28_REST_ORIGIN)
        mechanics.current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS);
        mechanics.set_destination_to_current();
        mechanics.line_to_destination(mechanics.homing_feedrate_mm_s[Z_AXIS]);
        stepper.synchronize();
      #endif
    }

  #endif

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

    /**
     * Print calibration results for plotting or manual frame adjustment.
     */
    void Bedlevel::print_2d_array(const uint8_t sx, const uint8_t sy, const uint8_t precision, float (*fn)(const uint8_t, const uint8_t)) {

      #if DISABLED(SCAD_MESH_OUTPUT)
        SERIAL_STR(ECHO);
        for (uint8_t x = 0; x < sx; x++) {
          for (uint8_t i = 0; i < precision + 2 + (x < 10 ? 1 : 0); i++)
            SERIAL_CHR(' ');
          SERIAL_VAL((int)x);
        }
        SERIAL_EOL();
      #endif

      #if ENABLED(SCAD_MESH_OUTPUT)
        SERIAL_EM("measured_z = [");  // open 2D array
      #endif

      for (uint8_t y = 0; y < sy; y++) {
        #if ENABLED(SCAD_MESH_OUTPUT)
          SERIAL_MSG(" [");             // open sub-array
        #else
          SERIAL_STR(ECHO);
          if (y < 10) SERIAL_CHR(' ');
          SERIAL_VAL((int)y);
        #endif
        for (uint8_t x = 0; x < sx; x++) {
          SERIAL_CHR(' ');
          const float offset = fn(x, y);
          if (!isnan(offset)) {
            if (offset >= 0) SERIAL_CHR('+');
            SERIAL_VAL(offset, precision);
          }
          else {
            #if ENABLED(SCAD_MESH_OUTPUT)
              for (uint8_t i = 3; i < precision + 3; i++)
                SERIAL_CHR(' ');
              SERIAL_MSG("NAN");
            #else
              for (uint8_t i = 0; i < precision + 3; i++)
                SERIAL_CHR(i ? '=' : ' ');
            #endif
          }
          #if ENABLED(SCAD_MESH_OUTPUT)
            if (x < sx - 1) SERIAL_CHR(',');
          #endif
        }
        #if ENABLED(SCAD_MESH_OUTPUT)
          SERIAL_CHR(' ');
          SERIAL_CHR(']');                     // close sub-array
          if (y < sy - 1) SERIAL_CHR(',');
        #endif
        SERIAL_EOL();
      }

      #if ENABLED(SCAD_MESH_OUTPUT)
        SERIAL_MSG("\n];");                     // close 2D array
      #endif

      SERIAL_EOL();

    }

  #endif // ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

#endif // HAS_LEVELING
