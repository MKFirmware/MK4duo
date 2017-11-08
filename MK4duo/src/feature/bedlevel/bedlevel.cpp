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

#include "../../../MK4duo.h"

#if HAS_LEVELING

  Bedlevel bedlevel;

  #if HAS_LEVELING
    bool  Bedlevel::leveling_active = false; // Flag that auto bed leveling is enabled
    #if OLD_ABL
      int Bedlevel::xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
    #endif
  #endif

  #if ABL_PLANAR
    matrix_3x3 Bedlevel::matrix; // Transform to compensate for bed level
  #endif

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    float Bedlevel::z_fade_height,
          Bedlevel::inverse_z_fade_height,
          Bedlevel::last_fade_z;
  #endif

  #if ENABLED(PROBE_MANUALLY)
    bool Bedlevel::g29_in_progress = false;
  #else
    const bool Bedlevel::g29_in_progress = false;
  #endif

  #if PLANNER_LEVELING

    /**
     * rx, ry, rz - Cartesian positions in mm
     */
    void Bedlevel::apply_leveling(float &rx, float &ry, float &rz) {

      if (!leveling_active) return;

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        const float fade_scaling_factor = fade_scaling_factor_for_z(rz);
        if (!fade_scaling_factor) return;
      #else
        constexpr float fade_scaling_factor = 1.0;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        rz += ubl.get_z_correction(rx, ry) * fade_scaling_factor;

      #elif ENABLED(MESH_BED_LEVELING)

        rz += mbl.get_z(NATIVE_X_POSITION(rx), NATIVE_Y_POSITION(ry)
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            , fade_scaling_factor
          #endif
        );

      #elif ABL_PLANAR

        UNUSED(fade_scaling_factor);

        float dx = rx - (X_TILT_FULCRUM),
              dy = ry - (Y_TILT_FULCRUM);

        apply_rotation_xyz(matrix, dx, dy, rz);

        rx = dx + X_TILT_FULCRUM;
        ry = dy + Y_TILT_FULCRUM;

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        float tmp[XYZ] = { rx, ry, 0 };
        rz += abl.bilinear_z_offset(tmp) * fade_scaling_factor;

      #endif
    }

    void Bedlevel::unapply_leveling(float raw[XYZ]) {

      if (!leveling_active) return;

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (z_fade_height && raw[Z_AXIS] >= z_fade_height) return;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_UBL)

        const float z_correct = ubl.get_z_correction(raw[X_AXIS], raw[Y_AXIS]);
              float z_raw = raw[Z_AXIS] - z_correct;

        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

          // for P=physical_z, L=raw_z, M=mesh_z, H=fade_height,
          // Given P=L+M(1-L/H) (faded mesh correction formula for L<H)
          //  then L=P-M(1-L/H)
          //    so L=P-M+ML/H
          //    so L-ML/H=P-M
          //    so L(1-M/H)=P-M
          //    so L=(P-M)/(1-M/H) for L<H

          if (z_fade_height) {
            if (z_raw >= z_fade_height)
              z_raw = raw[Z_AXIS];
            else
              z_raw /= 1.0 - z_correct * inverse_z_fade_height;
          }

        #endif // ENABLE_LEVELING_FADE_HEIGHT

        raw[Z_AXIS] = z_raw;

      #elif ENABLED(MESH_BED_LEVELING)

        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          const float c = mbl.get_z(raw[X_AXIS], raw[Y_AXIS], 1.0);
          raw[Z_AXIS] = (z_fade_height * (raw[Z_AXIS] - c)) / (z_fade_height - c);
        #else
          raw[Z_AXIS] -= mbl.get_z(raw[X_AXIS], raw[Y_AXIS]);
        #endif

      #elif ABL_PLANAR

        matrix_3x3 inverse = matrix_3x3::transpose(matrix);

        float dx = raw[X_AXIS] - (X_TILT_FULCRUM),
              dy = raw[Y_AXIS] - (Y_TILT_FULCRUM),
              dz = raw[Z_AXIS];

        apply_rotation_xyz(inverse, dx, dy, dz);

        raw[X_AXIS] = dx + X_TILT_FULCRUM;
        raw[Y_AXIS] = dy + Y_TILT_FULCRUM;
        raw[Z_AXIS] = dz;

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          const float c = abl.bilinear_z_offset(raw);
          raw[Z_AXIS] = (z_fade_height * (raw[Z_AXIS] - c)) / (z_fade_height - c);
        #else
          raw[Z_AXIS] -= abl.bilinear_z_offset(raw);
        #endif

      #endif
    }

  #endif // PLANNER_LEVELING

  bool Bedlevel::leveling_is_valid() {
    #if ENABLED(MESH_BED_LEVELING)
      return mbl.has_mesh;
    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
      return !!abl.bilinear_grid_spacing[X_AXIS];
    #elif ENABLED(AUTO_BED_LEVELING_UBL)
      return true;
    #else // 3POINT, LINEAR
      return true;
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

    if (can_change && enable != leveling_active) {

      #if ENABLED(MESH_BED_LEVELING)

        if (!enable)
          apply_leveling(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS]);

        const bool enabling = enable && leveling_is_valid();
        leveling_active = enabling;
        if (enabling) unapply_leveling(mechanics.current_position);

      #elif ENABLED(AUTO_BED_LEVELING_UBL)

        #if PLANNER_LEVELING
          if (leveling_active) {                        // leveling from on to off
            // change unleveled current_position to physical current_position without moving steppers.
            apply_leveling(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS]);
            leveling_active = false;                    // disable only AFTER calling apply_leveling
          }
          else {                                        // leveling from off to on
            leveling_active = true;                     // enable BEFORE calling unapply_leveling, otherwise ignored
            // change physical current_position to unleveled current_position without moving steppers.
            unapply_leveling(mechanics.current_position);
          }
        #else
          leveling_active = enable;                     // just flip the bit, current_position will be wrong until next move.
        #endif

      #else // OLD_ABL

        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          // Force abl.bilinear_z_offset to re-calculate next time
          const float reset[XYZ] = { -9999.999, -9999.999, 0 };
          (void)abl.bilinear_z_offset(reset);
        #endif

        // Enable or disable leveling compensation in the planner
        leveling_active = enable;

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

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

    void Bedlevel::set_z_fade_height(const float zfh) {

      const bool level_active = leveling_active;

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        if (level_active) set_bed_leveling_enabled(false);  // turn off before changing fade height for proper apply/unapply leveling to maintain current_position
      #endif

      z_fade_height = zfh > 0 ? zfh : 0;
      inverse_z_fade_height = RECIPROCAL(z_fade_height);
      force_fade_recalc();

      if (level_active) {
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          set_bed_leveling_enabled(true);  // turn back on after changing fade height
        #else
          mechanics.set_current_from_steppers_for_axis(
            #if ABL_PLANAR
              ALL_AXES
            #else
              Z_AXIS
            #endif
          );
        #endif
      }
    }

  #endif // LEVELING_FADE_HEIGHT

  /**
   * Reset calibration results to zero.
   */
  void Bedlevel::reset() {
    set_bed_leveling_enabled(false);
    #if ENABLED(MESH_BED_LEVELING)
      if (leveling_is_valid()) {
        mbl.reset();
        mbl.has_mesh = false;
      }
    #else
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("Reset Bed Level");
      #endif
      #if ABL_PLANAR
        matrix.set_to_identity();
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        abl.bilinear_start[X_AXIS] = abl.bilinear_start[Y_AXIS] =
        abl.bilinear_grid_spacing[X_AXIS] = abl.bilinear_grid_spacing[Y_AXIS] = 0;
        for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
          for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
            abl.z_values[x][y] = NAN;
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        ubl.reset();
      #endif
    #endif
  }

  #if ENABLED(MESH_BED_LEVELING) 
    void Bedlevel::mesh_report() {
      SERIAL_EM("Num X,Y: " STRINGIFY(GRID_MAX_POINTS_X) "," STRINGIFY(GRID_MAX_POINTS_Y));
      SERIAL_EMV("Z offset: ", mbl.z_offset, 5);
      SERIAL_EM("Measured points:");
      bedlevel.print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 5,
        [](const uint8_t ix, const uint8_t iy) { return mbl.z_values[ix][iy]; }
      );
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
