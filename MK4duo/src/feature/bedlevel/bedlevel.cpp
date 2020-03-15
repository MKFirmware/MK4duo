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
 * bedlevel.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if HAS_LEVELING

Bedlevel bedlevel;

/** Public Parameters */
level_flag_t Bedlevel::flag;

#if OLD_ABL
  int Bedlevel::xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
#endif

#if ABL_PLANAR
  matrix_3x3 Bedlevel::matrix; // Transform to compensate for bed level
#endif

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
  float Bedlevel::z_fade_height,
        Bedlevel::inverse_z_fade_height,
        Bedlevel::last_fade_z;
#endif

/** Public Function */
void Bedlevel::factory_parameters() {
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    z_fade_height = 0.0f;
  #endif
  reset();
}

/**
 * raw  - Cartesian positions in mm
 *        Leveled XYZ on completion
 */
void Bedlevel::apply_leveling(xyz_pos_t &raw) {

  if (!flag.leveling_active) return;

  #if ABL_PLANAR

    xy_pos_t d = raw - level_fulcrum();
    apply_rotation_xyz(matrix, d.x, d.y, raw.z);
    raw = d + level_fulcrum();

  #elif HAS_MESH

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      const float fade_scaling_factor = fade_scaling_factor_for_z(raw.z);
    #else
      constexpr float fade_scaling_factor = 1.0;
    #endif

    raw.z += (
      #if ENABLED(MESH_BED_LEVELING)
        mbl.get_z(raw
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            , fade_scaling_factor
          #endif
        )
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        fade_scaling_factor ? fade_scaling_factor * ubl.get_z_correction(raw) : 0.0
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        fade_scaling_factor ? fade_scaling_factor * abl.bilinear_z_offset(raw) : 0.0
      #endif
    );

  #endif
}

void Bedlevel::unapply_leveling(xyz_pos_t &raw) {

  if (!flag.leveling_active) return;

  #if ABL_PLANAR

    matrix_3x3 inverse = matrix_3x3::transpose(matrix);

    xy_pos_t d = raw - level_fulcrum();
    apply_rotation_xyz(inverse, d.x, d.y, raw.z);
    raw = d + level_fulcrum();

  #elif HAS_MESH

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      const float fade_scaling_factor = fade_scaling_factor_for_z(raw.z);
    #elif HAS_MESH
      constexpr float fade_scaling_factor = 1.0;
    #endif

    raw.z -= (
      #if ENABLED(MESH_BED_LEVELING)
        mbl.get_z(raw
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            , fade_scaling_factor
          #endif
        )
      #elif ENABLED(AUTO_BED_LEVELING_UBL)
        fade_scaling_factor ? fade_scaling_factor * ubl.get_z_correction(raw.x, raw.y) : 0.0
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        fade_scaling_factor ? fade_scaling_factor * abl.bilinear_z_offset(raw) : 0.0
      #endif
    );

  #endif
}

bool Bedlevel::leveling_is_valid() {
  #if ENABLED(MESH_BED_LEVELING)
    return mbl.has_mesh();
  #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
    return !!abl.data.bilinear_grid_spacing.x;
  #elif ENABLED(AUTO_BED_LEVELING_UBL)
    return ubl.mesh_is_valid();
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

  flag.leveling_previous = flag.leveling_active;

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    const bool can_change = (!enable || leveling_is_valid());
  #else
    constexpr bool can_change = true;
  #endif

  if (can_change && enable != flag.leveling_active) {

    planner.synchronize();

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      // Force abl.bilinear_z_offset to re-calculate next time
      const xyz_pos_t reset = { -9999.999, -9999.999, 0 };
      (void)abl.bilinear_z_offset(reset);
    #endif

    if (flag.leveling_active) {      // leveling from on to off
      if (printer.debugFeature()) DEBUG_POS("Leveling ON", mechanics.position);
      // change unleveled position.x to physical position.x without moving steppers.
      apply_leveling(mechanics.position);
      flag.leveling_active = false;  // disable only AFTER calling apply_leveling
      if (printer.debugFeature()) DEBUG_POS("...Now OFF", mechanics.position);
    }
    else {                          // leveling from off to on
      if (printer.debugFeature()) DEBUG_POS("Leveling OFF", mechanics.position);
      flag.leveling_active = true;  // enable BEFORE calling unapply_leveling, otherwise ignored
      // change physical position.x to unleveled position.x without moving steppers.
      unapply_leveling(mechanics.position);
      if (printer.debugFeature()) DEBUG_POS("...Now ON", mechanics.position);
    }

    mechanics.sync_plan_position();
  }
}

#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

  void Bedlevel::set_z_fade_height(const float zfh, const bool do_report/*=true*/) {

    if (z_fade_height == zfh) return;

    const bool leveling_was_active = flag.leveling_active;
    set_bed_leveling_enabled(false);

    z_fade_height = zfh > 0 ? zfh : 0;
    inverse_z_fade_height = RECIPROCAL(z_fade_height);
    force_fade_recalc();

    if (leveling_was_active) {
      const xyz_pos_t oldpos = mechanics.position;
      set_bed_leveling_enabled();
      if (do_report && oldpos != mechanics.position)
        mechanics.report_position();
    }
  }

#endif // LEVELING_FADE_HEIGHT

/**
 * Reset calibration results to zero.
 */
void Bedlevel::reset() {

  if (printer.debugFeature()) DEBUG_EM("Reset Bed Level");

  #if ENABLED(AUTO_BED_LEVELING_UBL)
    ubl.reset();
  #else
    set_bed_leveling_enabled(false);
    #if ENABLED(MESH_BED_LEVELING)
      mbl.factory_parameters();
    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
      abl.data.bilinear_start.reset();
      abl.data.bilinear_grid_spacing.reset();
      GRID_LOOP(x, y) abl.data.z_values[x][y] = NAN;
    #elif ABL_PLANAR
      matrix.set_to_identity();
    #endif
  #endif
}

#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

  /**
   * Print calibration results for plotting or manual frame adjustment.
   */
  void Bedlevel::print_2d_array(const uint8_t sx, const uint8_t sy, const uint8_t precision, element_2d_fn fn) {

    #if DISABLED(SCAD_MESH_OUTPUT)
      for (uint8_t x = 0; x < sx; x++) {
        SERIAL_SP(precision + (x < 10 ? 3 : 2));
        SERIAL_VAL((int)x);
      }
      SERIAL_EOL();
    #endif

    #if ENABLED(SCAD_MESH_OUTPUT)
      SERIAL_EM("measured_z = [");    // open 2D array
    #endif

    for (uint8_t y = 0; y < sy; y++) {
      #if ENABLED(SCAD_MESH_OUTPUT)
        SERIAL_MSG(" [");             // open sub-array
      #else
        if (y < 10) SERIAL_CHR(' ');
        SERIAL_VAL((int)y);
      #endif
      for (uint8_t x = 0; x < sx; x++) {
        SERIAL_CHR(' ');
        const float offset = fn(x, y);
        if (!isnan(offset)) {
          if (offset >= 0) SERIAL_CHR('+');
          SERIAL_VAL(offset, int(precision));
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
        SERIAL_CHR(']');                      // close sub-array
        if (y < sy - 1) SERIAL_CHR(',');
      #endif
      SERIAL_EOL();
    }

    #if ENABLED(SCAD_MESH_OUTPUT)
      SERIAL_MSG("];");                       // close 2D array
    #endif

    SERIAL_EOL();

  }

#endif // ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

#if ENABLED(MESH_BED_LEVELING) || HAS_PROBE_MANUALLY

  void Bedlevel::manual_goto_xy(const xy_pos_t &pos) {

    #if MANUAL_PROBE_HEIGHT > 0
      const float prev_z = mechanics.position.z;
      mechanics.do_blocking_move_to_xy_z(pos, MANUAL_PROBE_HEIGHT);
      mechanics.do_blocking_move_to_z(prev_z);
    #else
      mechanics.do_blocking_move_to_xy(pos);
    #endif

    mechanics.position = pos;

    #if ENABLED(LCD_BED_LEVELING)
      lcdui.wait_for_move = false;
    #endif

  }

#endif

#endif // HAS_LEVELING
