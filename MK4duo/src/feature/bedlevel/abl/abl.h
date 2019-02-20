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
#pragma once

class AutoBedLevel {

  public: /** Constructor */

    AutoBedLevel() {}

  public: /** Public Parameters */

    static int    bilinear_grid_spacing[2],
                  bilinear_start[2];
    static float  z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

  private: /** Private Parameters */

    static float  bilinear_grid_factor[2];

    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      #define ABL_GRID_POINTS_VIRT_X (GRID_MAX_POINTS_X - 1) * (BILINEAR_SUBDIVISIONS) + 1
      #define ABL_GRID_POINTS_VIRT_Y (GRID_MAX_POINTS_Y - 1) * (BILINEAR_SUBDIVISIONS) + 1
      #define ABL_TEMP_POINTS_X (GRID_MAX_POINTS_X + 2)
      #define ABL_TEMP_POINTS_Y (GRID_MAX_POINTS_Y + 2)

      static float  bilinear_grid_factor_virt[2],
                    z_values_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
      static int    bilinear_grid_spacing_virt[2];
    #endif

  public: /** Public Function */

    static float bilinear_z_offset(const float raw[XYZ]);
    static void refresh_bed_level();

    /**
     * Fill in the unprobed points (corners of circular print surface)
     * using linear extrapolation, away from the center.
     */
    static void extrapolate_unprobed_bed_level();

    static void print_bilinear_leveling_grid();

    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      static void print_bilinear_leveling_grid_virt();
      static void virt_interpolate();
    #endif

    #if !IS_KINEMATIC
      void bilinear_line_to_destination(float fr_mm_s, uint16_t x_splits=0xFFFF, uint16_t y_splits=0xFFFF);
    #endif

  private: /** Private Function */

    /**
     * Extrapolate a single point from its neighbors
     */
    static void extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir);

    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      static float bed_level_virt_coord(const uint8_t x, const uint8_t y);
      static float bed_level_virt_cmr(const float p[4], const uint8_t i, const float t);
      static float bed_level_virt_2cmr(const uint8_t x, const uint8_t y, const float &tx, const float &ty);
    #endif

};

extern AutoBedLevel abl;

#define Z_VALUES(X,Y) abl.z_values[X][Y]
