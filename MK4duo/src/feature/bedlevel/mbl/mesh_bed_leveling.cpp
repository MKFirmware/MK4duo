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

#if ENABLED(MESH_BED_LEVELING)

  #include "mesh_bed_leveling.h"

  mesh_bed_leveling mbl;

  float mesh_bed_leveling::z_offset,
        mesh_bed_leveling::z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y],
        mesh_bed_leveling::index_to_xpos[GRID_MAX_POINTS_X],
        mesh_bed_leveling::index_to_ypos[GRID_MAX_POINTS_Y];

  void mesh_bed_leveling::reset() {
    for (uint8_t i = 0; i < GRID_MAX_POINTS_X; ++i)
      index_to_xpos[i] = MESH_MIN_X + i * (MESH_X_DIST);
    for (uint8_t i = 0; i < GRID_MAX_POINTS_Y; ++i)
      index_to_ypos[i] = MESH_MIN_Y + i * (MESH_Y_DIST);
    z_offset = 0;
    ZERO(z_values);
  }

  /**
   * Prepare a mesh-leveled linear move in a Cartesian setup,
   * splitting the move where it crosses mesh borders.
   */
  void mesh_bed_leveling::line_to_destination(float fr_mm_s, uint16_t x_splits/*=0xFFFF*/, uint16_t y_splits/*=0xFFFF*/) {
    int cx1 = cell_index_x(mechanics.current_position[X_AXIS]),
        cy1 = cell_index_y(mechanics.current_position[Y_AXIS]),
        cx2 = cell_index_x(mechanics.destination[X_AXIS]),
        cy2 = cell_index_y(mechanics.destination[Y_AXIS]);

    if (cx1 == cx2 && cy1 == cy2) {
      // Start and end on same mesh square
      mechanics.buffer_line_to_destination(fr_mm_s);
      mechanics.set_current_to_destination();
      return;
    }

    #define MBL_SEGMENT_END(A) (mechanics.current_position[_AXIS(A)] + (mechanics.destination[_AXIS(A)] - mechanics.current_position[_AXIS(A)]) * normalized_dist)

    float normalized_dist, end[XYZE];
    const int8_t gcx = MAX(cx1, cx2), gcy = MAX(cy1, cy2);

    // Crosses on the X and not already split on this X?
    // The x_splits flags are insurance against rounding errors.
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      // Split on the X grid line
      CBI(x_splits, gcx);
      COPY_ARRAY(end, mechanics.destination);
      mechanics.destination[X_AXIS] = index_to_xpos[gcx];
      normalized_dist = (mechanics.destination[X_AXIS] - mechanics.current_position[X_AXIS]) / (end[X_AXIS] - mechanics.current_position[X_AXIS]);
      mechanics.destination[Y_AXIS] = MBL_SEGMENT_END(Y);
    }
    // Crosses on the Y and not already split on this Y?
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      // Split on the Y grid line
      CBI(y_splits, gcy);
      COPY_ARRAY(end, mechanics.destination);
      mechanics.destination[Y_AXIS] = index_to_ypos[gcy];
      normalized_dist = (mechanics.destination[Y_AXIS] - mechanics.current_position[Y_AXIS]) / (end[Y_AXIS] - mechanics.current_position[Y_AXIS]);
      mechanics.destination[X_AXIS] = MBL_SEGMENT_END(X);
    }
    else {
      // Must already have been split on these border(s)
      // This should be a rare case.
      mechanics.buffer_line_to_destination(fr_mm_s);
      mechanics.set_current_to_destination();
      return;
    }

    mechanics.destination[Z_AXIS] = MBL_SEGMENT_END(Z);
    mechanics.destination[E_AXIS] = MBL_SEGMENT_END(E);

    // Do the split and look for more borders
    line_to_destination(fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    COPY_ARRAY(mechanics.destination, end);
    line_to_destination(fr_mm_s, x_splits, y_splits);  
  }

  void mesh_bed_leveling::report_mesh() {
    SERIAL_EM("Num X,Y: " STRINGIFY(GRID_MAX_POINTS_X) "," STRINGIFY(GRID_MAX_POINTS_Y));
    SERIAL_EMV("Z offset: ", z_offset, 5);
    SERIAL_EM("Measured points:");
    bedlevel.print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 5,
      [](const uint8_t ix, const uint8_t iy) { return z_values[ix][iy]; }
    );
  }

#endif  // MESH_BED_LEVELING
