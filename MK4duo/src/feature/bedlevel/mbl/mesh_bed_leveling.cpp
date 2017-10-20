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

#include "../../../../MK4duo.h"

#if ENABLED(MESH_BED_LEVELING)

  #include "mesh_bed_leveling.h"

  mesh_bed_leveling mbl;

  bool mesh_bed_leveling::has_mesh;

  float mesh_bed_leveling::z_offset,
        mesh_bed_leveling::z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y],
        mesh_bed_leveling::index_to_xpos[GRID_MAX_POINTS_X],
        mesh_bed_leveling::index_to_ypos[GRID_MAX_POINTS_Y];

  mesh_bed_leveling::mesh_bed_leveling() {
    for (uint8_t i = 0; i < GRID_MAX_POINTS_X; ++i)
      index_to_xpos[i] = MESH_MIN_X + i * (MESH_X_DIST);
    for (uint8_t i = 0; i < GRID_MAX_POINTS_Y; ++i)
      index_to_ypos[i] = MESH_MIN_Y + i * (MESH_Y_DIST);
    reset();
  }

  void mesh_bed_leveling::reset() {
    has_mesh = false;
    z_offset = 0;
    ZERO(z_values);
  }

  /**
   * Prepare a mesh-leveled linear move in a Cartesian setup,
   * splitting the move where it crosses mesh borders.
   */
  void mesh_bed_leveling::line_to_destination(float fr_mm_s, uint8_t x_splits/*= 0xFF*/, uint8_t y_splits/*= 0xFF*/) {
    int cx1 = mbl.cell_index_x(RAW_CURRENT_POSITION(X)),
        cy1 = mbl.cell_index_y(RAW_CURRENT_POSITION(Y)),
        cx2 = mbl.cell_index_x(RAW_X_POSITION(mechanics.destination[X_AXIS])),
        cy2 = mbl.cell_index_y(RAW_Y_POSITION(mechanics.destination[Y_AXIS]));
    NOMORE(cx1, GRID_MAX_POINTS_X - 2);
    NOMORE(cy1, GRID_MAX_POINTS_Y - 2);
    NOMORE(cx2, GRID_MAX_POINTS_X - 2);
    NOMORE(cy2, GRID_MAX_POINTS_Y - 2);

    if (cx1 == cx2 && cy1 == cy2) {
      // Start and end on same mesh square
      mechanics.line_to_destination(fr_mm_s);
      mechanics.set_current_to_destination();
      return;
    }

    #define MBL_SEGMENT_END(A) (mechanics.current_position[A ##_AXIS] + (mechanics.destination[A ##_AXIS] - mechanics.current_position[A ##_AXIS]) * normalized_dist)

    float normalized_dist, end[XYZE];

    // Split at the left/front border of the right/top square
    int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      COPY_ARRAY(end, mechanics.destination);
      mechanics.destination[X_AXIS] = LOGICAL_X_POSITION(index_to_xpos[gcx]);
      normalized_dist = (mechanics.destination[X_AXIS] - mechanics.current_position[X_AXIS]) / (end[X_AXIS] - mechanics.current_position[X_AXIS]);
      mechanics.destination[Y_AXIS] = MBL_SEGMENT_END(Y);
      CBI(x_splits, gcx);
    }
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      COPY_ARRAY(end, mechanics.destination);
      mechanics.destination[Y_AXIS] = LOGICAL_Y_POSITION(index_to_ypos[gcy]);
      normalized_dist = (mechanics.destination[Y_AXIS] - mechanics.current_position[Y_AXIS]) / (end[Y_AXIS] - mechanics.current_position[Y_AXIS]);
      mechanics.destination[X_AXIS] = MBL_SEGMENT_END(X);
      CBI(y_splits, gcy);
    }
    else {
      // Already split on a border
      mechanics.line_to_destination(fr_mm_s);
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

  void mesh_bed_leveling::probing_done() {
    has_mesh = true;
    mechanics.Home(true);
    bedlevel.set_bed_leveling_enabled(true);
    #if ENABLED(MESH_G28_REST_ORIGIN)
      mechanics.current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS);
      mechanics.set_destination_to_current();
      mechanics.line_to_destination(mechanics.homing_feedrate_mm_s[Z_AXIS]);
      stepper.synchronize();
    #endif
  }

#endif  // MESH_BED_LEVELING
