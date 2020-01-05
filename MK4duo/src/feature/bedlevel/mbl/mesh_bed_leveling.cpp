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

#if ENABLED(MESH_BED_LEVELING)

#include "mesh_bed_leveling.h"

mesh_bed_leveling mbl;

/** Public Parameters */
mbl_data_t mesh_bed_leveling::data;

/** Public Function */
void mesh_bed_leveling::factory_parameters() {
  for (uint8_t i = 0; i < GRID_MAX_POINTS_X; ++i)
    data.index_to_xpos[i] = MESH_MIN_X + i * (MESH_X_DIST);
  for (uint8_t i = 0; i < GRID_MAX_POINTS_Y; ++i)
    data.index_to_ypos[i] = MESH_MIN_Y + i * (MESH_Y_DIST);
  data.z_offset = 0;
  ZERO(data.z_values);
}

/**
 * Prepare a mesh-leveled linear move in a Cartesian setup,
 * splitting the move where it crosses mesh borders.
 */
void mesh_bed_leveling::line_to_destination(const feedrate_t &scaled_fr_mm_s, uint8_t x_splits/*=0xFF*/, uint8_t y_splits/*=0xFF*/) {
  // Get current and destination cells for this line
  xy_int8_t scel = cell_indexes(mechanics.position), ecel = cell_indexes(mechanics.destination);
  NOMORE(scel.x, GRID_MAX_POINTS_X - 2);
  NOMORE(scel.y, GRID_MAX_POINTS_Y - 2);
  NOMORE(ecel.x, GRID_MAX_POINTS_X - 2);
  NOMORE(ecel.y, GRID_MAX_POINTS_Y - 2);

  // Start and end in the same cell? No split needed.
  if (scel == ecel) {
    mechanics.line_to_destination(scaled_fr_mm_s);
    mechanics.position = mechanics.destination;
    return;
  }

  #define MBL_SEGMENT_END(A) (mechanics.position.A + (mechanics.destination.A - mechanics.position.A) * normalized_dist)

  float normalized_dist;
  xyze_pos_t end;
  const int8_t gcx = MAX(scel.x, ecel.x), gcy = MAX(scel.y, ecel.y);

  // Crosses on the X and not already split on this X?
  // The x_splits flags are insurance against rounding errors.
  if (ecel.x != scel.x && TEST(x_splits, gcx)) {
    // Split on the X grid line
    CBI(x_splits, gcx);
    end = mechanics.destination;
    mechanics.destination.x = data.index_to_xpos[gcx];
    normalized_dist = (mechanics.destination.x - mechanics.position.x) / (end.x - mechanics.position.x);
    mechanics.destination.y = MBL_SEGMENT_END(y);
  }
  // Crosses on the Y and not already split on this Y?
  else if (ecel.y != scel.y && TEST(y_splits, gcy)) {
    // Split on the Y grid line
    CBI(y_splits, gcy);
    end = mechanics.destination;
    mechanics.destination.y = data.index_to_ypos[gcy];
    normalized_dist = (mechanics.destination.y - mechanics.position.y) / (end.y - mechanics.position.y);
    mechanics.destination.x = MBL_SEGMENT_END(x);
  }
  else {
    // Must already have been split on these border(s)
    // This should be a rare case.
    mechanics.line_to_destination(scaled_fr_mm_s);
    mechanics.position = mechanics.destination;
    return;
  }

  mechanics.destination.z = MBL_SEGMENT_END(z);
  mechanics.destination.e = MBL_SEGMENT_END(e);

  // Do the split and look for more borders
  line_to_destination(scaled_fr_mm_s, x_splits, y_splits);

  // Restore destination from stack
  mechanics.destination = end;
  line_to_destination(scaled_fr_mm_s, x_splits, y_splits);
}

void mesh_bed_leveling::report_mesh() {
  SERIAL_MSG("Num X,Y: " STRINGIFY(GRID_MAX_POINTS_X) "," STRINGIFY(GRID_MAX_POINTS_Y));
  SERIAL_EMV(" mesh. Z offset: ", data.z_offset, 5);
  SERIAL_EM("Measured points:");
  bedlevel.print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 5,
    [](const uint8_t ix, const uint8_t iy) { return data.z_values[ix][iy]; }
  );
}

#endif  // MESH_BED_LEVELING
