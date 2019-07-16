/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

#include "../../../MK4duo.h"

/** Public Function */
#if ENABLED(NOZZLE_CLEAN_FEATURE)

  void Nozzle::clean(const uint8_t &pattern, const uint8_t &strokes, const float &radius, const uint8_t &objects, const bool clean_x, const bool clean_y, const bool clean_z) {

    point_t start = NOZZLE_CLEAN_START_POINT;
    point_t end   = NOZZLE_CLEAN_END_POINT;
    if (!clean_x) start.x = end.x = mechanics.current_position[X_AXIS];
    if (!clean_y) start.y = end.y = mechanics.current_position[Y_AXIS];
    if (!clean_z) start.z = end.z = mechanics.current_position[Z_AXIS];

    #if MECH(DELTA)
      if (mechanics.current_position[Z_AXIS] > mechanics.delta_clip_start_height)
        mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
    #endif

    switch (pattern) {
      case 1:
        zigzag(start, end, strokes, objects);
        break;

      case 2:
        circle(NOZZLE_CLEAN_START_POINT, NOZZLE_CLEAN_CIRCLE_MIDDLE, strokes, radius);
        break;

      default:
        stroke(start, end, strokes);
    }
  }

#endif // ENABLED(NOZZLE_CLEAN_FEATURE)

#if ENABLED(NOZZLE_PARK_FEATURE)

  void Nozzle::park(const uint8_t z_action, const point_t &park/*= tools.data.park_point*/) {

    const float fr_xy = NOZZLE_PARK_XY_FEEDRATE;
    const float fr_z  = NOZZLE_PARK_Z_FEEDRATE;

    switch (z_action) {
      case 1: // force Z-park height
        mechanics.do_blocking_move_to_z(park.z, fr_z);
        break;

      case 2: // Raise by Z-park height
        mechanics.do_blocking_move_to_z(MIN(mechanics.current_position[Z_AXIS] + park.z, Z_MAX_BED), fr_z);
        break;

      default: // Raise to Z-park height if lower
        mechanics.do_blocking_move_to_z(MAX(park.z, mechanics.current_position[Z_AXIS]), fr_z);
    }

    mechanics.do_blocking_move_to_xy(park.x, park.y, fr_xy);
  }

#endif // ENABLED(NOZZLE_PARK_FEATURE)

/** Private Function */
#if ENABLED(NOZZLE_CLEAN_FEATURE)

  void Nozzle::stroke(const point_t &start, const point_t &end, const uint8_t &strokes) {

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const float initial[XYZ] = {
        mechanics.current_position[X_AXIS],
        mechanics.current_position[Y_AXIS],
        mechanics.current_position[Z_AXIS]
      };
    #endif

    // Move to the starting point
    #if ENABLED(NOZZLE_CLEAN_NO_Z)
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    #else
      mechanics.do_blocking_move_to(start.x, start.y, start.z);
    #endif

    // Start the stroke pattern
    for (uint8_t i = 0; i < (strokes >>1); i++) {
      mechanics.do_blocking_move_to_xy(end.x, end.y);
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    }

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to(initial);
    #endif
  }

  void Nozzle::zigzag(const point_t &start, const point_t &end, const uint8_t &strokes, const uint8_t &objects) {

    const float diffx = end.x - start.x,
                diffy = end.y - start.y;

    if (!diffx || !diffy) return;

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const float initial[XYZ] = {
        mechanics.current_position[X_AXIS],
        mechanics.current_position[Y_AXIS],
        mechanics.current_position[Z_AXIS]
      };
    #endif

    #if ENABLED(NOZZLE_CLEAN_NO_Z)
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    #else
      mechanics.do_blocking_move_to(start.x, start.y, start.z);
    #endif

    const uint8_t zigs = objects << 1;
    const bool horiz = ABS(diffx) >= ABS(diffy);    // Do a horizontal wipe?
    const float P = (horiz ? diffx : diffy) / zigs;   // Period of each zig / zag
    const point_t *side;

    for (uint8_t j = 0; j < strokes; j++) {
      for (int8_t i = 0; i < zigs; i++) {
        side = (i & 1) ? &end : &start;
        if (horiz)
          mechanics.do_blocking_move_to_xy(start.x + i * P, side->y);
        else
          mechanics.do_blocking_move_to_xy(side->x, start.y + i * P);
      }

      for (int8_t i = zigs; i >= 0; i--) {
        side = (i & 1) ? &end : &start;
        if (horiz)
          mechanics.do_blocking_move_to_xy(start.x + i * P, side->y);
        else
          mechanics.do_blocking_move_to_xy(side->x, start.y + i * P);
      }
    }

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to(initial);
    #endif
  }

  void Nozzle::circle(const point_t &start, const point_t &middle, const uint8_t &strokes, const float &radius) {

    if (strokes == 0) return;

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const float initial[XYZ] = {
        mechanics.current_position[X_AXIS],
        mechanics.current_position[Y_AXIS],
        mechanics.current_position[Z_AXIS]
      };
    #endif

    #if ENABLED(NOZZLE_CLEAN_NO_Z)
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    #else
      mechanics.do_blocking_move_to(start.x, start.y, start.z);
    #endif

    for (uint8_t s = 0; s < strokes; s++) {
      for (uint8_t i = 0; i < NOZZLE_CLEAN_CIRCLE_FN; i++) {
        mechanics.do_blocking_move_to_xy(
          middle.x + SIN((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius,
          middle.y + COS((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius
        );
      }
    }

    // Let's be safe
    mechanics.do_blocking_move_to_xy(start.x, start.y);

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to_xy(initial[X_AXIS], initial[Y_AXIS], initial[Z_AXIS]);
    #endif
  }

#endif // ENABLED(NOZZLE_CLEAN_FEATURE)
