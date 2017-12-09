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

#include "../../../MK4duo.h"

#if ENABLED(NOZZLE_CLEAN_FEATURE)

  constexpr float nozzle_clean_start_point[XYZ] = NOZZLE_CLEAN_START_POINT, // One corner of the wipe area
                  nozzle_clean_end_point[XYZ]   = NOZZLE_CLEAN_END_POINT,   // Opposite corner of the wipe area
                  nozzle_clean_length           = FABS(nozzle_clean_start_point[X_AXIS] - nozzle_clean_end_point[X_AXIS]),  // X size of wipe area
                  nozzle_clean_height           = FABS(nozzle_clean_start_point[Y_AXIS] - nozzle_clean_end_point[Y_AXIS]);  // Y size of wipe area
  constexpr bool  nozzle_clean_horizontal       = nozzle_clean_length >= nozzle_clean_height;                               // whether to zig-zag horizontally or vertically

  /**
   * @brief Stroke clean pattern
   * @details Wipes the nozzle back and forth in a linear movement
   *
   * @param start defining the starting point
   * @param end defining the ending point
   * @param strokes number of strokes to execute
   */
  void Nozzle::stroke(const float *start, const float *end, const uint8_t &strokes) {

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const float initial[XYZ] = {
        mechanics.current_position[X_AXIS],
        mechanics.current_position[Y_AXIS],
        mechanics.current_position[Z_AXIS]
      };
    #endif // NOZZLE_CLEAN_GOBACK

    // Move to the starting point
    mechanics.do_blocking_move_to(start[X_AXIS], start[Y_AXIS], start[Z_AXIS]);

    // Start the stroke pattern
    for (uint8_t i = 0; i < (strokes >>1); i++) {
      mechanics.do_blocking_move_to_xy(end[X_AXIS], end[Y_AXIS]);
      mechanics.do_blocking_move_to_xy(start[X_AXIS], start[Y_AXIS]);
    }

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to(initial[X_AXIS], initial[Y_AXIS], initial[Z_AXIS]);
    #endif // NOZZLE_CLEAN_GOBACK

  }

  /**
   * @brief Zig-zag clean pattern
   * @details Apply a zig-zag cleanning pattern
   *
   * @param start defining the starting point
   * @param end defining the ending point
   * @param strokes number of strokes to execute
   * @param objects number of objects to create
   */
  void Nozzle::zigzag(const float *start, const float *end, const uint8_t &strokes, const uint8_t &objects) {

    const float diffx = end[X_AXIS] - start[X_AXIS],
                diffy = end[Y_AXIS] - start[Y_AXIS];
    
    if (!diffx || !diffy) return;

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const float initial[XYZ] = {
        mechanics.current_position[X_AXIS],
        mechanics.current_position[Y_AXIS],
        mechanics.current_position[Z_AXIS]
      };
    #endif // NOZZLE_CLEAN_GOBACK

    mechanics.do_blocking_move_to(start[X_AXIS], start[Y_AXIS], start[Z_AXIS]);

    const uint8_t zigs = objects << 1;
    const bool horiz = FABS(diffx) >= FABS(diffy);    // Do a horizontal wipe?
    const float P = (horiz ? diffx : diffy) / zigs;   // Period of each zig / zag
    float side[XYZ];

    for (uint8_t j = 0; j < strokes; j++) {
      for (int8_t i = 0; i < zigs; i++) {
        (i & 1) ? COPY_ARRAY(side, end) : COPY_ARRAY(side, start);
        if (horiz)
          mechanics.do_blocking_move_to_xy(start[X_AXIS] + i * P, side[Y_AXIS]);
        else
          mechanics.do_blocking_move_to_xy(side[X_AXIS], start[Y_AXIS] + i * P);
      }

      for (int8_t i = zigs; i >= 0; i--) {
        (i & 1) ? COPY_ARRAY(side, end) : COPY_ARRAY(side, start);
        if (horiz)
          mechanics.do_blocking_move_to_xy(start[X_AXIS] + i * P, side[Y_AXIS]);
        else
          mechanics.do_blocking_move_to_xy(side[X_AXIS], start[Y_AXIS] + i * P);
      }
    }

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to(initial[X_AXIS], initial[Y_AXIS], initial[Z_AXIS]);
    #endif // NOZZLE_CLEAN_GOBACK

  }

  /**
   * @brief Circular clean pattern
   * @details Apply a circular cleaning pattern
   *
   * @param start defining the starting point
   * @param middle defining the middle of circle
   * @param strokes number of strokes to execute
   * @param radius of circle
   */
  void Nozzle::circle(const float *start, const float *middle, const uint8_t &strokes, const float &radius) {

    if (strokes == 0) return;

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const float initial[XYZ] = {
        mechanics.current_position[X_AXIS],
        mechanics.current_position[Y_AXIS],
        mechanics.current_position[Z_AXIS]
      };
    #endif // NOZZLE_CLEAN_GOBACK

    mechanics.do_blocking_move_to(start[X_AXIS], start[Y_AXIS], start[Z_AXIS]);

    for (uint8_t s = 0; s < strokes; s++) {
      for (uint8_t i = 0; i < NOZZLE_CLEAN_CIRCLE_FN; i++) {
        mechanics.do_blocking_move_to_xy(
          middle[X_AXIS] + SIN((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius,
          middle[Y_AXIS] + COS((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius
        );
      }
    }

    // Let's be safe
    mechanics.do_blocking_move_to_xy(start[X_AXIS], start[Y_AXIS]);

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      mechanics.do_blocking_move_to_xy(initial[X_AXIS], initial[Y_AXIS], initial[Z_AXIS]);
    #endif

  }

  /**
   * @brief Clean the nozzle
   * @details Starts the selected clean procedure pattern
   *
   * @param pattern one of the available patterns
   * @param argument depends on the cleaning pattern
   */
  void Nozzle::clean(const uint8_t &pattern, const uint8_t &strokes, const float &radius, const uint8_t &objects) {

    const float start[] = NOZZLE_CLEAN_START_POINT,
                end[]   = NOZZLE_CLEAN_END_POINT,
                middle[]= NOZZLE_CLEAN_CIRCLE_MIDDLE;

    #if MECH(DELTA)
      if (mechanics.current_position[Z_AXIS] > mechanics.delta_clip_start_height)
        mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
    #endif

    switch (pattern) {
      case 1:
        Nozzle::zigzag(start, end, strokes, objects);
        break;

      case 2:
        Nozzle::circle(start, middle, strokes, radius);
        break;

      default:
        Nozzle::stroke(start, end, strokes);
    }
  }

#endif // ENABLED(NOZZLE_CLEAN_FEATURE)

#if ENABLED(NOZZLE_PARK_FEATURE)

  void Nozzle::park(const uint8_t &z_action) {

    const float park[]  = NOZZLE_PARK_POINT;

    switch(z_action) {
      case 1: // force Z-park height
        mechanics.do_blocking_move_to_z(park[Z_AXIS]);
        break;

      case 2: // Raise by Z-park height
        mechanics.do_blocking_move_to_z(min(mechanics.current_position[Z_AXIS] + park[Z_AXIS], Z_MAX_POS));
        break;

      default: // Raise to Z-park height if lower
        mechanics.do_blocking_move_to_z(max(park[Z_AXIS], mechanics.current_position[Z_AXIS]));
    }

    mechanics.do_blocking_move_to_xy(park[X_AXIS], park[Y_AXIS]);
  }

#endif // ENABLED(NOZZLE_PARK_FEATURE)
