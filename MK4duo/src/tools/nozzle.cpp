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

#include "../../MK4duo.h"

#if ENABLED(NOZZLE_CLEAN_FEATURE)

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
    mechanics.do_blocking_move_to_xy(start[X_AXIS], start[Y_AXIS]);
    mechanics.do_blocking_move_to_z(start[Z_AXIS]);

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

    const float A = nozzle_clean_horizontal ? nozzle_clean_height : nozzle_clean_length, // [twice the] Amplitude
                P = (nozzle_clean_horizontal ? nozzle_clean_length : nozzle_clean_height) / (objects << 1); // Period

    // Don't allow impossible triangles
    if (A <= 0.0f || P <= 0.0f ) return;

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const float initial[XYZ] = {
        mechanics.current_position[X_AXIS],
        mechanics.current_position[Y_AXIS],
        mechanics.current_position[Z_AXIS]
      };
    #endif // NOZZLE_CLEAN_GOBACK

    for (uint8_t j = 0; j < strokes; j++) {
      for (uint8_t i = 0; i < (objects << 1); i++) {
        const float x = start[X_AXIS] + ( nozzle_clean_horizontal ? i * P : (A / P) * (P - FABS(FMOD((i * P), (2 * P)) - P)) ),
                    y = start[Y_AXIS] + (!nozzle_clean_horizontal ? i * P : (A / P) * (P - FABS(FMOD((i * P), (2 * P)) - P)) );

        mechanics.do_blocking_move_to_xy(x, y);
        if (i == 0) mechanics.do_blocking_move_to_z(start[Z_AXIS]);
      }

      for (int i = (objects << 1); i > -1; i--) {
        const float x = start[X_AXIS] + ( nozzle_clean_horizontal ? i * P : (A / P) * (P - FABS(FMOD((i * P), (2 * P)) - P)) ),
                    y = start[Y_AXIS] + (!nozzle_clean_horizontal ? i * P : (A / P) * (P - FABS(FMOD((i * P), (2 * P)) - P)) );

        mechanics.do_blocking_move_to_xy(x, y);
      }
    }

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to_z(initial[Z_AXIS]);
      mechanics.do_blocking_move_to_xy(initial[X_AXIS], initial[Y_AXIS]);
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

    if (start[Z_AXIS] <= mechanics.current_position[Z_AXIS]) {
      // Order of movement is pretty darn important here
      mechanics.do_blocking_move_to_xy(start[X_AXIS], start[Y_AXIS]);
      mechanics.do_blocking_move_to_z(start[Z_AXIS]);
    }
    else {
      mechanics.do_blocking_move_to_z(start[Z_AXIS]);
      mechanics.do_blocking_move_to_xy(start[X_AXIS], start[Y_AXIS]);
    }

    float x, y;
    for (uint8_t s = 0; s < strokes; s++) {
      for (uint8_t i = 0; i < NOZZLE_CLEAN_CIRCLE_FN; i++) {
        x = middle[X_AXIS] + sin((M_2_PI / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius;
        y = middle[Y_AXIS] + cos((M_2_PI / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius;

        mechanics.do_blocking_move_to_xy(x, y);
      }
    }

    // Let's be safe
    mechanics.do_blocking_move_to_xy(start[X_AXIS], start[Y_AXIS]);

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      if (start[Z_AXIS] <= initial[Z_AXIS]) {
        // As above order is important
        mechanics.do_blocking_move_to_z(initial[Z_AXIS]);
        mechanics.do_blocking_move_to_xy(initial[X_AXIS], initial[Y_AXIS]);
      }
      else {
        mechanics.do_blocking_move_to_xy(initial[X_AXIS], initial[Y_AXIS]);
        mechanics.do_blocking_move_to_z(initial[Z_AXIS]);
      }
    #endif // NOZZLE_CLEAN_GOBACK

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

    const float z       = mechanics.current_position[Z_AXIS],
                park[]  = NOZZLE_PARK_POINT;

    switch(z_action) {
      case 1: // force Z-park height
        mechanics.do_blocking_move_to_z(park[Z_AXIS]);
        break;

      case 2: // Raise by Z-park height
        mechanics.do_blocking_move_to_z(
          (z + park[Z_AXIS] > Z_MAX_POS) ? Z_MAX_POS : z + park[Z_AXIS]);
        break;

      default: // Raise to Z-park height if lower
        if (mechanics.current_position[Z_AXIS] < park[Z_AXIS])
          mechanics.do_blocking_move_to_z(park[Z_AXIS]);
    }

    mechanics. do_blocking_move_to_xy(park[X_AXIS], park[Y_AXIS]);
  }

#endif // ENABLED(NOZZLE_PARK_FEATURE)
