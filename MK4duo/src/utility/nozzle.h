/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

#ifndef __NOZZLE_H__
#define __NOZZLE_H__

#include "../../base.h"

#if ENABLED(NOZZLE_CLEAN_FEATURE)
  constexpr float nozzle_clean_start_point[XYZ] = NOZZLE_CLEAN_START_POINT,
                  nozzle_clean_end_point[XYZ]   = NOZZLE_CLEAN_END_POINT,
                  nozzle_clean_length = FABS(nozzle_clean_start_point[X_AXIS] - nozzle_clean_end_point[X_AXIS]), //abs x size of wipe pad
                  nozzle_clean_height = FABS(nozzle_clean_start_point[Y_AXIS] - nozzle_clean_end_point[Y_AXIS]); //abs y size of wipe pad
  constexpr bool nozzle_clean_horizontal = nozzle_clean_length >= nozzle_clean_height; //whether to zig-zag horizontally or vertically
#endif //NOZZLE_CLEAN_FEATURE

/**
 * @brief Nozzle class
 *
 * @todo: Do not ignore the end.z value and allow XYZ movements
 */
class Nozzle {

  private:

    /**
     * @brief Stroke clean pattern
     * @details Wipes the nozzle back and forth in a linear movement
     *
     * @param start defining the starting point
     * @param end defining the ending point
     * @param strokes number of strokes to execute
     */
    static void stroke(const float *start, const float *end, const uint8_t &strokes);

    /**
     * @brief Zig-zag clean pattern
     * @details Apply a zig-zag cleanning pattern
     *
     * @param start defining the starting point
     * @param end defining the ending point
     * @param strokes number of strokes to execute
     * @param objects number of objects to create
     */
    static void zigzag(const float *start, const float *end, const uint8_t &strokes, const uint8_t &objects);

    /**
     * @brief Circular clean pattern
     * @details Apply a circular cleaning pattern
     *
     * @param start defining the starting point
     * @param middle defining the middle of circle
     * @param strokes number of strokes to execute
     * @param radius of circle
     */
    static void circle(const float *start, const float *middle, const uint8_t &strokes, const float &radius);

  public:
    /**
     * @brief Clean the nozzle
     * @details Starts the selected clean procedure pattern
     *
     * @param pattern one of the available patterns
     * @param argument depends on the cleaning pattern
     */
    static void clean(const uint8_t &pattern, const uint8_t &strokes, const float &radius, const uint8_t &objects = 0);

    static void park(const uint8_t &z_action);

};

#endif /* __NOZZLE_H__ */
