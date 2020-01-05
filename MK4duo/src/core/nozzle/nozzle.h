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
#pragma once

// Struct Nozzle data
struct nozzle_data_t {
  xyz_pos_t hotend_offset[MAX_HOTEND];
  #if ENABLED(NOZZLE_PARK_FEATURE) || MAX_EXTRUDER > 1
    xyz_pos_t park_point;
  #endif
};

class Nozzle {

  public: /** Constructor */

    Nozzle() {};

  public: /** Public Parameters */

    static nozzle_data_t data;

  public: /** Public Function */

    static void factory_parameters();

    #if HAS_LCD
      static void set_heating_message();
    #endif

    #if ENABLED(NOZZLE_PARK_FEATURE) || MAX_EXTRUDER > 1
      static void print_M217();
    #endif

    #if MAX_HOTEND > 1
      static void print_M218();
    #endif

    #if ENABLED(NOZZLE_CLEAN_FEATURE)
      /**
       * @brief Clean the nozzle
       * @details Starts the selected clean procedure pattern
       *
       * @param pattern one of the available patterns
       * @param argument depends on the cleaning pattern
       */
      static void clean(const uint8_t &pattern, const uint8_t &strokes, const float &radius, const uint8_t &objects, const uint8_t cleans);
    #endif

    #if ENABLED(NOZZLE_PARK_FEATURE)
      static void park(const uint8_t z_action, const xyz_pos_t &park_p=data.park_point);
    #endif

  private: /** Private Function */

    #if ENABLED(NOZZLE_CLEAN_FEATURE)

      /**
       * @brief Stroke clean pattern
       * @details Wipes the nozzle back and forth in a linear movement
       *
       * @param start defining the starting point
       * @param end defining the ending point
       * @param strokes number of strokes to execute
       */
      static void stroke(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t &strokes) _Os;

      /**
       * @brief Zig-zag clean pattern
       * @details Apply a zig-zag cleanning pattern
       *
       * @param start defining the starting point
       * @param end defining the ending point
       * @param strokes number of strokes to execute
       * @param objects number of objects to create
       */
      static void zigzag(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t &strokes, const uint8_t &objects) _Os;

      /**
       * @brief Circular clean pattern
       * @details Apply a circular cleaning pattern
       *
       * @param start defining the starting point
       * @param middle defining the middle of circle
       * @param strokes number of strokes to execute
       * @param radius of circle
       */
      static void circle(const xyz_pos_t &start, const xyz_pos_t &middle, const uint8_t &strokes, const float &radius) _Os;

    #endif

};

extern Nozzle nozzle;
