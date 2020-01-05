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

/**
 * babystep.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(BABYSTEPPING)

#if IS_CORE || ENABLED(BABYSTEP_XY)
  #define BS_TODO_AXIS(A) A
#else
  #define BS_TODO_AXIS(A) 0
#endif

#if HAS_LCD_MENU && ENABLED(BABYSTEP_DISPLAY_TOTAL)
  #if ENABLED(BABYSTEP_XY)
    #define BS_TOTAL_AXIS(A) A
  #else
    #define BS_TOTAL_AXIS(A) 0
  #endif
#endif

class Babystep {

  public: /** Constructor */

    Babystep() {}

  public: /** Public Parameters */

    static volatile int16_t steps[BS_TODO_AXIS(Z_AXIS) + 1];

      #if HAS_LCD_MENU
        static int16_t accum;                                   // Total babysteps in current edit
        #if ENABLED(BABYSTEP_DISPLAY_TOTAL)
          static int16_t axis_total[BS_TOTAL_AXIS(Z_AXIS) + 1]; // Total babysteps since G28
        #endif
      #endif

  public: /** Public Function */

    #if HAS_LCD_MENU && ENABLED(BABYSTEP_DISPLAY_TOTAL)
      static inline void reset_total(const AxisEnum axis) {
        #if ENABLED(BABYSTEP_XY)
          if (axis == Z_AXIS)
        #endif
            axis_total[BS_TOTAL_AXIS(axis)] = 0;
      }
    #endif

    static void add_steps(const AxisEnum axis, const int16_t distance);
    static void add_mm(const AxisEnum axis, const float &mm);
    static void spin();

  private: /** Private Function */

    static void step_axis(const AxisEnum axis);

};

extern Babystep babystep;

#endif // ENABLED(BABYSTEPPING)
