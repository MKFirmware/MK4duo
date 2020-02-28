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
  #define BS_AXIS_IND(A)  A
  #define BS_AXIS(I)      AxisEnum(I)
#else
  #define BS_AXIS_IND(A)  0
  #define BS_AXIS(I)      Z_AXIS
#endif

#if HAS_LCD_MENU && ENABLED(BABYSTEP_DISPLAY_TOTAL)
  #if ENABLED(BABYSTEP_XY)
    #define BS_TOTAL_IND(A) A
  #else
    #define BS_TOTAL_IND(A) 0
  #endif
#endif

class Babystep {

  public: /** Constructor */

    Babystep() {}

  public: /** Public Parameters */

    static volatile int16_t steps[BS_AXIS_IND(Z_AXIS) + 1];

    static int16_t accum;                                       // Total babysteps in current edit

    #if HAS_LCD_MENU && ENABLED(BABYSTEP_DISPLAY_TOTAL)
      static int16_t axis_total[BS_TOTAL_IND(Z_AXIS) + 1]; // Total babysteps since G28
    #endif

  public: /** Public Function */

    #if HAS_LCD_MENU && ENABLED(BABYSTEP_DISPLAY_TOTAL)
      static inline void reset_total(const AxisEnum axis) {
        #if ENABLED(BABYSTEP_XY)
          if (axis == Z_AXIS)
        #endif
            axis_total[BS_TOTAL_IND(axis)] = 0;
      }
    #endif

    static void add_mm(const AxisEnum axis, const float &mm);
    static void add_steps(const AxisEnum axis, const int16_t distance);

    static inline bool has_steps() {
      return steps[BS_AXIS_IND(X_AXIS)] || steps[BS_AXIS_IND(Y_AXIS)] || steps[BS_AXIS_IND(Z_AXIS)];
    }

    static inline void spin() {
      #if ENABLED(BABYSTEP_XY)
        LOOP_XYZ(axis) step_axis((AxisEnum)axis);
      #else
        step_axis(Z_AXIS);
      #endif
    }

  private: /** Private Function */

    static void step_axis(const AxisEnum axis);

};

extern Babystep babystep;

#endif // ENABLED(BABYSTEPPING)
