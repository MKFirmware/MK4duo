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
#pragma once

/**
 * bezier.h
 *
 * Compute and buffer movement commands for bezier curves
 *
 */

#if ENABLED(G5_BEZIER)

  class Bezier {

    public: /** Public Parameters */

    public: /** Public Function */

      static void cubic_b_spline(
                    const float position[NUM_AXIS], // current position
                    const float target[NUM_AXIS],   // target position
                    const float offset[4],          // a pair of offsets
                    float fr_mm_s,
                    uint8_t extruder
                  );

    private: /** Private Parameters */

    private: /** Private Function */

      /* Compute the linear interpolation between to real numbers.
      */
      static inline float interp(float a, float b, float t) { return (1.0 - t) * a + t * b; }

      static inline float eval_bezier(float a, float b, float c, float d, float t) {
        float iab = interp(a, b, t);
        float ibc = interp(b, c, t);
        float icd = interp(c, d, t);
        float iabc = interp(iab, ibc, t);
        float ibcd = interp(ibc, icd, t);
        float iabcd = interp(iabc, ibcd, t);
        return iabcd;
      }

      /**
       * We approximate Euclidean distance with the sum of the coordinates
       * offset (so-called "norm 1"), which is quicker to compute.
       */
      static inline float dist1(float x1, float y1, float x2, float y2) { return ABS(x1 - x2) + ABS(y1 - y2); }
  };

#endif // ENABLED(G5_BEZIER)
