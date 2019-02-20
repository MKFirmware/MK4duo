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
 * bedlevel.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if OLD_ABL
  #define XY_PROBE_FEEDRATE_MM_S bedlevel.xy_probe_feedrate_mm_s
#elif ENABLED(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)
#else
  #define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #define _GET_MESH_X(I) abl.bilinear_start[X_AXIS] + I * abl.bilinear_grid_spacing[X_AXIS]
  #define _GET_MESH_Y(J) abl.bilinear_start[Y_AXIS] + J * abl.bilinear_grid_spacing[Y_AXIS]
#elif ENABLED(AUTO_BED_LEVELING_UBL)
  #define _GET_MESH_X(I) ubl.mesh_index_to_xpos(I)
  #define _GET_MESH_Y(J) ubl.mesh_index_to_ypos(J)
#elif ENABLED(MESH_BED_LEVELING)
  #define _GET_MESH_X(I) mbl.index_to_xpos[I]
  #define _GET_MESH_Y(J) mbl.index_to_ypos[J]
#endif

union flaglevel_t {
  bool all;
  struct {
    bool  leveling_active : 1;
    bool  g26_debug       : 1;
    bool  g29_in_progress : 1;
    bool  bit3            : 1;
    bool  bit4            : 1;
    bool  bit5            : 1;
    bool  bit6            : 1;
    bool  bit7            : 1;
  };
  flaglevel_t() { all = 0; }
};

#if HAS_LEVELING

  typedef struct {
    int8_t x_index, y_index;
    float distance;
  } mesh_index_pair;

  #if ABL_PLANAR || ENABLED(AUTO_BED_LEVELING_UBL)
    #include "math/vector_3.h"
    #include "math/least_squares_fit.h"
  #endif
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    #include "abl/abl.h"
  #elif ENABLED(MESH_BED_LEVELING)
    #include "mbl/mesh_bed_leveling.h"
  #elif ENABLED(AUTO_BED_LEVELING_UBL)
    #include "ubl/ubl.h"
  #endif

  class Bedlevel {

    public: /** Constructor */

      Bedlevel() {}

    public: /** Public Parameters */

      static flaglevel_t flag;

       #if OLD_ABL
        static int xy_probe_feedrate_mm_s;
      #endif
      #if ABL_PLANAR
        static matrix_3x3 matrix; // Transform to compensate for bed level
      #endif

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        static float z_fade_height, inverse_z_fade_height;
      #endif

    private: /** Private Parameters */

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        static float last_fade_z;
      #endif

    public: /** Public Function */

      /**
       * Apply leveling to transform a cartesian position
       * as it will be given to the planner and steppers.
       */
      static void apply_leveling(float &rx, float &ry, float &rz);
      FORCE_INLINE static void apply_leveling(float (&raw)[XYZ])  { apply_leveling(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS]); }
      FORCE_INLINE static void apply_leveling(float (&raw)[XYZE]) { apply_leveling(raw[X_AXIS], raw[Y_AXIS], raw[Z_AXIS]); }

      static void unapply_leveling(float raw[XYZ]);

      static bool leveling_is_valid();
      static void set_bed_leveling_enabled(const bool enable=true);
      static void reset();

      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

        static void set_z_fade_height(const float zfh, const bool do_report=true);

        /**
         * Get the Z leveling fade factor based on the given Z height,
         * re-calculating only when needed.
         *
         *  Returns 1.0 if planner.z_fade_height is 0.0.
         *  Returns 0.0 if Z is past the specified 'Fade Height'.
         */
        static inline float fade_scaling_factor_for_z(const float &rz) {
          static float z_fade_factor = 1.0;
          if (z_fade_height) {
            if (rz >= z_fade_height) return 0.0;
            if (last_fade_z != rz) {
              last_fade_z = rz;
              z_fade_factor = 1.0 - rz * inverse_z_fade_height;
            }
            return z_fade_factor;
          }
          return 1.0;
        }

        FORCE_INLINE static void force_fade_recalc() { last_fade_z = -999.999; }

        FORCE_INLINE static bool leveling_active_at_z(const float &rz) {
          return !z_fade_height || rz < z_fade_height;
        }

      #else

        FORCE_INLINE static float fade_scaling_factor_for_z(const float &rz) {
          UNUSED(rz);
          return 1.0;
        }

        FORCE_INLINE static bool leveling_active_at_z(const float &rz) { UNUSED(rz); return true; }

      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

        /**
         * Print calibration results for plotting or manual frame adjustment.
         */
        static void print_2d_array(const uint8_t sx, const uint8_t sy, const uint8_t precision, float (*fn)(const uint8_t, const uint8_t));

      #endif

      #if ENABLED(MESH_BED_LEVELING) || ENABLED(PROBE_MANUALLY)
        /**
         * Manual goto xy for Mesh Bed level or Probe Manually
         */
        void manual_goto_xy(const float &rx, const float &ry);
      #endif

  };

  extern Bedlevel bedlevel;

#endif // HAS_LEVELING
