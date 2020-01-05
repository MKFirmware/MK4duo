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
 * bedlevel.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

 /**
 * Set Probe feedrate
 */
#if OLD_ABL
  #define XY_PROBE_FEEDRATE_MM_S bedlevel.xy_probe_feedrate_mm_s
#elif ENABLED(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)
#else
  #define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()
#endif

#if HAS_LEVELING

#if HAS_MESH

  typedef float bed_mesh_t[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    #include "abl/abl.h"
  #elif ENABLED(AUTO_BED_LEVELING_UBL)
    #include "ubl/ubl.h"
  #elif ENABLED(MESH_BED_LEVELING)
    #include "mbl/mesh_bed_leveling.h"
  #endif

  #define Z_VALUES(X,Y)     Z_VALUES_ARR[X][Y]
  #define _GET_MESH_POS(M)  { _GET_MESH_X(M.a), _GET_MESH_Y(M.b) }

#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)
  typedef float (*element_2d_fn)(const uint8_t, const uint8_t);
#endif

struct mesh_index_pair {
  xy_int8_t pos;
  float distance;   // When populated, the distance from the search location
  void invalidate()                   { pos = -1; }
  bool valid()                  const { return pos.x >= 0 && pos.y >= 0; }
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    xy_pos_t meshpos() {
      return { ubl.mesh_index_to_xpos(pos.x), ubl.mesh_index_to_ypos(pos.y) };
    }
  #endif
  operator        xy_int8_t&()        { return pos; }
  operator const  xy_int8_t&()  const { return pos; }
};

union level_flag_t {
  bool all;
  struct {
    bool  leveling_active   : 1;
    bool  leveling_previous : 1;
    bool  g26_debug         : 1;
    bool  g29_in_progress   : 1;
    bool  bit4              : 1;
    bool  bit5              : 1;
    bool  bit6              : 1;
    bool  bit7              : 1;
  };
  level_flag_t() { all = false; }
};

class Bedlevel {

  public: /** Constructor */

    Bedlevel() {}

  public: /** Public Parameters */

    static level_flag_t flag;

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

    static void factory_parameters();

    /**
     * Apply leveling to transform a cartesian position
     * as it will be given to the planner and steppers.
     */
    static void apply_leveling(xyz_pos_t &raw);
    static void unapply_leveling(xyz_pos_t &raw);
    FORCE_INLINE static void force_unapply_leveling(xyz_pos_t &raw) {
      flag.leveling_active = true;
      unapply_leveling(raw);
      flag.leveling_active = false;
    }

    static bool leveling_is_valid();
    static void set_bed_leveling_enabled(const bool enable=true);
    static void reset();

    FORCE_INLINE static void restore_bed_leveling_state() { set_bed_leveling_enabled(flag.leveling_previous); }

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
      static void print_2d_array(const uint8_t sx, const uint8_t sy, const uint8_t precision, element_2d_fn fn);

    #endif

    #if ENABLED(MESH_BED_LEVELING) || HAS_PROBE_MANUALLY
      /**
       * Manual goto xy for Mesh Bed level or Probe Manually
       */
      static void manual_goto_xy(const xy_pos_t &pos);
    #endif

  private:

    static inline xy_pos_t level_fulcrum() {
      #if ENABLED(Z_SAFE_HOMING)
        return { Z_SAFE_HOMING_X_POINT, Z_SAFE_HOMING_Y_POINT };
      #else
        return { X_HOME_POS, Y_HOME_POS };
      #endif
    };

};

extern Bedlevel bedlevel;

#endif // HAS_LEVELING
