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
 * extruder.h - extruder object
 */
class Driver;
class Heater;

// Struct Extruder data
struct extruder_data_t {
  uint8_t   driver,               // Pointer to Driver
            hotend;               // Pointer to Hotend
  float     axis_steps_per_mm,
            max_feedrate_mm_s,
            retract_acceleration,
            max_jerk;
  uint32_t  max_acceleration_mm_per_s2;
  #if ENABLED(LIN_ADVANCE)
    float   advance_K;
  #endif
  #if ENABLED(VOLUMETRIC_EXTRUSION)
    float   filament_size;  // Diameter of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the toolManager.
  #endif
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    float   load_length,
            unload_length;
  #endif
  #if ENABLED(TOOL_CHANGE_FIL_SWAP)
    float   swap_length,
            purge_lenght;
    int16_t prime_speed,
            retract_speed;
  #endif
};

class Extruder {

  public: /** Constructor */

    Extruder() {}

  public: /** Public Parameters */

    extruder_data_t data;

    float     steps_to_mm,
              e_factor;

    uint32_t  max_acceleration_steps_per_s2;

    int16_t   flow_percentage,        // Extrusion factor for each extruder
              density_percentage,     // Extrusion density factor for each extruder
              singlenozzle_temp;      // Single nozzle temp for each extuder

    #if ENABLED(VOLUMETRIC_EXTRUSION)
      float   volumetric_multiplier;
    #endif

    #if ENABLED(DISABLE_INACTIVE_EXTRUDER)
      uint8_t g_uc_extruder_last_move;
    #endif

  private: /** Private Parameters */

    #if ENABLED(IDLE_OOZING_PREVENT)
      static bool IDLE_OOZING_retracted;
    #endif

  public: /** Public Function */

    void refresh_e_factor();

    inline uint8_t get_driver() { return data.driver; }
    inline uint8_t get_hotend() { return data.hotend; }

    #if ENABLED(VOLUMETRIC_EXTRUSION)
      FORCE_INLINE void set_filament_size(const float &v) { data.filament_size = v; }
    #endif

    #if ENABLED(TOOL_CHANGE_FIL_SWAP)
      void print_M217(const uint8_t e);
    #endif

};

#if MAX_EXTRUDER > 0
  extern Extruder* extruders[MAX_EXTRUDER];
#endif
