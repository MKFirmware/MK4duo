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
 * restart.h - Restart an SD print after power-loss
 */

#if HAS_SD_RESTART

//#define DEBUG_RESTART

#if ENABLED(COLOR_MIXING_EXTRUDER) && HAS_GRADIENT_MIX
  #include "../mixing/mixing.h"
#endif

typedef struct {

  uint8_t valid_head;

  // SD file e position
  char fileName[MAX_PATH_NAME_LENGHT];
  volatile uint32_t sdpos;

  // Mechanics state
  xyze_pos_t  axis_position_mm;

  #if ENABLED(WORKSPACE_OFFSETS)
    xyz_pos_t home_offset,
              position_shift;
  #endif

  uint16_t feedrate;

  #if HAS_HOTENDS
    int16_t target_temperature[MAX_HOTEND];
  #endif
  #if HAS_BEDS
    int16_t bed_target_temperature[MAX_BED];
  #endif
  #if HAS_CHAMBERS
    int16_t chamber_target_temperature[MAX_CHAMBER];
  #endif

  #if HAS_FAN
    uint8_t fan_speed[MAX_FAN];
  #endif

  // Extruders
  #if MAX_EXTRUDER > 1
    uint8_t active_extruder;
  #endif

  int16_t flow_percentage[MAX_EXTRUDER],
          density_percentage[MAX_EXTRUDER];

  // Leveling
  #if HAS_LEVELING
    bool  leveling;
    float z_fade_height;
  #endif

  // Color Mixing gradient
  #if ENABLED(COLOR_MIXING_EXTRUDER) && HAS_GRADIENT_MIX
    gradient_t gradient;
  #endif

  // Relative axis modes
  uint8_t axis_relative_modes;

  // Job elapsed time
  millis_l print_job_counter_elapsed;

  uint8_t valid_foot;

} restart_job_t;

class Restart {

  public: /** Constructor */

    Restart() {};

  public: /** Public Parameters */

    static SdFile job_file;

    static restart_job_t job_info;

    static bool enabled;

    static uint32_t cmd_sdpos,
                    sdpos[BUFSIZE];

  public: /** Public Function */

    static void enable(const bool onoff);
    static void changed();
    static void check();

    static void start_job();
    static void purge_job();
    static void load_job();
    static void save_job(const bool force_save=true);
    static void resume_job();

    static inline bool exists()               { return card.exist_restart_file(); }
    static inline void open(const bool read)  { card.open_restart_file(read); }
    static inline void close()                { job_file.close(); }

    static inline void factory_parameters()   { enable(true); }
    static inline bool valid()                { return job_info.valid_head && job_info.valid_head == job_info.valid_foot; }

    static inline uint32_t get_sdpos()        { return sdpos[commands.buffer_ring.head()]; }
    static inline void set_sdpos()            { sdpos[commands.buffer_ring.tail()] = cmd_sdpos; }

  private: /** Private Function */

    static void clear_job();

    static void write_job();

    #if ENABLED(DEBUG_RESTART)
      static void debug_info(PGM_P const prefix);
    #else
      static inline void debug_info(PGM_P const prefix) { UNUSED(prefix); }
    #endif

};

extern Restart restart;

#endif // HAS_SD_RESTART
