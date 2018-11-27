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
#pragma once

/**
 * restart.h - Restart an SD print after power-loss
 */

#if HAS_SD_RESTART

  //#define DEBUG_RESTART

  typedef struct {
    uint8_t valid_head;

    // SD file e position
    char fileName[MAX_PATH_NAME_LENGHT];
    uint32_t sdpos;

    // Mechanics state
    float     current_position[XYZE];
    uint16_t  feedrate;

    #if HEATER_COUNT > 0
      int16_t target_temperature[HEATER_COUNT];
    #endif

    #if FAN_COUNT > 0
      uint8_t fan_speed[FAN_COUNT];
    #endif

    // Extruders
    #if EXTRUDERS > 1
      uint8_t active_extruder;
    #endif

    // Leveling
    #if HAS_LEVELING
      bool  leveling;
      float z_fade_height;
    #endif

    // Command buffer
    uint8_t buffer_head,
            buffer_count;
    char    buffer_ring[BUFSIZE][MAX_CMD_SIZE];

    // Job elapsed time
    millis_t print_job_counter_elapsed;

    // Utility
    bool just_restart;

    uint8_t valid_foot;

  } restart_job_t;

  class Restart {

    public: /** Constructor */

      Restart() {};

    public: /** Public Parameters */

      static SdFile file;

      static restart_job_t job_info;

      static bool enabled;

    public: /** Public Function */

      static void init_job();

      static void enable(const bool onoff);
      static void changed();

      static void check();

      static inline bool exists() { return card.exist_restart_file(); }
      static inline void open(const bool read) { card.open_restart_file(read); }
      static inline void close() { file.close(); }

      static void purge_job();
      static void load_job();
      static void save_job(const bool force_save=false, const bool save_count=true);
      static void resume_job();

      static inline bool valid() { return job_info.valid_head && job_info.valid_head == job_info.valid_foot; }

    private: /** Private Function */

      static void write_job();

      #if ENABLED(DEBUG_RESTART)
        static void debug_info(PGM_P const prefix);
      #endif

  };

  extern Restart restart;

#endif // HAS_SD_RESTART
