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

#ifndef _RESTART_H_
#define _RESTART_H_

#if HAS_SD_RESTART

  //#define DEBUG_RESTART

  #define APPEND_CMD_COUNT 3

  typedef struct {
    uint8_t valid_head;

    // SD file e position
    char fileName[LONG_FILENAME_LENGTH];
    uint32_t sdpos;

    // Mechanics state
    float   current_position[XYZE];
    int16_t target_temperature[HEATER_COUNT];
    uint8_t fan_speed[FAN_COUNT];

    // Extruders
    #if EXTRUDERS > 1
      uint8_t active_extruder;
    #endif

    // Leveling
    #if HAS_LEVELING
      bool leveling;
      float z_fade_height;
    #endif

    // Command buffer
    uint8_t buffer_index_r,
            buffer_lenght;
    char buffer_ring[BUFSIZE][MAX_CMD_SIZE];

    // Job elapsed time
    millis_t print_job_counter_elapsed;

    // Utility
    bool auto_restart;
    bool just_restart;

    uint8_t valid_foot;

  } restart_job_t;

  enum restart_phase : unsigned char {
    RESTART_IDLE,
    RESTART_MAYBE,
    RESTART_YES,
    RESTART_DONE
  };

  class Restart {

    public: /** Constructor */

      Restart() {};

    public: /** Public Parameters */

      static restart_job_t  job_info;
      static restart_phase  job_phase;

      static char buffer_ring[BUFSIZE + APPEND_CMD_COUNT][MAX_CMD_SIZE];
      static uint8_t count;

    private: /** Private Parameters */

    public: /** Public Function */

      static void do_print_job();
      static void start_job();
      static void save_data(const bool force_save=false);

    private: /** Private Function */

      #if ENABLED(DEBUG_RESTART)
        static void debug_info(const bool restart);
      #endif

  };

  extern Restart restart;

#endif // HAS_SD_RESTART

#endif /* _RESTART_H_ */
