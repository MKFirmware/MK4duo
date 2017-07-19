/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

/**
 * commands.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "parser.h"

#ifndef _COMMANDS_H_
#define _COMMANDS_H_

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
  #if ENABLED(M100_FREE_MEMORY_DUMPER)
    void M100_dump_routine(const char * const title, const char *start, const char *end);
  #endif
#endif

class Commands {

  public: /** Constructor */

    Commands() {};

  public: /** Public Parameters */

    static long gcode_N,
                gcode_LastN,
                Stopped_gcode_LastN;

    static bool send_ok[BUFSIZE];

    static uint8_t mk_debug_flags;

    static char command_queue[BUFSIZE][MAX_CMD_SIZE];

    static millis_t previous_cmd_ms;

  public: /** Public Function */

    static void command_loop();

    static void get_serial_commands();
    #if HAS_SDSUPPORT
      static void get_sdcard_commands();
    #endif

    static void process_next_gcode();
    static void unknown_command_error();

    static void FlushSerialRequestResend();
    static void ok_to_send();
    static void get_available_commands();
    static void clear_command_queue();
    static void commit_command(bool say_ok);

    static void enqueue_and_echo_commands_P(const char * const pgcode);
    static bool drain_injected_commands_P();
    static bool enqueuecommand(const char* cmd, bool say_ok=false);
    static bool enqueue_and_echo_command(const char* cmd, bool say_ok=false);

    FORCE_INLINE static void save_last_gcode() { Stopped_gcode_LastN = gcode_LastN; }
    FORCE_INLINE static char* get_command_queue()   { return command_queue[cmd_queue_index_r]; }
    FORCE_INLINE static void reset_send_ok()        { for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true; }
    FORCE_INLINE static void refresh_cmd_timeout()  { previous_cmd_ms = millis(); }

  private: /** Private Parameters */

    static uint8_t  commands_in_queue,
                    cmd_queue_index_r,  // Ring buffer read position
                    cmd_queue_index_w;  // Ring buffer write position

    static int serial_count;

    static const char *injected_commands_P;

  private: /** Private Function */

    static void gcode_line_error(const char* err, const bool doFlush=true);

};

extern Commands commands;

#endif /* _COMMANDS_H_ */
