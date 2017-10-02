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

/**
 * commands.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "parser.h"

#ifndef _COMMANDS_H_
#define _COMMANDS_H_

class Commands {

  public: /** Constructor */

    Commands() {}

  public: /** Public Parameters */

    static char command_queue[BUFSIZE][MAX_CMD_SIZE];

    static long gcode_N,
                gcode_LastN,
                Stopped_gcode_LastN;

    static millis_t previous_cmd_ms;

  private: /** Private Parameters */

    static bool send_ok[BUFSIZE];

    static uint8_t  commands_in_queue,
                    cmd_queue_index_r,  // Ring buffer read position
                    cmd_queue_index_w;  // Ring buffer write position

    static int serial_count;

    static const char *injected_commands_P;

  public: /** Public Function */

    static void loop();

    static void flush_and_request_resend();
    static void ok_to_send();
    static void get_available_commands();
    static void clear_command_queue();

    static bool enqueue_and_echo_command(const char* cmd, bool say_ok=false);
    static void enqueue_and_echo_commands_P(const char * const pgcode);

    FORCE_INLINE static void save_last_gcode()      { Stopped_gcode_LastN = gcode_LastN; }
    FORCE_INLINE static void reset_send_ok()        { for (uint8_t i = 0; i < COUNT(send_ok); i++) send_ok[i] = true; }
    FORCE_INLINE static void refresh_cmd_timeout()  { previous_cmd_ms = millis(); }

  private: /** Private Function */

    static void get_serial_commands();
    #if HAS_SDSUPPORT
      static void get_sdcard_commands();
    #endif

    static void process_next_command();
    static void commit_command(bool say_ok);
    static void unknown_command_error();
    static void gcode_line_error(const char* err, const bool doFlush=true);

    static bool enqueue_command(const char* cmd, bool say_ok=false);
    static bool drain_injected_commands_P();

};

extern Commands commands;

#endif /* _COMMANDS_H_ */
