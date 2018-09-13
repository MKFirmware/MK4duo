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

    static uint8_t  buffer_lenght,  // Number of commands in the Buffer Ring
                    buffer_index_r, // Read position in Buffer Ring
                    buffer_index_w; // Write position in Buffer Ring

    static char buffer_ring[BUFSIZE][MAX_CMD_SIZE];

    static long gcode_LastN;

  private: /** Private Parameters */

    static long gcode_N;

    static bool send_ok[BUFSIZE];

    static int serial_count;

    static const char *injected_commands_P;

    static watch_t last_command_watch;

  public: /** Public Function */

    static void flush_and_request_resend();
    static void ok_to_send();
    static void get_available();
    static void advance_queue();
    static void clear_queue();

    static bool enqueue_and_echo(const char* cmd);
    static void enqueue_and_echo_P(const char * const pgcode);
    static void enqueue_and_echo_now(const char* cmd);
    static void enqueue_and_echo_now_P(const char * const cmd);

    static void get_destination();
    static bool get_target_tool(const uint16_t code);
    static bool get_target_heater(int8_t &h, const bool only_hotend=false);

    FORCE_INLINE static void setup() { for (uint8_t i = 0; i < COUNT(send_ok); i++) send_ok[i] = true; }

  private: /** Private Function */

    static void get_serial();
    #if HAS_SD_SUPPORT
      static void get_sdcard();
    #endif

    static void process_next();
    static void process_parsed();
    static void commit(bool say_ok);
    static void unknown_error();
    static void gcode_line_error(const char* err);

    static bool enqueue(const char* cmd, bool say_ok=false);
    static bool drain_injected_P();

    #if HAS_SD_RESTART
      static bool enqueue_restart();
    #endif
};

extern Commands commands;

#endif /* _COMMANDS_H_ */
