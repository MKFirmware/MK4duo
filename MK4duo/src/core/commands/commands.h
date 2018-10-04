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

struct gcode_t {
  char    gcode[MAX_CMD_SIZE];  // Char for gcode
  int8_t  s_port = -1;          // Serial port for print information:
                                //    -1 for all port
                                //    -2 for SD or null port
};

class Commands {

  public: /** Constructor */

    Commands() {}

  public: /** Public Parameters */

    static Circular_Queue<gcode_t, BUFSIZE> buffer_ring;

    static long gcode_LastN;

  private: /** Private Parameters */

    static long gcode_N;

    static int serial_count[NUM_SERIAL];

    static PGM_P injected_commands_P;

    static watch_t last_command_watch;

  public: /** Public Function */

    static void flush_and_request_resend();
    static void ok_to_send();
    static void get_available();
    static void advance_queue();
    static void clear_queue();

    static bool enqueue_and_echo(PGM_P cmd);
    static void enqueue_and_echo_P(PGM_P const pgcode);
    static void enqueue_and_echo_now(PGM_P cmd);
    static void enqueue_and_echo_now_P(PGM_P const cmd);

    static void get_destination();
    static bool get_target_tool(const uint16_t code);
    static bool get_target_heater(int8_t &h, const bool only_hotend=false);

  private: /** Private Function */

    static void get_serial();
    #if HAS_SD_SUPPORT
      static void get_sdcard();
    #endif

    static void process_next();
    static void process_parsed();
    static void unknown_error();
    static void gcode_line_error(PGM_P err, const int8_t tmp_port);

    static bool enqueue(PGM_P cmd, int8_t port=-2);
    static bool drain_injected_P();

    #if HAS_SD_RESTART
      static bool enqueue_restart();
    #endif
};

extern Commands commands;

#endif /* _COMMANDS_H_ */
