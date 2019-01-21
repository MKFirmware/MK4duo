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
 * commands.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "parser.h"

struct gcode_t {
  char    gcode[MAX_CMD_SIZE];  // Char for gcode
  bool    send_ok = true;       // Send "ok" after commands by default
  int8_t  s_port  = -1;         // Serial port for print information:
                                //    -1 for all port
                                //    -2 for SD or null port
};

class Commands {

  public: /** Constructor */

    Commands() {}

  public: /** Public Parameters */

    /**
     * GCode Command Buffer Ring
     * A simple ring buffer of BUFSIZE command strings.
     *
     * Commands are copied into this buffer by the command injectors
     * (immediate, serial, sd card) and they are processed sequentially by
     * the main loop. The process_next function parses the next
     * command and hands off execution to individual handler functions.
     */
    static Circular_Queue<gcode_t, BUFSIZE> buffer_ring;

    /**
     * GCode line number handling. Hosts may opt to include line numbers when
     * sending commands to MK4duo, and lines will be checked for sequentiality.
     * M110 N<int> sets the current line number.
     */
    static long gcode_LastN;

  private: /** Private Parameters */

    static long gcode_N;

    static int serial_count[NUM_SERIAL];

    /**
     * Next Injected Command pointer. NULL if no commands are being injected.
     * Used by MK4duo internally to ensure that commands initiated from within
     * are enqueued ahead of any pending serial or sd card
     */
    static PGM_P injected_commands_P;

    static watch_t last_command_watch;

  public: /** Public Function */

    /**
     * Send a "Resend: nnn" message to the host to
     * indicate that a command needs to be re-sent.
     */
    static void flush_and_request_resend();

    /**
     * Add to the buffer ring the next command from:
     *  - The command-injection queue (injected_commands_P)
     *  - The active serial input (usually USB)
     *  - The SD card file being actively printed
     */
    static void get_available();

    /**
     * Get the next command in the buffer_ring, optionally log it to SD, then dispatch it
     */
    static void advance_queue();

    /**
     * Clear the MK4duo command buffer_ring
     */
    static void clear_queue();

    /**
     * Record one or many commands to run from program memory.
     * Aborts the current queue, if any.
     * Note: drain_injected_P() must be called repeatedly to drain the commands afterwards
     */
    static void enqueue_and_echo_P(PGM_P const pgcode);

    /**
     * Enqueue with Serial Echo
     */
    static bool enqueue_and_echo(const char * cmd);

    /**
     * Enqueue from program memory and return only when commands are actually enqueued
     */
    static void enqueue_and_echo_now_P(PGM_P const cmd);

    /**
     * Enqueue and return only when commands are actually enqueued
     */
    static void enqueue_and_echo_now(const char * cmd);

    /**
     * Run a series of commands, bypassing the command queue to allow
     * G-code "macros" to be called from within other G-code handlers.
     */
    static void process_now_P(PGM_P pgcode);
    static void process_now(char * gcode);

    /**
     * Set XYZE mechanics.destination and mechanics.feedrate_mm_s from the current GCode command
     *
     *  - Set mechanics.destination from included axis codes
     *  - Set to current for missing axis codes
     *  - Set the mechanics.feedrate_mm_s, if included
     */
    static void get_destination();

    /**
     * Set target tool from the T parameter or the active_tool
     *
     * Returns TRUE if the target is invalid
     */
    static bool get_target_tool(const uint16_t code);

    /**
     * Set target heather from the H parameter
     *
     * Returns TRUE if the target is invalid
     */
    static bool get_target_heater(int8_t &h, const bool only_hotend=false);

    #if ENABLED(COLOR_MIXING_EXTRUDER)
      /**
       * Set target driver from the T parameter or the active_driver
       *
       * Returns TRUE if the target is invalid
       */
      static bool get_target_driver(const uint16_t code);
    #endif

    #if FAN_COUNT > 0
      /**
       * Set target fan from the P parameter
       *
       * Returns TRUE if the target is invalid
       */
      static bool get_target_fan(uint8_t &f);
    #endif

  private: /** Private Function */

    /**
     * Send an "ok" message to the host, indicating
     * that a command was successfully processed.
     *
     * If ADVANCED_OK is enabled also include:
     *   N<int>  Line number of the command, if any
     *   P<int>  Planner space remaining
     *   B<int>  Block queue space remaining
     */
    static void ok_to_send();

    /**
     * Get all commands waiting on the serial port and queue them.
     * Exit when the buffer is full or when no more characters are
     * left on the serial port.
     */
    static void get_serial();

    /**
     * Get commands from the SD Card until the command buffer is full
     * or until the end of the file is reached. The special character '#'
     * can also interrupt buffering.
     */
    #if HAS_SD_SUPPORT
      static void get_sdcard();
    #endif

    /**
     * Process a single command and dispatch it to its handler
     * This is called from the main loop()
     */
    static void process_next();

    static void unknown_error();

    static void gcode_line_error(PGM_P err, const int8_t tmp_port);

    /**
     * Copy a command from RAM into the main command buffer.
     * Return true if the command was successfully added.
     * Return false for a full buffer, or if the 'command' is a comment.
     */
    static bool enqueue(const char * cmd, bool say_ok=false, int8_t port=-2);

    /**
     * Inject the next "immediate" command, when possible, onto the front of the buffer_ring.
     * Return true if any immediate commands remain to inject.
     */
    static bool drain_injected_P();

    /**
     * Process parsed gcode and execute command
     */
    static void process_parsed(const bool say_ok=true);

};

extern Commands commands;
