/**
 * MK4duo 3D Commands Firmware
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
 * commands.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "gcode/gcode.h"

Commands commands;

/**
 * Public Parameters
 */

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
long  Commands::gcode_N             = 0,
      Commands::gcode_LastN         = 0;

bool Commands::send_ok[BUFSIZE];

char Commands::queue[BUFSIZE][MAX_CMD_SIZE];

// Inactivity shutdown
millis_t Commands::previous_cmd_ms = 0;

/**
 * Private Parameters
 */

/**
 * GCode Command Queue
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The process_next function parses the next
 * command and hands off execution to individual handler functions.
 */
uint8_t Commands::queue_count   = 0,  // Count of commands in the queue
        Commands::queue_index_r = 0,  // Ring buffer read position
        Commands::queue_index_w = 0;  // Ring buffer write position

int Commands::serial_count = 0;

/**
 * Next Injected Command pointer. NULL if no commands are being injected.
 * Used by MK4duo internally to ensure that commands initiated from within
 * are enqueued ahead of any pending serial or sd card
 */
const char *Commands::injected_commands_P = NULL;

/**
 * Public Function
 */

/**
 * Get all commands waiting on the serial port and queue them.
 * Exit when the buffer is full or when no more characters are
 * left on the serial port.
 */
void Commands::get_serial() {

  static char serial_line_buffer[MAX_CMD_SIZE];
  static bool serial_comment_mode = false;

  #if HAS_DOOR
    if (READ(DOOR_OPEN_PIN) != endstops.Is_logic(DOOR_OPEN)) {
      KEEPALIVE_STATE(DOOR_OPEN);
      return;  // do nothing while door is open
    }
  #endif

  // If the command buffer is empty for too long,
  // send "wait" to indicate MK4duo is still waiting.
  #if ENABLED(NO_TIMEOUTS) && NO_TIMEOUTS > 0
    static millis_t last_command_time = 0;
    millis_t ms = millis();
    if (queue_count == 0 && !MKSERIAL.available() && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
      SERIAL_STR(WT);
      SERIAL_EOL();
      last_command_time = ms;
    }
  #endif

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  int c;
  while (queue_count < BUFSIZE && (c = MKSERIAL.read()) >= 0) {

    char serial_char = c;

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false;                      // end of line == end of comment

      if (!serial_count) continue;                      // Skip empty lines

      serial_line_buffer[serial_count] = 0;             // Terminate string
      serial_count = 0;                                 // Reset buffer

      char *command = serial_line_buffer;

      while (*command == ' ') command++;                // Skip leading spaces

      char *npos = (*command == 'N') ? command : NULL;  // Require the N parameter to start the line
      if (npos) {

        bool M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char *n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        char *apos = strrchr(command, '*');
        if (apos) {
          uint8_t checksum = 0, count = uint8_t(apos - command);
          while (count) checksum ^= command[--count];
          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
      }

      // Movement commands alert when stopped
      if (!printer.IsRunning()) {
        char *gpos = strchr(command, 'G');
        if (gpos) {
          const int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_LM(ER, MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      #if DISABLED(EMERGENCY_PARSER)
        // If command was e-stop process now
        if (strcmp(command, "M108") == 0) {
          printer.setWaitForHeatUp(false);
          #if ENABLED(ULTIPANEL)
            printer.setWaitForUser(false);
          #endif
        }
        if (strcmp(command, "M112") == 0) printer.kill(PSTR(MSG_KILLED));
        if (strcmp(command, "M410") == 0) stepper.quickstop_stepper();
      #endif

      #if ENABLED(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      enqueue(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') { // Handle escapes
      // if we have one more character, copy it over
      if ((c = MKSERIAL.read()) >= 0 && !serial_comment_mode)
        serial_line_buffer[serial_count++] = serial_char;
    }
    else { // its not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }
  } // queue has space, serial has data
}

#if HAS_SDSUPPORT

  /**
   * Get commands from the SD Card until the command buffer is full
   * or until the end of the file is reached. The special character '#'
   * can also interrupt buffering.
   */
  void Commands::get_sdcard() {
    static bool stop_buffering = false,
                sd_comment_mode = false;

    if (!IS_SD_PRINTING) return;

    #if HAS_DOOR
      if (READ(DOOR_OPEN_PIN) != endstops.Is_logic(DOOR_OPEN)) {
        KEEPALIVE_STATE(DOOR_OPEN);
        return;  // do nothing while door is open
      }
    #endif

    #if HAS_POWER_CHECK
      if (READ(POWER_CHECK_PIN) != endstops.Is_logic(POWEER_CHECK)) {
        card.stopSDPrint();
        return;
      }
    #endif

    /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

    if (queue_count == 0) stop_buffering = false;

    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (queue_count < BUFSIZE && !card_eof && !stop_buffering) {
      const int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n'  || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {
          SERIAL_EM(MSG_FILE_PRINTED);
          card.printingHasFinished();

          if (card.sdprinting)
            sd_count = 0; // If a sub-file was printing, continue from call point
          else {
            #if ENABLED(PRINTER_EVENT_LEDS)
              LCD_MESSAGEPGM(MSG_INFO_COMPLETED_PRINTS);
              leds.set_green();
              #if HAS_RESUME_CONTINUE
                enqueue_and_echo_P(PSTR("M0")); // end of the queue!
              #else
                printer.safe_delay(1000);
              #endif
              leds.set_off();
            #endif
            card.checkautostart(true);
          }
        }
        else if (n == -1) {
          SERIAL_LM(ER, MSG_SD_ERR_READ);
        }
        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; // for new command

        if (!sd_count) continue; // skip empty lines (and comment lines)

        queue[queue_index_w][sd_count] = '\0'; // terminate string
        planner.add_block_length(sd_count);

        sd_count = 0; // clear sd line buffer

        commit(false);
      }
      else if (sd_count >= MAX_CMD_SIZE - 1) {
        /**
         * Keep fetching, but ignore normal characters beyond the max length
         * The command will be injected when EOL is reached
         */
      }
      else {
        if (sd_char == ';') sd_comment_mode = true;
        if (!sd_comment_mode) queue[queue_index_w][sd_count++] = sd_char;
      }
    }

    printer.progress = card.percentDone();
  }

#endif // SDSUPPORT

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void Commands::flush_and_request_resend() {
  //char queue[queue_index_r][100]="Resend:";
  HAL::serialFlush();
  SERIAL_LV(RESEND, gcode_LastN + 1);
  ok_to_send();
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void Commands::ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[queue_index_r]) return;
  SERIAL_STR(OK);
  #if ENABLED(ADVANCED_OK)
    char* p = queue[queue_index_r];
    if (*p == 'N') {
      SERIAL_CHR(' ');
      SERIAL_CHR(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_CHR(*p++);
    }
    SERIAL_MV(" P", (int)(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_MV(" B", BUFSIZE - queue_count);
  #endif
  SERIAL_EOL();
}

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void Commands::get_available() {

  if (queue_count >= BUFSIZE) return;

  // if any immediate commands remain, don't get other commands yet
  if (drain_injected_P()) return;

  get_serial();

  #if HAS_SDSUPPORT
    get_sdcard();
  #endif
}

/**
 * Get the next command in the queue, optionally log it to SD, then dispatch it
 */
void Commands::advance_queue() {

  if (!queue_count) return;

  #if HAS_SDSUPPORT

    if (card.saving) {
      char* command = queue[queue_index_r];
      if (strstr_P(command, PSTR("M29"))) {
        // M29 closes the file
        card.finishWrite();

        #if ENABLED(SERIAL_STATS_DROPPED_RX)
          SERIAL_EMV("Dropped bytes: ", MKSERIAL.dropped());
        #endif

        #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
          SERIAL_EMV("Max RX Queue Size: ", MKSERIAL.rxMaxEnqueued());
        #endif

        ok_to_send();
      }
      else {
        // Write the string from the read buffer to SD
        card.write_command(command);
        ok_to_send();
      }
    }
    else
      process_next();

  #else // !HAS_SDSUPPORT

    process_next();

  #endif // !HAS_SDSUPPORT

  // The queue may be reset by a command handler or by code invoked by idle() within a handler
  if (queue_count) {
    --queue_count;
    if (++queue_index_r >= BUFSIZE) queue_index_r = 0;
  }
}

/**
 * Enqueue with Serial Echo
 */
bool Commands::enqueue_and_echo(const char* cmd, bool say_ok/*=false*/) {
  if (enqueue(cmd, say_ok)) {
    SERIAL_SMT(ECHO, MSG_ENQUEUEING, cmd);
    SERIAL_CHR('"');
    SERIAL_EOL();
    return true;
  }
  return false;
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_injected_P() must be called repeatedly to drain the commands afterwards
 */
void Commands::enqueue_and_echo_P(const char * const pgcode) {
  injected_commands_P = pgcode;
  (void)drain_injected_P(); // first command executed asap (when possible)
}

/**
 * Enqueue and return only when commands are actually enqueued
 */
void Commands::enqueue_and_echo_now(const char* cmd, bool say_ok/*=false*/) {
  while (!enqueue_and_echo(cmd, say_ok)) printer.idle();
}

/**
 * Enqueue from program memory and return only when commands are actually enqueued
 */
void Commands::enqueue_and_echo_P_now(const char * const pgcode) {
  enqueue_and_echo_P(pgcode);
  while (drain_injected_P()) printer.idle();
}

/**
 * Clear the MK4duo command queue
 */
void Commands::clear_queue() {
  queue_index_r = queue_index_w = 0;
  queue_count = 0;
  ZERO(queue[queue_index_r]);
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
void Commands::commit(bool say_ok) {
  send_ok[queue_index_w] = say_ok;
  if (++queue_index_w >= BUFSIZE) queue_index_w = 0;
  queue_count++;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
bool Commands::enqueue(const char* cmd, bool say_ok/*=false*/) {
  if (*cmd == ';' || queue_count >= BUFSIZE) return false;
  strcpy(queue[queue_index_w], cmd);
  commit(say_ok);
  return true;
}

/**
 * Inject the next "immediate" command, when possible, onto the front of the queue.
 * Return true if any immediate commands remain to inject.
 */
bool Commands::drain_injected_P() {
  if (injected_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, injected_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
    cmd[i] = '\0';
    if (enqueue_and_echo(cmd))     // success?
      injected_commands_P = c ? injected_commands_P + i + 1 : NULL; // next command or done
  }
  return (injected_commands_P != NULL);    // return whether any more remain
}

/**
 * Set XYZE mechanics.destination and mechanics.feedrate_mm_s from the current GCode command
 *
 *  - Set mechanics.destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the mechanics.feedrate_mm_s, if included
 */
void Commands::get_destination() {

  #if ENABLED(IDLE_OOZING_PREVENT)
    if (parser.seen('E')) printer.IDLE_OOZING_retract(false);
  #endif

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const float v = parser.value_axis_units((AxisEnum)i);
      mechanics.destination[i] = (printer.axis_relative_modes[i] || printer.isRelativeMode())
        ? mechanics.current_position[i] + v
        : (i == E_AXIS) ? v : mechanics.logical_to_native(v, (AxisEnum)i);
    }
    else
      mechanics.destination[i] = mechanics.current_position[i];
  }

  if (parser.linearval('F') > 0.0)
    mechanics.feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());

  if (parser.seen('P'))
    mechanics.destination[E_AXIS] = (parser.value_axis_units(E_AXIS) * tools.density_percentage[tools.previous_extruder] / 100) + mechanics.current_position[E_AXIS];

  if (!printer.debugDryrun() && !printer.debugSimulation()) {
    const float diff = mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS];
    print_job_counter.data.filamentUsed += diff;
    #if ENABLED(RFID_MODULE)
      rfid522.RfidData[tools.active_extruder].data.lenght -= diff;
    #endif
  }

  #if ENABLED(COLOR_MIXING_EXTRUDER)
    get_mix_from_command();
  #endif

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      if ((parser.seen('X') || parser.seen('Y')) && parser.seen('E'))
        gfx_line_to(mechanics.destination[X_AXIS] + (X_MAX_POS), mechanics.destination[Y_AXIS] + (Y_MAX_POS), mechanics.destination[Z_AXIS]);
      else
        gfx_cursor_to(mechanics.destination[X_AXIS] + (X_MAX_POS), mechanics.destination[Y_AXIS] + (Y_MAX_POS), mechanics.destination[Z_AXIS]);
    #else
      if ((parser.seen('X') || parser.seen('Y')) && parser.seen('E'))
        gfx_line_to(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS]);
      else
        gfx_cursor_to(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS]);
    #endif
  #endif
}

/**
 * Set target_tool from the T parameter or the active_tool
 *
 * Returns TRUE if the target is invalid
 */
bool Commands::get_target_tool(const uint16_t code) {
  if (parser.seenval('T')) {
    const int8_t t = parser.value_byte();
    if (t >= EXTRUDERS) {
      SERIAL_SMV(ECHO, "M", code);
      SERIAL_EMV(" " MSG_INVALID_EXTRUDER, t);
      return true;
    }
    tools.target_extruder = t;
  }
  else
    tools.target_extruder = tools.active_extruder;

  return false;
}

bool Commands::get_target_heater(int8_t &h) {

  if (WITHIN(h, 0 , HOTENDS -1)) return true;
  #if HAS_HEATER_BED
    else if (h == -1) {
      h = BED_INDEX;
      return true;
    }
  #endif
  #if HAS_HEATER_CHAMBER
    else if (h == -2) {
      h = CHAMBER_INDEX;
      return true;
    }
  #endif
  #if HAS_HEATER_COOLER
    else if (h == -3) {
      h = COOLER_INDEX;
      return true;
    }
  #endif
  else {
    SERIAL_LM(ER, MSG_INVALID_HEATER);
    return false;
  }
}

/**
 * Private Function
 */

void Commands::gcode_line_error(const char* err, const bool doFlush/*=true*/) {
  SERIAL_STR(ER);
  SERIAL_PS(err);
  SERIAL_EV(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) flush_and_request_resend();
  serial_count = 0;
}

void Commands::unknown_error() {
  SERIAL_SMV(ECHO, MSG_UNKNOWN_COMMAND, parser.command_ptr);
  SERIAL_CHR('"');
  SERIAL_EOL();
}

/* G0 and G1 are sent to the machine way more frequently than any other GCode.
 * We want to make sure that their use is optimized to its maximum.
 */
#if IS_SCARA
  #define EXECUTE_G0_G1(NUM) gcode_G0_G1(NUM == 0)
#elif ENABLED(LASER)
  #define EXECUTE_G0_G1(NUM) gcode_G0_G1(NUM == 1)
#else
  #define EXECUTE_G0_G1(NUM) gcode_G0_G1()
#endif

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void Commands::process_next() {

  char * const current_command = queue[queue_index_r];

  if (printer.debugEcho()) SERIAL_LT(ECHO, current_command);

  KEEPALIVE_STATE(IN_HANDLER);

  // Parse the next command in the queue
  parser.parse(current_command);

  // Handle a known G, M, or T
  switch (parser.command_letter) {

    case 'G': {
      const uint16_t code_num = parser.codenum;
      G_CODE_TYPE start   = 0,
                  middle  = 0,
                  end     = COUNT(GCode_Table) - 1;

      if (code_num <= 1) { // Execute directly the most common Gcodes
        EXECUTE_G0_G1(code_num);
      }
      else if (WITHIN(code_num, GCode_Table[start].code, GCode_Table[end].code)) {
        while (start <= end) {
          middle = (start + end) >> 1;
          if (GCode_Table[middle].code == code_num) {
            GCode_Table[middle].command(); // Command found, execute it
            break;
          }
          else if (GCode_Table[middle].code < code_num)
            start = middle + 1;
          else
            end = middle - 1;
        }
      }
    }
    break;

    case 'M': {
      const uint16_t code_num = parser.codenum;
      M_CODE_TYPE start   = 0,
                  middle  = 0,
                  end     = COUNT(MCode_Table) - 1;

      if (WITHIN(code_num, MCode_Table[start].code, MCode_Table[end].code)) {
        while (start <= end) {
          middle = (start + end) >> 1;
          if (MCode_Table[middle].code == code_num) {
            MCode_Table[middle].command(); // Command found, execute it
            break;
          }
          else if (MCode_Table[middle].code < code_num)
            start = middle + 1;
          else
            end = middle - 1;
        }
      }

      // With M105 "ok" already sended
      if (code_num == 105) {
        KEEPALIVE_STATE(NOT_BUSY);
        return;
      }

      #if ENABLED(ARDUINO_ARCH_SAM)
        // Banzai code for erase bootloader on DUE
        if (code_num == 9999) initiateReset(1000);
      #endif
    }
    break;

    case 'T':
      gcode_T(parser.codenum); // Tn: Tool Change
    break;

    default: unknown_error();
  }

  KEEPALIVE_STATE(NOT_BUSY);

  ok_to_send();
}
