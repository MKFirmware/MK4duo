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
 * sending commands to MK4duo, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
long  Commands::gcode_N             = 0,
      Commands::gcode_LastN         = 0;

bool Commands::send_ok[BUFSIZE];

/**
 * GCode Command Buffer Ring
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The process_next function parses the next
 * command and hands off execution to individual handler functions.
 */
uint8_t Commands::buffer_lenght   = 0,  // Number of commands in the Buffer Ring
        Commands::buffer_index_r  = 0,  // Read position in Buffer Ring
        Commands::buffer_index_w  = 0;  // Write position in Buffer Ring

char Commands::buffer_ring[BUFSIZE][MAX_CMD_SIZE];

/**
 * Private Parameters
 */
int Commands::serial_count = 0;

watch_t Commands::last_command_watch(NO_TIMEOUTS);

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

  #if HAS_DOOR_OPEN
    if (READ(DOOR_OPEN_PIN) != endstops.isLogic(DOOR_OPEN_SENSOR)) {
      printer.keepalive(DoorOpen);
      return;  // do nothing while door is open
    }
  #endif

  // Buffer Ring is full
  if (buffer_lenght >= BUFSIZE) {
    printer.keepalive(InProcess);
    return;
  }

  // If the command buffer is empty for too long,
  // send "wait" to indicate MK4duo is still waiting.
  #if NO_TIMEOUTS > 0
    if (buffer_lenght == 0 && !MKSERIAL.available() && last_command_watch.elapsed()) {
      SERIAL_STR(WT);
      SERIAL_EOL();
      last_command_watch.start();
    }
  #endif

  /**
   * Loop while serial characters are incoming and the buffer_ring is not full
   */
  while (buffer_lenght < BUFSIZE && HAL::serialByteAvailable()) {
    int c;

    last_command_watch.start();
    printer.max_inactivity_watch.start();

    if ((c = MKSERIAL.read()) < 0) continue;

    char serial_char = c;

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false;                      // end of line == end of comment

      // Skip empty lines and comments
      if (!serial_count) { printer.check_periodical_actions(); continue; }

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
      #if HAS_SDSUPPORT
        else if (card.isSaving()) {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }
      #endif

      // Movement commands alert when stopped
      if (!printer.isRunning()) {
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
        if (strcmp(command, "M410") == 0) printer.quickstop_stepper();
      #endif

      // Add the command to the buffer_ring
      enqueue(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') { // Handle escapes
      // if we have one more character, copy it over
      if ((c = MKSERIAL.read()) >= 0 && !serial_comment_mode)
        serial_line_buffer[serial_count++] = (char)c;
    }
    else { // its not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }
  }
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

    #if HAS_DOOR_OPEN
      if (READ(DOOR_OPEN_PIN) != endstops.isLogic(DOOR_OPEN_SENSOR)) {
        printer.keepalive(DoorOpen);
        return;  // do nothing while door is open
      }
    #endif

    #if HAS_POWER_CHECK
      if (READ(POWER_CHECK_PIN) != endstops.isLogic(POWER_CHECK_SENSOR)) {
        printer.setAbortSDprinting(true);
        return;
      }
    #endif

    /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

    if (buffer_lenght == 0) stop_buffering = false;

    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (buffer_lenght < BUFSIZE && !card_eof && !stop_buffering) {
      const int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      last_command_watch.start();
      printer.max_inactivity_watch.start();
      if (card_eof || n == -1
          || sd_char == '\n'  || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {

          card.printingHasFinished();

          if (IS_SD_PRINTING)
            sd_count = 0; // If a sub-file was printing, continue from call point
          else {
            SERIAL_EM(MSG_FILE_PRINTED);
            #if ENABLED(PRINTER_EVENT_LEDS)
              LCD_MESSAGEPGM(MSG_INFO_COMPLETED_PRINTS);
              leds.set_green();
              #if HAS_RESUME_CONTINUE
                enqueue_and_echo_P(PSTR("M0 S"
                  #if HAS_LCD
                    "1800"
                  #else
                    "60"
                  #endif
                ));
              #else
                printer.safe_delay(2000);
                leds.set_off();
              #endif
            #endif // ENABLED(PRINTER_EVENT_LEDS)
          }
        }
        else if (n == -1) {
          SERIAL_LM(ER, MSG_SD_ERR_READ);
        }
        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; // for new command

        // Skip empty lines and comments
        if (!sd_count) { printer.check_periodical_actions(); continue; }

        buffer_ring[buffer_index_w][sd_count] = '\0'; // terminate string
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
        if (!sd_comment_mode) buffer_ring[buffer_index_w][sd_count++] = sd_char;
      }
    }

    printer.progress = card.percentDone();
  }

#endif // HAS_SDSUPPORT

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void Commands::flush_and_request_resend() {
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
  if (!send_ok[buffer_index_r]) return;
  SERIAL_STR(OK);
  #if ENABLED(ADVANCED_OK)
    char* p = buffer_ring[buffer_index_r];
    if (*p == 'N') {
      SERIAL_CHR(' ');
      SERIAL_CHR(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_CHR(*p++);
    }
    SERIAL_MV(" P", (int)(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_MV(" B", BUFSIZE - buffer_lenght);
  #endif
  SERIAL_EOL();
}

/**
 * Add to the buffer ring the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void Commands::get_available() {

  if (buffer_lenght >= BUFSIZE) return;

  // if any immediate commands remain, don't get other commands yet
  if (drain_injected_P()) return;

  get_serial();

  #if HAS_SD_RESTART
    if (restart.job_phase == RESTART_YES && enqueue_restart()) return;
  #endif

  #if HAS_SDSUPPORT
    get_sdcard();
  #endif
}

/**
 * Get the next command in the buffer_ring, optionally log it to SD, then dispatch it
 */
void Commands::advance_queue() {

  if (!buffer_lenght) return;

  #if HAS_SDSUPPORT

    if (card.isSaving()) {
      char* command = buffer_ring[buffer_index_r];
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
    else {
      process_next();
      #if HAS_SD_RESTART
        if (IS_SD_PRINTING) restart.save_data();
      #endif
    }

  #else // !HAS_SDSUPPORT

    process_next();

  #endif // !HAS_SDSUPPORT

  // The buffer_ring may be reset by a command handler or by code invoked by idle() within a handler
  if (buffer_lenght) {
    --buffer_lenght;
    if (++buffer_index_r >= BUFSIZE) buffer_index_r = 0;
  }
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
void Commands::enqueue_and_echo_now(const char* cmd) {
  while (!enqueue_and_echo(cmd)) printer.idle();
}

/**
 * Enqueue from program memory and return only when commands are actually enqueued
 */
void Commands::enqueue_and_echo_now_P(const char * const cmd) {
  enqueue_and_echo_P(cmd);
  while (drain_injected_P()) printer.idle();
}

/**
 * Clear the MK4duo command buffer_ring
 */
void Commands::clear_queue() {
  buffer_index_r = buffer_index_w = buffer_lenght = 0;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
bool Commands::enqueue(const char* cmd, bool say_ok/*=false*/) {
  if (*cmd == ';' || buffer_lenght >= BUFSIZE) return false;
  strcpy(buffer_ring[buffer_index_w], cmd);
  commit(say_ok);
  return true;
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

  if (parser.linearval('F') > 0)
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

bool Commands::get_target_heater(int8_t &h, const bool only_hotend/*=false*/) {
  h = parser.seen('H') ? parser.value_int() : 0;
  if (WITHIN(h, 0 , HOTENDS -1)) return true;
  if (!only_hotend) {
    #if HAS_HEATER_BED
      if (h == -1) {
        h = BED_INDEX;
        return true;
      }
    #endif
    #if HAS_HEATER_CHAMBER
      if (h == -2) {
        h = CHAMBER_INDEX;
        return true;
      }
    #endif
    #if HAS_HEATER_COOLER
      if (h == -3) {
        h = COOLER_INDEX;
        return true;
      }
    #endif
    SERIAL_LM(ER, MSG_INVALID_HEATER);
    return false;
  }
  else {
    SERIAL_LM(ER, MSG_INVALID_HOTEND);
    return false;
  }
}

/**
 * Private Function
 */

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

  char * const current_command = buffer_ring[buffer_index_r];

  if (printer.debugEcho()) SERIAL_LT(ECHO, current_command);

  printer.move_watch.start(); // Keep steppers powered

  // Parse the next command in the buffer_ring
  parser.parse(current_command);
  process_parsed();

}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
void Commands::commit(bool say_ok) {
  send_ok[buffer_index_w] = say_ok;
  if (++buffer_index_w >= BUFSIZE) buffer_index_w = 0;
  buffer_lenght++;
}

void Commands::unknown_error() {
  SERIAL_SMV(ECHO, MSG_UNKNOWN_COMMAND, parser.command_ptr);
  SERIAL_CHR('"');
  SERIAL_EOL();
}

void Commands::gcode_line_error(const char* err) {
  SERIAL_STR(ER);
  SERIAL_PS(err);
  SERIAL_EV(gcode_LastN);
  flush_and_request_resend();
  serial_count = 0;
}

/**
 * Enqueue with Serial Echo
 */
bool Commands::enqueue_and_echo(const char* cmd) {
  if (enqueue(cmd)) {
    SERIAL_SMT(ECHO, MSG_ENQUEUEING, cmd);
    SERIAL_CHR('"');
    SERIAL_EOL();
    return true;
  }
  return false;
}

/**
 * Inject the next "immediate" command, when possible, onto the front of the buffer_ring.
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

#if HAS_SD_RESTART

  bool Commands::enqueue_restart() {
    static uint8_t restart_commands_index = 0;
    if (restart.count) {
      if (enqueue(restart.buffer_ring[restart_commands_index])) {
        ++restart_commands_index;
        if (!--restart.count) restart.job_phase = RESTART_DONE;
      }
      return true;
    }
    else
      return false;
  }

#endif

// Process parsed code
void Commands::process_parsed() {

  printer.keepalive(InHandler);

  #if ENABLED(FASTER_GCODE_EXECUTE) || ENABLED(ARDUINO_ARCH_SAM)

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
          printer.keepalive(NotBusy);
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

  #else

    // Handle a known G, M, or T
    switch (parser.command_letter) {

      case 'G': switch (parser.codenum) {

        case 0: case 1: EXECUTE_G0_G1(parser.codenum); break;
        #if ENABLED(CODE_G2)
          case 2: gcode_G2(); break;
        #endif
        #if ENABLED(CODE_G3)
          case 3: gcode_G3(); break;
        #endif
        #if ENABLED(CODE_G4)
          case 4: gcode_G4(); break;
        #endif
        #if ENABLED(CODE_G5)
          case 5: gcode_G5(); break;
        #endif
        #if ENABLED(CODE_G6)
          case 6: gcode_G6(); break;
        #endif
        #if ENABLED(CODE_G7)
          case 7: gcode_G7(); break;
        #endif
        #if ENABLED(CODE_G8)
          case 8: gcode_G8(); break;
        #endif
        #if ENABLED(CODE_G9)
          case 9: gcode_G9(); break;
        #endif
        #if ENABLED(CODE_G10)
          case 10: gcode_G10(); break;
        #endif
        #if ENABLED(CODE_G11)
          case 11: gcode_G11(); break;
        #endif
        #if ENABLED(CODE_G12)
          case 12: gcode_G12(); break;
        #endif
        #if ENABLED(CODE_G13)
          case 13: gcode_G13(); break;
        #endif
        #if ENABLED(CODE_G14)
          case 14: gcode_G14(); break;
        #endif
        #if ENABLED(CODE_G15)
          case 15: gcode_G15(); break;
        #endif
        #if ENABLED(CODE_G16)
          case 16: gcode_G16(); break;
        #endif
        #if ENABLED(CODE_G17)
          case 17: gcode_G17(); break;
        #endif
        #if ENABLED(CODE_G18)
          case 18: gcode_G18(); break;
        #endif
        #if ENABLED(CODE_G19)
          case 19: gcode_G19(); break;
        #endif
        #if ENABLED(CODE_G20)
          case 20: gcode_G20(); break;
        #endif
        #if ENABLED(CODE_G21)
          case 21: gcode_G21(); break;
        #endif
        #if ENABLED(CODE_G22)
          case 22: gcode_G22(); break;
        #endif
        #if ENABLED(CODE_G23)
          case 23: gcode_G23(); break;
        #endif
        #if ENABLED(CODE_G24)
          case 24: gcode_G24(); break;
        #endif
        #if ENABLED(CODE_G25)
          case 25: gcode_G25(); break;
        #endif
        #if ENABLED(CODE_G26)
          case 26: gcode_G26(); break;
        #endif
        #if ENABLED(CODE_G27)
          case 27: gcode_G27(); break;
        #endif
        #if ENABLED(CODE_G28)
          case 28: gcode_G28(); break;
        #endif
        #if ENABLED(CODE_G29)
          case 29: gcode_G29(); break;
        #endif
        #if ENABLED(CODE_G30)
          case 30: gcode_G30(); break;
        #endif
        #if ENABLED(CODE_G31)
          case 31: gcode_G31(); break;
        #endif
        #if ENABLED(CODE_G32)
          case 32: gcode_G32(); break;
        #endif
        #if ENABLED(CODE_G33)
          case 33: gcode_G33(); break;
        #endif
        #if ENABLED(CODE_G34)
          case 34: gcode_G34(); break;
        #endif
        #if ENABLED(CODE_G35)
          case 35: gcode_G35(); break;
        #endif
        #if ENABLED(CODE_G36)
          case 36: gcode_G36(); break;
        #endif
        #if ENABLED(CODE_G37)
          case 37: gcode_G37(); break;
        #endif
        #if ENABLED(CODE_G38)
          case 38: gcode_G38(); break;
        #endif
        #if ENABLED(CODE_G39)
          case 39: gcode_G39(); break;
        #endif
        #if ENABLED(CODE_G40)
          case 40: gcode_G40(); break;
        #endif
        #if ENABLED(CODE_G41)
          case 41: gcode_G41(); break;
        #endif
        #if ENABLED(CODE_G42)
          case 42: gcode_G42(); break;
        #endif
        #if ENABLED(CODE_G43)
          case 43: gcode_G43(); break;
        #endif
        #if ENABLED(CODE_G44)
          case 44: gcode_G44(); break;
        #endif
        #if ENABLED(CODE_G45)
          case 45: gcode_G45(); break;
        #endif
        #if ENABLED(CODE_G46)
          case 46: gcode_G46(); break;
        #endif
        #if ENABLED(CODE_G47)
          case 47: gcode_G47(); break;
        #endif
        #if ENABLED(CODE_G48)
          case 48: gcode_G48(); break;
        #endif
        #if ENABLED(CODE_G49)
          case 49: gcode_G49(); break;
        #endif
        #if ENABLED(CODE_G50)
          case 50: gcode_G50(); break;
        #endif
        #if ENABLED(CODE_G51)
          case 51: gcode_G51(); break;
        #endif
        #if ENABLED(CODE_G52)
          case 52: gcode_G52(); break;
        #endif
        #if ENABLED(CODE_G53)
          case 53: gcode_G53(); break;
        #endif
        #if ENABLED(CODE_G54)
          case 54: gcode_G54(); break;
        #endif
        #if ENABLED(CODE_G55)
          case 55: gcode_G55(); break;
        #endif
        #if ENABLED(CODE_G56)
          case 56: gcode_G56(); break;
        #endif
        #if ENABLED(CODE_G57)
          case 57: gcode_G57(); break;
        #endif
        #if ENABLED(CODE_G58)
          case 58: gcode_G58(); break;
        #endif
        #if ENABLED(CODE_G59)
          case 59: gcode_G59(); break;
        #endif
        #if ENABLED(CODE_G60)
          case 60: gcode_G60(); break;
        #endif
        #if ENABLED(CODE_G61)
          case 61: gcode_G61(); break;
        #endif
        #if ENABLED(CODE_G62)
          case 62: gcode_G62(); break;
        #endif
        #if ENABLED(CODE_G63)
          case 63: gcode_G63(); break;
        #endif
        #if ENABLED(CODE_G64)
          case 64: gcode_G64(); break;
        #endif
        #if ENABLED(CODE_G65)
          case 65: gcode_G65(); break;
        #endif
        #if ENABLED(CODE_G66)
          case 66: gcode_G66(); break;
        #endif
        #if ENABLED(CODE_G67)
          case 67: gcode_G67(); break;
        #endif
        #if ENABLED(CODE_G68)
          case 68: gcode_G68(); break;
        #endif
        #if ENABLED(CODE_G69)
          case 69: gcode_G69(); break;
        #endif
        #if ENABLED(CODE_G70)
          case 70: gcode_G70(); break;
        #endif
        #if ENABLED(CODE_G71)
          case 71: gcode_G71(); break;
        #endif
        #if ENABLED(CODE_G72)
          case 72: gcode_G72(); break;
        #endif
        #if ENABLED(CODE_G73)
          case 73: gcode_G73(); break;
        #endif
        #if ENABLED(CODE_G74)
          case 74: gcode_G74(); break;
        #endif
        #if ENABLED(CODE_G75)
          case 75: gcode_G75(); break;
        #endif
        #if ENABLED(CODE_G76)
          case 76: gcode_G76(); break;
        #endif
        #if ENABLED(CODE_G77)
          case 77: gcode_G77(); break;
        #endif
        #if ENABLED(CODE_G78)
          case 78: gcode_G78(); break;
        #endif
        #if ENABLED(CODE_G79)
          case 79: gcode_G79(); break;
        #endif
        #if ENABLED(CODE_G80)
          case 80: gcode_G80(); break;
        #endif
        #if ENABLED(CODE_G81)
          case 81: gcode_G81(); break;
        #endif
        #if ENABLED(CODE_G82)
          case 82: gcode_G82(); break;
        #endif
        #if ENABLED(CODE_G83)
          case 83: gcode_G83(); break;
        #endif
        #if ENABLED(CODE_G84)
          case 84: gcode_G84(); break;
        #endif
        #if ENABLED(CODE_G85)
          case 85: gcode_G85(); break;
        #endif
        #if ENABLED(CODE_G86)
          case 86: gcode_G86(); break;
        #endif
        #if ENABLED(CODE_G87)
          case 87: gcode_G87(); break;
        #endif
        #if ENABLED(CODE_G88)
          case 88: gcode_G88(); break;
        #endif
        #if ENABLED(CODE_G89)
          case 89: gcode_G89(); break;
        #endif
        #if ENABLED(CODE_G90)
          case 90: gcode_G90(); break;
        #endif
        #if ENABLED(CODE_G91)
          case 91: gcode_G91(); break;
        #endif
        #if ENABLED(CODE_G92)
          case 92: gcode_G92(); break;
        #endif
        #if ENABLED(CODE_G93)
          case 93: gcode_G93(); break;
        #endif
        #if ENABLED(CODE_G94)
          case 94: gcode_G94(); break;
        #endif
        #if ENABLED(CODE_G95)
          case 95: gcode_G95(); break;
        #endif
        #if ENABLED(CODE_G96)
          case 96: gcode_G96(); break;
        #endif
        #if ENABLED(CODE_G97)
          case 97: gcode_G97(); break;
        #endif
        #if ENABLED(CODE_G98)
          case 98: gcode_G98(); break;
        #endif
        #if ENABLED(CODE_G99)
          case 99: gcode_G99(); break;
        #endif

        default: unknown_error(); break;
      }
      break;

      case 'M': switch (parser.codenum) {

        #if ENABLED(CODE_M0)
          case 0: gcode_M0_M1(); break;
        #endif
        #if ENABLED(CODE_M1)
          case 1: gcode_M0_M1(); break;
        #endif
        #if ENABLED(CODE_M2)
          case 2: gcode_M2(); break;
        #endif
        #if ENABLED(CODE_M3)
          case 3: gcode_M3(); break;
        #endif
        #if ENABLED(CODE_M4)
          case 4: gcode_M4(); break;
        #endif
        #if ENABLED(CODE_M5)
          case 5: gcode_M5(); break;
        #endif
        #if ENABLED(CODE_M6)
          case 6: gcode_M6(); break;
        #endif
        #if ENABLED(CODE_M7)
          case 7: gcode_M7(); break;
        #endif
        #if ENABLED(CODE_M8)
          case 8: gcode_M8(); break;
        #endif
        #if ENABLED(CODE_M9)
          case 9: gcode_M9(); break;
        #endif
        #if ENABLED(CODE_M10)
          case 10: gcode_M10(); break;
        #endif
        #if ENABLED(CODE_M11)
          case 11: gcode_M11(); break;
        #endif
        #if ENABLED(CODE_M12)
          case 12: gcode_M12(); break;
        #endif
        #if ENABLED(CODE_M13)
          case 13: gcode_M13(); break;
        #endif
        #if ENABLED(CODE_M14)
          case 14: gcode_M14(); break;
        #endif
        #if ENABLED(CODE_M15)
          case 15: gcode_M15(); break;
        #endif
        #if ENABLED(CODE_M16)
          case 16: gcode_M16(); break;
        #endif
        #if ENABLED(CODE_M17)
          case 17: gcode_M17(); break;
        #endif
        #if ENABLED(CODE_M18)
          case 18: gcode_M18_M84(); break;
        #endif
        #if ENABLED(CODE_M19)
          case 19: gcode_M19(); break;
        #endif
        #if ENABLED(CODE_M20)
          case 20: gcode_M20(); break;
        #endif
        #if ENABLED(CODE_M21)
          case 21: gcode_M21(); break;
        #endif
        #if ENABLED(CODE_M22)
          case 22: gcode_M22(); break;
        #endif
        #if ENABLED(CODE_M23)
          case 23: gcode_M23(); break;
        #endif
        #if ENABLED(CODE_M24)
          case 24: gcode_M24(); break;
        #endif
        #if ENABLED(CODE_M25)
          case 25: gcode_M25(); break;
        #endif
        #if ENABLED(CODE_M26)
          case 26: gcode_M26(); break;
        #endif
        #if ENABLED(CODE_M27)
          case 27: gcode_M27(); break;
        #endif
        #if ENABLED(CODE_M28)
          case 28: gcode_M28(); break;
        #endif
        #if ENABLED(CODE_M29)
          case 29: gcode_M29(); break;
        #endif
        #if ENABLED(CODE_M30)
          case 30: gcode_M30(); break;
        #endif
        #if ENABLED(CODE_M31)
          case 31: gcode_M31(); break;
        #endif
        #if ENABLED(CODE_M32)
          case 32: gcode_M32(); break;
        #endif
        #if ENABLED(CODE_M33)
          case 33: gcode_M33(); break;
        #endif
        #if ENABLED(CODE_M34)
          case 34: gcode_M34(); break;
        #endif
        #if ENABLED(CODE_M35)
          case 35: gcode_M35(); break;
        #endif
        #if ENABLED(CODE_M36)
          case 36: gcode_M36(); break;
        #endif
        #if ENABLED(CODE_M37)
          case 37: gcode_M37(); break;
        #endif
        #if ENABLED(CODE_M38)
          case 38: gcode_M38(); break;
        #endif
        #if ENABLED(CODE_M39)
          case 39: gcode_M39(); break;
        #endif
        #if ENABLED(CODE_M40)
          case 40: gcode_M40(); break;
        #endif
        #if ENABLED(CODE_M41)
          case 41: gcode_M41(); break;
        #endif
        #if ENABLED(CODE_M42)
          case 42: gcode_M42(); break;
        #endif
        #if ENABLED(CODE_M43)
          case 43: gcode_M43(); break;
        #endif
        #if ENABLED(CODE_M44)
          case 44: gcode_M44(); break;
        #endif
        #if ENABLED(CODE_M45)
          case 45: gcode_M45(); break;
        #endif
        #if ENABLED(CODE_M46)
          case 46: gcode_M46(); break;
        #endif
        #if ENABLED(CODE_M47)
          case 47: gcode_M47(); break;
        #endif
        #if ENABLED(CODE_M48)
          case 48: gcode_M48(); break;
        #endif
        #if ENABLED(CODE_M49)
          case 49: gcode_M49(); break;
        #endif
        #if ENABLED(CODE_M50)
          case 50: gcode_M50(); break;
        #endif
        #if ENABLED(CODE_M51)
          case 51: gcode_M51(); break;
        #endif
        #if ENABLED(CODE_M52)
          case 52: gcode_M52(); break;
        #endif
        #if ENABLED(CODE_M53)
          case 53: gcode_M53(); break;
        #endif
        #if ENABLED(CODE_M54)
          case 54: gcode_M54(); break;
        #endif
        #if ENABLED(CODE_M55)
          case 55: gcode_M55(); break;
        #endif
        #if ENABLED(CODE_M56)
          case 56: gcode_M56(); break;
        #endif
        #if ENABLED(CODE_M57)
          case 57: gcode_M57(); break;
        #endif
        #if ENABLED(CODE_M58)
          case 58: gcode_M58(); break;
        #endif
        #if ENABLED(CODE_M59)
          case 59: gcode_M59(); break;
        #endif
        #if ENABLED(CODE_M60)
          case 60: gcode_M60(); break;
        #endif
        #if ENABLED(CODE_M61)
          case 61: gcode_M61(); break;
        #endif
        #if ENABLED(CODE_M62)
          case 62: gcode_M62(); break;
        #endif
        #if ENABLED(CODE_M63)
          case 63: gcode_M63(); break;
        #endif
        #if ENABLED(CODE_M64)
          case 64: gcode_M64(); break;
        #endif
        #if ENABLED(CODE_M65)
          case 65: gcode_M65(); break;
        #endif
        #if ENABLED(CODE_M66)
          case 66: gcode_M66(); break;
        #endif
        #if ENABLED(CODE_M67)
          case 67: gcode_M67(); break;
        #endif
        #if ENABLED(CODE_M68)
          case 68: gcode_M68(); break;
        #endif
        #if ENABLED(CODE_M69)
          case 69: gcode_M69(); break;
        #endif
        #if ENABLED(CODE_M70)
          case 70: gcode_M70(); break;
        #endif
        #if ENABLED(CODE_M71)
          case 71: gcode_M71(); break;
        #endif
        #if ENABLED(CODE_M72)
          case 72: gcode_M72(); break;
        #endif
        #if ENABLED(CODE_M73)
          case 73: gcode_M73(); break;
        #endif
        #if ENABLED(CODE_M74)
          case 74: gcode_M74(); break;
        #endif
        #if ENABLED(CODE_M75)
          case 75: gcode_M75(); break;
        #endif
        #if ENABLED(CODE_M76)
          case 76: gcode_M76(); break;
        #endif
        #if ENABLED(CODE_M77)
          case 77: gcode_M77(); break;
        #endif
        #if ENABLED(CODE_M78)
          case 78: gcode_M78(); break;
        #endif
        #if ENABLED(CODE_M79)
          case 79: gcode_M79(); break;
        #endif
        #if ENABLED(CODE_M80)
          case 80: gcode_M80(); break;
        #endif
        #if ENABLED(CODE_M81)
          case 81: gcode_M81(); break;
        #endif
        #if ENABLED(CODE_M82)
          case 82: gcode_M82(); break;
        #endif
        #if ENABLED(CODE_M83)
          case 83: gcode_M83(); break;
        #endif
        #if ENABLED(CODE_M84)
          case 84: gcode_M18_M84(); break;
        #endif
        #if ENABLED(CODE_M85)
          case 85: gcode_M85(); break;
        #endif
        #if ENABLED(CODE_M86)
          case 86: gcode_M86(); break;
        #endif
        #if ENABLED(CODE_M87)
          case 87: gcode_M87(); break;
        #endif
        #if ENABLED(CODE_M88)
          case 88: gcode_M88(); break;
        #endif
        #if ENABLED(CODE_M89)
          case 89: gcode_M89(); break;
        #endif
        #if ENABLED(CODE_M90)
          case 90: gcode_M90(); break;
        #endif
        #if ENABLED(CODE_M91)
          case 91: gcode_M91(); break;
        #endif
        #if ENABLED(CODE_M92)
          case 92: gcode_M92(); break;
        #endif
        #if ENABLED(CODE_M93)
          case 93: gcode_M93(); break;
        #endif
        #if ENABLED(CODE_M94)
          case 94: gcode_M94(); break;
        #endif
        #if ENABLED(CODE_M95)
          case 95: gcode_M95(); break;
        #endif
        #if ENABLED(CODE_M96)
          case 96: gcode_M96(); break;
        #endif
        #if ENABLED(CODE_M97)
          case 97: gcode_M97(); break;
        #endif
        #if ENABLED(CODE_M98)
          case 98: gcode_M98(); break;
        #endif
        #if ENABLED(CODE_M99)
          case 99: gcode_M99(); break;
        #endif
        #if ENABLED(CODE_M100)
          case 100: gcode_M100(); break;
        #endif
        #if ENABLED(CODE_M101)
          case 101: gcode_M101(); break;
        #endif
        #if ENABLED(CODE_M102)
          case 102: gcode_M102(); break;
        #endif
        #if ENABLED(CODE_M103)
          case 103: gcode_M103(); break;
        #endif
        #if ENABLED(CODE_M104)
          case 104: gcode_M104(); break;
        #endif
        #if ENABLED(CODE_M105)
          case 105: gcode_M105(); printer.keepalive(NotBusy); return;
        #endif
        #if ENABLED(CODE_M106)
          case 106: gcode_M106(); break;
        #endif
        #if ENABLED(CODE_M107)
          case 107: gcode_M107(); break;
        #endif
        #if ENABLED(CODE_M108)
          case 108: gcode_M108(); break;
        #endif
        #if ENABLED(CODE_M109)
          case 109: gcode_M109(); break;
        #endif
        #if ENABLED(CODE_M110)
          case 110: gcode_M110(); break;
        #endif
        #if ENABLED(CODE_M111)
          case 111: gcode_M111(); break;
        #endif
        #if ENABLED(CODE_M112)
          case 112: gcode_M112(); break;
        #endif
        #if ENABLED(CODE_M113)
          case 113: gcode_M113(); break;
        #endif
        #if ENABLED(CODE_M114)
          case 114: gcode_M114(); break;
        #endif
        #if ENABLED(CODE_M115)
          case 115: gcode_M115(); break;
        #endif
        #if ENABLED(CODE_M116)
          case 116: gcode_M116(); break;
        #endif
        #if ENABLED(CODE_M117)
          case 117: gcode_M117(); break;
        #endif
        #if ENABLED(CODE_M118)
          case 118: gcode_M118(); break;
        #endif
        #if ENABLED(CODE_M119)
          case 119: gcode_M119(); break;
        #endif
        #if ENABLED(CODE_M120)
          case 120: gcode_M120(); break;
        #endif
        #if ENABLED(CODE_M121)
          case 121: gcode_M121(); break;
        #endif
        #if ENABLED(CODE_M122)
          case 122: gcode_M122(); break;
        #endif
        #if ENABLED(CODE_M123)
          case 123: gcode_M123(); break;
        #endif
        #if ENABLED(CODE_M124)
          case 124: gcode_M124(); break;
        #endif
        #if ENABLED(CODE_M125)
          case 125: gcode_M125(); break;
        #endif
        #if ENABLED(CODE_M126)
          case 126: gcode_M126(); break;
        #endif
        #if ENABLED(CODE_M127)
          case 127: gcode_M127(); break;
        #endif
        #if ENABLED(CODE_M128)
          case 128: gcode_M128(); break;
        #endif
        #if ENABLED(CODE_M129)
          case 129: gcode_M129(); break;
        #endif
        #if ENABLED(CODE_M130)
          case 130: gcode_M130(); break;
        #endif
        #if ENABLED(CODE_M131)
          case 131: gcode_M131(); break;
        #endif
        #if ENABLED(CODE_M132)
          case 132: gcode_M132(); break;
        #endif
        #if ENABLED(CODE_M133)
          case 133: gcode_M133(); break;
        #endif
        #if ENABLED(CODE_M134)
          case 134: gcode_M134(); break;
        #endif
        #if ENABLED(CODE_M135)
          case 135: gcode_M135(); break;
        #endif
        #if ENABLED(CODE_M136)
          case 136: gcode_M136(); break;
        #endif
        #if ENABLED(CODE_M137)
          case 137: gcode_M137(); break;
        #endif
        #if ENABLED(CODE_M138)
          case 138: gcode_M138(); break;
        #endif
        #if ENABLED(CODE_M139)
          case 139: gcode_M139(); break;
        #endif
        #if ENABLED(CODE_M140)
          case 140: gcode_M140(); break;
        #endif
        #if ENABLED(CODE_M141)
          case 141: gcode_M141(); break;
        #endif
        #if ENABLED(CODE_M142)
          case 142: gcode_M142(); break;
        #endif
        #if ENABLED(CODE_M143)
          case 143: gcode_M143(); break;
        #endif
        #if ENABLED(CODE_M144)
          case 144: gcode_M144(); break;
        #endif
        #if ENABLED(CODE_M145)
          case 145: gcode_M145(); break;
        #endif
        #if ENABLED(CODE_M146)
          case 146: gcode_M146(); break;
        #endif
        #if ENABLED(CODE_M147)
          case 147: gcode_M147(); break;
        #endif
        #if ENABLED(CODE_M148)
          case 148: gcode_M148(); break;
        #endif
        #if ENABLED(CODE_M149)
          case 149: gcode_M149(); break;
        #endif
        #if ENABLED(CODE_M150)
          case 150: gcode_M150(); break;
        #endif
        #if ENABLED(CODE_M151)
          case 151: gcode_M151(); break;
        #endif
        #if ENABLED(CODE_M152)
          case 152: gcode_M152(); break;
        #endif
        #if ENABLED(CODE_M153)
          case 153: gcode_M153(); break;
        #endif
        #if ENABLED(CODE_M154)
          case 154: gcode_M154(); break;
        #endif
        #if ENABLED(CODE_M155)
          case 155: gcode_M155(); break;
        #endif
        #if ENABLED(CODE_M156)
          case 156: gcode_M156(); break;
        #endif
        #if ENABLED(CODE_M157)
          case 157: gcode_M157(); break;
        #endif
        #if ENABLED(CODE_M158)
          case 158: gcode_M158(); break;
        #endif
        #if ENABLED(CODE_M159)
          case 159: gcode_M159(); break;
        #endif
        #if ENABLED(CODE_M160)
          case 160: gcode_M160(); break;
        #endif
        #if ENABLED(CODE_M161)
          case 161: gcode_M161(); break;
        #endif
        #if ENABLED(CODE_M162)
          case 162: gcode_M162(); break;
        #endif
        #if ENABLED(CODE_M163)
          case 163: gcode_M163(); break;
        #endif
        #if ENABLED(CODE_M164)
          case 164: gcode_M164(); break;
        #endif
        #if ENABLED(CODE_M165)
          case 165: gcode_M165(); break;
        #endif
        #if ENABLED(CODE_M166)
          case 166: gcode_M166(); break;
        #endif
        #if ENABLED(CODE_M167)
          case 167: gcode_M167(); break;
        #endif
        #if ENABLED(CODE_M168)
          case 168: gcode_M168(); break;
        #endif
        #if ENABLED(CODE_M169)
          case 169: gcode_M169(); break;
        #endif
        #if ENABLED(CODE_M170)
          case 170: gcode_M170(); break;
        #endif
        #if ENABLED(CODE_M171)
          case 171: gcode_M171(); break;
        #endif
        #if ENABLED(CODE_M172)
          case 172: gcode_M172(); break;
        #endif
        #if ENABLED(CODE_M173)
          case 173: gcode_M173(); break;
        #endif
        #if ENABLED(CODE_M174)
          case 174: gcode_M174(); break;
        #endif
        #if ENABLED(CODE_M175)
          case 175: gcode_M175(); break;
        #endif
        #if ENABLED(CODE_M176)
          case 176: gcode_M176(); break;
        #endif
        #if ENABLED(CODE_M177)
          case 177: gcode_M177(); break;
        #endif
        #if ENABLED(CODE_M178)
          case 178: gcode_M178(); break;
        #endif
        #if ENABLED(CODE_M179)
          case 179: gcode_M179(); break;
        #endif
        #if ENABLED(CODE_M180)
          case 180: gcode_M180(); break;
        #endif
        #if ENABLED(CODE_M181)
          case 181: gcode_M181(); break;
        #endif
        #if ENABLED(CODE_M182)
          case 182: gcode_M182(); break;
        #endif
        #if ENABLED(CODE_M183)
          case 183: gcode_M183(); break;
        #endif
        #if ENABLED(CODE_M184)
          case 184: gcode_M184(); break;
        #endif
        #if ENABLED(CODE_M185)
          case 185: gcode_M185(); break;
        #endif
        #if ENABLED(CODE_M186)
          case 186: gcode_M186(); break;
        #endif
        #if ENABLED(CODE_M187)
          case 187: gcode_M187(); break;
        #endif
        #if ENABLED(CODE_M188)
          case 188: gcode_M188(); break;
        #endif
        #if ENABLED(CODE_M189)
          case 189: gcode_M189(); break;
        #endif
        #if ENABLED(CODE_M190)
          case 190: gcode_M190(); break;
        #endif
        #if ENABLED(CODE_M191)
          case 191: gcode_M191(); break;
        #endif
        #if ENABLED(CODE_M192)
          case 192: gcode_M192(); break;
        #endif
        #if ENABLED(CODE_M193)
          case 193: gcode_M193(); break;
        #endif
        #if ENABLED(CODE_M194)
          case 194: gcode_M194(); break;
        #endif
        #if ENABLED(CODE_M195)
          case 195: gcode_M195(); break;
        #endif
        #if ENABLED(CODE_M196)
          case 196: gcode_M196(); break;
        #endif
        #if ENABLED(CODE_M197)
          case 197: gcode_M197(); break;
        #endif
        #if ENABLED(CODE_M198)
          case 198: gcode_M198(); break;
        #endif
        #if ENABLED(CODE_M199)
          case 199: gcode_M199(); break;
        #endif
        #if ENABLED(CODE_M200)
          case 200: gcode_M200(); break;
        #endif
        #if ENABLED(CODE_M201)
          case 201: gcode_M201(); break;
        #endif
        #if ENABLED(CODE_M202)
          case 202: gcode_M202(); break;
        #endif
        #if ENABLED(CODE_M203)
          case 203: gcode_M203(); break;
        #endif
        #if ENABLED(CODE_M204)
          case 204: gcode_M204(); break;
        #endif
        #if ENABLED(CODE_M205)
          case 205: gcode_M205(); break;
        #endif
        #if ENABLED(CODE_M206)
          case 206: gcode_M206(); break;
        #endif
        #if ENABLED(CODE_M207)
          case 207: gcode_M207(); break;
        #endif
        #if ENABLED(CODE_M208)
          case 208: gcode_M208(); break;
        #endif
        #if ENABLED(CODE_M209)
          case 209: gcode_M209(); break;
        #endif
        #if ENABLED(CODE_M210)
          case 210: gcode_M210(); break;
        #endif
        #if ENABLED(CODE_M211)
          case 211: gcode_M211(); break;
        #endif
        #if ENABLED(CODE_M212)
          case 212: gcode_M212(); break;
        #endif
        #if ENABLED(CODE_M213)
          case 213: gcode_M213(); break;
        #endif
        #if ENABLED(CODE_M214)
          case 214: gcode_M214(); break;
        #endif
        #if ENABLED(CODE_M215)
          case 215: gcode_M215(); break;
        #endif
        #if ENABLED(CODE_M216)
          case 216: gcode_M216(); break;
        #endif
        #if ENABLED(CODE_M217)
          case 217: gcode_M217(); break;
        #endif
        #if ENABLED(CODE_M218)
          case 218: gcode_M218(); break;
        #endif
        #if ENABLED(CODE_M219)
          case 219: gcode_M219(); break;
        #endif
        #if ENABLED(CODE_M220)
          case 220: gcode_M220(); break;
        #endif
        #if ENABLED(CODE_M221)
          case 221: gcode_M221(); break;
        #endif
        #if ENABLED(CODE_M222)
          case 222: gcode_M222(); break;
        #endif
        #if ENABLED(CODE_M223)
          case 223: gcode_M223(); break;
        #endif
        #if ENABLED(CODE_M224)
          case 224: gcode_M224(); break;
        #endif
        #if ENABLED(CODE_M225)
          case 225: gcode_M225(); break;
        #endif
        #if ENABLED(CODE_M226)
          case 226: gcode_M226(); break;
        #endif
        #if ENABLED(CODE_M227)
          case 227: gcode_M227(); break;
        #endif
        #if ENABLED(CODE_M228)
          case 228: gcode_M228(); break;
        #endif
        #if ENABLED(CODE_M229)
          case 229: gcode_M229(); break;
        #endif
        #if ENABLED(CODE_M230)
          case 230: gcode_M230(); break;
        #endif
        #if ENABLED(CODE_M231)
          case 231: gcode_M231(); break;
        #endif
        #if ENABLED(CODE_M232)
          case 232: gcode_M232(); break;
        #endif
        #if ENABLED(CODE_M233)
          case 233: gcode_M233(); break;
        #endif
        #if ENABLED(CODE_M234)
          case 234: gcode_M234(); break;
        #endif
        #if ENABLED(CODE_M235)
          case 235: gcode_M235(); break;
        #endif
        #if ENABLED(CODE_M236)
          case 236: gcode_M236(); break;
        #endif
        #if ENABLED(CODE_M237)
          case 237: gcode_M237(); break;
        #endif
        #if ENABLED(CODE_M238)
          case 238: gcode_M238(); break;
        #endif
        #if ENABLED(CODE_M239)
          case 239: gcode_M239(); break;
        #endif
        #if ENABLED(CODE_M240)
          case 240: gcode_M240(); break;
        #endif
        #if ENABLED(CODE_M241)
          case 241: gcode_M241(); break;
        #endif
        #if ENABLED(CODE_M242)
          case 242: gcode_M242(); break;
        #endif
        #if ENABLED(CODE_M243)
          case 243: gcode_M243(); break;
        #endif
        #if ENABLED(CODE_M244)
          case 244: gcode_M244(); break;
        #endif
        #if ENABLED(CODE_M245)
          case 245: gcode_M245(); break;
        #endif
        #if ENABLED(CODE_M246)
          case 246: gcode_M246(); break;
        #endif
        #if ENABLED(CODE_M247)
          case 247: gcode_M247(); break;
        #endif
        #if ENABLED(CODE_M248)
          case 248: gcode_M248(); break;
        #endif
        #if ENABLED(CODE_M249)
          case 249: gcode_M249(); break;
        #endif
        #if ENABLED(CODE_M250)
          case 250: gcode_M250(); break;
        #endif
        #if ENABLED(CODE_M251)
          case 251: gcode_M251(); break;
        #endif
        #if ENABLED(CODE_M252)
          case 252: gcode_M252(); break;
        #endif
        #if ENABLED(CODE_M253)
          case 253: gcode_M253(); break;
        #endif
        #if ENABLED(CODE_M254)
          case 254: gcode_M254(); break;
        #endif
        #if ENABLED(CODE_M255)
          case 255: gcode_M255(); break;
        #endif
        #if ENABLED(CODE_M256)
          case 256: gcode_M256(); break;
        #endif
        #if ENABLED(CODE_M257)
          case 257: gcode_M257(); break;
        #endif
        #if ENABLED(CODE_M258)
          case 258: gcode_M258(); break;
        #endif
        #if ENABLED(CODE_M259)
          case 259: gcode_M259(); break;
        #endif
        #if ENABLED(CODE_M260)
          case 260: gcode_M260(); break;
        #endif
        #if ENABLED(CODE_M261)
          case 261: gcode_M261(); break;
        #endif
        #if ENABLED(CODE_M262)
          case 262: gcode_M262(); break;
        #endif
        #if ENABLED(CODE_M263)
          case 263: gcode_M263(); break;
        #endif
        #if ENABLED(CODE_M264)
          case 264: gcode_M264(); break;
        #endif
        #if ENABLED(CODE_M265)
          case 265: gcode_M265(); break;
        #endif
        #if ENABLED(CODE_M266)
          case 266: gcode_M266(); break;
        #endif
        #if ENABLED(CODE_M267)
          case 267: gcode_M267(); break;
        #endif
        #if ENABLED(CODE_M268)
          case 268: gcode_M268(); break;
        #endif
        #if ENABLED(CODE_M269)
          case 269: gcode_M269(); break;
        #endif
        #if ENABLED(CODE_M270)
          case 270: gcode_M270(); break;
        #endif
        #if ENABLED(CODE_M271)
          case 271: gcode_M271(); break;
        #endif
        #if ENABLED(CODE_M272)
          case 272: gcode_M272(); break;
        #endif
        #if ENABLED(CODE_M273)
          case 273: gcode_M273(); break;
        #endif
        #if ENABLED(CODE_M274)
          case 274: gcode_M274(); break;
        #endif
        #if ENABLED(CODE_M275)
          case 275: gcode_M275(); break;
        #endif
        #if ENABLED(CODE_M276)
          case 276: gcode_M276(); break;
        #endif
        #if ENABLED(CODE_M277)
          case 277: gcode_M277(); break;
        #endif
        #if ENABLED(CODE_M278)
          case 278: gcode_M278(); break;
        #endif
        #if ENABLED(CODE_M279)
          case 279: gcode_M279(); break;
        #endif
        #if ENABLED(CODE_M280)
          case 280: gcode_M280(); break;
        #endif
        #if ENABLED(CODE_M281)
          case 281: gcode_M281(); break;
        #endif
        #if ENABLED(CODE_M282)
          case 282: gcode_M282(); break;
        #endif
        #if ENABLED(CODE_M283)
          case 283: gcode_M283(); break;
        #endif
        #if ENABLED(CODE_M284)
          case 284: gcode_M284(); break;
        #endif
        #if ENABLED(CODE_M285)
          case 285: gcode_M285(); break;
        #endif
        #if ENABLED(CODE_M286)
          case 286: gcode_M286(); break;
        #endif
        #if ENABLED(CODE_M287)
          case 287: gcode_M287(); break;
        #endif
        #if ENABLED(CODE_M288)
          case 288: gcode_M288(); break;
        #endif
        #if ENABLED(CODE_M289)
          case 289: gcode_M289(); break;
        #endif
        #if ENABLED(CODE_M290)
          case 290: gcode_M290(); break;
        #endif
        #if ENABLED(CODE_M291)
          case 291: gcode_M291(); break;
        #endif
        #if ENABLED(CODE_M292)
          case 292: gcode_M292(); break;
        #endif
        #if ENABLED(CODE_M293)
          case 293: gcode_M293(); break;
        #endif
        #if ENABLED(CODE_M294)
          case 294: gcode_M294(); break;
        #endif
        #if ENABLED(CODE_M295)
          case 295: gcode_M295(); break;
        #endif
        #if ENABLED(CODE_M296)
          case 296: gcode_M296(); break;
        #endif
        #if ENABLED(CODE_M297)
          case 297: gcode_M297(); break;
        #endif
        #if ENABLED(CODE_M298)
          case 298: gcode_M298(); break;
        #endif
        #if ENABLED(CODE_M299)
          case 299: gcode_M299(); break;
        #endif
        #if ENABLED(CODE_M300)
          case 300: gcode_M300(); break;
        #endif
        #if ENABLED(CODE_M301)
          case 301: gcode_M301(); break;
        #endif
        #if ENABLED(CODE_M302)
          case 302: gcode_M302(); break;
        #endif
        #if ENABLED(CODE_M303)
          case 303: gcode_M303(); break;
        #endif
        #if ENABLED(CODE_M304)
          case 304: gcode_M304(); break;
        #endif
        #if ENABLED(CODE_M305)
          case 305: gcode_M305(); break;
        #endif
        #if ENABLED(CODE_M306)
          case 306: gcode_M306(); break;
        #endif
        #if ENABLED(CODE_M307)
          case 307: gcode_M307(); break;
        #endif
        #if ENABLED(CODE_M308)
          case 308: gcode_M308(); break;
        #endif
        #if ENABLED(CODE_M309)
          case 309: gcode_M309(); break;
        #endif
        #if ENABLED(CODE_M310)
          case 310: gcode_M310(); break;
        #endif
        #if ENABLED(CODE_M311)
          case 311: gcode_M311(); break;
        #endif
        #if ENABLED(CODE_M312)
          case 312: gcode_M312(); break;
        #endif
        #if ENABLED(CODE_M313)
          case 313: gcode_M313(); break;
        #endif
        #if ENABLED(CODE_M314)
          case 314: gcode_M314(); break;
        #endif
        #if ENABLED(CODE_M315)
          case 315: gcode_M315(); break;
        #endif
        #if ENABLED(CODE_M316)
          case 316: gcode_M316(); break;
        #endif
        #if ENABLED(CODE_M317)
          case 317: gcode_M317(); break;
        #endif
        #if ENABLED(CODE_M318)
          case 318: gcode_M318(); break;
        #endif
        #if ENABLED(CODE_M319)
          case 319: gcode_M319(); break;
        #endif
        #if ENABLED(CODE_M320)
          case 320: gcode_M320(); break;
        #endif
        #if ENABLED(CODE_M321)
          case 321: gcode_M321(); break;
        #endif
        #if ENABLED(CODE_M322)
          case 322: gcode_M322(); break;
        #endif
        #if ENABLED(CODE_M323)
          case 323: gcode_M323(); break;
        #endif
        #if ENABLED(CODE_M324)
          case 324: gcode_M324(); break;
        #endif
        #if ENABLED(CODE_M325)
          case 325: gcode_M325(); break;
        #endif
        #if ENABLED(CODE_M326)
          case 326: gcode_M326(); break;
        #endif
        #if ENABLED(CODE_M327)
          case 327: gcode_M327(); break;
        #endif
        #if ENABLED(CODE_M328)
          case 328: gcode_M328(); break;
        #endif
        #if ENABLED(CODE_M329)
          case 329: gcode_M329(); break;
        #endif
        #if ENABLED(CODE_M330)
          case 330: gcode_M330(); break;
        #endif
        #if ENABLED(CODE_M331)
          case 331: gcode_M331(); break;
        #endif
        #if ENABLED(CODE_M332)
          case 332: gcode_M332(); break;
        #endif
        #if ENABLED(CODE_M333)
          case 333: gcode_M333(); break;
        #endif
        #if ENABLED(CODE_M334)
          case 334: gcode_M334(); break;
        #endif
        #if ENABLED(CODE_M335)
          case 335: gcode_M335(); break;
        #endif
        #if ENABLED(CODE_M336)
          case 336: gcode_M336(); break;
        #endif
        #if ENABLED(CODE_M337)
          case 337: gcode_M337(); break;
        #endif
        #if ENABLED(CODE_M338)
          case 338: gcode_M338(); break;
        #endif
        #if ENABLED(CODE_M339)
          case 339: gcode_M339(); break;
        #endif
        #if ENABLED(CODE_M340)
          case 340: gcode_M340(); break;
        #endif
        #if ENABLED(CODE_M341)
          case 341: gcode_M341(); break;
        #endif
        #if ENABLED(CODE_M342)
          case 342: gcode_M342(); break;
        #endif
        #if ENABLED(CODE_M343)
          case 343: gcode_M343(); break;
        #endif
        #if ENABLED(CODE_M344)
          case 344: gcode_M344(); break;
        #endif
        #if ENABLED(CODE_M345)
          case 345: gcode_M345(); break;
        #endif
        #if ENABLED(CODE_M346)
          case 346: gcode_M346(); break;
        #endif
        #if ENABLED(CODE_M347)
          case 347: gcode_M347(); break;
        #endif
        #if ENABLED(CODE_M348)
          case 348: gcode_M348(); break;
        #endif
        #if ENABLED(CODE_M349)
          case 349: gcode_M349(); break;
        #endif
        #if ENABLED(CODE_M350)
          case 350: gcode_M350(); break;
        #endif
        #if ENABLED(CODE_M351)
          case 351: gcode_M351(); break;
        #endif
        #if ENABLED(CODE_M352)
          case 352: gcode_M352(); break;
        #endif
        #if ENABLED(CODE_M353)
          case 353: gcode_M353(); break;
        #endif
        #if ENABLED(CODE_M354)
          case 354: gcode_M354(); break;
        #endif
        #if ENABLED(CODE_M355)
          case 355: gcode_M355(); break;
        #endif
        #if ENABLED(CODE_M356)
          case 356: gcode_M356(); break;
        #endif
        #if ENABLED(CODE_M357)
          case 357: gcode_M357(); break;
        #endif
        #if ENABLED(CODE_M358)
          case 358: gcode_M358(); break;
        #endif
        #if ENABLED(CODE_M359)
          case 359: gcode_M359(); break;
        #endif
        #if ENABLED(CODE_M360)
          case 360: gcode_M360(); break;
        #endif
        #if ENABLED(CODE_M361)
          case 361: gcode_M361(); break;
        #endif
        #if ENABLED(CODE_M362)
          case 362: gcode_M362(); break;
        #endif
        #if ENABLED(CODE_M363)
          case 363: gcode_M363(); break;
        #endif
        #if ENABLED(CODE_M364)
          case 364: gcode_M364(); break;
        #endif
        #if ENABLED(CODE_M365)
          case 365: gcode_M365(); break;
        #endif
        #if ENABLED(CODE_M366)
          case 366: gcode_M366(); break;
        #endif
        #if ENABLED(CODE_M367)
          case 367: gcode_M367(); break;
        #endif
        #if ENABLED(CODE_M368)
          case 368: gcode_M368(); break;
        #endif
        #if ENABLED(CODE_M369)
          case 369: gcode_M369(); break;
        #endif
        #if ENABLED(CODE_M370)
          case 370: gcode_M370(); break;
        #endif
        #if ENABLED(CODE_M371)
          case 371: gcode_M371(); break;
        #endif
        #if ENABLED(CODE_M372)
          case 372: gcode_M372(); break;
        #endif
        #if ENABLED(CODE_M373)
          case 373: gcode_M373(); break;
        #endif
        #if ENABLED(CODE_M374)
          case 374: gcode_M374(); break;
        #endif
        #if ENABLED(CODE_M375)
          case 375: gcode_M375(); break;
        #endif
        #if ENABLED(CODE_M376)
          case 376: gcode_M376(); break;
        #endif
        #if ENABLED(CODE_M377)
          case 377: gcode_M377(); break;
        #endif
        #if ENABLED(CODE_M378)
          case 378: gcode_M378(); break;
        #endif
        #if ENABLED(CODE_M379)
          case 379: gcode_M379(); break;
        #endif
        #if ENABLED(CODE_M380)
          case 380: gcode_M380(); break;
        #endif
        #if ENABLED(CODE_M381)
          case 381: gcode_M381(); break;
        #endif
        #if ENABLED(CODE_M382)
          case 382: gcode_M382(); break;
        #endif
        #if ENABLED(CODE_M383)
          case 383: gcode_M383(); break;
        #endif
        #if ENABLED(CODE_M384)
          case 384: gcode_M384(); break;
        #endif
        #if ENABLED(CODE_M385)
          case 385: gcode_M385(); break;
        #endif
        #if ENABLED(CODE_M386)
          case 386: gcode_M386(); break;
        #endif
        #if ENABLED(CODE_M387)
          case 387: gcode_M387(); break;
        #endif
        #if ENABLED(CODE_M388)
          case 388: gcode_M388(); break;
        #endif
        #if ENABLED(CODE_M389)
          case 389: gcode_M389(); break;
        #endif
        #if ENABLED(CODE_M390)
          case 390: gcode_M390(); break;
        #endif
        #if ENABLED(CODE_M391)
          case 391: gcode_M391(); break;
        #endif
        #if ENABLED(CODE_M392)
          case 392: gcode_M392(); break;
        #endif
        #if ENABLED(CODE_M393)
          case 393: gcode_M393(); break;
        #endif
        #if ENABLED(CODE_M394)
          case 394: gcode_M394(); break;
        #endif
        #if ENABLED(CODE_M395)
          case 395: gcode_M395(); break;
        #endif
        #if ENABLED(CODE_M396)
          case 396: gcode_M396(); break;
        #endif
        #if ENABLED(CODE_M397)
          case 397: gcode_M397(); break;
        #endif
        #if ENABLED(CODE_M398)
          case 398: gcode_M398(); break;
        #endif
        #if ENABLED(CODE_M399)
          case 399: gcode_M399(); break;
        #endif
        #if ENABLED(CODE_M400)
          case 400: gcode_M400(); break;
        #endif
        #if ENABLED(CODE_M401)
          case 401: gcode_M401(); break;
        #endif
        #if ENABLED(CODE_M402)
          case 402: gcode_M402(); break;
        #endif
        #if ENABLED(CODE_M403)
          case 403: gcode_M403(); break;
        #endif
        #if ENABLED(CODE_M404)
          case 404: gcode_M404(); break;
        #endif
        #if ENABLED(CODE_M405)
          case 405: gcode_M405(); break;
        #endif
        #if ENABLED(CODE_M406)
          case 406: gcode_M406(); break;
        #endif
        #if ENABLED(CODE_M407)
          case 407: gcode_M407(); break;
        #endif
        #if ENABLED(CODE_M408)
          case 408: gcode_M408(); break;
        #endif
        #if ENABLED(CODE_M409)
          case 409: gcode_M409(); break;
        #endif
        #if ENABLED(CODE_M410)
          case 410: gcode_M410(); break;
        #endif
        #if ENABLED(CODE_M411)
          case 411: gcode_M411(); break;
        #endif
        #if ENABLED(CODE_M412)
          case 412: gcode_M412(); break;
        #endif
        #if ENABLED(CODE_M413)
          case 413: gcode_M413(); break;
        #endif
        #if ENABLED(CODE_M414)
          case 414: gcode_M414(); break;
        #endif
        #if ENABLED(CODE_M415)
          case 415: gcode_M415(); break;
        #endif
        #if ENABLED(CODE_M416)
          case 416: gcode_M416(); break;
        #endif
        #if ENABLED(CODE_M417)
          case 417: gcode_M417(); break;
        #endif
        #if ENABLED(CODE_M418)
          case 418: gcode_M418(); break;
        #endif
        #if ENABLED(CODE_M419)
          case 419: gcode_M419(); break;
        #endif
        #if ENABLED(CODE_M420)
          case 420: gcode_M420(); break;
        #endif
        #if ENABLED(CODE_M421)
          case 421: gcode_M421(); break;
        #endif
        #if ENABLED(CODE_M422)
          case 422: gcode_M422(); break;
        #endif
        #if ENABLED(CODE_M423)
          case 423: gcode_M423(); break;
        #endif
        #if ENABLED(CODE_M424)
          case 424: gcode_M424(); break;
        #endif
        #if ENABLED(CODE_M425)
          case 425: gcode_M425(); break;
        #endif
        #if ENABLED(CODE_M426)
          case 426: gcode_M426(); break;
        #endif
        #if ENABLED(CODE_M427)
          case 427: gcode_M427(); break;
        #endif
        #if ENABLED(CODE_M428)
          case 428: gcode_M428(); break;
        #endif
        #if ENABLED(CODE_M429)
          case 429: gcode_M429(); break;
        #endif
        #if ENABLED(CODE_M430)
          case 430: gcode_M430(); break;
        #endif
        #if ENABLED(CODE_M431)
          case 431: gcode_M431(); break;
        #endif
        #if ENABLED(CODE_M432)
          case 432: gcode_M432(); break;
        #endif
        #if ENABLED(CODE_M433)
          case 433: gcode_M433(); break;
        #endif
        #if ENABLED(CODE_M434)
          case 434: gcode_M434(); break;
        #endif
        #if ENABLED(CODE_M435)
          case 435: gcode_M435(); break;
        #endif
        #if ENABLED(CODE_M436)
          case 436: gcode_M436(); break;
        #endif
        #if ENABLED(CODE_M437)
          case 437: gcode_M437(); break;
        #endif
        #if ENABLED(CODE_M438)
          case 438: gcode_M438(); break;
        #endif
        #if ENABLED(CODE_M439)
          case 439: gcode_M439(); break;
        #endif
        #if ENABLED(CODE_M440)
          case 440: gcode_M440(); break;
        #endif
        #if ENABLED(CODE_M441)
          case 441: gcode_M441(); break;
        #endif
        #if ENABLED(CODE_M442)
          case 442: gcode_M442(); break;
        #endif
        #if ENABLED(CODE_M443)
          case 443: gcode_M443(); break;
        #endif
        #if ENABLED(CODE_M444)
          case 444: gcode_M444(); break;
        #endif
        #if ENABLED(CODE_M445)
          case 445: gcode_M445(); break;
        #endif
        #if ENABLED(CODE_M446)
          case 446: gcode_M446(); break;
        #endif
        #if ENABLED(CODE_M447)
          case 447: gcode_M447(); break;
        #endif
        #if ENABLED(CODE_M448)
          case 448: gcode_M448(); break;
        #endif
        #if ENABLED(CODE_M449)
          case 449: gcode_M449(); break;
        #endif
        #if ENABLED(CODE_M450)
          case 450: gcode_M450(); break;
        #endif
        #if ENABLED(CODE_M451)
          case 451: gcode_M451(); break;
        #endif
        #if ENABLED(CODE_M452)
          case 452: gcode_M452(); break;
        #endif
        #if ENABLED(CODE_M453)
          case 453: gcode_M453(); break;
        #endif
        #if ENABLED(CODE_M454)
          case 454: gcode_M454(); break;
        #endif
        #if ENABLED(CODE_M455)
          case 455: gcode_M455(); break;
        #endif
        #if ENABLED(CODE_M456)
          case 456: gcode_M456(); break;
        #endif
        #if ENABLED(CODE_M457)
          case 457: gcode_M457(); break;
        #endif
        #if ENABLED(CODE_M458)
          case 458: gcode_M458(); break;
        #endif
        #if ENABLED(CODE_M459)
          case 459: gcode_M459(); break;
        #endif
        #if ENABLED(CODE_M460)
          case 460: gcode_M460(); break;
        #endif
        #if ENABLED(CODE_M461)
          case 461: gcode_M461(); break;
        #endif
        #if ENABLED(CODE_M462)
          case 462: gcode_M462(); break;
        #endif
        #if ENABLED(CODE_M463)
          case 463: gcode_M463(); break;
        #endif
        #if ENABLED(CODE_M464)
          case 464: gcode_M464(); break;
        #endif
        #if ENABLED(CODE_M465)
          case 465: gcode_M465(); break;
        #endif
        #if ENABLED(CODE_M466)
          case 466: gcode_M466(); break;
        #endif
        #if ENABLED(CODE_M467)
          case 467: gcode_M467(); break;
        #endif
        #if ENABLED(CODE_M468)
          case 468: gcode_M468(); break;
        #endif
        #if ENABLED(CODE_M469)
          case 469: gcode_M469(); break;
        #endif
        #if ENABLED(CODE_M470)
          case 470: gcode_M470(); break;
        #endif
        #if ENABLED(CODE_M471)
          case 471: gcode_M471(); break;
        #endif
        #if ENABLED(CODE_M472)
          case 472: gcode_M472(); break;
        #endif
        #if ENABLED(CODE_M473)
          case 473: gcode_M473(); break;
        #endif
        #if ENABLED(CODE_M474)
          case 474: gcode_M474(); break;
        #endif
        #if ENABLED(CODE_M475)
          case 475: gcode_M475(); break;
        #endif
        #if ENABLED(CODE_M476)
          case 476: gcode_M476(); break;
        #endif
        #if ENABLED(CODE_M477)
          case 477: gcode_M477(); break;
        #endif
        #if ENABLED(CODE_M478)
          case 478: gcode_M478(); break;
        #endif
        #if ENABLED(CODE_M479)
          case 479: gcode_M479(); break;
        #endif
        #if ENABLED(CODE_M480)
          case 480: gcode_M480(); break;
        #endif
        #if ENABLED(CODE_M481)
          case 481: gcode_M481(); break;
        #endif
        #if ENABLED(CODE_M482)
          case 482: gcode_M482(); break;
        #endif
        #if ENABLED(CODE_M483)
          case 483: gcode_M483(); break;
        #endif
        #if ENABLED(CODE_M484)
          case 484: gcode_M484(); break;
        #endif
        #if ENABLED(CODE_M485)
          case 485: gcode_M485(); break;
        #endif
        #if ENABLED(CODE_M486)
          case 486: gcode_M486(); break;
        #endif
        #if ENABLED(CODE_M487)
          case 487: gcode_M487(); break;
        #endif
        #if ENABLED(CODE_M488)
          case 488: gcode_M488(); break;
        #endif
        #if ENABLED(CODE_M489)
          case 489: gcode_M489(); break;
        #endif
        #if ENABLED(CODE_M490)
          case 490: gcode_M490(); break;
        #endif
        #if ENABLED(CODE_M491)
          case 491: gcode_M491(); break;
        #endif
        #if ENABLED(CODE_M492)
          case 492: gcode_M492(); break;
        #endif
        #if ENABLED(CODE_M493)
          case 493: gcode_M493(); break;
        #endif
        #if ENABLED(CODE_M494)
          case 494: gcode_M494(); break;
        #endif
        #if ENABLED(CODE_M495)
          case 495: gcode_M495(); break;
        #endif
        #if ENABLED(CODE_M496)
          case 496: gcode_M496(); break;
        #endif
        #if ENABLED(CODE_M497)
          case 497: gcode_M497(); break;
        #endif
        #if ENABLED(CODE_M498)
          case 498: gcode_M498(); break;
        #endif
        #if ENABLED(CODE_M499)
          case 499: gcode_M499(); break;
        #endif
        #if ENABLED(CODE_M500)
          case 500: gcode_M500(); break;
        #endif
        #if ENABLED(CODE_M501)
          case 501: gcode_M501(); break;
        #endif
        #if ENABLED(CODE_M502)
          case 502: gcode_M502(); break;
        #endif
        #if ENABLED(CODE_M503)
          case 503: gcode_M503(); break;
        #endif
        #if ENABLED(CODE_M504)
          case 504: gcode_M504(); break;
        #endif
        #if ENABLED(CODE_M505)
          case 505: gcode_M505(); break;
        #endif
        #if ENABLED(CODE_M506)
          case 506: gcode_M506(); break;
        #endif
        #if ENABLED(CODE_M507)
          case 507: gcode_M507(); break;
        #endif
        #if ENABLED(CODE_M508)
          case 508: gcode_M508(); break;
        #endif
        #if ENABLED(CODE_M509)
          case 509: gcode_M509(); break;
        #endif
        #if ENABLED(CODE_M510)
          case 510: gcode_M510(); break;
        #endif
        #if ENABLED(CODE_M511)
          case 511: gcode_M511(); break;
        #endif
        #if ENABLED(CODE_M512)
          case 512: gcode_M512(); break;
        #endif
        #if ENABLED(CODE_M513)
          case 513: gcode_M513(); break;
        #endif
        #if ENABLED(CODE_M514)
          case 514: gcode_M514(); break;
        #endif
        #if ENABLED(CODE_M515)
          case 515: gcode_M515(); break;
        #endif
        #if ENABLED(CODE_M516)
          case 516: gcode_M516(); break;
        #endif
        #if ENABLED(CODE_M517)
          case 517: gcode_M517(); break;
        #endif
        #if ENABLED(CODE_M518)
          case 518: gcode_M518(); break;
        #endif
        #if ENABLED(CODE_M519)
          case 519: gcode_M519(); break;
        #endif
        #if ENABLED(CODE_M520)
          case 520: gcode_M520(); break;
        #endif
        #if ENABLED(CODE_M521)
          case 521: gcode_M521(); break;
        #endif
        #if ENABLED(CODE_M522)
          case 522: gcode_M522(); break;
        #endif
        #if ENABLED(CODE_M523)
          case 523: gcode_M523(); break;
        #endif
        #if ENABLED(CODE_M524)
          case 524: gcode_M524(); break;
        #endif
        #if ENABLED(CODE_M525)
          case 525: gcode_M525(); break;
        #endif
        #if ENABLED(CODE_M526)
          case 526: gcode_M526(); break;
        #endif
        #if ENABLED(CODE_M527)
          case 527: gcode_M527(); break;
        #endif
        #if ENABLED(CODE_M528)
          case 528: gcode_M528(); break;
        #endif
        #if ENABLED(CODE_M529)
          case 529: gcode_M529(); break;
        #endif
        #if ENABLED(CODE_M530)
          case 530: gcode_M530(); break;
        #endif
        #if ENABLED(CODE_M531)
          case 531: gcode_M531(); break;
        #endif
        #if ENABLED(CODE_M532)
          case 532: gcode_M532(); break;
        #endif
        #if ENABLED(CODE_M533)
          case 533: gcode_M533(); break;
        #endif
        #if ENABLED(CODE_M534)
          case 534: gcode_M534(); break;
        #endif
        #if ENABLED(CODE_M535)
          case 535: gcode_M535(); break;
        #endif
        #if ENABLED(CODE_M536)
          case 536: gcode_M536(); break;
        #endif
        #if ENABLED(CODE_M537)
          case 537: gcode_M537(); break;
        #endif
        #if ENABLED(CODE_M538)
          case 538: gcode_M538(); break;
        #endif
        #if ENABLED(CODE_M539)
          case 539: gcode_M539(); break;
        #endif
        #if ENABLED(CODE_M540)
          case 540: gcode_M540(); break;
        #endif
        #if ENABLED(CODE_M541)
          case 541: gcode_M541(); break;
        #endif
        #if ENABLED(CODE_M542)
          case 542: gcode_M542(); break;
        #endif
        #if ENABLED(CODE_M543)
          case 543: gcode_M543(); break;
        #endif
        #if ENABLED(CODE_M544)
          case 544: gcode_M544(); break;
        #endif
        #if ENABLED(CODE_M545)
          case 545: gcode_M545(); break;
        #endif
        #if ENABLED(CODE_M546)
          case 546: gcode_M546(); break;
        #endif
        #if ENABLED(CODE_M547)
          case 547: gcode_M547(); break;
        #endif
        #if ENABLED(CODE_M548)
          case 548: gcode_M548(); break;
        #endif
        #if ENABLED(CODE_M549)
          case 549: gcode_M549(); break;
        #endif
        #if ENABLED(CODE_M550)
          case 550: gcode_M550(); break;
        #endif
        #if ENABLED(CODE_M551)
          case 551: gcode_M551(); break;
        #endif
        #if ENABLED(CODE_M552)
          case 552: gcode_M552(); break;
        #endif
        #if ENABLED(CODE_M553)
          case 553: gcode_M553(); break;
        #endif
        #if ENABLED(CODE_M554)
          case 554: gcode_M554(); break;
        #endif
        #if ENABLED(CODE_M555)
          case 555: gcode_M555(); break;
        #endif
        #if ENABLED(CODE_M556)
          case 556: gcode_M556(); break;
        #endif
        #if ENABLED(CODE_M557)
          case 557: gcode_M557(); break;
        #endif
        #if ENABLED(CODE_M558)
          case 558: gcode_M558(); break;
        #endif
        #if ENABLED(CODE_M559)
          case 559: gcode_M559(); break;
        #endif
        #if ENABLED(CODE_M560)
          case 560: gcode_M560(); break;
        #endif
        #if ENABLED(CODE_M561)
          case 561: gcode_M561(); break;
        #endif
        #if ENABLED(CODE_M562)
          case 562: gcode_M562(); break;
        #endif
        #if ENABLED(CODE_M563)
          case 563: gcode_M563(); break;
        #endif
        #if ENABLED(CODE_M564)
          case 564: gcode_M564(); break;
        #endif
        #if ENABLED(CODE_M565)
          case 565: gcode_M565(); break;
        #endif
        #if ENABLED(CODE_M566)
          case 566: gcode_M566(); break;
        #endif
        #if ENABLED(CODE_M567)
          case 567: gcode_M567(); break;
        #endif
        #if ENABLED(CODE_M568)
          case 568: gcode_M568(); break;
        #endif
        #if ENABLED(CODE_M569)
          case 569: gcode_M569(); break;
        #endif
        #if ENABLED(CODE_M570)
          case 570: gcode_M570(); break;
        #endif
        #if ENABLED(CODE_M571)
          case 571: gcode_M571(); break;
        #endif
        #if ENABLED(CODE_M572)
          case 572: gcode_M572(); break;
        #endif
        #if ENABLED(CODE_M573)
          case 573: gcode_M573(); break;
        #endif
        #if ENABLED(CODE_M574)
          case 574: gcode_M574(); break;
        #endif
        #if ENABLED(CODE_M575)
          case 575: gcode_M575(); break;
        #endif
        #if ENABLED(CODE_M576)
          case 576: gcode_M576(); break;
        #endif
        #if ENABLED(CODE_M577)
          case 577: gcode_M577(); break;
        #endif
        #if ENABLED(CODE_M578)
          case 578: gcode_M578(); break;
        #endif
        #if ENABLED(CODE_M579)
          case 579: gcode_M579(); break;
        #endif
        #if ENABLED(CODE_M580)
          case 580: gcode_M580(); break;
        #endif
        #if ENABLED(CODE_M581)
          case 581: gcode_M581(); break;
        #endif
        #if ENABLED(CODE_M582)
          case 582: gcode_M582(); break;
        #endif
        #if ENABLED(CODE_M583)
          case 583: gcode_M583(); break;
        #endif
        #if ENABLED(CODE_M584)
          case 584: gcode_M584(); break;
        #endif
        #if ENABLED(CODE_M585)
          case 585: gcode_M585(); break;
        #endif
        #if ENABLED(CODE_M586)
          case 586: gcode_M586(); break;
        #endif
        #if ENABLED(CODE_M587)
          case 587: gcode_M587(); break;
        #endif
        #if ENABLED(CODE_M588)
          case 588: gcode_M588(); break;
        #endif
        #if ENABLED(CODE_M589)
          case 589: gcode_M589(); break;
        #endif
        #if ENABLED(CODE_M590)
          case 590: gcode_M590(); break;
        #endif
        #if ENABLED(CODE_M591)
          case 591: gcode_M591(); break;
        #endif
        #if ENABLED(CODE_M592)
          case 592: gcode_M592(); break;
        #endif
        #if ENABLED(CODE_M593)
          case 593: gcode_M593(); break;
        #endif
        #if ENABLED(CODE_M594)
          case 594: gcode_M594(); break;
        #endif
        #if ENABLED(CODE_M595)
          case 595: gcode_M595(); break;
        #endif
        #if ENABLED(CODE_M596)
          case 596: gcode_M596(); break;
        #endif
        #if ENABLED(CODE_M597)
          case 597: gcode_M597(); break;
        #endif
        #if ENABLED(CODE_M598)
          case 598: gcode_M598(); break;
        #endif
        #if ENABLED(CODE_M599)
          case 599: gcode_M599(); break;
        #endif
        #if ENABLED(CODE_M600)
          case 600: gcode_M600(); break;
        #endif
        #if ENABLED(CODE_M601)
          case 601: gcode_M601(); break;
        #endif
        #if ENABLED(CODE_M602)
          case 602: gcode_M602(); break;
        #endif
        #if ENABLED(CODE_M603)
          case 603: gcode_M603(); break;
        #endif
        #if ENABLED(CODE_M604)
          case 604: gcode_M604(); break;
        #endif
        #if ENABLED(CODE_M605)
          case 605: gcode_M605(); break;
        #endif
        #if ENABLED(CODE_M606)
          case 606: gcode_M606(); break;
        #endif
        #if ENABLED(CODE_M607)
          case 607: gcode_M607(); break;
        #endif
        #if ENABLED(CODE_M608)
          case 608: gcode_M608(); break;
        #endif
        #if ENABLED(CODE_M609)
          case 609: gcode_M609(); break;
        #endif
        #if ENABLED(CODE_M610)
          case 610: gcode_M610(); break;
        #endif
        #if ENABLED(CODE_M611)
          case 611: gcode_M611(); break;
        #endif
        #if ENABLED(CODE_M612)
          case 612: gcode_M612(); break;
        #endif
        #if ENABLED(CODE_M613)
          case 613: gcode_M613(); break;
        #endif
        #if ENABLED(CODE_M614)
          case 614: gcode_M614(); break;
        #endif
        #if ENABLED(CODE_M615)
          case 615: gcode_M615(); break;
        #endif
        #if ENABLED(CODE_M616)
          case 616: gcode_M616(); break;
        #endif
        #if ENABLED(CODE_M617)
          case 617: gcode_M617(); break;
        #endif
        #if ENABLED(CODE_M618)
          case 618: gcode_M618(); break;
        #endif
        #if ENABLED(CODE_M619)
          case 619: gcode_M619(); break;
        #endif
        #if ENABLED(CODE_M620)
          case 620: gcode_M620(); break;
        #endif
        #if ENABLED(CODE_M621)
          case 621: gcode_M621(); break;
        #endif
        #if ENABLED(CODE_M622)
          case 622: gcode_M622(); break;
        #endif
        #if ENABLED(CODE_M623)
          case 623: gcode_M623(); break;
        #endif
        #if ENABLED(CODE_M624)
          case 624: gcode_M624(); break;
        #endif
        #if ENABLED(CODE_M625)
          case 625: gcode_M625(); break;
        #endif
        #if ENABLED(CODE_M626)
          case 626: gcode_M626(); break;
        #endif
        #if ENABLED(CODE_M627)
          case 627: gcode_M627(); break;
        #endif
        #if ENABLED(CODE_M628)
          case 628: gcode_M628(); break;
        #endif
        #if ENABLED(CODE_M629)
          case 629: gcode_M629(); break;
        #endif
        #if ENABLED(CODE_M630)
          case 630: gcode_M630(); break;
        #endif
        #if ENABLED(CODE_M631)
          case 631: gcode_M631(); break;
        #endif
        #if ENABLED(CODE_M632)
          case 632: gcode_M632(); break;
        #endif
        #if ENABLED(CODE_M633)
          case 633: gcode_M633(); break;
        #endif
        #if ENABLED(CODE_M634)
          case 634: gcode_M634(); break;
        #endif
        #if ENABLED(CODE_M635)
          case 635: gcode_M635(); break;
        #endif
        #if ENABLED(CODE_M636)
          case 636: gcode_M636(); break;
        #endif
        #if ENABLED(CODE_M637)
          case 637: gcode_M637(); break;
        #endif
        #if ENABLED(CODE_M638)
          case 638: gcode_M638(); break;
        #endif
        #if ENABLED(CODE_M639)
          case 639: gcode_M639(); break;
        #endif
        #if ENABLED(CODE_M640)
          case 640: gcode_M640(); break;
        #endif
        #if ENABLED(CODE_M641)
          case 641: gcode_M641(); break;
        #endif
        #if ENABLED(CODE_M642)
          case 642: gcode_M642(); break;
        #endif
        #if ENABLED(CODE_M643)
          case 643: gcode_M643(); break;
        #endif
        #if ENABLED(CODE_M644)
          case 644: gcode_M644(); break;
        #endif
        #if ENABLED(CODE_M645)
          case 645: gcode_M645(); break;
        #endif
        #if ENABLED(CODE_M646)
          case 646: gcode_M646(); break;
        #endif
        #if ENABLED(CODE_M647)
          case 647: gcode_M647(); break;
        #endif
        #if ENABLED(CODE_M648)
          case 648: gcode_M648(); break;
        #endif
        #if ENABLED(CODE_M649)
          case 649: gcode_M649(); break;
        #endif
        #if ENABLED(CODE_M650)
          case 650: gcode_M650(); break;
        #endif
        #if ENABLED(CODE_M651)
          case 651: gcode_M651(); break;
        #endif
        #if ENABLED(CODE_M652)
          case 652: gcode_M652(); break;
        #endif
        #if ENABLED(CODE_M653)
          case 653: gcode_M653(); break;
        #endif
        #if ENABLED(CODE_M654)
          case 654: gcode_M654(); break;
        #endif
        #if ENABLED(CODE_M655)
          case 655: gcode_M655(); break;
        #endif
        #if ENABLED(CODE_M656)
          case 656: gcode_M656(); break;
        #endif
        #if ENABLED(CODE_M657)
          case 657: gcode_M657(); break;
        #endif
        #if ENABLED(CODE_M658)
          case 658: gcode_M658(); break;
        #endif
        #if ENABLED(CODE_M659)
          case 659: gcode_M659(); break;
        #endif
        #if ENABLED(CODE_M660)
          case 660: gcode_M660(); break;
        #endif
        #if ENABLED(CODE_M661)
          case 661: gcode_M661(); break;
        #endif
        #if ENABLED(CODE_M662)
          case 662: gcode_M662(); break;
        #endif
        #if ENABLED(CODE_M663)
          case 663: gcode_M663(); break;
        #endif
        #if ENABLED(CODE_M664)
          case 664: gcode_M664(); break;
        #endif
        #if ENABLED(CODE_M665)
          case 665: gcode_M665(); break;
        #endif
        #if ENABLED(CODE_M666)
          case 666: gcode_M666(); break;
        #endif
        #if ENABLED(CODE_M667)
          case 667: gcode_M667(); break;
        #endif
        #if ENABLED(CODE_M668)
          case 668: gcode_M668(); break;
        #endif
        #if ENABLED(CODE_M669)
          case 669: gcode_M669(); break;
        #endif
        #if ENABLED(CODE_M670)
          case 670: gcode_M670(); break;
        #endif
        #if ENABLED(CODE_M671)
          case 671: gcode_M671(); break;
        #endif
        #if ENABLED(CODE_M672)
          case 672: gcode_M672(); break;
        #endif
        #if ENABLED(CODE_M673)
          case 673: gcode_M673(); break;
        #endif
        #if ENABLED(CODE_M674)
          case 674: gcode_M674(); break;
        #endif
        #if ENABLED(CODE_M675)
          case 675: gcode_M675(); break;
        #endif
        #if ENABLED(CODE_M676)
          case 676: gcode_M676(); break;
        #endif
        #if ENABLED(CODE_M677)
          case 677: gcode_M677(); break;
        #endif
        #if ENABLED(CODE_M678)
          case 678: gcode_M678(); break;
        #endif
        #if ENABLED(CODE_M679)
          case 679: gcode_M679(); break;
        #endif
        #if ENABLED(CODE_M680)
          case 680: gcode_M680(); break;
        #endif
        #if ENABLED(CODE_M681)
          case 681: gcode_M681(); break;
        #endif
        #if ENABLED(CODE_M682)
          case 682: gcode_M682(); break;
        #endif
        #if ENABLED(CODE_M683)
          case 683: gcode_M683(); break;
        #endif
        #if ENABLED(CODE_M684)
          case 684: gcode_M684(); break;
        #endif
        #if ENABLED(CODE_M685)
          case 685: gcode_M685(); break;
        #endif
        #if ENABLED(CODE_M686)
          case 686: gcode_M686(); break;
        #endif
        #if ENABLED(CODE_M687)
          case 687: gcode_M687(); break;
        #endif
        #if ENABLED(CODE_M688)
          case 688: gcode_M688(); break;
        #endif
        #if ENABLED(CODE_M689)
          case 689: gcode_M689(); break;
        #endif
        #if ENABLED(CODE_M690)
          case 690: gcode_M690(); break;
        #endif
        #if ENABLED(CODE_M691)
          case 691: gcode_M691(); break;
        #endif
        #if ENABLED(CODE_M692)
          case 692: gcode_M692(); break;
        #endif
        #if ENABLED(CODE_M693)
          case 693: gcode_M693(); break;
        #endif
        #if ENABLED(CODE_M694)
          case 694: gcode_M694(); break;
        #endif
        #if ENABLED(CODE_M695)
          case 695: gcode_M695(); break;
        #endif
        #if ENABLED(CODE_M696)
          case 696: gcode_M696(); break;
        #endif
        #if ENABLED(CODE_M697)
          case 697: gcode_M697(); break;
        #endif
        #if ENABLED(CODE_M698)
          case 698: gcode_M698(); break;
        #endif
        #if ENABLED(CODE_M699)
          case 699: gcode_M699(); break;
        #endif
        #if ENABLED(CODE_M700)
          case 700: gcode_M700(); break;
        #endif
        #if ENABLED(CODE_M701)
          case 701: gcode_M701(); break;
        #endif
        #if ENABLED(CODE_M702)
          case 702: gcode_M702(); break;
        #endif
        #if ENABLED(CODE_M703)
          case 703: gcode_M703(); break;
        #endif
        #if ENABLED(CODE_M704)
          case 704: gcode_M704(); break;
        #endif
        #if ENABLED(CODE_M705)
          case 705: gcode_M705(); break;
        #endif
        #if ENABLED(CODE_M706)
          case 706: gcode_M706(); break;
        #endif
        #if ENABLED(CODE_M707)
          case 707: gcode_M707(); break;
        #endif
        #if ENABLED(CODE_M708)
          case 708: gcode_M708(); break;
        #endif
        #if ENABLED(CODE_M709)
          case 709: gcode_M709(); break;
        #endif
        #if ENABLED(CODE_M710)
          case 710: gcode_M710(); break;
        #endif
        #if ENABLED(CODE_M711)
          case 711: gcode_M711(); break;
        #endif
        #if ENABLED(CODE_M712)
          case 712: gcode_M712(); break;
        #endif
        #if ENABLED(CODE_M713)
          case 713: gcode_M713(); break;
        #endif
        #if ENABLED(CODE_M714)
          case 714: gcode_M714(); break;
        #endif
        #if ENABLED(CODE_M715)
          case 715: gcode_M715(); break;
        #endif
        #if ENABLED(CODE_M716)
          case 716: gcode_M716(); break;
        #endif
        #if ENABLED(CODE_M717)
          case 717: gcode_M717(); break;
        #endif
        #if ENABLED(CODE_M718)
          case 718: gcode_M718(); break;
        #endif
        #if ENABLED(CODE_M719)
          case 719: gcode_M719(); break;
        #endif
        #if ENABLED(CODE_M720)
          case 720: gcode_M720(); break;
        #endif
        #if ENABLED(CODE_M721)
          case 721: gcode_M721(); break;
        #endif
        #if ENABLED(CODE_M722)
          case 722: gcode_M722(); break;
        #endif
        #if ENABLED(CODE_M723)
          case 723: gcode_M723(); break;
        #endif
        #if ENABLED(CODE_M724)
          case 724: gcode_M724(); break;
        #endif
        #if ENABLED(CODE_M725)
          case 725: gcode_M725(); break;
        #endif
        #if ENABLED(CODE_M726)
          case 726: gcode_M726(); break;
        #endif
        #if ENABLED(CODE_M727)
          case 727: gcode_M727(); break;
        #endif
        #if ENABLED(CODE_M728)
          case 728: gcode_M728(); break;
        #endif
        #if ENABLED(CODE_M729)
          case 729: gcode_M729(); break;
        #endif
        #if ENABLED(CODE_M730)
          case 730: gcode_M730(); break;
        #endif
        #if ENABLED(CODE_M731)
          case 731: gcode_M731(); break;
        #endif
        #if ENABLED(CODE_M732)
          case 732: gcode_M732(); break;
        #endif
        #if ENABLED(CODE_M733)
          case 733: gcode_M733(); break;
        #endif
        #if ENABLED(CODE_M734)
          case 734: gcode_M734(); break;
        #endif
        #if ENABLED(CODE_M735)
          case 735: gcode_M735(); break;
        #endif
        #if ENABLED(CODE_M736)
          case 736: gcode_M736(); break;
        #endif
        #if ENABLED(CODE_M737)
          case 737: gcode_M737(); break;
        #endif
        #if ENABLED(CODE_M738)
          case 738: gcode_M738(); break;
        #endif
        #if ENABLED(CODE_M739)
          case 739: gcode_M739(); break;
        #endif
        #if ENABLED(CODE_M740)
          case 740: gcode_M740(); break;
        #endif
        #if ENABLED(CODE_M741)
          case 741: gcode_M741(); break;
        #endif
        #if ENABLED(CODE_M742)
          case 742: gcode_M742(); break;
        #endif
        #if ENABLED(CODE_M743)
          case 743: gcode_M743(); break;
        #endif
        #if ENABLED(CODE_M744)
          case 744: gcode_M744(); break;
        #endif
        #if ENABLED(CODE_M745)
          case 745: gcode_M745(); break;
        #endif
        #if ENABLED(CODE_M746)
          case 746: gcode_M746(); break;
        #endif
        #if ENABLED(CODE_M747)
          case 747: gcode_M747(); break;
        #endif
        #if ENABLED(CODE_M748)
          case 748: gcode_M748(); break;
        #endif
        #if ENABLED(CODE_M749)
          case 749: gcode_M749(); break;
        #endif
        #if ENABLED(CODE_M750)
          case 750: gcode_M750(); break;
        #endif
        #if ENABLED(CODE_M751)
          case 751: gcode_M751(); break;
        #endif
        #if ENABLED(CODE_M752)
          case 752: gcode_M752(); break;
        #endif
        #if ENABLED(CODE_M753)
          case 753: gcode_M753(); break;
        #endif
        #if ENABLED(CODE_M754)
          case 754: gcode_M754(); break;
        #endif
        #if ENABLED(CODE_M755)
          case 755: gcode_M755(); break;
        #endif
        #if ENABLED(CODE_M756)
          case 756: gcode_M756(); break;
        #endif
        #if ENABLED(CODE_M757)
          case 757: gcode_M757(); break;
        #endif
        #if ENABLED(CODE_M758)
          case 758: gcode_M758(); break;
        #endif
        #if ENABLED(CODE_M759)
          case 759: gcode_M759(); break;
        #endif
        #if ENABLED(CODE_M760)
          case 760: gcode_M760(); break;
        #endif
        #if ENABLED(CODE_M761)
          case 761: gcode_M761(); break;
        #endif
        #if ENABLED(CODE_M762)
          case 762: gcode_M762(); break;
        #endif
        #if ENABLED(CODE_M763)
          case 763: gcode_M763(); break;
        #endif
        #if ENABLED(CODE_M764)
          case 764: gcode_M764(); break;
        #endif
        #if ENABLED(CODE_M765)
          case 765: gcode_M765(); break;
        #endif
        #if ENABLED(CODE_M766)
          case 766: gcode_M766(); break;
        #endif
        #if ENABLED(CODE_M767)
          case 767: gcode_M767(); break;
        #endif
        #if ENABLED(CODE_M768)
          case 768: gcode_M768(); break;
        #endif
        #if ENABLED(CODE_M769)
          case 769: gcode_M769(); break;
        #endif
        #if ENABLED(CODE_M770)
          case 770: gcode_M770(); break;
        #endif
        #if ENABLED(CODE_M771)
          case 771: gcode_M771(); break;
        #endif
        #if ENABLED(CODE_M772)
          case 772: gcode_M772(); break;
        #endif
        #if ENABLED(CODE_M773)
          case 773: gcode_M773(); break;
        #endif
        #if ENABLED(CODE_M774)
          case 774: gcode_M774(); break;
        #endif
        #if ENABLED(CODE_M775)
          case 775: gcode_M775(); break;
        #endif
        #if ENABLED(CODE_M776)
          case 776: gcode_M776(); break;
        #endif
        #if ENABLED(CODE_M777)
          case 777: gcode_M777(); break;
        #endif
        #if ENABLED(CODE_M778)
          case 778: gcode_M778(); break;
        #endif
        #if ENABLED(CODE_M779)
          case 779: gcode_M779(); break;
        #endif
        #if ENABLED(CODE_M780)
          case 780: gcode_M780(); break;
        #endif
        #if ENABLED(CODE_M781)
          case 781: gcode_M781(); break;
        #endif
        #if ENABLED(CODE_M782)
          case 782: gcode_M782(); break;
        #endif
        #if ENABLED(CODE_M783)
          case 783: gcode_M783(); break;
        #endif
        #if ENABLED(CODE_M784)
          case 784: gcode_M784(); break;
        #endif
        #if ENABLED(CODE_M785)
          case 785: gcode_M785(); break;
        #endif
        #if ENABLED(CODE_M786)
          case 786: gcode_M786(); break;
        #endif
        #if ENABLED(CODE_M787)
          case 787: gcode_M787(); break;
        #endif
        #if ENABLED(CODE_M788)
          case 788: gcode_M788(); break;
        #endif
        #if ENABLED(CODE_M789)
          case 789: gcode_M789(); break;
        #endif
        #if ENABLED(CODE_M790)
          case 790: gcode_M790(); break;
        #endif
        #if ENABLED(CODE_M791)
          case 791: gcode_M791(); break;
        #endif
        #if ENABLED(CODE_M792)
          case 792: gcode_M792(); break;
        #endif
        #if ENABLED(CODE_M793)
          case 793: gcode_M793(); break;
        #endif
        #if ENABLED(CODE_M794)
          case 794: gcode_M794(); break;
        #endif
        #if ENABLED(CODE_M795)
          case 795: gcode_M795(); break;
        #endif
        #if ENABLED(CODE_M796)
          case 796: gcode_M796(); break;
        #endif
        #if ENABLED(CODE_M797)
          case 797: gcode_M797(); break;
        #endif
        #if ENABLED(CODE_M798)
          case 798: gcode_M798(); break;
        #endif
        #if ENABLED(CODE_M799)
          case 799: gcode_M799(); break;
        #endif
        #if ENABLED(CODE_M800)
          case 800: gcode_M800(); break;
        #endif
        #if ENABLED(CODE_M801)
          case 801: gcode_M801(); break;
        #endif
        #if ENABLED(CODE_M802)
          case 802: gcode_M802(); break;
        #endif
        #if ENABLED(CODE_M803)
          case 803: gcode_M803(); break;
        #endif
        #if ENABLED(CODE_M804)
          case 804: gcode_M804(); break;
        #endif
        #if ENABLED(CODE_M805)
          case 805: gcode_M805(); break;
        #endif
        #if ENABLED(CODE_M806)
          case 806: gcode_M806(); break;
        #endif
        #if ENABLED(CODE_M807)
          case 807: gcode_M807(); break;
        #endif
        #if ENABLED(CODE_M808)
          case 808: gcode_M808(); break;
        #endif
        #if ENABLED(CODE_M809)
          case 809: gcode_M809(); break;
        #endif
        #if ENABLED(CODE_M810)
          case 810: gcode_M810(); break;
        #endif
        #if ENABLED(CODE_M811)
          case 811: gcode_M811(); break;
        #endif
        #if ENABLED(CODE_M812)
          case 812: gcode_M812(); break;
        #endif
        #if ENABLED(CODE_M813)
          case 813: gcode_M813(); break;
        #endif
        #if ENABLED(CODE_M814)
          case 814: gcode_M814(); break;
        #endif
        #if ENABLED(CODE_M815)
          case 815: gcode_M815(); break;
        #endif
        #if ENABLED(CODE_M816)
          case 816: gcode_M816(); break;
        #endif
        #if ENABLED(CODE_M817)
          case 817: gcode_M817(); break;
        #endif
        #if ENABLED(CODE_M818)
          case 818: gcode_M818(); break;
        #endif
        #if ENABLED(CODE_M819)
          case 819: gcode_M819(); break;
        #endif
        #if ENABLED(CODE_M820)
          case 820: gcode_M820(); break;
        #endif
        #if ENABLED(CODE_M821)
          case 821: gcode_M821(); break;
        #endif
        #if ENABLED(CODE_M822)
          case 822: gcode_M822(); break;
        #endif
        #if ENABLED(CODE_M823)
          case 823: gcode_M823(); break;
        #endif
        #if ENABLED(CODE_M824)
          case 824: gcode_M824(); break;
        #endif
        #if ENABLED(CODE_M825)
          case 825: gcode_M825(); break;
        #endif
        #if ENABLED(CODE_M826)
          case 826: gcode_M826(); break;
        #endif
        #if ENABLED(CODE_M827)
          case 827: gcode_M827(); break;
        #endif
        #if ENABLED(CODE_M828)
          case 828: gcode_M828(); break;
        #endif
        #if ENABLED(CODE_M829)
          case 829: gcode_M829(); break;
        #endif
        #if ENABLED(CODE_M830)
          case 830: gcode_M830(); break;
        #endif
        #if ENABLED(CODE_M831)
          case 831: gcode_M831(); break;
        #endif
        #if ENABLED(CODE_M832)
          case 832: gcode_M832(); break;
        #endif
        #if ENABLED(CODE_M833)
          case 833: gcode_M833(); break;
        #endif
        #if ENABLED(CODE_M834)
          case 834: gcode_M834(); break;
        #endif
        #if ENABLED(CODE_M835)
          case 835: gcode_M835(); break;
        #endif
        #if ENABLED(CODE_M836)
          case 836: gcode_M836(); break;
        #endif
        #if ENABLED(CODE_M837)
          case 837: gcode_M837(); break;
        #endif
        #if ENABLED(CODE_M838)
          case 838: gcode_M838(); break;
        #endif
        #if ENABLED(CODE_M839)
          case 839: gcode_M839(); break;
        #endif
        #if ENABLED(CODE_M840)
          case 840: gcode_M840(); break;
        #endif
        #if ENABLED(CODE_M841)
          case 841: gcode_M841(); break;
        #endif
        #if ENABLED(CODE_M842)
          case 842: gcode_M842(); break;
        #endif
        #if ENABLED(CODE_M843)
          case 843: gcode_M843(); break;
        #endif
        #if ENABLED(CODE_M844)
          case 844: gcode_M844(); break;
        #endif
        #if ENABLED(CODE_M845)
          case 845: gcode_M845(); break;
        #endif
        #if ENABLED(CODE_M846)
          case 846: gcode_M846(); break;
        #endif
        #if ENABLED(CODE_M847)
          case 847: gcode_M847(); break;
        #endif
        #if ENABLED(CODE_M848)
          case 848: gcode_M848(); break;
        #endif
        #if ENABLED(CODE_M849)
          case 849: gcode_M849(); break;
        #endif
        #if ENABLED(CODE_M850)
          case 850: gcode_M850(); break;
        #endif
        #if ENABLED(CODE_M851)
          case 851: gcode_M851(); break;
        #endif
        #if ENABLED(CODE_M852)
          case 852: gcode_M852(); break;
        #endif
        #if ENABLED(CODE_M853)
          case 853: gcode_M853(); break;
        #endif
        #if ENABLED(CODE_M854)
          case 854: gcode_M854(); break;
        #endif
        #if ENABLED(CODE_M855)
          case 855: gcode_M855(); break;
        #endif
        #if ENABLED(CODE_M856)
          case 856: gcode_M856(); break;
        #endif
        #if ENABLED(CODE_M857)
          case 857: gcode_M857(); break;
        #endif
        #if ENABLED(CODE_M858)
          case 858: gcode_M858(); break;
        #endif
        #if ENABLED(CODE_M859)
          case 859: gcode_M859(); break;
        #endif
        #if ENABLED(CODE_M860)
          case 860: gcode_M860(); break;
        #endif
        #if ENABLED(CODE_M861)
          case 861: gcode_M861(); break;
        #endif
        #if ENABLED(CODE_M862)
          case 862: gcode_M862(); break;
        #endif
        #if ENABLED(CODE_M863)
          case 863: gcode_M863(); break;
        #endif
        #if ENABLED(CODE_M864)
          case 864: gcode_M864(); break;
        #endif
        #if ENABLED(CODE_M865)
          case 865: gcode_M865(); break;
        #endif
        #if ENABLED(CODE_M866)
          case 866: gcode_M866(); break;
        #endif
        #if ENABLED(CODE_M867)
          case 867: gcode_M867(); break;
        #endif
        #if ENABLED(CODE_M868)
          case 868: gcode_M868(); break;
        #endif
        #if ENABLED(CODE_M869)
          case 869: gcode_M869(); break;
        #endif
        #if ENABLED(CODE_M870)
          case 870: gcode_M870(); break;
        #endif
        #if ENABLED(CODE_M871)
          case 871: gcode_M871(); break;
        #endif
        #if ENABLED(CODE_M872)
          case 872: gcode_M872(); break;
        #endif
        #if ENABLED(CODE_M873)
          case 873: gcode_M873(); break;
        #endif
        #if ENABLED(CODE_M874)
          case 874: gcode_M874(); break;
        #endif
        #if ENABLED(CODE_M875)
          case 875: gcode_M875(); break;
        #endif
        #if ENABLED(CODE_M876)
          case 876: gcode_M876(); break;
        #endif
        #if ENABLED(CODE_M877)
          case 877: gcode_M877(); break;
        #endif
        #if ENABLED(CODE_M878)
          case 878: gcode_M878(); break;
        #endif
        #if ENABLED(CODE_M879)
          case 879: gcode_M879(); break;
        #endif
        #if ENABLED(CODE_M880)
          case 880: gcode_M880(); break;
        #endif
        #if ENABLED(CODE_M881)
          case 881: gcode_M881(); break;
        #endif
        #if ENABLED(CODE_M882)
          case 882: gcode_M882(); break;
        #endif
        #if ENABLED(CODE_M883)
          case 883: gcode_M883(); break;
        #endif
        #if ENABLED(CODE_M884)
          case 884: gcode_M884(); break;
        #endif
        #if ENABLED(CODE_M885)
          case 885: gcode_M885(); break;
        #endif
        #if ENABLED(CODE_M886)
          case 886: gcode_M886(); break;
        #endif
        #if ENABLED(CODE_M887)
          case 887: gcode_M887(); break;
        #endif
        #if ENABLED(CODE_M888)
          case 888: gcode_M888(); break;
        #endif
        #if ENABLED(CODE_M889)
          case 889: gcode_M889(); break;
        #endif
        #if ENABLED(CODE_M890)
          case 890: gcode_M890(); break;
        #endif
        #if ENABLED(CODE_M891)
          case 891: gcode_M891(); break;
        #endif
        #if ENABLED(CODE_M892)
          case 892: gcode_M892(); break;
        #endif
        #if ENABLED(CODE_M893)
          case 893: gcode_M893(); break;
        #endif
        #if ENABLED(CODE_M894)
          case 894: gcode_M894(); break;
        #endif
        #if ENABLED(CODE_M895)
          case 895: gcode_M895(); break;
        #endif
        #if ENABLED(CODE_M896)
          case 896: gcode_M896(); break;
        #endif
        #if ENABLED(CODE_M897)
          case 897: gcode_M897(); break;
        #endif
        #if ENABLED(CODE_M898)
          case 898: gcode_M898(); break;
        #endif
        #if ENABLED(CODE_M899)
          case 899: gcode_M899(); break;
        #endif
        #if ENABLED(CODE_M900)
          case 900: gcode_M900(); break;
        #endif
        #if ENABLED(CODE_M901)
          case 901: gcode_M901(); break;
        #endif
        #if ENABLED(CODE_M902)
          case 902: gcode_M902(); break;
        #endif
        #if ENABLED(CODE_M903)
          case 903: gcode_M903(); break;
        #endif
        #if ENABLED(CODE_M904)
          case 904: gcode_M904(); break;
        #endif
        #if ENABLED(CODE_M905)
          case 905: gcode_M905(); break;
        #endif
        #if ENABLED(CODE_M906)
          case 906: gcode_M906(); break;
        #endif
        #if ENABLED(CODE_M907)
          case 907: gcode_M907(); break;
        #endif
        #if ENABLED(CODE_M908)
          case 908: gcode_M908(); break;
        #endif
        #if ENABLED(CODE_M909)
          case 909: gcode_M909(); break;
        #endif
        #if ENABLED(CODE_M910)
          case 910: gcode_M910(); break;
        #endif
        #if ENABLED(CODE_M911)
          case 911: gcode_M911(); break;
        #endif
        #if ENABLED(CODE_M912)
          case 912: gcode_M912(); break;
        #endif
        #if ENABLED(CODE_M913)
          case 913: gcode_M913(); break;
        #endif
        #if ENABLED(CODE_M914)
          case 914: gcode_M914(); break;
        #endif
        #if ENABLED(CODE_M915)
          case 915: gcode_M915(); break;
        #endif
        #if ENABLED(CODE_M916)
          case 916: gcode_M916(); break;
        #endif
        #if ENABLED(CODE_M917)
          case 917: gcode_M917(); break;
        #endif
        #if ENABLED(CODE_M918)
          case 918: gcode_M918(); break;
        #endif
        #if ENABLED(CODE_M919)
          case 919: gcode_M919(); break;
        #endif
        #if ENABLED(CODE_M920)
          case 920: gcode_M920(); break;
        #endif
        #if ENABLED(CODE_M921)
          case 921: gcode_M921(); break;
        #endif
        #if ENABLED(CODE_M922)
          case 922: gcode_M922(); break;
        #endif
        #if ENABLED(CODE_M923)
          case 923: gcode_M923(); break;
        #endif
        #if ENABLED(CODE_M924)
          case 924: gcode_M924(); break;
        #endif
        #if ENABLED(CODE_M925)
          case 925: gcode_M925(); break;
        #endif
        #if ENABLED(CODE_M926)
          case 926: gcode_M926(); break;
        #endif
        #if ENABLED(CODE_M927)
          case 927: gcode_M927(); break;
        #endif
        #if ENABLED(CODE_M928)
          case 928: gcode_M928(); break;
        #endif
        #if ENABLED(CODE_M929)
          case 929: gcode_M929(); break;
        #endif
        #if ENABLED(CODE_M930)
          case 930: gcode_M930(); break;
        #endif
        #if ENABLED(CODE_M931)
          case 931: gcode_M931(); break;
        #endif
        #if ENABLED(CODE_M932)
          case 932: gcode_M932(); break;
        #endif
        #if ENABLED(CODE_M933)
          case 933: gcode_M933(); break;
        #endif
        #if ENABLED(CODE_M934)
          case 934: gcode_M934(); break;
        #endif
        #if ENABLED(CODE_M935)
          case 935: gcode_M935(); break;
        #endif
        #if ENABLED(CODE_M936)
          case 936: gcode_M936(); break;
        #endif
        #if ENABLED(CODE_M937)
          case 937: gcode_M937(); break;
        #endif
        #if ENABLED(CODE_M938)
          case 938: gcode_M938(); break;
        #endif
        #if ENABLED(CODE_M939)
          case 939: gcode_M939(); break;
        #endif
        #if ENABLED(CODE_M940)
          case 940: gcode_M940(); break;
        #endif
        #if ENABLED(CODE_M941)
          case 941: gcode_M941(); break;
        #endif
        #if ENABLED(CODE_M942)
          case 942: gcode_M942(); break;
        #endif
        #if ENABLED(CODE_M943)
          case 943: gcode_M943(); break;
        #endif
        #if ENABLED(CODE_M944)
          case 944: gcode_M944(); break;
        #endif
        #if ENABLED(CODE_M945)
          case 945: gcode_M945(); break;
        #endif
        #if ENABLED(CODE_M946)
          case 946: gcode_M946(); break;
        #endif
        #if ENABLED(CODE_M947)
          case 947: gcode_M947(); break;
        #endif
        #if ENABLED(CODE_M948)
          case 948: gcode_M948(); break;
        #endif
        #if ENABLED(CODE_M949)
          case 949: gcode_M949(); break;
        #endif
        #if ENABLED(CODE_M950)
          case 950: gcode_M950(); break;
        #endif
        #if ENABLED(CODE_M951)
          case 951: gcode_M951(); break;
        #endif
        #if ENABLED(CODE_M952)
          case 952: gcode_M952(); break;
        #endif
        #if ENABLED(CODE_M953)
          case 953: gcode_M953(); break;
        #endif
        #if ENABLED(CODE_M954)
          case 954: gcode_M954(); break;
        #endif
        #if ENABLED(CODE_M955)
          case 955: gcode_M955(); break;
        #endif
        #if ENABLED(CODE_M956)
          case 956: gcode_M956(); break;
        #endif
        #if ENABLED(CODE_M957)
          case 957: gcode_M957(); break;
        #endif
        #if ENABLED(CODE_M958)
          case 958: gcode_M958(); break;
        #endif
        #if ENABLED(CODE_M959)
          case 959: gcode_M959(); break;
        #endif
        #if ENABLED(CODE_M960)
          case 960: gcode_M960(); break;
        #endif
        #if ENABLED(CODE_M961)
          case 961: gcode_M961(); break;
        #endif
        #if ENABLED(CODE_M962)
          case 962: gcode_M962(); break;
        #endif
        #if ENABLED(CODE_M963)
          case 963: gcode_M963(); break;
        #endif
        #if ENABLED(CODE_M964)
          case 964: gcode_M964(); break;
        #endif
        #if ENABLED(CODE_M965)
          case 965: gcode_M965(); break;
        #endif
        #if ENABLED(CODE_M966)
          case 966: gcode_M966(); break;
        #endif
        #if ENABLED(CODE_M967)
          case 967: gcode_M967(); break;
        #endif
        #if ENABLED(CODE_M968)
          case 968: gcode_M968(); break;
        #endif
        #if ENABLED(CODE_M969)
          case 969: gcode_M969(); break;
        #endif
        #if ENABLED(CODE_M970)
          case 970: gcode_M970(); break;
        #endif
        #if ENABLED(CODE_M971)
          case 971: gcode_M971(); break;
        #endif
        #if ENABLED(CODE_M972)
          case 972: gcode_M972(); break;
        #endif
        #if ENABLED(CODE_M973)
          case 973: gcode_M973(); break;
        #endif
        #if ENABLED(CODE_M974)
          case 974: gcode_M974(); break;
        #endif
        #if ENABLED(CODE_M975)
          case 975: gcode_M975(); break;
        #endif
        #if ENABLED(CODE_M976)
          case 976: gcode_M976(); break;
        #endif
        #if ENABLED(CODE_M977)
          case 977: gcode_M977(); break;
        #endif
        #if ENABLED(CODE_M978)
          case 978: gcode_M978(); break;
        #endif
        #if ENABLED(CODE_M979)
          case 979: gcode_M979(); break;
        #endif
        #if ENABLED(CODE_M980)
          case 980: gcode_M980(); break;
        #endif
        #if ENABLED(CODE_M981)
          case 981: gcode_M981(); break;
        #endif
        #if ENABLED(CODE_M982)
          case 982: gcode_M982(); break;
        #endif
        #if ENABLED(CODE_M983)
          case 983: gcode_M983(); break;
        #endif
        #if ENABLED(CODE_M984)
          case 984: gcode_M984(); break;
        #endif
        #if ENABLED(CODE_M985)
          case 985: gcode_M985(); break;
        #endif
        #if ENABLED(CODE_M986)
          case 986: gcode_M986(); break;
        #endif
        #if ENABLED(CODE_M987)
          case 987: gcode_M987(); break;
        #endif
        #if ENABLED(CODE_M988)
          case 988: gcode_M988(); break;
        #endif
        #if ENABLED(CODE_M989)
          case 989: gcode_M989(); break;
        #endif
        #if ENABLED(CODE_M990)
          case 990: gcode_M990(); break;
        #endif
        #if ENABLED(CODE_M991)
          case 991: gcode_M991(); break;
        #endif
        #if ENABLED(CODE_M992)
          case 992: gcode_M992(); break;
        #endif
        #if ENABLED(CODE_M993)
          case 993: gcode_M993(); break;
        #endif
        #if ENABLED(CODE_M994)
          case 994: gcode_M994(); break;
        #endif
        #if ENABLED(CODE_M995)
          case 995: gcode_M995(); break;
        #endif
        #if ENABLED(CODE_M996)
          case 996: gcode_M996(); break;
        #endif
        #if ENABLED(CODE_M997)
          case 997: gcode_M997(); break;
        #endif
        #if ENABLED(CODE_M998)
          case 998: gcode_M998(); break;
        #endif
        #if ENABLED(CODE_M999)
          case 999: gcode_M999(); break;
        #endif
        #if ENABLED(ARDUINO_ARCH_SAM)
          case 9999: initiateReset(1000); break;
        #endif

        default: unknown_error(); break;
      }
      break;

      case 'T': gcode_T(parser.codenum);
      break;

      default: unknown_error();
    }

  #endif

  printer.keepalive(NotBusy);

  ok_to_send();

}
