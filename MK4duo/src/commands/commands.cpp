/**
 * MK4duo 3D Commands Firmware
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
 * commands.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"
#include "commands.h"
#include "../gcode/gcode.h"

typedef void (*lpfGCode) (void);

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
      Commands::gcode_LastN         = 0,
      Commands::Stopped_gcode_LastN = 0;

bool Commands::send_ok[BUFSIZE];

uint8_t Commands::mk_debug_flags = DEBUG_NONE;

char Commands::command_queue[BUFSIZE][MAX_CMD_SIZE];

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
 * the main loop. The process_next_command function parses the next
 * command and hands off execution to individual handler functions.
 */
uint8_t Commands::commands_in_queue       = 0,
        Commands::cmd_queue_index_r       = 0,  // Ring buffer read position
        Commands::cmd_queue_index_w       = 0;  // Ring buffer write position

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
void Commands::get_serial_commands() {

  static char serial_line_buffer[MAX_CMD_SIZE];
  static bool serial_comment_mode = false;

  #if HAS_DOOR
    if (READ(DOOR_PIN) != DOOR_PIN_INVERTING) {
      KEEPALIVE_STATE(DOOR_OPEN);
      return;  // do nothing while door is open
    }
  #endif

  // If the command buffer is empty for too long,
  // send "wait" to indicate MK4duo is still waiting.
  #if ENABLED(NO_TIMEOUTS) && NO_TIMEOUTS > 0
    static millis_t last_command_time = 0;
    millis_t ms = millis();
    if (commands_in_queue == 0 && !HAL::serialByteAvailable() && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
      SERIAL_L(WT);
      last_command_time = ms;
    }
  #endif

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (commands_in_queue < BUFSIZE && HAL::serialByteAvailable() > 0) {

    char serial_char = HAL::serialReadByte();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false; // end of line == end of comment

      if (!serial_count) continue; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; // reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
      char* apos = strchr(command, '*');

      if (npos) {

        const bool M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];

          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
          // if no errors, continue parsing
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing
      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        return;
      }

      // Movement commands alert when stopped
      if (printer.IsStopped()) {
        char* gpos = strchr(command, 'G');
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
          thermalManager.wait_for_heatup = false;
          #if ENABLED(ULTIPANEL)
            printer.wait_for_user = false;
          #endif
        }
        if (strcmp(command, "M112") == 0) printer.kill(PSTR(MSG_KILLED));
        if (strcmp(command, "M410") == 0) { printer.quickstop_stepper(); }
      #endif

      #if ENABLED(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') { // Handle escapes
      if (HAL::serialByteAvailable()) {
        // if we have one more character, copy it over
        serial_char = HAL::serialReadByte();
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // its not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }
  } // queue has space, serial has data
}

/**
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call idle
 */
void Commands::command_loop() {

  get_available_commands();

  #if HAS_EEPROM_SD
    static uint8_t wait_for_host_init_string_to_finish = 1;
    if (wait_for_host_init_string_to_finish) {
      if (commands_in_queue != 0 && wait_for_host_init_string_to_finish == 1) wait_for_host_init_string_to_finish = 2;
      if (commands_in_queue == 0 && wait_for_host_init_string_to_finish >= 2) wait_for_host_init_string_to_finish++;
      if (wait_for_host_init_string_to_finish >= 250) {
        wait_for_host_init_string_to_finish = 0;
        // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
        eeprom.Load_Settings();
      }
    }
  #endif

  #if HAS_SDSUPPORT
    card.checkautostart(false);
  #endif

  if (commands_in_queue) {

    #if HAS_SDSUPPORT

      if (card.saving) {
        char* command = command_queue[cmd_queue_index_r];
        if (strstr_P(command, PSTR("M29"))) {
          // M29 closes the file
          card.finishWrite();
          ok_to_send();
        }
        else {
          // Write the string from the read buffer to SD
          card.write_command(command);
          ok_to_send();
        }
      }
      else
        process_next_gcode();

    #else

      process_next_gcode();

    #endif // SDSUPPORT

    // The queue may be reset by a command handler or by code invoked by idle() within a handler
    if (commands_in_queue) {
      --commands_in_queue;
      if (++cmd_queue_index_r >= BUFSIZE) cmd_queue_index_r = 0;
    }
  }

  endstops.report_state();
  printer.idle();

}

#if HAS_SDSUPPORT

  /**
   * Get commands from the SD Card until the command buffer is full
   * or until the end of the file is reached. The special character '#'
   * can also interrupt buffering.
   */
  void Commands::get_sdcard_commands() {
    static bool stop_buffering = false,
                sd_comment_mode = false;

    if (!card.sdprinting) return;

    #if HAS_DOOR
      if (READ(DOOR_PIN) != DOOR_PIN_INVERTING) {
        KEEPALIVE_STATE(DOOR_OPEN);
        return;  // do nothing while door is open
      }
    #endif

    #if HAS_POWER_CHECK
      if (READ(POWER_CHECK_PIN) != POWER_CHECK_PIN_INVERTING) {
        stopSDPrint(true);
        return;
      }
    #endif

    /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

    if (commands_in_queue == 0) stop_buffering = false;

    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (commands_in_queue < BUFSIZE && !card_eof && !stop_buffering) {
      const int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n' || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {
          SERIAL_EM(MSG_FILE_PRINTED);
          card.printingHasFinished();
          #if ENABLED(PRINTER_EVENT_LEDS)
            LCD_MESSAGEPGM(MSG_INFO_COMPLETED_PRINTS);
            printer.set_led_color(0, 255, 0); // Green
            #if HAS(RESUME_CONTINUE)
              enqueue_and_echo_commands_P(PSTR("M0")); // end of the queue!
            #else
              safe_delay(1000);
            #endif
            printer.set_led_color(0, 0, 0);   // OFF
          #endif
          card.checkautostart(true);
        }
        else if (n == -1) {
          SERIAL_LM(ER, MSG_SD_ERR_READ);
        }
        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; // for new command

        if (!sd_count) continue; // skip empty lines (and comment lines)

        command_queue[cmd_queue_index_w][sd_count] = '\0'; // terminate string
        sd_count = 0; // clear sd line buffer

        commit_command(false);
      }
      else if (sd_count >= MAX_CMD_SIZE - 1) {
        /**
         * Keep fetching, but ignore normal characters beyond the max length
         * The command will be injected when EOL is reached
         */
      }
      else {
        if (sd_char == ';') sd_comment_mode = true;
        if (!sd_comment_mode) command_queue[cmd_queue_index_w][sd_count++] = sd_char;
      }
    }
  }

#endif // SDSUPPORT

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void Commands::FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
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
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_STR(OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      SERIAL_CHR(' ');
      SERIAL_CHR(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_CHR(*p++);
    }
    SERIAL_MV(" P", (int)(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_MV(" B", BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL();
}

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void Commands::get_available_commands() {

  if (commands_in_queue >= BUFSIZE) return;

  // if any immediate commands remain, don't get other commands yet
  if (drain_injected_commands_P()) return;

  get_serial_commands();

  #if HAS_SDSUPPORT
    get_sdcard_commands();
  #endif

}

/**
 * Inject the next "immediate" command, when possible, onto the front of the queue.
 * Return true if any immediate commands remain to inject.
 */
bool Commands::drain_injected_commands_P() {
  if (injected_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, injected_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
    cmd[i] = '\0';
    if (enqueue_and_echo_command(cmd))     // success?
      injected_commands_P = c ? injected_commands_P + i + 1 : NULL; // next command or done
  }
  return (injected_commands_P != NULL);    // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_injected_commands_P() must be called repeatedly to drain the commands afterwards
 */
void Commands::enqueue_and_echo_commands_P(const char * const pgcode) {
  injected_commands_P = pgcode;
  drain_injected_commands_P(); // first command executed asap (when possible)
}

/**
 * Clear the MK4duo command queue
 */
void Commands::clear_command_queue() {
  cmd_queue_index_r = cmd_queue_index_w;
  commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
void Commands::commit_command(bool say_ok) {
  send_ok[cmd_queue_index_w] = say_ok;
  if (++cmd_queue_index_w >= BUFSIZE) cmd_queue_index_w = 0;
  commands_in_queue++;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
bool Commands::enqueuecommand(const char* cmd, bool say_ok/*=false*/) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  commit_command(say_ok);
  return true;
}

/**
 * Enqueue with Serial Echo
 */
bool Commands::enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
  if (enqueuecommand(cmd, say_ok)) {
    SERIAL_SMT(ECHO, MSG_ENQUEUEING, cmd);
    SERIAL_CHR('"');
    SERIAL_EOL();
    return true;
  }
  return false;
}

/**
 * Private Function
 */

void Commands::gcode_line_error(const char* err, const bool doFlush/*=true*/) {
  SERIAL_STR(ER);
  SERIAL_PS(err);
  SERIAL_EV(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

void Commands::unknown_command_error() {
  SERIAL_SMV(ECHO, MSG_UNKNOWN_COMMAND, parser.command_ptr);
  SERIAL_CHR('"');
  SERIAL_EOL();
}

#define GCODE_G(N) \
  case ##N: gcode_G##N(); break

#define GCODE_M(N) \
  case ##N: gcode_M##N(); break

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void Commands::process_next_gcode() {

  char * const current_command = get_command_queue();

  if (DEBUGGING(ECHO)) SERIAL_LV(ECHO, current_command);

  KEEPALIVE_STATE(IN_HANDLER);

  // Parse the next command in the queue
  parser.parse(current_command);

  // Handle a known G, M, or T
  switch(parser.command_letter) {

    case 'G': {
      const int gcode_cmd = parser.codenum;
      for (int tblIndex = 0; tblIndex < COUNT(GCode_Table); tblIndex ++) {
        if (gcode_cmd == GCode_Table[tblIndex][0]) {
          ((lpfGCode)(GCode_Table[tblIndex][1]))(); // Command found, execute
          break; 
        }
      }
    }
    break;

    case 'M': switch (parser.codenum) {

      #if ENABLED(ULTIPANEL) || ENABLED(EMERGENCY_PARSER)
        case 0: // M0: Unconditional stop - Wait for user button press on LCD
        case 1: // M1: Conditional stop - Wait for user button press on LCD
          gcode_M0_M1(); break;
      #endif // ULTIPANEL || EMERGENCY_PARSER

      #if ENABLED(LASER) || ENABLED(CNCROUTER)
        case 3: // M03: Setting laser beam or CNC clockwise speed
        case 4: // M04: Turn on laser beam or CNC counter clockwise speed
          gcode_M3_M4(parser.codenum == 3); break;
        case 5: // M05: Turn off laser beam or CNC stop
          gcode_M5(); break;
      #endif // LASER || CNCROUTER

      #if ENABLED(CNCROUTER)
        case 6: // M06: Tool change CNC
          gcode_M6(); break;
        // case 7: // M07 - Mist coolant CNC XXX
        // case 8: // M08 - Flood coolant CNC XXX
        // case 9: // M09 - Coolant off CNC XXX
        // case 10: // M10 - Vacuum on CNC XXX
        // case 11: // M11 - Vacuum off CNC XXX
      #endif // CNCROUTER

      case 17: // M17: Enable/Power all stepper motors
        gcode_M17(); break;

      #if HAS_SDSUPPORT
        case 20: // M20: list SD card
          gcode_M20(); break;
        case 21: // M21: init SD card
          gcode_M21(); break;
        case 22: // M22: release SD card
          gcode_M22(); break;
        case 23: // M23: Select file
          gcode_M23(); break;
        case 24: // M24: Start SD print
          gcode_M24(); break;
        case 25: // M25: Pause SD print
          gcode_M25(); break;
        case 26: // M26: Set SD index
          gcode_M26(); break;
        case 27: // M27: Get SD status
          gcode_M27(); break;
        case 28: // M28: Start SD write
          gcode_M28(); break;
        case 29: // M29: Stop SD write
          gcode_M29(); break;
        case 30: // M30 <filename> Delete File
          gcode_M30(); break;
      #endif // SDSUPPORT

      case 31: // M31: Report time since the start of SD print or last M109
        gcode_M31(); break;

      #if HAS_SDSUPPORT
        case 32: // M32: Make directory
          gcode_M32(); break;
        case 33: // M33: Stop printing, close file and save restart.gcode
          gcode_M33(); break;
        case 34: // M34: Select file and start SD print
          gcode_M34(); break;
        #if ENABLED(NEXTION)
          case 35: // M35: Upload Firmware to Nextion from SD
            gcode_M35(); break;
        #endif
      #endif // SDSUPPORT

      case 42: // M42: Change pin state
        gcode_M42(); break;

      #if ENABLED(PINS_DEBUGGING)
        case 43: // M43: Read pin state
          gcode_M43(); break;
      #endif

      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: // M48: Z probe repeatability test
          gcode_M48(); break;
      #endif

      #if HAS_POWER_CONSUMPTION_SENSOR
        case 70: // M70: Power consumption sensor calibration
          gcode_M70(); break;
      #endif

      case 75: // M75: Start print timer
        gcode_M75(); break;
      case 76: // M76: Pause print timer
        gcode_M76(); break;
      case 77: // M77: Stop print timer
        gcode_M77(); break;
      case 78: // M78: Show print statistics
        gcode_M78(); break;

      #if HAS_POWER_SWITCH
        case 80: // M80: Turn on Power Supply
          gcode_M80(); break;
      #endif

      case 81: // M81: Turn off Power, including Power Supply, if possible
        gcode_M81(); break;
      case 82: // M82: Set E axis normal mode (same as other axes)
        gcode_M82(); break;
      case 83: // M83: Set E axis relative mode
        gcode_M83(); break;
      case 18: // M18 => M84
      case 84: // M84: Disable all steppers or set timeout
        gcode_M18_M84(); break;
      case 85: // M85: Set inactivity stepper shutdown timeout
        gcode_M85(); break;
      case 92: // M92: Set the steps-per-unit for one or more axes
        gcode_M92(); break;

      #if ENABLED(ZWOBBLE)
        case 96: // M96: Print ZWobble value
          gcode_M96(); break;
        case 97: // M97: Set ZWobble parameter
          gcode_M97(); break;
      #endif

      #if ENABLED(HYSTERESIS)
        case 98: // M98: Print Hysteresis value
          gcode_M98(); break;
        case 99: // M99: Set Hysteresis parameter
          gcode_M99(); break;
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100: // M100: Free Memory Report
          gcode_M100(); break;
      #endif

      #if HAS_TEMP_HOTEND
        case 104: // M104: Set hot end temperature
          gcode_M104(); break;
      #endif

      case 105: // M105: Report current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed

      #if FAN_COUNT > 0
        case 106: // M106: Fan On
          gcode_M106(); break;
        case 107: // M107: Fan Off
          gcode_M107(); break;
      #endif // FAN_COUNT > 0

      #if DISABLED(EMERGENCY_PARSER)
        case 108: // M108: Cancel heatup
          gcode_M108(); break;
      #endif

      #if HAS_TEMP_HOTEND
        case 109: // M109: Wait for hotend temperature to reach target
          gcode_M109(); break;
      #endif

      case 110: // M110: Set Current Line Number
        gcode_M110(); break;

      case 111: // M111: Set debug level
        gcode_M111(); break;

      #if DISABLED(EMERGENCY_PARSER)
        case 112: //  M112: Emergency Stop
          gcode_M112(); break;
      #endif

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: // M113: Set Host Keepalive interval
          gcode_M113(); break;
      #endif

      case 114: // M114: Report current position
        gcode_M114(); break;

      case 115: // M115: Report capabilities
        gcode_M115(); break;

      case 117: // M117: Set LCD message text, if possible
        gcode_M117(); break;

      case 118: // M118: Print to Host the message text
        gcode_M118(); break;

      case 119: // M119: Report endstop states
        gcode_M119(); break;
      case 120: // M120: Enable endstops
        gcode_M120(); break;
      case 121: // M121: Disable endstops
        gcode_M121(); break;
      case 122: // M122: Disable or enable software endstops
        gcode_M122(); break;

      #if ENABLED(HAVE_TMC2130DRIVER)
        case 123: // M123: Diagnose, used to debug TMC2130
          gcode_M123(); break;
      #endif

      #if ENABLED(PARK_HEAD_ON_PAUSE)
        case 125: // M125: Store current position and move to pause park position
          gcode_M125(); break;
      #endif

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS_HEATER_1
          case 126: // M126 valve open
            gcode_M126(); break;
          case 127: // M127 valve closed
            gcode_M127(); break;
        #endif // HAS_HEATER_1

        // PWM for HEATER_2_PIN
        #if HAS_HEATER_2
          case 128: // M128 valve open
            gcode_M128(); break;
          case 129: // M129 valve closed
            gcode_M129(); break;
        #endif // HAS_HEATER_2
      #endif // BARICUDA

      #if HAS_TEMP_BED
        case 140: // M140 - Set bed temp
          gcode_M140(); break;
      #endif

      #if HAS_TEMP_CHAMBER
        case 141: // M141 - Set chamber temp
          gcode_M141(); break;
      #endif

      #if HAS_TEMP_COOLER
        case 142: // M142 - Set cooler temp
          gcode_M142(); break;
      #endif

      #if ENABLED(ULTIPANEL) && HAS_TEMP_0
        case 145: // M145: Set material heatup parameters
          gcode_M145(); break;
      #endif

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149: // M149: Set temperature units
          gcode_M149(); break;
      #endif

      #if HAS_COLOR_LEDS
        case 150: // M150
          gcode_M150(); break;
      #endif //BLINKM

      #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
        case 155: // M155: Set temperature auto-report interval
          gcode_M155(); break;
      #endif

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        case 163: // M163 S<int> P<float> set weight for a mixing extruder
          gcode_M163(); break;
        #if MIXING_VIRTUAL_TOOLS > 1
          case 164: // M164 S<int> save current mix as a virtual tools
            gcode_M164(); break;
        #endif
        case 165: // M165 [ABCDHI]<float> set multiple mix weights
          gcode_M165(); break;
      #endif

      #if HAS_TEMP_BED
        case 190: // M190 - Wait for bed heater to reach target.
          gcode_M190(); break;
      #endif // TEMP_BED

      #if HAS_TEMP_CHAMBER
        case 191: // M191 - Wait for chamber heater to reach target.
          gcode_M191(); break;
      #endif

      #if HAS_TEMP_COOLER
        case 192: // M192 - Wait for chamber heater to reach target.
          gcode_M192(); break;
      #endif

      case 200: // // M200 D<diameter> Set filament diameter and set E axis units to cubic. (Use S0 to revert to linear units.)
        gcode_M200(); break;
      case 201: // M201
        gcode_M201(); break;
      #if 0 // Not used for Sprinter/grbl gen6
      case 202: // M202
        gcode_M202();
        break;
      #endif
      case 203: // M203 max mechanics.feedrate_mm_s units/sec
        gcode_M203(); break;
      case 204: // M204 mechanics.acceleration S normal moves T filament only moves
        gcode_M204(); break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
        gcode_M205(); break;

      #if ENABLED(WORKSPACE_OFFSETS)
        case 206: // M206: Set home offsets
          gcode_M206(); break;
      #endif

      #if ENABLED(FWRETRACT)
        case 207: //M207 - M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>1
          gcode_M207(); break;
        case 208: // M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>
          gcode_M208(); break;
        case 209: // M209 - Turn Automatic Retract Detection on/off: S<bool> (For slicers that don't support G10/11). Every normal extrude-only move will be classified as retract depending on the direction.
          gcode_M209(); break;
      #endif

      case 218: // M218: Set a tool offset: T<index> X<offset> Y<offset> Z<offset>
        gcode_M218(); break;
      case 220: // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)
        gcode_M220(); break;
      case 221: // M221: Set Flow Percentage: T<extruder> S<percent>
        gcode_M221(); break;
      case 222: // M222: Set Purge Percentage: T<extruder> S<percent>
        gcode_M222(); break;
      case 226: // M226: P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226(); break;

      #if HAS_CHDK || HAS_PHOTOGRAPH
        case 240: // M240: Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240(); break;
      #endif

      #if HAS(LCD_CONTRAST)
        case 250: // M250: Set LCD contrast value: C<value> (value 0..63)
          gcode_M250(); break;
      #endif

      #if HAS_SERVOS
        case 280: // M280: Set servo position absolute
          gcode_M280(); break;
      #endif

      #if HAS_BUZZER
        case 300: // M300: Play beep tone
          gcode_M300(); break;
      #endif

      #if ENABLED(PIDTEMP)
        case 301: // M301: Set hotend PID parameters
          gcode_M301(); break;
      #endif

      #if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
        case 302: // M302: Allow cold extrudes (set the minimum extrude temperature)
          gcode_M302(); break;
      #endif

      #if ENABLED(PIDTEMP)
        case 303: // M303: PID autotune
          gcode_M303(); break;
      #endif

      #if ENABLED(PIDTEMPBED)
        case 304: // M304: Set Bed PID
          gcode_M304(); break;
      #endif

      #if ENABLED(PIDTEMPCHAMBER)
        case 305: // M305: Set Chamber PID
          gcode_M305(); break;
      #endif

      #if ENABLED(PIDTEMPCOOLER)
        case 306: // M306: Set Cooler PID
          gcode_M306(); break;
      #endif

      #if HAS_ABL
        case 320: // M320: Activate ABL
          gcode_M320(); break;
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          case 321: // M321: Set a Auto Bed Leveling Z coordinate
            gcode_M321(); break;
        #endif
        case 322: // M322: Reset auto leveling matrix
          gcode_M322(); break;
      #endif

      #if HAS_MICROSTEPS
        case 350: // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350(); break;
        case 351: // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351(); break;
      #endif

      #if HAS_CASE_LIGHT
        case 355: // M355: Turn case lights on/off
          gcode_M355(); break;
      #endif

      #if MECH(MORGAN_SCARA)
        case 360:  // M360: SCARA Theta pos1
          if (gcode_M360()) return; break;
        case 361:  // M361: SCARA Theta pos2
          if (gcode_M361()) return; break;
        case 362:  // M362: SCARA Psi pos1
          if (gcode_M362()) return; break;
        case 363:  // M363: SCARA Psi pos2
          if (gcode_M363()) return; break;
        case 364:  // M364: SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return; break;
      #endif

      case 400: // M400 finish all moves
        gcode_M400(); break;

      #if HAS_BED_PROBE
        case 401: // M401: Engage Z Servo endstop if available
          gcode_M401(); break;
        case 402: // M402: Retract Z Servo endstop if enabled
          gcode_M402(); break;
      #endif

      #if ENABLED(FILAMENT_SENSOR)
        case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404(); break;
        case 405:  //M405 Turn on filament sensor for control
          gcode_M405(); break;
        case 406:  //M406 Turn off filament sensor for control
          gcode_M406(); break;
        case 407:   //M407 Display measured filament diameter
          gcode_M407(); break;
      #endif

      #if ENABLED(JSON_OUTPUT)
        case 408: // M408 JSON STATUS OUTPUT
          gcode_M408(); break;
      #endif // JSON_OUTPUT

      #if DISABLED(EMERGENCY_PARSER)
        case 410: // M410 quickstop - Abort all the planned moves.
          gcode_M410(); break;
      #endif

      #if ENABLED(MESH_BED_LEVELING)
        case 420: // M420 Enable/Disable Mesh Bed Leveling
          gcode_M420(); break;
        case 421: // M421 Set a Mesh Bed Leveling Z coordinate
          gcode_M421(); break;
      #endif

      #if ENABLED(WORKSPACE_OFFSETS)
        case 428: // M428 Apply current_position to home_offset
          gcode_M428(); break;
      #endif

      #if HAS_MULTI_MODE
        case 450:
          gcode_M450(); break; // report printer mode
        case 451:
          gcode_M451(); break;    // set printer mode printer
        #if ENABLED(LASER)
          case 452:
            gcode_M452(); break;  // set printer mode laser
        #endif
        #if ENABLED(CNCROUTER)
          case 453:
            gcode_M453(); break;  // set printer mode router
        #endif
      #endif

      case 500: // M500: Store settings in EEPROM
        gcode_M500(); break;
      case 501: // M501: Read settings from EEPROM
        gcode_M501(); break;
      case 502: // M502: Revert to default settings
        gcode_M502(); break;
      case 503: // M503: print settings currently in memory
        gcode_M503(); break;

      #if ENABLED(RFID_MODULE)
        case 522: // M422: Read or Write on card. M522 T<extruders> R<read> or W<write>
          gcode_M522(); break;
      #endif

      case 530: // M530: S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
        gcode_M530(); break;
      case 531: // M531: filename - Define filename being printed
        gcode_M531(); break;
      case 532: // M532: X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
        gcode_M532(); break;

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540: // M540: Set abort on endstop hit for SD printing
          gcode_M540(); break;
      #endif

      #if HEATER_USES_AD595
        case 595: // M595 set Hotends AD595 offset & gain
          gcode_M595(); break;
      #endif

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 600: // Pause Park X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600(); break;
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        case 605:
          gcode_M605(); break;
      #endif

      #if ENABLED(LASER)
        case 649: // M649 set laser options
          gcode_M649(); break;
      #endif 

      #if MECH(MUVE3D)
        case 650: // M650: Set peel distance
          gcode_M650(); break;
        case 651: // M651: Run peel move and return back to start.
          gcode_M651(); break;
        case 653: // M653: Execute tilt move
          gcode_M653(); break;
        case 654: // M654 - execute untilt move
          gcode_M654(); break;
        case 655: // M655: Send projector control commands via serial
          gcode_M655(); break;
      #endif

      #if HAS_BED_PROBE || MECH(DELTA)
        case 666: // M666 Set Z probe offset or set delta endstop and geometry adjustment
          gcode_M666(); break;
      #endif

      #if ENABLED(LIN_ADVANCE)
        case 900: // M900 Set advance factor.
          gcode_M900(); break;
      #endif

      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3) || ENABLED(HAVE_TMC2130)
        case 906: // M906 Set motor currents XYZ T0-4 E
          gcode_M906(); break;
      #endif

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907(); break;

      #if HAS_DIGIPOTSS
        case 908: // M908 Control digital trimpot directly.
          gcode_M908(); break;
      #endif // HAS_DIGIPOTSS

      #if ENABLED(HAVE_TMC2130)
        case 911: // M911: Report TMC2130 prewarn triggered flags
          gcode_M911(); break;
        case 912: // M912: Clear TMC2130 prewarn triggered flags
          gcode_M912(); break;

        #if ENABLED(HYBRID_THRESHOLD)
          case 913: // M913: Set HYBRID_THRESHOLD speed.
            gcode_M913(); break;
        #endif

        #if ENABLED(SENSORLESS_HOMING)
          case 914: // M914: Set SENSORLESS_HOMING sensitivity.
            gcode_M914(); break;
        #endif
      #endif

      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        case 995: // M995 Nextion origin
          gcode_M995(); break;
        case 996: // M996 Nextion scale
          gcode_M996(); break;
      #endif

      #if ENABLED(NPR2)
        case 997: // M997 Cxx Move Carter xx gradi
          gcode_M997(); break;
      #endif // NPR2

      case 999: // M999: Restart after being Stopped
        gcode_M999(); break;

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800: // GCode Parser Test
          parser.debug(); break;
      #endif
    }
    break;

    case 'T':
      gcode_T(parser.codenum);
    break;

    default: unknown_command_error();
  }

  KEEPALIVE_STATE(NOT_BUSY);

  ok_to_send();
}
