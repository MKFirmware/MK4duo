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
 * parser.cpp - Parser for a GCode line, providing a parameter interface.
 */

#include "../../base.h"

// Must be declared for allocation and to satisfy the linker
// Zero values need no initialisation.

#if ENABLED(INCH_MODE_SUPPORT)
  float GCodeParser::linear_unit_factor, GCodeParser::volumetric_unit_factor;
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit GCodeParser::input_temp_units = TEMPUNIT_C;
#endif

char *GCodeParser::command_ptr,
     *GCodeParser::string_arg,
     *GCodeParser::value_ptr,
      GCodeParser::command_letter;

uint16_t GCodeParser::codenum;

#if USE_GCODE_SUBCODES
  uint8_t GCodeParser::subcode;
#endif

#if ENABLED(FASTER_GCODE_PARSER)
  // Optimized Parameters
  byte GCodeParser::codebits[4];   // found bits
  uint8_t GCodeParser::param[26];  // parameter offsets from command_ptr
#else
  char *GCodeParser::command_args; // start of parameters
#endif

// Create a global instance of the GCodeParser singleton
GCodeParser parser;

// Populate all fields by parsing a single line of GCode
// 58 bytes of SRAM are used to speed up seen/value
void GCodeParser::parse(char *p) {

  reset(); // No codes to report

  while (*p == ' ') ++p;  // Skip spaces

  // Skip N[-0-9] if included in the command line
  if (*p == 'N' && NUMERIC_SIGNED(p[1])) {
    #if ENABLED(FASTER_GCODE_PARSER)
      //set('N', p + 1);      // (optional) Set the 'N' parameter value
    #endif
    p += 2;                   // skip N[-0-9]
    while (NUMERIC(*p)) ++p;  // skip [0-9]*
    while (*p == ' ') ++p;    // skip [ ]*
  }

  // *p now points to the current command, which should be G, M, or T
  command_ptr = p;

  // Get the command letter, which must be G, M, or T
  const char letter = *p++;

  // Nullify asterisk and trailing whitespace
  char *starpos = strchr(p, '*');
  if (starpos) {
    --starpos;
    while (*starpos == ' ') --starpos;  // remove previous spaces...
    starpos[1] = '\0';
  }

  // Bail if the letter is not G, M, or T
  switch (letter) { case 'G': case 'M': case 'T': break; default: return; }

  // Skip spaces to get the numeric part
  while (*p == ' ') ++p;

  // Bail if there's no command code number
  if (!NUMERIC(*p)) return;

  // Save the command letter at this point
  // A '?' signifies an unknown command
  command_letter = letter;

  // Get the code number - integer digits only
  codenum = 0;
  do {
    codenum *= 10, codenum += *p++ - '0';
  } while (NUMERIC(*p));

  // Allow for decimal point in command
  #if USE_GCODE_SUBCODES
    if (*p == '.') {
      p++;
      while (NUMERIC(*p))
        subcode *= 10, subcode += *p++ - '0';
    }
  #endif

  // Skip all spaces to get to the first argument, or null
  while (*p == ' ') ++p;

  // The command parameters (if any) start here, for sure!

  #if DISABLED(FASTER_GCODE_PARSER)
    command_args = p; // Scan for parameters in seen()
  #endif

  // Only use string_arg for these M codes
  if (letter == 'M') switch (codenum) { case 23: case 28: case 30: case 117: case 118: case 928: string_arg = p; return; default: break; }

  #if ENABLED(DEBUG_GCODE_PARSER)
    const bool debug = (codenum == 800);
  #endif

  /**
   * Find all parameters, set flags and pointers for fast parsing
   *
   * Most codes ignore 'string_arg', but those that want a string will get the right pointer.
   * The following loop assigns the first "parameter" having no numeric value to 'string_arg'.
   * This allows M0/M1 with expire time to work: "M0 S5 You Win!"
   */
  string_arg = NULL;
  while (char code = *p++) {                    // Get the next parameter. A '\0' ends the loop

    // Special handling for M32 [P] !/path/to/file.g#
    // The path must be the last parameter
    if (code == '!' && letter == 'M' && codenum == 32) {
      string_arg = p;                           // Name starts after '!'
      char * const lb = strchr(p, '#');         // Already seen '#' as SD char (to pause buffering)
      if (lb) *lb = '\0';                       // Safe to mark the end of the filename
      return;
    }

    // Arguments MUST be uppercase for fast GCode parsing
    #if ENABLED(FASTER_GCODE_PARSER)
      #define PARAM_TEST WITHIN(code, 'A', 'Z')
    #else
      #define PARAM_TEST true
    #endif

    if (PARAM_TEST) {

      while (*p == ' ') ++p;                    // skip spaces between parameters & values
      const bool has_num = DECIMAL_SIGNED(*p);  // The parameter has a number [-+0-9.]

      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) {
          SERIAL_MV("Got letter ", code); // DEBUG
          SERIAL_MV(" at index ", (int)(p - command_ptr - 1)); // DEBUG
          if (has_num) SERIAL_MSG(" (has_num)");
        }
      #endif

      if (!has_num && !string_arg) {            // No value? First time, keep as string_arg
        string_arg = p - 1;
        #if ENABLED(DEBUG_GCODE_PARSER)
          if (debug) SERIAL_MV(" string_arg: ", hex_address((void*)string_arg)); // DEBUG
        #endif
      }

      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_EOL();
      #endif

      #if ENABLED(FASTER_GCODE_PARSER)
        set(code, has_num ? p : NULL            // Set parameter exists and pointer (NULL for no number)
          #if ENABLED(DEBUG_GCODE_PARSER)
            , debug
          #endif
        );
      #endif
    }
    else if (!string_arg) {                     // Not A-Z? First time, keep as the string_arg
      string_arg = p - 1;
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_MV(" string_arg: ", hex_address((void*)string_arg)); // DEBUG
      #endif
    }

    if (!WITHIN(*p, 'A', 'Z')) {
      while (*p && NUMERIC(*p)) p++;            // Skip over the value section of a parameter
      while (*p == ' ') ++p;                    // Skip over all spaces
    }
  }
}

#if ENABLED(DEBUG_GCODE_PARSER)

  void GCodeParser::debug() {
    SERIAL_MV("Command: ", command_ptr);
    SERIAL_MV(" (", command_letter);
    SERIAL_VAL(codenum);
    SERIAL_EM(")");
    #if ENABLED(FASTER_GCODE_PARSER)
      SERIAL_MSG(" args: \"");
      for (char c = 'A'; c <= 'Z'; ++c)
        if (seen(c)) { SERIAL_CHR(c); SERIAL_CHR(' '); }
    #else
      SERIAL_MV(" args: \"", command_args);
    #endif
    SERIAL_MSG("\"");
    if (string_arg) {
      SERIAL_MSG(" string: \"");
      SERIAL_TXT(string_arg);
      SERIAL_CHR('"');
    }
    SERIAL_MSG("\n\n");
    for (char c = 'A'; c <= 'Z'; ++c) {
      if (seen(c)) {
        SERIAL_MV("Code '", c); SERIAL_MSG("':");
        if (has_value()) {
          SERIAL_MV("\n    float: ", value_float());
          SERIAL_MV("\n     long: ", value_long());
          SERIAL_MV("\n    ulong: ", value_ulong());
          SERIAL_MV("\n   millis: ", value_millis());
          SERIAL_MV("\n   sec-ms: ", value_millis_from_seconds());
          SERIAL_MV("\n      int: ", value_int());
          SERIAL_MV("\n   ushort: ", value_ushort());
          SERIAL_MV("\n     byte: ", (int)value_byte());
          SERIAL_MV("\n     bool: ", (int)value_bool());
          SERIAL_MV("\n   linear: ", value_linear_units());
          SERIAL_MV("\n  celsius: ", value_celsius());
        }
        else
          SERIAL_MSG(" (no value)");
        SERIAL_MSG("\n\n");
      }
    }
  }

#endif // DEBUG_GCODE_PARSER
