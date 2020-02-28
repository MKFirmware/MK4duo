/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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

#include "../../MK4duo.h"

// Must be declared for allocation and to satisfy the linker
// Zero values need no initialisation.

#if ENABLED(INCH_MODE_SUPPORT)
  float GCodeParser::linear_unit_factor,
        GCodeParser::volumetric_unit_factor;
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnitEnum GCodeParser::input_temp_units = TEMPUNIT_C;
#endif

char *GCodeParser::command_ptr,
     *GCodeParser::string_arg,
     *GCodeParser::value_ptr;

char  GCodeParser::command_letter;

uint16_t GCodeParser::codenum;

#if USE_GCODE_SUBCODES
  uint8_t GCodeParser::subcode;
#endif

#if ENABLED(FASTER_GCODE_PARSER)
  // Optimized Parameters
  uint32_t  GCodeParser::codebits;  // found bits
  uint8_t   GCodeParser::param[26]; // parameter offsets from command_ptr
#else
  char *GCodeParser::command_args; // start of parameters
#endif

// Create a global instance of the GCodeParser singleton
GCodeParser parser;

/**
 * Clear all code-seen (and value pointers)
 *
 * Since each param is set/cleared on seen codes,
 * this may be optimized by commenting out ZERO(param)
 */
void GCodeParser::reset() {
  string_arg = nullptr;               // No whole line argument
  command_letter = '?';               // No command letter
  codenum = 0;                        // No command code
  #if USE_GCODE_SUBCODES
    subcode = 0;                      // No command sub-code
  #endif
  #if ENABLED(FASTER_GCODE_PARSER)
    codebits = 0;                     // No codes yet
    //ZERO(param);                    // No parameters (should be safe to comment out this line)
  #endif
}

// Pass the address after the first quote (if any)
char* GCodeParser::unescape_string(char* &src) {
  if (*src == '"') ++src;     // Skip the leading quote
  char * const out = src;     // Start of the string
  char *dst = src;            // Prepare to unescape and terminate
  for (;;) {
    char c = *src++;          // Get the next char
    switch (c) {
      case '\\': c = *src++;  break;  // Get the escaped char
      case '"' : c = '\0';    break;  // Convert bare quote to nul
    }
    if (!(*dst++ = c)) break; // Copy and break on nul
  }
  return out;
}

// Populate all fields by parsing a single line of GCode
// 58 bytes of SRAM are used to speed up seen/value
void GCodeParser::parse(char *p) {

  reset(); // No codes to report

  auto uppercase = [](char c) {
    if (WITHIN(c, 'a', 'z')) c += 'A' - 'a';
    return c;
  };

  // Skip spaces
  while (*p == ' ') ++p;

  // Skip N[-0-9] if included in the command line
  if (uppercase(*p) == 'N' && NUMERIC_SIGNED(p[1])) {
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
  const char letter = uppercase(*p++);

  // Nullify asterisk and trailing whitespace
  char *starpos = strchr(p, '*');
  if (starpos) {
    --starpos;
    while (*starpos == ' ') --starpos;  // remove previous spaces...
    starpos[1] = '\0';
  }

  // Bail if the letter is not G, M, or T
  switch (letter) {

    case 'G': case 'M': case 'T':

      // Skip spaces to get the numeric part
      while (*p == ' ') ++p;

      #if HAS_MMU2
        if (letter == 'T') {
          // check for special MMU2 T?/Tx/Tc commands
          if (*p == '?' || *p == 'x' || *p == 'c') {
            command_letter = letter;
            string_arg = p;
            return;
          }
        }
      #endif

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

      break;

    default: return;
  }

  // The command parameters (if any) start here, for sure!

  #if DISABLED(FASTER_GCODE_PARSER)
    command_args = p; // Scan for parameters in seen()
  #endif

  // Only use string_arg for these M codes
  if (letter == 'M') switch (codenum) {
    case 23: case 28: case 30: case 117: case 118: case 928:
      string_arg = unescape_string(p);
      return;
    default: break;
  }

  #if ENABLED(DEBUG_GCODE_PARSER)
    const bool debug = (codenum == 1000);
  #endif

  /**
   * Find all parameters, set flags and pointers for fast parsing
   *
   * Most codes ignore 'string_arg', but those that want a string will get the right pointer.
   * The following loop assigns the first "parameter" having no numeric value to 'string_arg'.
   * This allows M0/M1 with expire time to work: "M0 S5 You Win!"
   * For 'M118' you must use 'E1' and 'A1' rather than just 'E' or 'A'
   */
  bool quoted_string_arg = false;
  string_arg = nullptr;
  while (const char param = uppercase(*p++)) {  // Get the next parameter. A NUL ends the loop

    // Special handling for M32 [P] !/path/to/file.g#
    // The path must be the last parameter
    if (param == '!' && letter == 'M' && codenum == 32) {
      string_arg = p;                           // Name starts after '!'
      char * const lb = strchr(p, '#');         // Already seen '#' as SD char (to pause buffering)
      if (lb) *lb = '\0';                       // Safe to mark the end of the filename
      return;
    }

    if (!quoted_string_arg && param == '"') {
      quoted_string_arg = true;
      string_arg = unescape_string(p);
    }

    #if ENABLED(FASTER_GCODE_PARSER)
      // Arguments MUST be uppercase for fast GCode parsing
      #define PARAM_OK(P) WITHIN((P), 'A', 'Z')
    #else
      #define PARAM_OK(P) true
    #endif

    if (PARAM_OK(param)) {

      while (*p == ' ') ++p;                    // skip spaces between parameters & values

      const bool is_str = (*p == '"'), has_val = is_str || valid_float(p);
      char * const valptr = has_val ? is_str ? unescape_string(p) : p : nullptr;

      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) {
          SERIAL_MT("Got param ", param);
          SERIAL_MV(" at index ", (int)(p - command_ptr - 1));
          if (has_val) SERIAL_MSG(" (has_val)");
        }
      #endif

      if (!has_val && !string_arg) {            // No value? First time, keep as string_arg
        string_arg = p - 1;
        #if ENABLED(DEBUG_GCODE_PARSER)
          if (debug) SERIAL_MV(" string_arg: ", hex_address((void*)string_arg)); // DEBUG
        #endif
      }

      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_EOL();
      #endif

      #if ENABLED(FASTER_GCODE_PARSER)
        set(param, valptr);                     // Set parameter exists and pointer (nullptr for no value)
      #endif
    }
    else if (!string_arg) {                     // Not A-Z? First time, keep as the string_arg
      string_arg = p - 1;
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (debug) SERIAL_MV(" string_arg: ", hex_address((void*)string_arg)); // DEBUG
      #endif
    }

    if (!WITHIN(*p, 'A', 'Z')) {                // Another parameter right away?
      while (*p && DECIMAL_SIGNED(*p)) p++;     // Skip over the value section of a parameter
      while (*p == ' ') ++p;                    // Skip over all spaces
    }
  }
}

#if ENABLED(INCH_MODE_SUPPORT)

  float GCodeParser::axis_unit_factor(const AxisEnum axis) {
    return (axis >= E_AXIS && toolManager.isVolumetric() ? volumetric_unit_factor : linear_unit_factor);
  }

#endif

#if ENABLED(DEBUG_GCODE_PARSER)

  void GCodeParser::debug() {
    SERIAL_MT("Command: ", command_ptr);
    SERIAL_MT(" (", command_letter);
    SERIAL_VAL(codenum);
    SERIAL_EM(")");
    #if ENABLED(FASTER_GCODE_PARSER)
      SERIAL_MSG(" args: \"");
      for (char c = 'A'; c <= 'Z'; ++c)
        if (seen(c)) { SERIAL_CHR(c); SERIAL_CHR(' '); }
    #else
      SERIAL_MV(" args: \"", command_args);
    #endif
    SERIAL_CHR('"');
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
