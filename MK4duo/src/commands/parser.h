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
 * parser.h - Parser for a GCode line, providing a parameter interface.
 *            Codes like M149 control the way the GCode parser behaves,
 *            so settings for these codes are located in this class.
 */

#ifndef _PARSER_H_
#define _PARSER_H_

//#define DEBUG_GCODE_PARSER

// Index so that 'X' falls on index 24
#define PARAM_IND(N)  ((N) >> 3)
#define PARAM_BIT(N)  ((N) & 0x7)
#define LETTER_OFF(N) ((N) - 'A')
#define LETTER_IND(N) PARAM_IND(LETTER_OFF(N))
#define LETTER_BIT(N) PARAM_BIT(LETTER_OFF(N))

typedef enum {
  TEMPUNIT_C,
  TEMPUNIT_K,
  TEMPUNIT_F
} TempUnit;

/**
 * Parser Gcode
 *
 *  - Parse a single gcode line for its letter, code, subcode, and parameters
 *  - FASTER_GCODE_PARSER:
 *    - Flags existing params (1 bit each)
 *    - Stores value offsets (1 byte each)
 *  - Provide accessors for parameters:
 *    - Parameter exists
 *    - Parameter has value
 *    - Parameter value in different units and types
 */
class GCodeParser {

  public: /** Public Parameters */

    // Global states for GCode-level units features
    #if ENABLED(INCH_MODE_SUPPORT)
      static float  linear_unit_factor,
                    volumetric_unit_factor;
    #endif

    #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
      static TempUnit input_temp_units;
    #endif

    // Command line state
    static char *command_ptr,     // The command, so it can be echoed
                *string_arg,      // string of command line
                 command_letter;  // G, M, or T

    static uint16_t codenum;      // 123

    #if USE_GCODE_SUBCODES
      static uint8_t subcode;     // .1
    #endif

  private: /** Private Parameters */

    static char *value_ptr;       // Set by seen, used to fetch the value

    #if ENABLED(FASTER_GCODE_PARSER)
      static byte codebits[4];    // Parameters pre-scanned
      static uint8_t param[26];   // For A-Z, offsets into command args
    #else
      static char *command_args;  // Args start here, for slow scan
    #endif

  public: /** Public Function */

    #if ENABLED(DEBUG_GCODE_PARSER)
      void debug();
    #endif

    #if ENABLED(FASTER_GCODE_PARSER)

      // Set the flag and pointer for a parameter
      static void set(const char c, char * const ptr
        #if ENABLED(DEBUG_GCODE_PARSER)
          , const bool debug=false
        #endif
      ) {
        const uint8_t ind = LETTER_OFF(c);
        if (ind >= COUNT(param)) return;                // Only A-Z
        SBI(codebits[PARAM_IND(ind)], PARAM_BIT(ind));  // parameter exists
        param[ind] = ptr ? ptr - command_ptr : 0;       // parameter offset or 0
        #if ENABLED(DEBUG_GCODE_PARSER)
          if (debug) {
            SERIAL_MV("Set bit ", (int)PARAM_BIT(ind));
            SERIAL_MV(" of index ", (int)PARAM_IND(ind));
            SERIAL_EMV(" | param = ",(int)param[ind]);
          }
        #endif
      }

      // Code seen bit was set. If not found, value_ptr is unchanged.
      // This allows "if (seen('A')||seen('B'))" to use the last-found value.
      // This is volatile because its side-effects are important
      static bool seen(const char c) {
        const uint8_t ind = LETTER_OFF(c);
        if (ind >= COUNT(param)) return false; // Only A-Z
        const bool b = TEST(codebits[PARAM_IND(ind)], PARAM_BIT(ind));
        if (b) value_ptr = param[ind] ? command_ptr + param[ind] : (char*)NULL;
        return b;
      }

      FORCE_INLINE static bool seen_any() { return codebits[3] || codebits[2] || codebits[1] || codebits[0]; }

      #define SEEN_TEST(L) TEST(codebits[LETTER_IND(L)], LETTER_BIT(L))

    #else // !FASTER_GCODE_PARSER

      // Code is found in the string. If not found, value_ptr is unchanged.
      // This allows "if (seen('A')||seen('B'))" to use the last-found value.
      // This is volatile because its side-effects are important
      // p DEVE ESSERE CHAR e non CONST CHAR
      static bool seen(const char c) {
        char *p = strchr(command_args, c);
        const bool b = !!p;
        if (b) value_ptr = DECIMAL_SIGNED(p[1]) ? &p[1] : (char*)NULL;
        return b;
      }

      FORCE_INLINE static bool seen_any() { return *command_args == '\0'; }

      #define SEEN_TEST(L) !!strchr(command_args, L)

    #endif // FASTER_GCODE_PARSER

    // Seen any axis parameter
    FORCE_INLINE static bool seen_axis() { return SEEN_TEST('X') || SEEN_TEST('Y') || SEEN_TEST('Z') || SEEN_TEST('E') ; }

    // Populate all fields by parsing a single line of GCode
    // This uses 54 bytes of SRAM to speed up seen/value
    static void parse(char * p);

    // Code value pointer was set
    FORCE_INLINE static bool has_value() { return value_ptr != NULL; }

    // Seen a parameter with a value
    FORCE_INLINE static bool seenval(const char c) { return seen(c) && has_value(); }

    // Float removes 'E' to prevent scientific notation interpretation
    inline static float value_float() {
      if (value_ptr) {
        char *e = value_ptr;
        for (;;) {
          const char c = *e;
          if (c == '\0' || c == ' ') break;
          if (c == 'E' || c == 'e') {
            *e = '\0';
            const float ret = strtod(value_ptr, NULL);
            *e = c;
            return ret;
          }
          ++e;
        }
        return strtod(value_ptr, NULL);
      }
      return 0.0;
    }

    // Code value as a long or ulong
    FORCE_INLINE static int32_t   value_long()  { return value_ptr ? strtol(value_ptr, NULL, 10) : 0L; }
    FORCE_INLINE static uint32_t  value_ulong() { return value_ptr ? strtoul(value_ptr, NULL, 10) : 0UL; }

    // Code value for use as time
    FORCE_INLINE static millis_t  value_millis()              { return value_ulong(); }
    FORCE_INLINE static millis_t  value_millis_from_seconds() { return value_float() * 1000UL; }

    // Reduce to fewer bits
    FORCE_INLINE static int16_t   value_int()     { return  (int16_t) value_long(); }
    FORCE_INLINE static uint16_t  value_ushort()  { return (uint16_t) value_long(); }
    FORCE_INLINE static uint8_t   value_byte()    { return  (uint8_t) constrain(value_long(), 0, 255); }

    // Bool is true with no value or non-zero
    FORCE_INLINE static bool      value_bool()    { return !has_value() || value_byte(); }

    // Units modes: Inches, Fahrenheit, Kelvin

    #if ENABLED(INCH_MODE_SUPPORT)

      FORCE_INLINE static void set_input_linear_units(float factor) {
        linear_unit_factor = factor;
        volumetric_unit_factor = POW(linear_unit_factor, 3.0);
      }

      FORCE_INLINE static float axis_unit_factor(const AxisEnum axis) {
        return (axis >= E_AXIS && tools.volumetric_enabled ? volumetric_unit_factor : linear_unit_factor);
      }

      FORCE_INLINE static float value_linear_units()                     { return value_float() * linear_unit_factor; }
      FORCE_INLINE static float value_axis_units(const AxisEnum axis)    { return value_float() * axis_unit_factor(axis); }
      FORCE_INLINE static float value_per_axis_unit(const AxisEnum axis) { return value_float() / axis_unit_factor(axis); }

    #else

      FORCE_INLINE static float value_linear_units()                  {            return value_float(); }
      FORCE_INLINE static float value_axis_units(const AxisEnum a)    { UNUSED(a); return value_float(); }
      FORCE_INLINE static float value_per_axis_unit(const AxisEnum a) { UNUSED(a); return value_float(); }

    #endif

    #if ENABLED(TEMPERATURE_UNITS_SUPPORT)

      FORCE_INLINE static void set_input_temp_units(TempUnit units) { input_temp_units = units; }

      #if ENABLED(ULTIPANEL) && DISABLED(DISABLE_M503)

        FORCE_INLINE static char temp_units_code() {
          return input_temp_units == TEMPUNIT_K ? 'K' : input_temp_units == TEMPUNIT_F ? 'F' : 'C';
        }

        FORCE_INLINE static char* temp_units_name() {
          return input_temp_units == TEMPUNIT_K ? PSTR("Kelvin") : input_temp_units == TEMPUNIT_F ? PSTR("Fahrenheit") : PSTR("Celsius");
        }

        inline static float to_temp_units(const float &f) {
          switch (input_temp_units) {
            case TEMPUNIT_F:
              return f * 0.5555555556 + 32.0;
            case TEMPUNIT_K:
              return f + 273.15;
            case TEMPUNIT_C:
            default:
              return f;
          }
        }

      #endif // ULTIPANEL && !DISABLE_M503

      inline static float value_celsius() {
        const float f = value_float();
        switch (input_temp_units) {
          case TEMPUNIT_F:
            return (f - 32.0) * 0.5555555556;
          case TEMPUNIT_K:
            return f - 273.15;
          case TEMPUNIT_C:
          default:
            return f;
        }
      }

      inline static float value_celsius_diff() {
        switch (input_temp_units) {
          case TEMPUNIT_F:
            return value_float() * 0.5555555556;
          case TEMPUNIT_C:
          case TEMPUNIT_K:
          default:
            return value_float();
        }
      }

    #else // !TEMPERATURE_UNITS_SUPPORT

      FORCE_INLINE static float value_celsius()      { return value_float(); }
      FORCE_INLINE static float value_celsius_diff() { return value_float(); }

    #endif // !TEMPERATURE_UNITS_SUPPORT

    FORCE_INLINE static float value_feedrate() { return value_linear_units(); }

    // Provide simple value accessors with default option
    FORCE_INLINE static float    floatval(const char c, const float dval=0.0)   { return seenval(c) ? value_float()        : dval; }
    FORCE_INLINE static bool     boolval(const char c, const bool dval=false)   { return seen(c)    ? value_bool()         : dval; }
    FORCE_INLINE static uint8_t  byteval(const char c, const uint8_t dval=0)    { return seenval(c) ? value_byte()         : dval; }
    FORCE_INLINE static int16_t  intval(const char c, const int16_t dval=0)     { return seenval(c) ? value_int()          : dval; }
    FORCE_INLINE static uint16_t ushortval(const char c, const uint16_t dval=0) { return seenval(c) ? value_ushort()       : dval; }
    FORCE_INLINE static int32_t  longval(const char c, const int32_t dval=0)    { return seenval(c) ? value_long()         : dval; }
    FORCE_INLINE static uint32_t ulongval(const char c, const uint32_t dval=0)  { return seenval(c) ? value_ulong()        : dval; }
    FORCE_INLINE static float    linearval(const char c, const float dval=0.0)  { return seenval(c) ? value_linear_units() : dval; }
    FORCE_INLINE static float    celsiusval(const char c, const float dval=0.0) { return seenval(c) ? value_celsius()      : dval; }

  private: /** Private Function */

    /**
     * Clear all code-seen (and value pointers)
     *
     * Since each param is set/cleared on seen codes,
     * this may be optimized by commenting out ZERO(param)
     *
     * Reset is done only before parsing!
     */
    FORCE_INLINE static void reset() {
      string_arg = NULL;                  // No whole line argument
      command_letter = '?';               // No command letter
      codenum = 0;                        // No command code
      #if USE_GCODE_SUBCODES
        subcode = 0;                      // No command sub-code
      #endif
      #if ENABLED(FASTER_GCODE_PARSER)
        ZERO(codebits);                   // No codes yet
        //ZERO(param);                    // No parameters (should be safe to comment out this line)
      #endif
    }

};

extern GCodeParser parser;

#endif /* _PARSER_H_ */
