/**
 * MK & MK4due 3D Printer Firmware
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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

/**
 * Define debug bit-masks
 */
enum DebugFlags {
  DEBUG_NONE          = 0,
  DEBUG_ECHO          = _BV(0), ///< Echo commands in order as they are processed
  DEBUG_INFO          = _BV(1), ///< Print messages for code that has debug output
  DEBUG_ERRORS        = _BV(2), ///< Not implemented
  DEBUG_DRYRUN        = _BV(3), ///< Ignore temperature setting and E movement commands
  DEBUG_COMMUNICATION = _BV(4), ///< Not implemented
  DEBUG_LEVELING      = _BV(5), ///< Print detailed output for homing and leveling
  DEBUG_MESH_ADJUST   = _BV(6), ///< UBL bed leveling
  DEBUG_ALL           = 0xFF
};

extern uint8_t mk_debug_flags;
#define DEBUGGING(F)    (mk_debug_flags & (DEBUG_## F))

#define START           "start"               // start for host
#define OK              "ok"                  // ok answer for host
#define OKSPACE         "ok "                 // ok plus space 
#define ER              "Error:"              // error for host
#define WT              "wait"                // wait for host
#define ECHO            "Echo:"               // message for user
#define CFG             "Config:"             // config for host
#define CAP             "Cap:"                // capabilities for host
#define INFO            "Info:"               // info for host
#define BUSY            "busy:"               // buys for host
#define RESEND          "Resend:"             // resend for host
#define WARNING         "Warning:"            // warning for host
#define TNAN            "NAN"                 // NAN for host
#define TINF            "INF"                 // INF for host
#define PAUSE           "//action:pause"      // command for host that support action
#define RESUME          "//action:resume"     // command for host that support action
#define DISCONNECT      "//action:disconnect" // command for host that support action
#define REQUEST_PAUSE   "RequestPause:"       // command for host that support action

#define SERIAL_INIT(baud)                   do{ MKSERIAL.begin(baud); HAL::delayMilliseconds(1); }while(0)

// Functions for serial printing from PROGMEM. (Saves loads of SRAM.)
void serialprintPGM(const char* str);

#define SERIAL_PS(message)                  (serialprintPGM(message))
#define SERIAL_PGM(message)                 (serialprintPGM(PSTR(message)))

#define SERIAL_STR(str)                     SERIAL_PGM(str)
#define SERIAL_MSG(msg)                     SERIAL_PGM(msg)
#define SERIAL_TXT(txt)                     (serial_print(txt))
#define SERIAL_VAL(val, ...)                (serial_print(val, ## __VA_ARGS__))
#define SERIAL_CHR(c)                       ((void)MKSERIAL.write(c))
#define SERIAL_EOL()                        ((void)MKSERIAL.write('\n'))

#define SERIAL_SP(C)                        serial_spaces(C)

#define SERIAL_MT(msg, txt)                 (serial_print_pair(PSTR(msg), txt))
#define SERIAL_MV(msg, val, ...)            (serial_print_pair(PSTR(msg), val, ## __VA_ARGS__))

#define SERIAL_SM(str, msg)                 do{ SERIAL_STR(str); SERIAL_MSG(msg); }while(0)
#define SERIAL_ST(str, txt)                 do{ SERIAL_STR(str); SERIAL_TXT(txt); }while(0)
#define SERIAL_SV(str, val, ...)            do{ SERIAL_STR(str); SERIAL_VAL(val, ## __VA_ARGS__); }while(0)
#define SERIAL_SMT(str, msg, txt)           do{ SERIAL_STR(str); SERIAL_MT(msg, txt); }while(0)
#define SERIAL_SMV(str, msg, val, ...)      do{ SERIAL_STR(str); SERIAL_MV(msg, val, ## __VA_ARGS__); }while(0)

#define SERIAL_EM(msg)                      (serialprintPGM(PSTR(msg "\n")))
#define SERIAL_ET(txt)                      do{ SERIAL_TXT(txt); SERIAL_EOL(); }while(0)
#define SERIAL_EV(val, ...)                 do{ SERIAL_VAL(val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)
#define SERIAL_EMT(msg, txt)                do{ SERIAL_MT(msg, txt); SERIAL_EOL(); }while(0)
#define SERIAL_EMV(msg, val, ...)           do{ SERIAL_MV(msg, val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)

#define SERIAL_L(str)                       do{ SERIAL_STR(str); SERIAL_EOL(); }while(0)
#define SERIAL_LM(str, msg)                 do{ SERIAL_STR(str); SERIAL_EM(msg); }while(0)
#define SERIAL_LT(str, txt)                 do{ SERIAL_STR(str); SERIAL_TXT(txt); SERIAL_EOL(); }while(0)
#define SERIAL_LV(str, val, ...)            do{ SERIAL_STR(str); SERIAL_VAL(val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)
#define SERIAL_LMT(str, msg, txt)           do{ SERIAL_STR(str); SERIAL_MT(msg, txt); SERIAL_EOL(); }while(0)
#define SERIAL_LMV(str, msg, val, ...)      do{ SERIAL_STR(str); SERIAL_MV(msg, val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)

FORCE_INLINE void serial_print(const char *v)   { MKSERIAL.print(v); }
FORCE_INLINE void serial_print(char v)          { MKSERIAL.print(v); }
FORCE_INLINE void serial_print(int v)           { MKSERIAL.print(v); }
FORCE_INLINE void serial_print(long v)          { MKSERIAL.print(v); }
FORCE_INLINE void serial_print(double v)        { MKSERIAL.print(v); }
FORCE_INLINE void serial_print(float v, int n)  { MKSERIAL.print(v, n); }
FORCE_INLINE void serial_print(uint8_t v)       { MKSERIAL.print((int)v); }
FORCE_INLINE void serial_print(uint16_t v)      { MKSERIAL.print((int)v); }
FORCE_INLINE void serial_print(uint32_t v)      { MKSERIAL.print((long)v); }
FORCE_INLINE void serial_print(bool v)          { MKSERIAL.print((int)v); }
FORCE_INLINE void serial_print(void *v)         { MKSERIAL.print((int)v); }

void serial_print_pair(const char* msg, const char *v);
void serial_print_pair(const char* msg, char v);
void serial_print_pair(const char* msg, int v);
void serial_print_pair(const char* msg, long v);
void serial_print_pair(const char* msg, float v, int n);
void serial_print_pair(const char* msg, double v);
void serial_print_pair(const char* msg, uint8_t v);
void serial_print_pair(const char* msg, uint16_t v);
void serial_print_pair(const char* msg, uint32_t v);
void serial_print_pair(const char* msg, bool v);
void serial_print_pair(const char* msg, void *v);

void serial_spaces(uint8_t count);

#if ENABLED(DEBUG_LEVELING_FEATURE)
  void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z);
  void print_xyz(const char* prefix, const char* suffix, const float xyz[]);
  #if HAS_PLANAR
    void print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz);
  #endif
  #define DEBUG_POS(SUFFIX,VAR)       do{ \
    print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); }while(0)
#endif

#endif /* COMMUNICATION_H */
