/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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

#if ENABLED(DEBUG_FEATURE)

  class Debug {

    public: /** Constructor */

      Debug() {}

    public: /** Public Function */

      static void log_machine_info();

      static void print_xyz(PGM_P prefix, PGM_P suffix, const float x, const float y, const float z);
      static void print_xyz(PGM_P prefix, PGM_P suffix, const float xyz[]);
      #if HAS_PLANAR
        static void print_xyz(PGM_P prefix, PGM_P suffix, const vector_3 &xyz);
      #endif

  };

  #define DEBUG_LOG_INFO()                Debug::log_machine_info()

  #define DEBUG_POS(SUFFIX,VAR)           Debug::print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR)
  #define DEBUG_XYZ(PREF,SUFF,X,Y,Z)      Debug::print_xyz(PREF, SUFF, X, Y, Z)

  #define DEBUG_STR(str)                  Com::printPGM(str)
  #define DEBUG_MSG(msg)                  Com::printPGM(PSTR(msg))
  #define DEBUG_TXT(txt)                  Com::print(txt)
  #define DEBUG_VAL(val, ...)             Com::print(val, ## __VA_ARGS__)
  #define DEBUG_CHR(c)                    Com::write(c)
  #define DEBUG_EOL()                     Com::println()

  #define DEBUG_SP(C)                     Com::print_spaces(C)
  #define DEBUG_LOGIC(msg, val)           Com::print_logic(PSTR(msg), val)
  #define DEBUG_ONOFF(msg, val)           Com::print_onoff(PSTR(msg), val)

  #define DEBUG_ELOGIC(msg, val)          do{ DEBUG_LOGIC(msg, val); DEBUG_EOL(); }while(0)
  #define DEBUG_EONOFF(msg, val)          do{ DEBUG_ONOFF(msg, val); DEBUG_EOL(); }while(0)

  #define DEBUG_MT(msg, txt)              do{ DEBUG_MSG(msg); DEBUG_TXT(txt); }while(0)
  #define DEBUG_MV(msg, val, ...)         do{ DEBUG_MSG(msg); DEBUG_VAL(val, ## __VA_ARGS__); }while(0)

  #define DEBUG_SM(str, msg)              do{ DEBUG_STR(str); DEBUG_MSG(msg); }while(0)
  #define DEBUG_ST(str, txt)              do{ DEBUG_STR(str); DEBUG_TXT(txt); }while(0)
  #define DEBUG_SV(str, val, ...)         do{ DEBUG_STR(str); DEBUG_VAL(val, ## __VA_ARGS__); }while(0)
  #define DEBUG_SMT(str, msg, txt)        do{ DEBUG_STR(str); DEBUG_MT(msg, txt); }while(0)
  #define DEBUG_SMV(str, msg, val, ...)   do{ DEBUG_STR(str); DEBUG_MV(msg, val, ## __VA_ARGS__); }while(0)

  #define DEBUG_EM(msg)                   do{ DEBUG_MSG(msg); DEBUG_EOL(); }while(0)
  #define DEBUG_ET(txt)                   do{ DEBUG_TXT(txt); DEBUG_EOL(); }while(0)
  #define DEBUG_EV(val, ...)              do{ DEBUG_VAL(val, ## __VA_ARGS__); DEBUG_EOL(); }while(0)
  #define DEBUG_EMT(msg, txt)             do{ DEBUG_MT(msg, txt); DEBUG_EOL(); }while(0)
  #define DEBUG_EMV(msg, val, ...)        do{ DEBUG_MV(msg, val, ## __VA_ARGS__); DEBUG_EOL(); }while(0)

  #define DEBUG_L(str)                    do{ DEBUG_STR(str); DEBUG_EOL(); }while(0)
  #define DEBUG_LM(str, msg)              do{ DEBUG_STR(str); DEBUG_MSG(msg); DEBUG_EOL(); }while(0)
  #define DEBUG_LT(str, txt)              do{ DEBUG_STR(str); DEBUG_TXT(txt); DEBUG_EOL(); }while(0)
  #define DEBUG_LV(str, val, ...)         do{ DEBUG_STR(str); DEBUG_VAL(val, ## __VA_ARGS__); DEBUG_EOL(); }while(0)
  #define DEBUG_LMT(str, msg, txt)        do{ DEBUG_STR(str); DEBUG_MT(msg, txt); DEBUG_EOL(); }while(0)
  #define DEBUG_LMV(str, msg, val, ...)   do{ DEBUG_STR(str); DEBUG_MV(msg, val, ## __VA_ARGS__); DEBUG_EOL(); }while(0)

#else

  #define DEBUG_LOG_INFO()                NOOP

  #define DEBUG_POS(...)                  NOOP
  #define DEBUG_XYZ(...)                  NOOP

  #define DEBUG_STR(...)                  NOOP
  #define DEBUG_MSG(...)                  NOOP
  #define DEBUG_TXT(...)                  NOOP
  #define DEBUG_VAL(...)                  NOOP
  #define DEBUG_CHR(...)                  NOOP
  #define DEBUG_EOL()                     NOOP

  #define DEBUG_SP(...)                   NOOP
  #define DEBUG_LOGIC(...)                NOOP
  #define DEBUG_ONOFF(...)                NOOP

  #define DEBUG_ELOGIC(...)               NOOP
  #define DEBUG_EONOFF(...)               NOOP

  #define DEBUG_MT(...)                   NOOP
  #define DEBUG_MV(...)                   NOOP

  #define DEBUG_SM(...)                   NOOP
  #define DEBUG_ST(...)                   NOOP
  #define DEBUG_SV(...)                   NOOP
  #define DEBUG_SMT(...)                  NOOP
  #define DEBUG_SMV(...)                  NOOP

  #define DEBUG_EM(...)                   NOOP
  #define DEBUG_ET(...)                   NOOP
  #define DEBUG_EV(...)                   NOOP
  #define DEBUG_EMT(...)                  NOOP
  #define DEBUG_EMV(...)                  NOOP

  #define DEBUG_L(...)                    NOOP
  #define DEBUG_LM(...)                   NOOP
  #define DEBUG_LT(...)                   NOOP
  #define DEBUG_LV(...)                   NOOP
  #define DEBUG_LMT(...)                  NOOP
  #define DEBUG_LMV(...)                  NOOP

#endif // ENABLED(DEBUG_FEATURE)
