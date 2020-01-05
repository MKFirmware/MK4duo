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
#pragma once

#if ENABLED(DEBUG_FEATURE)

  class Debug {

    public: /** Constructor */

      Debug() {}

    public: /** Public Function */

      static void log_machine_info();

      static void print_xyz(const float &x, const float &y, const float &z, PGM_P const prefix=nullptr, PGM_P const suffix=nullptr);

      static inline void print_xyz(const xyz_pos_t &pos, PGM_P const prefix=nullptr, PGM_P const suffix=nullptr) {
        print_xyz(pos.x, pos.y, pos.z, prefix, suffix);
      }
      #if HAS_PLANAR
        static inline void print_xyz(const vector_3 &pos, PGM_P const prefix=nullptr, PGM_P const suffix=nullptr) {
          print_xyz(pos.x, pos.y, pos.z, prefix, suffix);
        }
      #endif

  };

  #define DEBUG_LOG_INFO()            do { Debug::log_machine_info(); }while(0)

  #define DEBUG_POS(SUFFIX,VAR)       do { Debug::print_xyz(VAR, PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"));  }while(0)
  #define DEBUG_XYZ(PREFIX,V...)      do { Debug::print_xyz(V, PSTR(PREFIX), nullptr); }while(0)

  #define DEBUG_STR                   SERIAL_STR
  #define DEBUG_MSG                   SERIAL_MSG
  #define DEBUG_TXT                   SERIAL_TXT
  #define DEBUG_VAL                   SERIAL_VAL
  #define DEBUG_CHR                   SERIAL_CHR
  #define DEBUG_EOL                   SERIAL_EOL

  #define DEBUG_SP                    SERIAL_SP
  #define DEBUG_LOGIC                 SERIAL_LOGIC
  #define DEBUG_ONOFF                 SERIAL_ONOFF

  #define DEBUG_ELOGIC                SERIAL_ELOGIC
  #define DEBUG_EONOFF                SERIAL_EONOFF

  #define DEBUG_MT                    SERIAL_MT
  #define DEBUG_MV                    SERIAL_MV
  #define DEBUG_MC                    SERIAL_MC

  #define DEBUG_SM                    SERIAL_SM
  #define DEBUG_ST                    SERIAL_ST
  #define DEBUG_SV                    SERIAL_SV
  #define DEBUG_SSM                   SERIAL_SSM
  #define DEBUG_SMT                   SERIAL_SMT
  #define DEBUG_SMV                   SERIAL_SMV

  #define DEBUG_EM                    SERIAL_EM
  #define DEBUG_ET                    SERIAL_ET
  #define DEBUG_EV                    SERIAL_EV
  #define DEBUG_EMT                   SERIAL_EMT
  #define DEBUG_EMV                   SERIAL_EMV

  #define DEBUG_L                     SERIAL_L
  #define DEBUG_LS                    SERIAL_LS
  #define DEBUG_LM                    SERIAL_LM
  #define DEBUG_LT                    SERIAL_LT
  #define DEBUG_LV                    SERIAL_LV
  #define DEBUG_LSM                   SERIAL_LSM
  #define DEBUG_LMT                   SERIAL_LMT
  #define DEBUG_LMV                   SERIAL_LMV

#else

  #define DEBUG_LOG_INFO()            NOOP

  #define DEBUG_POS(...)              NOOP
  #define DEBUG_XYZ(...)              NOOP

  #define DEBUG_STR(...)              NOOP
  #define DEBUG_MSG(...)              NOOP
  #define DEBUG_TXT(...)              NOOP
  #define DEBUG_VAL(...)              NOOP
  #define DEBUG_CHR(...)              NOOP
  #define DEBUG_EOL()                 NOOP

  #define DEBUG_SP(...)               NOOP
  #define DEBUG_LOGIC(...)            NOOP
  #define DEBUG_ONOFF(...)            NOOP

  #define DEBUG_ELOGIC(...)           NOOP
  #define DEBUG_EONOFF(...)           NOOP

  #define DEBUG_MT(...)               NOOP
  #define DEBUG_MV(...)               NOOP
  #define DEBUG_MC(...)               NOOP

  #define DEBUG_SM(...)               NOOP
  #define DEBUG_ST(...)               NOOP
  #define DEBUG_SV(...)               NOOP
  #define DEBUG_SSM(...)              NOOP
  #define DEBUG_SMT(...)              NOOP
  #define DEBUG_SMV(...)              NOOP

  #define DEBUG_EM(...)               NOOP
  #define DEBUG_ET(...)               NOOP
  #define DEBUG_EV(...)               NOOP
  #define DEBUG_EMT(...)              NOOP
  #define DEBUG_EMV(...)              NOOP

  #define DEBUG_L(...)                NOOP
  #define DEBUG_LS(...)               NOOP
  #define DEBUG_LM(...)               NOOP
  #define DEBUG_LT(...)               NOOP
  #define DEBUG_LV(...)               NOOP
  #define DEBUG_LSM(...)              NOOP
  #define DEBUG_LMT(...)              NOOP
  #define DEBUG_LMV(...)              NOOP

#endif // ENABLED(DEBUG_FEATURE)
