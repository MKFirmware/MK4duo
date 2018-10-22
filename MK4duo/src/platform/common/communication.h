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

#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#define DEC 10
#define HEX 16
#define OCT  8
#define BIN  2
#define BYTE 0

FSTRINGVAR(START);        // start for host
FSTRINGVAR(OK);           // ok answer for host
FSTRINGVAR(OKSPACE);      // ok space answer for host
FSTRINGVAR(ER);           // error for host
FSTRINGVAR(WT);           // wait for host
FSTRINGVAR(ECHO);         // message for user
FSTRINGVAR(CFG);          // config for host
FSTRINGVAR(CAP);          // capabilities for host
FSTRINGVAR(INFO);         // info for host
FSTRINGVAR(BUSY);         // buys for host
FSTRINGVAR(RESEND);       // resend for host
FSTRINGVAR(WARNING);      // warning for host
FSTRINGVAR(TNAN);         // NAN for host
FSTRINGVAR(TINF);         // INF for host
FSTRINGVAR(PAUSE);        // command for host that support action
FSTRINGVAR(RESUME);       // command for host that support action
FSTRINGVAR(DISCONNECT);   // command for host that support action
FSTRINGVAR(POWEROFF);     // command for host that support action
FSTRINGVAR(REQUESTPAUSE); // command for host that support action

class Com {

  public: /** Public Parameters */

    static int8_t serial_port;

  public: /** Public Function */

    static void setBaudrate();

    static void serialFlush();

    static int serialRead(const uint8_t index);

    static bool serialDataAvailable();

    // Functions for serial printing from PROGMEM. (Saves loads of SRAM.)
    static void printPGM(PGM_P);

    static void write(char);
    static void write(PGM_P);
    static void write(const uint8_t* buffer, size_t size);
    static void print(const String& s);
    static void print(PGM_P);

    static void print(char, int = BYTE);
    static void print(unsigned char, int = DEC);
    static void print(int, int = DEC);
    static void print(unsigned int, int = DEC);
    static void print(long, int = DEC);
    static void print(unsigned long, int = DEC);
    static void print(double, int = 2);

    static void println(void);
    operator bool() { return true; }

    static void print_spaces(uint8_t count);

    static void print_logic(PGM_P const label, const bool logic);

    #if ENABLED(DEBUG_FEATURE)
      static void print_xyz(PGM_P prefix, PGM_P suffix, const float x, const float y, const float z);
      static void print_xyz(PGM_P prefix, PGM_P suffix, const float xyz[]);
      #if HAS_PLANAR
        static void print_xyz(PGM_P prefix, PGM_P suffix, const vector_3 &xyz);
      #endif
      #define DEBUG_POS(SUFFIX,VAR)       do{ \
        Com::print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); }while(0)
    #endif

  private: /** Private Function */

      static void printNumber(unsigned long, const uint8_t);
      static void printFloat(double, uint8_t);

};

// MACRO FOR SERIAL
#define SERIAL_PORT(p)                      (Com::serial_port = p)

#define SERIAL_PS(message)                  (Com::printPGM(message))
#define SERIAL_PGM(message)                 (Com::printPGM(PSTR(message)))

#define SERIAL_STR(str)                     SERIAL_PS(str)
#define SERIAL_MSG(msg)                     SERIAL_PGM(msg)
#define SERIAL_TXT(txt)                     (Com::print(txt))
#define SERIAL_VAL(val, ...)                (Com::print(val, ## __VA_ARGS__))
#define SERIAL_CHR(c)                       (Com::write(c))
#define SERIAL_EOL()                        (Com::println())

#define SERIAL_SP(C)                        (Com::print_spaces(C))
#define SERIAL_LOGIC(msg, val)              (Com::print_logic(PSTR(msg), val))

#define SERIAL_MT(msg, txt)                 do{ SERIAL_MSG(msg); SERIAL_TXT(txt); }while(0)
#define SERIAL_MV(msg, val, ...)            do{ SERIAL_MSG(msg); SERIAL_VAL(val, ## __VA_ARGS__); }while(0)

#define SERIAL_SM(str, msg)                 do{ SERIAL_STR(str); SERIAL_MSG(msg); }while(0)
#define SERIAL_ST(str, txt)                 do{ SERIAL_STR(str); SERIAL_TXT(txt); }while(0)
#define SERIAL_SV(str, val, ...)            do{ SERIAL_STR(str); SERIAL_VAL(val, ## __VA_ARGS__); }while(0)
#define SERIAL_SMT(str, msg, txt)           do{ SERIAL_STR(str); SERIAL_MT(msg, txt); }while(0)
#define SERIAL_SMV(str, msg, val, ...)      do{ SERIAL_STR(str); SERIAL_MV(msg, val, ## __VA_ARGS__); }while(0)

#define SERIAL_EM(msg)                      do{ SERIAL_MSG(msg); SERIAL_EOL(); }while(0)
#define SERIAL_ET(txt)                      do{ SERIAL_TXT(txt); SERIAL_EOL(); }while(0)
#define SERIAL_EV(val, ...)                 do{ SERIAL_VAL(val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)
#define SERIAL_EMT(msg, txt)                do{ SERIAL_MT(msg, txt); SERIAL_EOL(); }while(0)
#define SERIAL_EMV(msg, val, ...)           do{ SERIAL_MV(msg, val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)

#define SERIAL_L(str)                       do{ SERIAL_STR(str); SERIAL_EOL(); }while(0)
#define SERIAL_LM(str, msg)                 do{ SERIAL_STR(str); SERIAL_MSG(msg); SERIAL_EOL(); }while(0)
#define SERIAL_LT(str, txt)                 do{ SERIAL_STR(str); SERIAL_TXT(txt); SERIAL_EOL(); }while(0)
#define SERIAL_LV(str, val, ...)            do{ SERIAL_STR(str); SERIAL_VAL(val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)
#define SERIAL_LMT(str, msg, txt)           do{ SERIAL_STR(str); SERIAL_MT(msg, txt); SERIAL_EOL(); }while(0)
#define SERIAL_LMV(str, msg, val, ...)      do{ SERIAL_STR(str); SERIAL_MV(msg, val, ## __VA_ARGS__); SERIAL_EOL(); }while(0)

#endif /* _COMMUNICATION_H_ */
