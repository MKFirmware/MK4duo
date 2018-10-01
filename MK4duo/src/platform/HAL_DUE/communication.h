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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#define DEC 10
#define HEX 16
#define OCT  8
#define BIN  2
#define BYTE 0

#if ENABLED(DEBUG_FEATURE)
  void print_xyz(PGM_P prefix, PGM_P suffix, const float x, const float y, const float z);
  void print_xyz(PGM_P prefix, PGM_P suffix, const float xyz[]);
  #if HAS_PLANAR
    void print_xyz(PGM_P prefix, PGM_P suffix, const vector_3 &xyz);
  #endif
  #define DEBUG_POS(SUFFIX,VAR)       do{ \
    print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); }while(0)
#endif

class Com {

  public: /** Public Parameters */

    FSTRINGVAR(tStart)                    // start for host
    FSTRINGVAR(tOk)                       // ok answer for host
    FSTRINGVAR(tOkSpace)                  // ok space answer for host
    FSTRINGVAR(tError)                    // error for host
    FSTRINGVAR(tWait)                     // wait for host
    FSTRINGVAR(tEcho)                     // message for user
    FSTRINGVAR(tConfig)                   // config for host
    FSTRINGVAR(tCap)                      // capabilities for host
    FSTRINGVAR(tInfo)                     // info for host
    FSTRINGVAR(tBusy)                     // buys for host
    FSTRINGVAR(tResend)                   // resend for host
    FSTRINGVAR(tWarning)                  // warning for host
    FSTRINGVAR(tNAN)                      // NAN for host
    FSTRINGVAR(tINF)                      // INF for host
    FSTRINGVAR(tPauseCommunication)       // command for host that support action
    FSTRINGVAR(tContinueCommunication)    // command for host that support action
    FSTRINGVAR(tDisconnectCommunication)  // command for host that support action
    FSTRINGVAR(tRequestPauseCommunication)// command for host that support action

  public: /** Public Function */

    static void serialprintPGM(FSTRINGPARAM(str));

    FORCE_INLINE static void write(const uint8_t c) { HAL::serialWriteByte(c); }
    FORCE_INLINE static void write(PGM_P str) { while (*str) write(*str++); }
    FORCE_INLINE static void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
    FORCE_INLINE static void print(const String& s) { for (int i = 0; i < (int)s.length(); i++) write(s[i]); }
    FORCE_INLINE static void print(PGM_P str) { write(str); }
    FORCE_INLINE static void print_spaces(uint8_t count) { count *= (PROPORTIONAL_FONT_RATIO); while (count--) HAL::serialWriteByte(' '); }

    static void print(char, int = BYTE);
    static void print(unsigned char, int = DEC);
    static void print(int, int = DEC);
    static void print(unsigned int, int = DEC);
    static void print(long, int = DEC);
    static void print(unsigned long, int = DEC);
    static void print(double, int = 2);
    static void println(void);
    operator bool() { return true; }

  private: /** Private Function */

    static void printNumber(unsigned long, const uint8_t);
    static void printFloat(double, uint8_t);

};

#define START           Com::tStart
#define OK              Com::tOk
#define OKSPACE         Com::tOkSpace
#define ER              Com::tError
#define WT              Com::tWait
#define ECHO            Com::tEcho
#define CFG             Com::tConfig
#define CAP             Com::tCap
#define INFO            Com::tInfo
#define BUSY            Com::tBusy
#define RESEND          Com::tResend
#define WARNING         Com::tWarning
#define TNAN            Com::tNAN
#define TINF            Com::tINF
#define PAUSE           Com::tPauseCommunication
#define RESUME          Com::tContinueCommunication
#define DISCONNECT      Com::tDisconnectCommunication
#define REQUESTPAUSE    Com::tRequestPauseCommunication

#define SERIAL_INIT(baud)                   HAL::serialSetBaudrate(baud)

#define SERIAL_PS(message)                  (Com::serialprintPGM(message))
#define SERIAL_PGM(message)                 (Com::serialprintPGM(PSTR(message)))

#define SERIAL_STR(str)                     SERIAL_PGM(str)
#define SERIAL_MSG(msg)                     SERIAL_PGM(msg)
#define SERIAL_TXT(txt)                     (Com::print(txt))
#define SERIAL_VAL(val, ...)                (Com::print(val, ## __VA_ARGS__))
#define SERIAL_CHR(c)                       (Com::write(c))
#define SERIAL_EOL()                        (Com::println())

#define SERIAL_SP(C)                        (Com::print_spaces(C))

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

#endif
