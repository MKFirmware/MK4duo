/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

class Com {
  public:
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
    FSTRINGVAR(tPowerUp)
    FSTRINGVAR(tBrownOut)
    FSTRINGVAR(tWatchdog)
    FSTRINGVAR(tSoftwareReset)
    FSTRINGVAR(tExternalReset)


    static void printInfoLN(FSTRINGPARAM(text));
    static void PS_PGM(FSTRINGPARAM(text));
    static void printNumber(uint32_t n);
    static void printFloat(float number, uint8_t digits);
    static void print(const char* text);
    static void print(long value);
    static inline void print(char c) { HAL::serialWriteByte(c); }
    static inline void print(uint32_t value) { printNumber(value); }
    static inline void print(int value) { print((int32_t)value); }
    static inline void print(uint16_t value) { print((int32_t)value); }
    static inline void print(float number) { printFloat(number, 6); }
    static inline void print(float number, uint8_t digits) { printFloat(number, digits); }
    static inline void print(double number) { printFloat(number, 6); }
    static inline void print(double number, uint8_t digits) { printFloat(number, digits); }
    static inline void println() { HAL::serialWriteByte('\r'); HAL::serialWriteByte('\n'); }

  protected:
  private:
};

#define START       Com::tStart
#define OK          Com::tOk
#define OKSPACE     Com::tOkSpace
#define ER          Com::tError
#define WT          Com::tWait
#define ECHO        Com::tEcho
#define CFG         Com::tConfig
#define CAP         Com::tCap
#define INFO        Com::tInfo
#define BUSY        Com::tBusy
#define RESEND      Com::tResend
#define WARNING     Com::tWarning
#define TNAN        Com::tNAN
#define TINF        Com::tINF
#define PAUSE       Com::tPauseCommunication
#define RESUME      Com::tContinueCommunication
#define DISCONNECT  Com::tDisconnectCommunication

#define SERIAL_INIT(baud)                   HAL::serialSetBaudrate(baud)
#define SERIAL_PRINT(val, ...)              Com::print(val, ## __VA_ARGS__)
#define SERIAL_EOL                          Com::println()

#define SERIAL_PS(message)                  Com::PS_PGM(message)
#define SERIAL_PGM(message)                 Com::PS_PGM(PSTR(message))

#define SERIAL_S(srt)                       Com::PS_PGM(srt)
#define SERIAL_M(msg)                       Com::PS_PGM(PSTR(msg))
#define SERIAL_T(txt)                       Com::print(txt)
#define SERIAL_V(val, ...)                  Com::print(val, ## __VA_ARGS__)
#define SERIAL_C(c)                         Com::print(c)
#define SERIAL_E                            Com::println()

#define SERIAL_MV(msg, val, ...)            SERIAL_M(msg),SERIAL_V(val, ## __VA_ARGS__)
#define SERIAL_MT(msg, txt)                 SERIAL_M(msg),SERIAL_T(txt)

#define SERIAL_SM(srt, msg)                 SERIAL_S(srt),SERIAL_M(msg)
#define SERIAL_SV(srt, val, ...)            SERIAL_S(srt),SERIAL_V(val, ## __VA_ARGS__)
#define SERIAL_ST(srt, txt)                 SERIAL_S(srt),SERIAL_T(txt)
#define SERIAL_SMT(srt, msg, txt)           SERIAL_S(srt),SERIAL_MT(msg, txt)
#define SERIAL_SMV(srt, msg, val, ...)      SERIAL_S(srt),SERIAL_MV(msg, val, ## __VA_ARGS__)

#define SERIAL_EM(msg)                      SERIAL_M(msg),SERIAL_E
#define SERIAL_EV(val, ...)                 SERIAL_V(val, ## __VA_ARGS__),SERIAL_E
#define SERIAL_ET(txt)                      SERIAL_T(txt),SERIAL_E
#define SERIAL_EMT(msg, txt)                SERIAL_MT(msg, txt),SERIAL_E
#define SERIAL_EMV(msg, val, ...)           SERIAL_MV(msg, val, ## __VA_ARGS__),SERIAL_E

#define SERIAL_L(srt)                       SERIAL_S(srt),SERIAL_E
#define SERIAL_LM(srt, msg)                 SERIAL_S(srt),SERIAL_M(msg),SERIAL_E
#define SERIAL_LT(srt, txt)                 SERIAL_S(srt),SERIAL_T(txt),SERIAL_E
#define SERIAL_LMT(srt, msg, txt)           SERIAL_S(srt),SERIAL_MT(msg, txt),SERIAL_E
#define SERIAL_LV(srt, val, ...)            SERIAL_S(srt),SERIAL_V(val, ## __VA_ARGS__),SERIAL_E
#define SERIAL_LMV(srt, msg, val, ...)      SERIAL_S(srt),SERIAL_MV(msg, val, ## __VA_ARGS__),SERIAL_E

#endif
