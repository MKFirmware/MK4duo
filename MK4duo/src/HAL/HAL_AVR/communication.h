/**
 * MK & MK4due 3D Printer Firmware
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

#define SERIAL_INIT(baud)                   MKSERIAL.begin(baud), HAL::delayMilliseconds(1)
#define SERIAL_WRITE(x)                     MKSERIAL.write(x)
#define SERIAL_PRINT(msg, ...)              MKSERIAL.print(msg, ## __VA_ARGS__)
#define SERIAL_EOL                          MKSERIAL.println()

// Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char* str) {
  while (char ch = pgm_read_byte(str++)) MKSERIAL.write(ch);
}

#define SERIAL_PS(message)                  serialprintPGM(message)
#define SERIAL_PGM(message)                 serialprintPGM(PSTR(message))

#define SERIAL_S(srt)                       SERIAL_PGM(srt)
#define SERIAL_M(msg)                       SERIAL_PGM(msg)
#define SERIAL_T(txt)                       SERIAL_PRINT(txt)
#define SERIAL_V(val, ...)                  SERIAL_PRINT(val, ## __VA_ARGS__)
#define SERIAL_C(c)                         SERIAL_WRITE(c)
#define SERIAL_E                            SERIAL_EOL

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
