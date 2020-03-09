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

// String for HOST in Progmem
FSTRINGVAR(START);              // start for host
FSTRINGVAR(OK);                 // ok answer for host
FSTRINGVAR(OKSPACE);            // ok space answer for host
FSTRINGVAR(ER);                 // error for host
FSTRINGVAR(WT);                 // wait for host
FSTRINGVAR(DEB);                // message for debug
FSTRINGVAR(ECHO);               // message for user
FSTRINGVAR(CFG);                // config for host
FSTRINGVAR(CAP);                // capabilities for host
FSTRINGVAR(INFO);               // info for host
FSTRINGVAR(BUSY);               // buys for host
FSTRINGVAR(RESEND);             // resend for host
FSTRINGVAR(WARNING);            // warning for host
FSTRINGVAR(TNAN);               // NAN for host
FSTRINGVAR(TINF);               // INF for host
FSTRINGVAR(REQUESTPAUSE);       // command for host that support action
FSTRINGVAR(REQUESTCONTINUE);    // command for host that support action
FSTRINGVAR(REQUESTSTOP);        // command for host that support action
FSTRINGVAR(NULL_STR);           // NULL string

// String for Gcode in Progmen
FSTRINGVAR(G28_CMD);            // G28 command
FSTRINGVAR(M21_CMD);            // M21 command
FSTRINGVAR(M22_CMD);            // M22 command
FSTRINGVAR(M23_CMD);            // M23 command
FSTRINGVAR(M24_CMD);            // M24 command

class Com {

  public: /** Public Parameters */

    static int8_t serial_port_index;

  public: /** Public Function */

    static void setBaudrate();

    static void serialFlush();

    static int serialRead(const uint8_t index);

    static bool serialDataAvailable();
    static bool serialDataAvailable(const uint8_t index);

    // Functions for serial printing from PROGMEM. (Saves loads of SRAM.)
    static void printPGM(PGM_P);

    static void print_spaces(uint8_t count);

    static void print_logic(PGM_P const label, const bool logic);
    static void print_onoff(PGM_P const label, const bool on);
    static void print_yesno(PGM_P const label, const bool yes);

    // A delay to provide brittle hosts time to receive bytes
    static void serial_delay(const millis_l ms);

};

// MACRO FOR SERIAL
#if ENABLED(SERIAL_PORT_2) && SERIAL_PORT_2 >= -1
  #define NUM_SERIAL 2
#else
  #define NUM_SERIAL 1
#endif

#if NUM_SERIAL > 1
  #define SERIAL_OUT(WHAT,V...) do{ \
    if (Com::serial_port_index == -1 || Com::serial_port_index == 0) (void)MKSERIAL1.WHAT(V); \
    if (Com::serial_port_index == -1 || Com::serial_port_index == 1) (void)MKSERIAL2.WHAT(V); \
  }while(0)
#else
  #define SERIAL_OUT(WHAT,V...)       (void)MKSERIAL1.WHAT(V)
#endif

#define SERIAL_PORT(p)                Com::serial_port_index = p

#define SERIAL_STR(str)               Com::printPGM(str)
#define SERIAL_MSG(msg)               Com::printPGM(PSTR(msg))
#define SERIAL_TXT(txt)               SERIAL_OUT(print, txt)
#define SERIAL_VAL(V...)              SERIAL_OUT(print, V)
#define SERIAL_CHR(c)                 SERIAL_OUT(write, c)
#define SERIAL_EOL()                  SERIAL_CHR('\n')

#define SERIAL_SP(C)                  Com::print_spaces(C)
#define SERIAL_LOGIC(msg,val)         Com::print_logic(PSTR(msg), val)
#define SERIAL_ONOFF(msg,val)         Com::print_onoff(PSTR(msg), val)
#define SERIAL_YESNO(msg,val)         Com::print_yesno(PSTR(msg), val)

#define SERIAL_ELOGIC(msg,val)        do{ SERIAL_LOGIC(msg, val);                     SERIAL_EOL(); }while(0)
#define SERIAL_EONOFF(msg,val)        do{ SERIAL_ONOFF(msg, val);                     SERIAL_EOL(); }while(0)
#define SERIAL_EYESNO(msg,val)        do{ SERIAL_YESNO(msg, val);                     SERIAL_EOL(); }while(0)

#define SERIAL_MT(msg,txt)            do{ SERIAL_MSG(msg);  SERIAL_TXT(txt);                        }while(0)
#define SERIAL_MV(msg,val,V...)       do{ SERIAL_MSG(msg);  SERIAL_VAL(val, ##V);                   }while(0)
#define SERIAL_MC(msg,c)              do{ SERIAL_MSG(msg);  SERIAL_CHR(c);                          }while(0)

#define SERIAL_SM(str,msg)            do{ SERIAL_STR(str);  SERIAL_MSG(msg);                        }while(0)
#define SERIAL_ST(str,txt)            do{ SERIAL_STR(str);  SERIAL_TXT(txt);                        }while(0)
#define SERIAL_SV(str,val,V...)       do{ SERIAL_STR(str);  SERIAL_VAL(val, ##V);                   }while(0)
#define SERIAL_SSM(str1,str2,msg)     do{ SERIAL_STR(str1); SERIAL_SM(str2, msg);                   }while(0)
#define SERIAL_SMT(str,msg,txt)       do{ SERIAL_STR(str);  SERIAL_MT(msg, txt);                    }while(0)
#define SERIAL_SMV(str,msg,val,V...)  do{ SERIAL_STR(str);  SERIAL_MV(msg, val, ##V);               }while(0)

#define SERIAL_EM(msg)                do{ SERIAL_MSG(msg);                            SERIAL_EOL(); }while(0)
#define SERIAL_ET(txt)                do{ SERIAL_TXT(txt);                            SERIAL_EOL(); }while(0)
#define SERIAL_EV(val,V...)           do{ SERIAL_VAL(val, ##V);                       SERIAL_EOL(); }while(0)
#define SERIAL_EMT(msg,txt)           do{ SERIAL_MT(msg, txt);                        SERIAL_EOL(); }while(0)
#define SERIAL_EMV(msg,val,V...)      do{ SERIAL_MV(msg, val, ##V);                   SERIAL_EOL(); }while(0)

#define SERIAL_L(str)                 do{ SERIAL_STR(str);                            SERIAL_EOL(); }while(0)
#define SERIAL_LS(str1,str2)          do{ SERIAL_STR(str1); SERIAL_STR(str2);         SERIAL_EOL(); }while(0)
#define SERIAL_LM(str,msg)            do{ SERIAL_STR(str);  SERIAL_MSG(msg);          SERIAL_EOL(); }while(0)
#define SERIAL_LT(str,txt)            do{ SERIAL_STR(str);  SERIAL_TXT(txt);          SERIAL_EOL(); }while(0)
#define SERIAL_LV(str,val,V...)       do{ SERIAL_STR(str);  SERIAL_VAL(val, ##V);     SERIAL_EOL(); }while(0)
#define SERIAL_LSM(str1,str2,msg)     do{ SERIAL_STR(str1); SERIAL_SM(str2, msg);     SERIAL_EOL(); }while(0)
#define SERIAL_LMT(str,msg,txt)       do{ SERIAL_STR(str);  SERIAL_MT(msg, txt);      SERIAL_EOL(); }while(0)
#define SERIAL_LMV(str,msg,val,V...)  do{ SERIAL_STR(str);  SERIAL_MV(msg, val, ##V); SERIAL_EOL(); }while(0)

// Flush
#define SERIAL_FLUSH()                Com::serialFlush()

// Serial overrun protection
#define SERIAL_DLY(ms)                Com::serial_delay(ms)
