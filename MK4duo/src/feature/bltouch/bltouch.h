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

/**
 * bltouch.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(BLTOUCH)

#define BLTOUCH_DEPLOY     10
#define BLTOUCH_SW_MODE    60
#define BLTOUCH_STOW       90
#define BLTOUCH_SELFTEST  120
#define BLTOUCH_5V_MODE   140
#define BLTOUCH_OD_MODE   150
#define BLTOUCH_RESET     160

/**
 * The following commands may require different delays.
 *
 * ANTClabs recommends 2000ms for 5V/OD commands. However it is
 * not common for other commands to immediately follow these,
 * and testing has shown that these complete in 500ms reliably.
 *
 * AntClabs recommends 750ms for Deploy/Stow, otherwise you will
 * not catch an alarm state until the following move command.
 */

#ifndef BLTOUCH_SET5V_DELAY
  #define BLTOUCH_SET5V_DELAY   BLTOUCH_DELAY
#endif
#ifndef BLTOUCH_SETOD_DELAY
  #define BLTOUCH_SETOD_DELAY   BLTOUCH_DELAY
#endif
#ifndef BLTOUCH_DEPLOY_DELAY
  #define BLTOUCH_DEPLOY_DELAY  750
#endif
#ifndef BLTOUCH_STOW_DELAY
  #define BLTOUCH_STOW_DELAY    750
#endif
#ifndef BLTOUCH_RESET_DELAY
  #define BLTOUCH_RESET_DELAY   BLTOUCH_DELAY
#endif

typedef unsigned char BLTCommand;

class BLTouch {

  public: /** Constructor */

    BLTouch() {};

  public: /** Public Function */

    static void init();
    static bool test();

    static bool deploy();
    static bool stow();
    static bool status();

    FORCE_INLINE static bool set_deployed(const bool dep) { if (dep) return deploy(); else return stow(); }

    FORCE_INLINE static void cmd_reset()    { command(BLTOUCH_RESET,    BLTOUCH_RESET_DELAY); }
    FORCE_INLINE static void cmd_selftest() { command(BLTOUCH_SELFTEST, BLTOUCH_DELAY);       }

    FORCE_INLINE static void cmd_SW_mode()  { command(BLTOUCH_SW_MODE, BLTOUCH_DELAY);        }
    FORCE_INLINE static void cmd_5V_mode()  { command(BLTOUCH_5V_MODE, BLTOUCH_SET5V_DELAY);  }
    FORCE_INLINE static void cmd_OD_mode()  { command(BLTOUCH_OD_MODE, BLTOUCH_SETOD_DELAY);  }

    FORCE_INLINE static void cmd_deploy()   { command(BLTOUCH_DEPLOY, BLTOUCH_DEPLOY_DELAY);  }
    FORCE_INLINE static void cmd_stow()     { command(BLTOUCH_STOW,   BLTOUCH_STOW_DELAY);    }

  private: /** Private Function */

    static void clear();
    static bool command(const BLTCommand cmd, const millis_s &ms);

    FORCE_INLINE static bool cmd_deploy_alarm() { return command(BLTOUCH_DEPLOY,  BLTOUCH_DEPLOY_DELAY);  }
    FORCE_INLINE static bool cmd_stow_alarm()   { return command(BLTOUCH_STOW,    BLTOUCH_STOW_DELAY);    }

};

extern BLTouch bltouch;

#endif // ENABLED(BLTOUCH)
