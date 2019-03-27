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

#define BLTOUCH_DEPLOY    10
#define BLTOUCH_SW_MODE   60
#define BLTOUCH_STOW      90
#define BLTOUCH_SELFTEST 120
#define BLTOUCH_5V_MODE  140
#define BLTOUCH_OD_MODE  150
#define BLTOUCH_RESET    160

class Bltouch {

  public: /** Constructor */

    Bltouch() {};

  public: /** Public Parameters */

  public: /** Public Function */

    static void init();
    static bool test();
    static void command(const int angle);

    // returns false for ok and true for failure
    static bool set_deployed(const bool deploy);

    FORCE_INLINE static bool deploy() { return set_deployed(true); }
    FORCE_INLINE static bool stow()   { return set_deployed(false); }

    FORCE_INLINE static void cmd_deploy()   { command(BLTOUCH_DEPLOY); }
    FORCE_INLINE static void cmd_stow()     { command(BLTOUCH_STOW); }

    FORCE_INLINE static void cmd_reset()    { command(BLTOUCH_RESET); }
    FORCE_INLINE static void cmd_selftest() { command(BLTOUCH_SELFTEST); }
    FORCE_INLINE static void cmd_5V_mode()  { command(BLTOUCH_5V_MODE); }
    FORCE_INLINE static void cmd_OD_mode()  { command(BLTOUCH_OD_MODE); }
    FORCE_INLINE static void cmd_SW_mode()  { command(BLTOUCH_SW_MODE); }

};

extern Bltouch bltouch;

#endif // ENABLED(BLTOUCH)
