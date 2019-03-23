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

class Host_Action {

  public: /** Constructor */

    Host_Action() {}

  public: /** Public Parameters */

    static HostPromptEnum prompt_reason;

  public: /** Public Function */

    static void say_m876_response(PGM_P const msg);

    static void response_handler(const uint8_t response);

    static void pause(const bool eol=true)  { print_action(PSTR("pause"), eol); }
    static void paused(const bool eol=true) { print_action(PSTR("paused"), eol); }
    static void resume()                    { print_action(PSTR("resume")); }
    static void resumed()                   { print_action(PSTR("resumed")); }
    static void cancel()                    { print_action(PSTR("cancel")); }
    static void power_off()                 { print_action(PSTR("poweroff")); }

    static void filrunout(const uint8_t t);

    static void prompt_begin(PGM_P const msg, const bool eol=true);
    static void prompt_choice(PGM_P const msg);
    static void prompt_button(PGM_P const msg);

    static void prompt_show() { print_prompt(PSTR("show")); }
    static void prompt_end()  { print_prompt(PSTR("end")); }

    static void prompt_do(const HostPromptEnum reason, PGM_P const pstr, PGM_P const pbtn=NULL);

    inline static void prompt_open(const HostPromptEnum reason, PGM_P const pstr, PGM_P const pbtn=NULL) {
      if (prompt_reason == PROMPT_NOT_DEFINED) prompt_do(reason, pstr, pbtn);
    }

  private: /** Private Function */

    static void print_action(PGM_P const msg, const bool eol=true);
    static void print_prompt(PGM_P const msg, const bool eol=true);
    static void print_prompt_plus(PGM_P const ptype, PGM_P const msg, const bool eol=true);

};

extern Host_Action host_action;
