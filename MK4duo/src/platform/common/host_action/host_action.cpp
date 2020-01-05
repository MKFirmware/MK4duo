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

#include "../../../../MK4duo.h"

//#define DEBUG_HOST_ACTIONS

Host_Action host_action;

/** Public Parameters */
HostPromptEnum Host_Action::prompt_reason = PROMPT_NOT_DEFINED;

/** Public Function */
void Host_Action::say_m876_response(PGM_P const msg) {
  SERIAL_MSG("M876 Responding PROMPT_");
  SERIAL_STR(msg);
  SERIAL_EOL();
}

void Host_Action::response_handler(const uint8_t response) {

  #ifdef DEBUG_HOST_ACTIONS
    SERIAL_MV("M876 Handle Reason: ", prompt_reason);
    SERIAL_MV("M876 Handle Response: ", response);
  #endif

  PGM_P msg = PSTR("UNKNOWN STATE");
  const HostPromptEnum temp_pr = prompt_reason;
  prompt_reason = PROMPT_NOT_DEFINED;

  switch (temp_pr) {
    case PROMPT_FILAMENT_RUNOUT:
      msg = PSTR("FILAMENT_RUNOUT");
      if (response == 0) {
        #if ENABLED(ADVANCED_PAUSE_FEATURE)
          advancedpause.menu_response = PAUSE_RESPONSE_EXTRUDE_MORE;
        #endif
        prompt_begin(PSTR("Paused"));
        prompt_button(PSTR("Purge More"));
        if (false
          #if HAS_FILAMENT_SENSOR
            || filamentrunout.sensor.isFilamentOut()
          #endif
        )
          prompt_button(PSTR("DisableRunout"));
        else {
          prompt_reason = PROMPT_FILAMENT_RUNOUT;
          prompt_button(PSTR("Continue"));
        }
        prompt_show();
      }
      else if (response == 1) {
        #if HAS_FILAMENT_SENSOR
          if (filamentrunout.sensor.isFilamentOut()) {
            filamentrunout.sensor.setEnabled(false);
            filamentrunout.reset();
          }
        #endif
        #if ENABLED(ADVANCED_PAUSE_FEATURE)
          advancedpause.menu_response = PAUSE_RESPONSE_RESUME_PRINT;
        #endif
      }
      break;
    case PROMPT_FILAMENT_RUNOUT_REHEAT:
      msg = PSTR("FILAMENT_RUNOUT_REHEAT");
      printer.setWaitForUser(false);
      break;
    case PROMPT_USER_CONTINUE:
      msg = PSTR("USER_CONTINUE");
      printer.setWaitForUser(false);
      break;
    case PROMPT_PAUSE_RESUME:
      msg = PSTR("LCD_PAUSE_RESUME");
      commands.inject_P(M24_CMD);
      break;
      case PROMPT_INFO:
        msg = PSTR("GCODE_INFO");
      break;
    default: break;
  }
  say_m876_response(msg);
}

void Host_Action::action_notify(const char * const msg) {
  print_action(PSTR("notification "), false);
  SERIAL_TXT(msg);
  SERIAL_EOL();
}

void Host_Action::action_notify_P(PGM_P const msg) {
  print_action(PSTR("notification "), false);
  SERIAL_STR(msg);
  SERIAL_EOL();
}

void Host_Action::filrunout(const uint8_t t) {
  print_action(PSTR("out_of_filament T"), false);
  SERIAL_EV(int(t));
}

void Host_Action::prompt_begin(PGM_P const msg, const bool eol/*=true*/) {
  prompt_end(); // ensure any current prompt is closed before we begin a new one
  print_prompt_plus(PSTR("begin"), msg, eol);
}

void Host_Action::prompt_choice(PGM_P const msg) {
  print_prompt_plus(PSTR("choice"), msg);
}

void Host_Action::prompt_button(PGM_P const msg) {
  print_prompt_plus(PSTR("button"), msg);
}

void Host_Action::prompt_do(const HostPromptEnum reason, PGM_P const pstr, PGM_P const pbtn/*=NULL*/) {
  prompt_reason = reason;
  prompt_begin(pstr);
  if (pbtn) prompt_button(pbtn);
  prompt_show();
}

/** Private Function */
void Host_Action::print_action(PGM_P const msg, const bool eol/*=true*/) {
  SERIAL_MSG("//action:");
  SERIAL_STR(msg);
  if (eol) SERIAL_EOL();
}

void Host_Action::print_prompt(PGM_P const msg, const bool eol/*=true*/) {
  print_action(PSTR("prompt_"), false);
  SERIAL_STR(msg);
  if (eol) SERIAL_EOL();
}

void Host_Action::print_prompt_plus(PGM_P const ptype, PGM_P const msg, const bool eol/*=true*/) {
  print_prompt(ptype, false);
  SERIAL_CHR(' ');
  SERIAL_STR(msg);
  if (eol) SERIAL_EOL();
}
