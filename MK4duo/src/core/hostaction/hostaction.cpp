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

/**
 * hostaction.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

//#define DEBUG_HOST_ACTIONS

Host_Action host_action;

/** Private Parameters */
HostPromptEnum Host_Action::prompt_reason = PROMPT_NOT_DEFINED;

/** Public Function */
void Host_Action::response_handler(const uint8_t response) {

  #ifdef DEBUG_HOST_ACTIONS
    SERIAL_MV("M876 Handle Reason: ", prompt_reason);
    SERIAL_MV("M876 Handle Response: ", response);
  #endif

  const char * msg = PSTR("UNKNOWN STATE");
  const HostPromptEnum temp_pr = prompt_reason;
  prompt_reason = PROMPT_NOT_DEFINED;

  switch (temp_pr) {

    case PROMPT_FILAMENT_RUNOUT:
      msg = PSTR("FILAMENT_RUNOUT");
      switch (response) {

        case 0: // "Purge More" button
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            advancedpause.menu_response = PAUSE_RESPONSE_EXTRUDE_MORE;
          #endif
          filament_load_prompt();
          break;

        case 1: // "Continue" / "Disable Runout" button
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            advancedpause.menu_response = PAUSE_RESPONSE_RESUME_PRINT;
          #endif
          #if HAS_FILAMENT_SENSOR
            if (filamentrunout.sensor.isFilamentOut()) {
              filamentrunout.sensor.setEnabled(false);
              filamentrunout.reset();
            }
          #endif
          break;
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

  SERIAL_MSG("M876 Responding PROMPT_");
  SERIAL_STR(msg);
  SERIAL_EOL();

}

void Host_Action::action_notify(const char * const pstr) {
  print_action(PSTR("notification "), false);
  SERIAL_TXT(pstr);
  SERIAL_EOL();
}

void Host_Action::filrunout(const uint8_t t) {
  print_action(PSTR("out_of_filament T"), false);
  SERIAL_EV(int(t));
}

void Host_Action::prompt_begin(const HostPromptEnum reason, const char * const pstr, const char extra_char/*='\0'*/) {
  prompt_end();
  prompt_reason = reason;
  print_prompt_plus(PSTR("begin"), pstr, extra_char);
}

void Host_Action::prompt_button(const char * const pstr) {
  print_prompt_plus(PSTR("button"), pstr);
}

void Host_Action::prompt_do(const HostPromptEnum reason, const char * const pstr, const char * const btn1/*=nullptr*/, const char * const btn2/*=nullptr*/) {
  prompt_begin(reason, pstr);
  if (btn1) prompt_button(btn1);
  if (btn2) prompt_button(btn1);
  prompt_show();
}

/** Private Function */
void Host_Action::print_action(const char * const pstr, const bool eol/*=true*/) {
  SERIAL_MSG("//action:");
  SERIAL_STR(pstr);
  if (eol) SERIAL_EOL();
}

void Host_Action::print_prompt(const char * const ptype, const bool eol/*=true*/) {
  print_action(PSTR("prompt_"), false);
  SERIAL_STR(ptype);
  if (eol) SERIAL_EOL();
}

void Host_Action::print_prompt_plus(const char * const ptype, const char * const pstr, const char extra_char/*='\0'*/) {
  print_prompt(ptype, false);
  SERIAL_CHR(' ');
  SERIAL_STR(pstr);
  if (extra_char != '\0') SERIAL_CHR(extra_char);
  SERIAL_EOL();
}

void Host_Action::filament_load_prompt() {
  const bool disable_to_continue = (false
    #if HAS_FILAMENT_SENSOR
      || filamentrunout.sensor.isFilamentOut()
    #endif
  );
  prompt_do(PROMPT_FILAMENT_RUNOUT, PSTR("Paused"), PSTR("PurgeMore"),
    disable_to_continue ? PSTR("DisableRunout") : CONTINUE_BTN
  );
}
