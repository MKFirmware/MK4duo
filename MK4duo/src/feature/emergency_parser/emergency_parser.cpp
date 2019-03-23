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

/**
 * emergency_parser.cpp - Intercept special commands directly in the serial stream
 */

#include "../../../MK4duo.h"

#if ENABLED(EMERGENCY_PARSER)

EmergencyParser emergency_parser;

/** Public Parameters */
bool  EmergencyParser::killed_by_M112 = false;

uint8_t EmergencyParser::M876_response = 0;

/** Private Parameters */
bool EmergencyParser::enabled = true;

//Public Function
void EmergencyParser::update(EmergencyStateEnum &state, const uint8_t c) {

  switch (state) {
    case EP_RESET:
      switch (c) {
        case ' ': break;
        case 'N': state = EP_N;      break;
        case 'M': state = EP_M;      break;
        default: state  = EP_IGNORE;
      }
      break;

    case EP_N:
      if (isdigit(c) || c == '-' || c == ' ') break;
      else if (c == 'M')
        state = EP_M;
      else
        state = EP_IGNORE;
      break;

    case EP_M:
      switch (c) {
        case ' ': break;
        case '1': state = EP_M1;     break;
        case '4': state = EP_M4;     break;
        case '8': state = EP_M8;     break;
        default: state  = EP_IGNORE;
      }
      break;

    case EP_M1:
      switch (c) {
        case '0': state = EP_M10;    break;
        case '1': state = EP_M11;    break;
        default: state  = EP_IGNORE;
      }
      break;

    case EP_M10:
      state = (c == '8') ? EP_M108 : EP_IGNORE;
      break;

    case EP_M11:
      state = (c == '2') ? EP_M112 : EP_IGNORE;
      break;

    case EP_M4:
      state = (c == '1') ? EP_M41 : EP_IGNORE;
      break;

    case EP_M41:
      state = (c == '0') ? EP_M410 : EP_IGNORE;
      break;

    case EP_M8:
      state = (c == '7') ? EP_M87 : EP_IGNORE;
      break;

    case EP_M87:
      state = (c == '6') ? EP_M876 : EP_IGNORE;
      break;

    case EP_M876:
      switch (c) {
        case ' ':
          break;
        case 'S':
          state = EP_M876S;
          break;
        default:
          state =  EP_IGNORE;
          break;
      }
      break;

    case EP_M876S:
      if (isdigit(c)) {
        state = EP_M876SN;
        M876_response = uint8_t(c - '0');
      }
      break;

    case EP_IGNORE:
      if (c == '\n') state = EP_RESET;
      break;

    default:
      if (c == '\n') {
        if (enabled) switch (state) {
          case EP_M108:
            printer.setWaitForUser(false);
            printer.setWaitForHeatUp(false);
            break;
          case EP_M112:
            killed_by_M112 = true;
            break;
          case EP_M410:
            printer.quickstop_stepper();
            break;
          case EP_M876SN:
            host_action.response_handler(M876_response);
            break;
          default:
            break;
        }
        state = EP_RESET;
      }
  }
}

#endif // EMERGENCY_PARSER
