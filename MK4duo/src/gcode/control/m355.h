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

/**
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_CASE_LIGHT

  #define CODE_M355

  /**
   * M355: Turn case light on/off and set brightness
   *
   *   P<byte>  Set case light brightness (PWM pin required - ignored otherwise)
   *
   *   S<bool>  Set case light on/off
   *
   *   When S turns on the light on a PWM pin then the current brightness level is used/restored
   *
   *   M355 P200 S0 turns off the light & sets the brightness level
   *   M355 S1 turns on the light with a brightness of 200 (assuming a PWM pin)
   */
  inline void gcode_M355(void) {
    uint8_t args = 0;

    if (parser.seen('P')) {
      ++args, case_light_brightness = parser.value_byte();
      case_light_arg_flag = false;
    }
    if (parser.seen('S')) {
      ++args, case_light_on = parser.value_bool();
      case_light_arg_flag = true;
    }
    if (args) update_case_light();

    // always report case light status
    SERIAL_SM(ECHO, "Case light: ");
    if (!case_light_on)
      SERIAL_EM("off");
    else {
      if (!USEABLE_HARDWARE_PWM(CASE_LIGHT_PIN)) SERIAL_EM("on");
      else SERIAL_EV((int)case_light_brightness);
    }
  }

#endif // HAS_CASE_LIGHT
