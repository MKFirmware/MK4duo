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
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#define CODE_M106
#define CODE_M107

/**
 * M106: Set Fan Speed
 *
 *  P<index>  Fan index, if more than one fan
 *  S<int>    Speed between 0-255
 *  F<int>    Set PWM frequency
 *  H<int>    Set Auto mode - H=7 for controller - H-1 for disabled
 *  T<int>    Set Triggered temperature
 *  U<int>    Fan Pin
 *  L<int>    Min Speed
 *  X<int>    Max Speed
 *  I<bool>   Inverted pin output
 */
inline void gcode_M106() {

  uint8_t f = 0;

  if (printer.debugSimulation() || !fanManager.get_target_fan(f)) return;

  const uint8_t new_speed = parser.byteval('S', 255);

  Fan* fan = fans[f];

  if (parser.seen('U')) {
    // Put off the fan
    fan->speed = 0;
    fan->data.pin = HAL::digital_value_pin();
    SERIAL_LM(ECHO, STR_CHANGE_PIN);
  }

  if (parser.seen('I'))
    fan->setHWinvert(parser.value_bool());

  if (parser.seen('H'))
    fan->set_auto_monitor(parser.value_int());

  if (f == 0)
    fanManager.data.frequency  = parser.ushortval('F', fanManager.data.frequency);

  fan->data.speed_limit.min     = parser.byteval('L', fan->data.speed_limit.min);
  fan->data.speed_limit.max     = parser.byteval('X', fan->data.speed_limit.max);
  fan->data.trigger_temperature = parser.ushortval('T', fan->data.trigger_temperature);

  fan->set_speed(new_speed);

  #if ENABLED(DUAL_X_CARRIAGE) && MAX_FAN > 1
    // Check for Clone fan
    if (f == 0 && mechanics.dxc_is_duplicating() && TEST(fans[1]->data.auto_monitor, 6))
      fans[1]->set_speed(new_speed);
  #endif

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M106 report.
    if (!parser.seen("SUIHLXFT")) fanManager.print_M106(f);
  #endif

}

/**
 * M107: Fan Off
 */
inline void gcode_M107() {
  uint8_t f = 0;
  if (printer.debugSimulation() || !fanManager.get_target_fan(f)) return;
  fans[f]->speed = 0;
}
