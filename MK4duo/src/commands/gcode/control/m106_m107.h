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
 * mcode
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if FAN_COUNT > 0

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
  inline void gcode_M106(void) {

    uint8_t f = 0;

    if (printer.debugSimulation() || !commands.get_target_fan(f)) return;

    const uint8_t speed = parser.byteval('S', 255);

    Fan *fan = &fans[f];

    if (parser.seen('U')) {
      // Put off the fan
      fan->Speed = 0;
      fan->data.pin = parser.value_pin();
      SERIAL_LM(ECHO, MSG_CHANGE_PIN);
    }

    if (parser.seen('I'))
      fan->setHWInverted(parser.value_bool());

    if (parser.seen('H'))
      fan->setAutoMonitored(parser.value_int());

    fan->data.min_Speed           = parser.byteval('L', fan->data.min_Speed);
    fan->data.max_Speed           = parser.byteval('X', fan->data.max_Speed);
    fan->data.freq                = parser.ushortval('F', fan->data.freq);
    fan->data.triggerTemperature  = parser.ushortval('T', fan->data.triggerTemperature);

    #if ENABLED(FAN_KICKSTART_TIME)
      if (fan->Kickstart == 0 && speed > fan->Speed && speed < 85) {
        if (fan->Speed) fan->Kickstart = FAN_KICKSTART_TIME / 100;
        else            fan->Kickstart = FAN_KICKSTART_TIME / 25;
      }
    #endif

    fan->Speed = constrain(speed, fan->data.min_Speed, fan->data.max_Speed);

    #if DISABLED(DISABLE_M503)
      // No arguments? Show M106 report.
      if (!parser.seen("SUIHLXFT")) fan->print_M106();
    #endif

  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107(void) {
    uint8_t f = 0;
    if (printer.debugSimulation() || !commands.get_target_fan(f)) return;
    fans[f].Speed = 0;
  }

#endif // FAN_COUNT > 0
