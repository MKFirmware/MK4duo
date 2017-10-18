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

#if FAN_COUNT > 0

  #define CODE_M106
  #define CODE_M107

  #if ENABLED(FAN_MIN_PWM)
    #define CALC_FAN_SPEED() (speed ? ( FAN_MIN_PWM + (speed * (255 - FAN_MIN_PWM)) / 255 ) : 0)
  #else
    #define CALC_FAN_SPEED() speed
  #endif

  /**
   * M106: Set Fan Speed
   *
   *  P<index>  Fan index, if more than one fan
   *  S<int>    Speed between 0-255
   *  F<int>    Fan Pin
   *  L<int>    Min Speed
   *  I<bool>   Inverted pin output
   */
  inline void gcode_M106(void) {
    const uint8_t speed = parser.byteval('S', 255),
                  f     = parser.byteval('P');

    if (f < FAN_COUNT) {

      Fan *fan = &fans[f];

      if (parser.seen('F')) {
        // Put off the fan
        fan->Speed = 0;
        fan->pin = parser.value_int();
        fan->init();
      }

      fan->hardwareInverted = parser.boolval('I');
      fan->min_Speed        = parser.byteval('L', FAN_MIN_PWM);

      #if ENABLED(FAN_KICKSTART_TIME)
        if (fan->Kickstart == 0 && speed > fan->Speed && speed < 85) {
          if (fan->Speed) fan->Kickstart = FAN_KICKSTART_TIME / 100;
          else            fan->Kickstart = FAN_KICKSTART_TIME / 25;
        }
      #endif

      fan->Speed = fan->min_Speed + (speed * (255 - fan->min_Speed)) / 255;

      if (!parser.seen('S')) {
        char response[50];
        sprintf_P(response, PSTR("Fan:%i pin:%i, min:%i inverted:%s"),
            (int)f,
            (int)fan->pin,
            (int)fan->min_Speed,
            (fan->hardwareInverted) ? "true" : "false"
        );
        SERIAL_TXT(response);
        SERIAL_EOL();
      }
    }
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107(void) {
    uint8_t f = parser.byteval('P');
    if (f < FAN_COUNT) fans[f].Speed = 0;
  }

#endif // FAN_COUNT > 0
