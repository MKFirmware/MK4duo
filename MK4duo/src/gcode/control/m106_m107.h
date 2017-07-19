/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
   *  S<int>   Speed between 0-255
   *  P<index> Fan index, if more than one fan
   */
  inline void gcode_M106(void) {
    uint8_t speed = parser.seen('S') ? parser.value_ushort() : 255,
            fan   = parser.seen('P') ? parser.value_ushort() : 0;

    if (fan >= FAN_COUNT || printer.fanSpeeds[fan] == speed)
      return;

    #if ENABLED(FAN_KICKSTART_TIME)
      if (printer.fanKickstart == 0 && speed > printer.fanSpeeds[fan] && speed < 85) {
        if (printer.fanSpeeds[fan]) printer.fanKickstart = FAN_KICKSTART_TIME / 100;
        else                printer.fanKickstart = FAN_KICKSTART_TIME / 25;
      }
    #endif

    printer.fanSpeeds[fan] = CALC_FAN_SPEED();
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107(void) {
    uint16_t p = parser.seen('P') ? parser.value_ushort() : 0;
    if (p < FAN_COUNT) printer.fanSpeeds[p] = 0;
  }

#endif // FAN_COUNT > 0
