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
   *  S<int>   Speed between 0-255
   *  P<index> Fan index, if more than one fan
   */
  inline void gcode_M106(void) {
    const uint8_t speed = parser.byteval('S', 255),
                  fan   = parser.byteval('P', 0);
    
    if (fan >= FAN_COUNT || fans[fan].Speed == speed)
      return;

    #if ENABLED(FAN_KICKSTART_TIME)
      if (fans[fan].Kickstart == 0 && speed > fans[fan].Speed && speed < 85) {
        if (fans[fan].Speed)  fans[fan].Kickstart = FAN_KICKSTART_TIME / 100;
        else                  fans[fan].Kickstart = FAN_KICKSTART_TIME / 25;
      }
    #endif
    fans[fan].Speed = CALC_FAN_SPEED();
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107(void) {
    uint16_t p = parser.seen('P') ? parser.value_ushort() : 0;
    if (p < FAN_COUNT) fans[p].Speed = 0;
  }

#endif // FAN_COUNT > 0
