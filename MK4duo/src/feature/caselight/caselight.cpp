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

#include "../../../MK4duo.h"

#if HAS_CASE_LIGHT

  uint8_t case_light_brightness = CASE_LIGHT_DEFAULT_BRIGHTNESS;
  bool case_light_on = CASE_LIGHT_DEFAULT_ON;

  #if DISABLED(INVERT_CASE_LIGHT)
    #define INVERT_CASE_LIGHT false
  #endif

  void update_case_light() {
    HAL::pinMode(CASE_LIGHT_PIN, OUTPUT);
    uint8_t case_light_bright = (uint8_t)case_light_brightness;
    if (case_light_on) {
      HAL::analogWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? 255 - case_light_brightness : case_light_brightness );
      HAL::digitalWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? LOW : HIGH);
    }
    else HAL::digitalWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? HIGH : LOW);
  }

#endif // HAS_CASE_LIGHT
