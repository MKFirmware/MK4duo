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

#define CODE_M352

/**
 * M352: Stepper driver control
 *
 *  X                 - Set X
 *  Y                 - Set Y
 *  Z                 - Set Z
 *  T[tools]          - Set Extruder
 *
 *  E[pin]            - Set Enable pin
 *  D[pin]            - Set Dir Pin
 *  S[pin]            - Set Step pin
 *
 *  L[bool]           - Set Enable Logic
 *  M[bool]           - Set Step Logic
 *
 */
inline void gcode_M352() {

  if (commands.get_target_tool(352)) return;
  
  #if DISABLED(DISABLE_M503)
    // No arguments? Show M352 report.
    if (!parser.seen("XYZEDSLM")) {
      stepper.print_M352();
      return;
    }
  #endif

  LOOP_XYZ(i) {
    if (parser.seen(axis_codes[i])) {
      if (driver[i]) {
        if (parser.seen('E')) driver[i]->data.pin.enable  = HAL::digital_value_pin();
        if (parser.seen('D')) driver[i]->data.pin.dir     = HAL::digital_value_pin();
        if (parser.seen('S')) driver[i]->data.pin.step    = HAL::digital_value_pin();
        if (parser.seen('L')) driver[i]->data.flag.enable = parser.value_bool();
        if (parser.seen('M')) driver[i]->data.flag.step   = parser.value_bool();
        driver[i]->init();
      }
    }
  }

  if (parser.seenval('T')) {
    const uint8_t d = extruders[toolManager.extruder.target]->get_driver();
    if (driver.e[d]) {
      if (parser.seen('E')) driver.e[d]->data.pin.enable   = HAL::digital_value_pin();
      if (parser.seen('D')) driver.e[d]->data.pin.dir      = HAL::digital_value_pin();
      if (parser.seen('S')) driver.e[d]->data.pin.step     = HAL::digital_value_pin();
      if (parser.seen('L')) driver.e[d]->data.flag.enable  = parser.value_bool();
      if (parser.seen('M')) driver.e[d]->data.flag.step    = parser.value_bool();
      driver.e[d]->init();
    }
  }

}
