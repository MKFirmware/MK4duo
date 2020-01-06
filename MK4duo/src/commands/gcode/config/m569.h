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

#define CODE_M569

/**
 * M569: Stepper driver control
 *
 *  X[bool]           - Set X direction
 *  Y[bool]           - Set Y direction
 *  Z[bool]           - Set Z direction
 *  T[tools] E[bool]  - Set Extruder direction
 *
 *  P[int]            - Set minimum pulse
 *  R[long]           - Set maximum rate
 *  D[long]           - Set Direction delay
 *
 */
inline void gcode_M569() {

  if (commands.get_target_tool(569)) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M569 report.
    if (!parser.seen("XYZEDPRQ")) {
      stepper.print_M569();
      return;
    }
  #endif

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      if (i == E_AXIS) {
        const uint8_t d = extruders[toolManager.extruder.target]->get_driver();
        if (driver.e[d]) driver.e[d]->setDir(parser.value_bool());
      }
      else {
        if (driver[i]) driver[i]->setDir(parser.value_bool());
      }
    }
  }

  // Set actually direction
  stepper.reset_drivers();

  if (parser.seen('D')) stepper.data.direction_delay  = parser.value_ulong();
  if (parser.seen('P')) stepper.data.minimum_pulse    = parser.value_byte();
  if (parser.seen('R')) stepper.data.maximum_rate     = parser.value_ulong();
  if (parser.seen('Q')) stepper.data.quad_stepping    = parser.value_bool();

  // Recalculate pulse cycle
  HAL_calc_pulse_cycle();

  /* // <- put a / for activate code
  DEBUG_EMV("HAL_min_pulse_cycle:",     HAL_min_pulse_cycle);
  DEBUG_EMV("HAL_frequency_limit[0]:",  HAL_frequency_limit[0]);
  DEBUG_EMV("HAL_frequency_limit[1]:",  HAL_frequency_limit[1]);
  DEBUG_EMV("HAL_frequency_limit[2]:",  HAL_frequency_limit[2]);
  DEBUG_EMV("HAL_frequency_limit[3]:",  HAL_frequency_limit[3]);
  DEBUG_EMV("HAL_frequency_limit[4]:",  HAL_frequency_limit[4]);
  DEBUG_EMV("HAL_frequency_limit[5]:",  HAL_frequency_limit[5]);
  DEBUG_EMV("HAL_frequency_limit[6]:",  HAL_frequency_limit[6]);
  DEBUG_EMV("HAL_frequency_limit[7]:",  HAL_frequency_limit[7]);
  //*/

}
