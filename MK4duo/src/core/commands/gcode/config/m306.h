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

#if HEATER_COUNT > 0

  #define CODE_M306

  /**
   * M306: Set Heaters parameters
   *
   *  H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 COOLER
   *
   *    A[int]    Pid Drive Min
   *    B[int]    Pid Drive Max
   *    C[int]    Pid Max
   *    L[int]    Min temperature
   *    O[int]    Max temperature
   *    U[bool]   Use Pid/bang bang
   *    I[bool]   Hardware Inverted
   *    P[int]    Sensor Pin
   *
   */
  inline void gcode_M306(void) {

    int8_t h = parser.seen('H') ? parser.value_int() : 0; // hotend being updated

    if (!commands.get_target_heater(h)) return;

    Heater *act = &heaters[h];

    act->pid.DriveMin = parser.intval('A', act->pid.DriveMin);
    act->pid.DriveMax = parser.intval('B', act->pid.DriveMax);
    act->pid.Max      = parser.intval('C', act->pid.Max);
    act->data.mintemp      = parser.intval('L', act->data.mintemp);
    act->data.maxtemp      = parser.intval('O', act->data.maxtemp);

    if (parser.seen('U'))
      act->setUsePid(parser.value_bool());
    if (parser.seen('I'))
      act->setHWInverted(parser.value_bool());

    if (parser.seen('P')) {
      // Put off the heaters
      act->setTarget(0);
      act->data.pin = parser.value_pin();
    }

    act->pid.update();
    act->print_heater_parameters();

 }

#endif // HEATER_COUNT > 0
