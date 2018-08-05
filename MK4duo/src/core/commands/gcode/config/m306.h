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

    act->pidDriveMin  = parser.intval('A', act->pidDriveMin);
    act->pidDriveMax  = parser.intval('B', act->pidDriveMax);
    act->pidMax       = parser.intval('C', act->pidMax);
    act->mintemp      = parser.intval('L', act->mintemp);
    act->maxtemp      = parser.intval('O', act->maxtemp);

    if (parser.seen('U'))
      act->setUsePid(parser.value_bool());
    if (parser.seen('I'))
      act->setHWInverted(parser.value_bool());

    if (parser.seen('P')) {
      // Put off the heaters
      act->setTarget(0);
      act->pin = parser.value_pin();
    }

    act->updatePID();
    act->print_heater_parameters();

 }

#endif // HEATER_COUNT > 0
