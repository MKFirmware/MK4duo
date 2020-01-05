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

#if HAS_HEATER

#define CODE_M306

#define MAX_PWM_FREQUENCY 1000U

/**
 * M306: Set Heaters parameters
 *
 *   H[heaters] H = 0-5 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 = COOLER
 *
 *    T[int]      0-3 For Select Beds or Chambers
 *
 *    A[int]    Pid Drive Min
 *    B[int]    Pid Drive Max
 *    C[int]    Pid Max
 *    F[int}    PWM frequency
 *    L[int]    Min temperature
 *    O[int]    Max temperature
 *    U[bool]   Use Pid/bang bang
 *    I[bool]   Hardware Inverted
 *    R[bool]   Thermal Protection
 *    P[int]    Heater Pin
 *    Q[bool]   PWM Hardware
 *
 */
inline void gcode_M306() {

  Heater * const act = commands.get_target_heater();

  if (!act) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M306 report.
    if (!parser.seen("ABCFLO") && !parser.seen("UIRPQ")) {
      act->print_M306();
      return;
    }
  #endif

  act->data.pid.drive.min = parser.intval('A', act->data.pid.drive.min);
  act->data.pid.drive.max = parser.intval('B', act->data.pid.drive.max);
  act->data.pid.Max       = parser.intval('C', act->data.pid.Max);
  act->data.temp.min      = parser.intval('L', act->data.temp.min);
  act->data.temp.max      = parser.intval('O', act->data.temp.max);
  act->data.freq          = MIN(parser.intval('F', act->data.freq), MAX_PWM_FREQUENCY);

  NOMORE(act->data.pid.drive.max, act->data.pid.Max);

  if (parser.seen('U'))
    act->setUsePid(parser.value_bool());
  if (parser.seen('I'))
    act->setHWinvert(parser.value_bool());
  if (parser.seen('Q'))
    act->setHWpwm(parser.value_bool());
  if (parser.seen('R'))
    act->setThermalProtection(parser.value_bool());

  if (parser.seen('P')) {
    // Put off the heaters
    act->set_target_temp(0);
    act->data.pin = HAL::digital_value_pin();
  }

}

#endif // HAS_HEATER
