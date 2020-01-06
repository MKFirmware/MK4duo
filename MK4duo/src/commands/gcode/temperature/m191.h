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

#if HAS_CHAMBERS

#define CODE_M191

/**
 * M191: Sxxx Wait for chamber current temp to reach target temp. Waits only when heating
 *       Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
 */
inline void gcode_M191() {
  if (printer.debugDryrun() || printer.debugSimulation()) return;

  const uint8_t c = parser.byteval('T');
  if (WITHIN(c, 0 , CHAMBERS - 1)) {
    bool no_wait_for_cooling = parser.seen('S');
    if (no_wait_for_cooling || parser.seen('R'))
      chambers[c]->set_target_temp(parser.value_celsius());
    else return;

    lcdui.set_status_P(chambers[c]->isHeating() ? GET_TEXT(MSG_CHAMBER_HEATING) : GET_TEXT(MSG_CHAMBER_COOLING));

    chambers[c]->wait_for_target(no_wait_for_cooling);
  }
}

#endif // HAS_CHAMBERS
