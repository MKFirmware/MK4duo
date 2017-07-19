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

#define CODE_M303

/**
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default target temperature = 150C)
 *       H<hotend> (-1 for the bed, -2 for chamber, -3 for cooler) (default 0)
 *       C<cycles>
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303(void) {

  #if HAS_PID_HEATING || HAS_PID_COOLING
    const int   h = parser.seen('H') ? parser.value_int() : 0,
                c = parser.seen('C') ? parser.value_int() : 5;
    const bool  u = parser.seen('U') && parser.value_bool() != 0;

    int16_t temp = parser.seen('S') ? parser.value_celsius() : (h < 0 ? 70 : 200);

    if (WITHIN(h, 0, HOTENDS - 1)) tools.target_extruder = h;

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(NOT_BUSY);
    #endif

    thermalManager.PID_autotune(temp, h, c, u);

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(IN_HANDLER);
    #endif

  #else

    SERIAL_LM(ER, MSG_ERR_M303_DISABLED);

  #endif

}
