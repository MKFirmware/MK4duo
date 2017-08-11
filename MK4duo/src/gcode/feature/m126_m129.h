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

#if ENABLED(BARICUDA)

  #if HAS_HEATER_1

    #define CODE_M126

    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126(void) { printer.baricuda_valve_pressure = parser.byteval('S', 255); }

    #define CODE_M127

    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127(void) { printer.baricuda_valve_pressure = 0; }

  #endif

  #if HAS_HEATER_2

    #define CODE_M128

    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128(void) { printer.baricuda_e_to_p_pressure = parser.byteval('S', 255); }

    #define CODE_M129

    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129(void) { printer.baricuda_e_to_p_pressure = 0; }

  #endif

#endif // BARICUDA
