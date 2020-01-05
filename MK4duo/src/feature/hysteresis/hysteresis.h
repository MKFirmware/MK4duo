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
#pragma once

#if ENABLED(HYSTERESIS_FEATURE)

// Struct Hysteresis data
typedef struct {
  float mm[XYZ],
        correction;
} hysteresis_data_t;

class Hysteresis {

  public: /** Constructor */

    Hysteresis() {};

  public: /** Public Parameters */

    static hysteresis_data_t data;

  public: /** Public Function */

    static void factory_parameters();
    static void add_correction_step(block_t * const block);
    static void print_M99();

};

extern Hysteresis hysteresis;

#endif // #if ENABLED(HYSTERESIS_FEATURE)
