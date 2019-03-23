/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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

/**
 * emergency_parser.h - Intercept special commands directly in the serial stream
 */

class EmergencyParser {

  public: /** Constructor */

    EmergencyParser() {}

  public: /** Public Parameters */

    static bool killed_by_M112;

    static uint8_t M876_response;

  private: /** Private Parameters */

    static bool enabled;

  public: /** Public Function */

    FORCE_INLINE static void enable()   { enabled = true; }
    FORCE_INLINE static void disable()  { enabled = false; }

    static void update(EmergencyStateEnum &state, const uint8_t c);

};

extern EmergencyParser emergency_parser;
