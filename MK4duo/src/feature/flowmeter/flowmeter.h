/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2013 Alberto Cotronei @MagoKimbra
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
 * flowmeter.h - Flowmeter control library for Arduino - Version 1
 * Copyright (c) 2016 Franco (nextime) Lanza.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#if ENABLED(FLOWMETER_SENSOR)

#define FLOWMETER_CALIBRATION (FLOWMETER_MAXFREQ / FLOWMETER_MAXFLOW)

class FlowMeter {

  public: /** Constructor */

    FlowMeter() {};

  public: /** Public Parameters */

    static bool   flow_firstread;
    static float  flowrate;
    static int    flowrate_pulsecount;  

  private: /** Private Parameters */

    static millis_l lastflow;

  public: /** Public Function */

    static void init();
    static void spin();
    static void print_flowrate();

};

extern FlowMeter flowmeter;

#endif // ENABLED(FLOWMETER_SENSOR)
