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
 * filament.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _FILAMENT_H_
#define _FILAMENT_H_

#if ENABLED(FILAMENT_SENSOR)
  extern bool     filament_sensor;                              // Flag that filament sensor readings should control extrusion
  extern float    filament_width_nominal,                       // Theoretical filament diameter i.e., 3.00 or 1.75
                  filament_width_meas;                          // Measured filament diameter
  extern uint8_t  meas_delay_cm,                                // Delay distance
                  measurement_delay[MAX_MEASUREMENT_DELAY + 1]; // Ring buffer to delay measurement
  extern int8_t   filwidth_delay_index[2];                      // Ring buffer indexes. Used by planner, temperature, and main code
#endif

#endif /* _FILAMENT_H_ */
