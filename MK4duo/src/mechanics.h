/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#ifndef MECHANICS_H
  #define MECHANICS_H

  // Macros for mechanics type
  #define MECH_UNKNOWN    -1
  #define MECH_CARTESIAN   0
  #define MECH_COREXY      1
  #define MECH_COREYX      2
  #define MECH_COREXZ      8
  #define MECH_COREZX      9
  #define MECH_DELTA       3
  #define MORGAN_SCARA     4
  #define MAKERARM_SCARA   5

  #define MECH(mech)    (MECHANISM == MECH_##mech)
  #define NOMECH(mech)  (MECHANISM != MECH_##mech)

#endif