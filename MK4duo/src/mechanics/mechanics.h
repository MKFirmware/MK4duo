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
 * mechanics.h
 *
 * Copyright (C) 2016 Alberto Cotronei @MagoKimbra
 */

#ifndef _MECHANICS_H_
#define _MECHANICS_H_

// DEBUG LEVELING
#if ENABLED(DEBUG_LEVELING_FEATURE)
  #define DEBUG_POS(SUFFIX,VAR)       do{ \
    Mechanics.print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); } while(0)
#endif

// Workspace offsets
#if ENABLED(WORKSPACE_OFFSETS)
  #define WORKSPACE_OFFSET(AXIS) Mechanics.workspace_offset[AXIS]
#else
  #define WORKSPACE_OFFSET(AXIS) 0
#endif

#define LOGICAL_POSITION(POS, AXIS) ((POS) + WORKSPACE_OFFSET(AXIS))
#define RAW_POSITION(POS, AXIS)     ((POS) - WORKSPACE_OFFSET(AXIS))

#if ENABLED(WORKSPACE_OFFSETS)
  #define LOGICAL_X_POSITION(POS)   LOGICAL_POSITION(POS, X_AXIS)
  #define LOGICAL_Y_POSITION(POS)   LOGICAL_POSITION(POS, Y_AXIS)
  #define LOGICAL_Z_POSITION(POS)   LOGICAL_POSITION(POS, Z_AXIS)
  #define RAW_X_POSITION(POS)       RAW_POSITION(POS, X_AXIS)
  #define RAW_Y_POSITION(POS)       RAW_POSITION(POS, Y_AXIS)
  #define RAW_Z_POSITION(POS)       RAW_POSITION(POS, Z_AXIS)
#else
  #define LOGICAL_X_POSITION(POS)   (POS)
  #define LOGICAL_Y_POSITION(POS)   (POS)
  #define LOGICAL_Z_POSITION(POS)   (POS)
  #define RAW_X_POSITION(POS)       (POS)
  #define RAW_Y_POSITION(POS)       (POS)
  #define RAW_Z_POSITION(POS)       (POS)
#endif

#define RAW_CURRENT_POSITION(A)     RAW_##A##_POSITION(Mechanics.current_position[A##_AXIS])

#if PLANNER_LEVELING || ENABLED(ZWOBBLE) || ENABLED(HYSTERESIS)
  #define ARG_X float lx
  #define ARG_Y float ly
  #define ARG_Z float lz
#else
  #define ARG_X const float &lx
  #define ARG_Y const float &ly
  #define ARG_Z const float &lz
#endif

#if IS_CARTESIAN
  #include "cartesian_mechanics.h"
#elif IS_CORE
  #include "core_mechanics.h"
#elif IS_DELTA
  #include "delta_mechanics.h"
#elif IS_SCARA
  #error "This version not supoorted scara for now, please use old version"
  //#include "scara_mechanism.h"
#endif

#endif /* _MECHANICS_H_ */
