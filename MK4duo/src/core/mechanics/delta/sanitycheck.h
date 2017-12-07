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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _MECH_DELTA_SANITYCHECK_H_
#define _MECH_DELTA_SANITYCHECK_H_

// Delta requirements
#if MECH(DELTA)

  #if ABL_GRID
    #if (GRID_MAX_POINTS_X & 1) == 0  || (GRID_MAX_POINTS_Y & 1) == 0
      #error "DELTA requires GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y to be odd numbers."
    #elif GRID_MAX_POINTS_X < 3 || GRID_MAX_POINTS_Y < 3
      #error "DELTA requires GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y to be 3 or higher."
    #endif
  #endif

  static_assert(1 >= 0
    #if ENABLED(DELTA_AUTO_CALIBRATION_1)
      +1
    #endif
    #if ENABLED(DELTA_AUTO_CALIBRATION_2)
      +1
    #endif
    , "Select only one of: DELTA_AUTO_CALIBRATION_1 or DELTA_AUTO_CALIBRATION_2"
  );

  #if DISABLED(DELTA_DIAGONAL_ROD)
    #error DEPENDENCY ERROR: Missing setting DELTA_DIAGONAL_ROD
  #endif
  #if DISABLED(DELTA_SMOOTH_ROD_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DELTA_SMOOTH_ROD_OFFSET
  #endif
  #if DISABLED(DELTA_CARRIAGE_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DELTA_CARRIAGE_OFFSET
  #endif
  #if DISABLED(DELTA_PRINTABLE_RADIUS)
    #error DEPENDENCY ERROR: Missing setting DELTA_PRINTABLE_RADIUS
  #endif
  #if DISABLED(TOWER_A_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_ENDSTOP_ADJ
  #endif
  #if DISABLED(TOWER_B_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_ENDSTOP_ADJ
  #endif
  #if DISABLED(TOWER_C_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_ENDSTOP_ADJ
  #endif
  #if DISABLED(TOWER_A_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_RADIUS_ADJ
  #endif
  #if DISABLED(TOWER_B_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_RADIUS_ADJ
  #endif
  #if DISABLED(TOWER_C_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_RADIUS_ADJ
  #endif
  #if DISABLED(TOWER_A_ANGLE_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_ANGLE_ADJ
  #endif
  #if DISABLED(TOWER_B_ANGLE_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_ANGLE_ADJ
  #endif
  #if DISABLED(TOWER_C_ANGLE_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_ANGLE_ADJ
  #endif
  #if DISABLED(TOWER_A_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_DIAGROD_ADJ
  #endif
  #if DISABLED(TOWER_B_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_DIAGROD_ADJ
  #endif
  #if DISABLED(TOWER_C_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_DIAGROD_ADJ
  #endif

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #if DISABLED(XY_PROBE_SPEED)
      #error DEPENDENCY ERROR: Missing setting XY_PROBE_SPEED
    #endif
    #if DISABLED(Z_PROBE_SPEED)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_SPEED
    #endif
    #if DISABLED(X_PROBE_OFFSET_FROM_NOZZLE)
      #error DEPENDENCY ERROR: Missing setting X_PROBE_OFFSET_FROM_NOZZLE
    #endif
    #if DISABLED(Y_PROBE_OFFSET_FROM_NOZZLE)
      #error DEPENDENCY ERROR: Missing setting Y_PROBE_OFFSET_FROM_NOZZLE
    #endif
    #if DISABLED(Z_PROBE_OFFSET_FROM_NOZZLE)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET_FROM_NOZZLE
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_START_LOCATION
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_END_LOCATION
    #endif
    #if DISABLED(Z_PROBE_RETRACT_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_START_LOCATION
    #endif
    #if DISABLED(Z_PROBE_RETRACT_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_END_LOCATION
    #endif
    #if DISABLED(Z_PROBE_BETWEEN_HEIGHT)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_BETWEEN_HEIGHT
    #endif
  #endif

  /**
   * Babystepping
   */
  #if ENABLED(BABYSTEPPING) && ENABLED(BABYSTEP_XY)
    #error "BABYSTEPPING only implemented for Z axis on deltabots."
  #endif

#endif // MECH(DELTA)

#endif /* _MECH_DELTA_SANITYCHECK_H_ */
