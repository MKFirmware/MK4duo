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

/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _MECH_SANITYCHECK_H_
#define _MECH_SANITYCHECK_H_

// Mechanism
#if DISABLED(MECHANISM)
  #error "DEPENDENCY ERROR: Missing setting MECHANISM."
#endif
#if DISABLED(KNOWN_MECH)
  #error "DEPENDENCY ERROR: You have to set a valid MECHANICS."
#endif

#if DISABLED(AXIS_RELATIVE_MODES)
  #error "DEPENDENCY ERROR: Missing setting AXIS_RELATIVE_MODES."
#endif

#if DISABLED(MINIMUM_PLANNER_SPEED)
  #error "DEPENDENCY ERROR: Missing setting MINIMUM_PLANNER_SPEED."
#endif

// Homing Bump
#if DISABLED(X_HOME_BUMP_MM)
  #error "DEPENDENCY ERROR: Missing setting X_HOME_BUMP_MM."
#endif
#if DISABLED(Y_HOME_BUMP_MM)
  #error "DEPENDENCY ERROR: Missing setting Y_HOME_BUMP_MM."
#endif
#if DISABLED(Z_HOME_BUMP_MM)
  #error "DEPENDENCY ERROR: Missing setting Z_HOME_BUMP_MM."
#endif
#if X_HOME_BUMP_MM < 0 || Y_HOME_BUMP_MM < 0 || Z_HOME_BUMP_MM < 0
  #error "DEPENDENCY ERROR: [XYZ]_HOME_BUMP_MM must be greater than or equal to 0."
#endif

#if DISABLED(HOMING_BUMP_DIVISOR)
  #error "DEPENDENCY ERROR: Missing setting HOMING_BUMP_DIVISOR."
#endif

// Home direction
#if DISABLED(X_HOME_DIR)
  #error "DEPENDENCY ERROR: Missing setting X_HOME_DIR."
#endif
#if DISABLED(Y_HOME_DIR)
  #error "DEPENDENCY ERROR: Missing setting Y_HOME_DIR."
#endif
#if DISABLED(Z_HOME_DIR)
  #error "DEPENDENCY ERROR: Missing setting Z_HOME_DIR."
#endif


// Manual home position
#if ENABLED(MANUAL_HOME_POSITIONS)
  #if DISABLED(MANUAL_X_HOME_POS)
    #error "DEPENDENCY ERROR: Missing setting MANUAL_X_HOME_POS."
  #endif
  #if DISABLED(MANUAL_Y_HOME_POS)
    #error "DEPENDENCY ERROR: Missing setting MANUAL_Y_HOME_POS."
  #endif
  #if DISABLED(MANUAL_Z_HOME_POS)
    #error "DEPENDENCY ERROR: Missing setting MANUAL_Z_HOME_POS."
  #endif
#endif


// Enable on
#if DISABLED(X_ENABLE_ON)
  #error "DEPENDENCY ERROR: Missing setting X_ENABLE_ON."
#endif
#if DISABLED(Y_ENABLE_ON)
  #error "DEPENDENCY ERROR: Missing setting Y_ENABLE_ON."
#endif
#if DISABLED(Z_ENABLE_ON)
  #error "DEPENDENCY ERROR: Missing setting Z_ENABLE_ON."
#endif
#if DISABLED(E_ENABLE_ON)
  #error "DEPENDENCY ERROR: Missing setting E_ENABLE_ON."
#endif


// Invert STEP pin
#if DISABLED(INVERT_X_STEP_PIN)
  #error "DEPENDENCY ERROR: Missing setting INVERT_X_STEP_PIN."
#endif
#if DISABLED(INVERT_Y_STEP_PIN)
  #error "DEPENDENCY ERROR: Missing setting INVERT_Y_STEP_PIN."
#endif
#if DISABLED(INVERT_Z_STEP_PIN)
  #error "DEPENDENCY ERROR: Missing setting INVERT_Z_STEP_PIN."
#endif
#if DISABLED(INVERT_E_STEP_PIN)
  #error "DEPENDENCY ERROR: Missing setting INVERT_E_STEP_PIN."
#endif


// Invert direction
#if DISABLED(INVERT_X_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_X_DIR."
#endif
#if DISABLED(INVERT_Y_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_Y_DIR."
#endif
#if DISABLED(INVERT_Z_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_Z_DIR."
#endif
#if DISABLED(INVERT_E0_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_E0_DIR."
#endif
#if DISABLED(INVERT_E1_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_E1_DIR."
#endif
#if DISABLED(INVERT_E2_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_E2_DIR."
#endif
#if DISABLED(INVERT_E3_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_E3_DIR."
#endif
#if DISABLED(INVERT_E4_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_E4_DIR."
#endif
#if DISABLED(INVERT_E5_DIR)
  #error "DEPENDENCY ERROR: Missing setting INVERT_E5_DIR."
#endif


// Disable axis
#if DISABLED(DISABLE_X)
  #error "DEPENDENCY ERROR: Missing setting DISABLE_X."
#endif
#if DISABLED(DISABLE_Y)
  #error "DEPENDENCY ERROR: Missing setting DISABLE_Y."
#endif
#if DISABLED(DISABLE_Z)
  #error "DEPENDENCY ERROR: Missing setting DISABLE_Z."
#endif
#if DISABLED(DISABLE_E)
  #error "DEPENDENCY ERROR: Missing setting DISABLE_E."
#endif


// Max/Min position for nomech DELTA
#if NOMECH(DELTA)
  #if DISABLED(X_MAX_POS)
    #error "DEPENDENCY ERROR: Missing setting X_MAX_POS."
  #endif
  #if DISABLED(X_MIN_POS)
    #error "DEPENDENCY ERROR: Missing setting X_MIN_POS."
  #endif
  #if DISABLED(Y_MAX_POS)
    #error "DEPENDENCY ERROR: Missing setting Y_MAX_POS."
  #endif
  #if DISABLED(Y_MIN_POS)
    #error "DEPENDENCY ERROR: Missing setting Y_MIN_POS."
  #endif
  #if DISABLED(Z_MAX_POS)
    #error "DEPENDENCY ERROR: Missing setting Z_MAX_POS."
  #endif
  #if DISABLED(Z_MIN_POS)
    #error "DEPENDENCY ERROR: Missing setting Z_MIN_POS."
  #endif
  #if DISABLED(E_MIN_POS)
    #error "DEPENDENCY ERROR: Missing setting E_MIN_POS."
  #endif
#endif


// Dual X carriage
#if ENABLED(DUAL_X_CARRIAGE)
  #if DISABLED(X2_MIN_POS)
    #error "DEPENDENCY ERROR: Missing setting X2_MIN_POS."
  #endif
  #if DISABLED(X2_MAX_POS)
    #error "DEPENDENCY ERROR: Missing setting X2_MAX_POS."
  #endif
  #if DISABLED(X2_HOME_DIR)
    #error "DEPENDENCY ERROR: Missing setting X2_HOME_DIR."
  #endif
  #if DISABLED(X2_HOME_POS)
    #error "DEPENDENCY ERROR: Missing setting X2_HOME_POS."
  #endif
  #if DISABLED(DEFAULT_DUAL_X_CARRIAGE_MODE)
    #error "DEPENDENCY ERROR: Missing setting DEFAULT_DUAL_X_CARRIAGE_MODE."
  #endif
  #if DISABLED(TOOLCHANGE_PARK_ZLIFT)
    #error "DEPENDENCY ERROR: Missing setting TOOLCHANGE_PARK_ZLIFT."
  #endif
  #if DISABLED(TOOLCHANGE_UNPARK_ZLIFT)
    #error "DEPENDENCY ERROR: Missing setting TOOLCHANGE_UNPARK_ZLIFT."
  #endif
  #if DISABLED(DEFAULT_DUPLICATION_X_OFFSET)
    #error "DEPENDENCY ERROR: Missing setting DEFAULT_DUPLICATION_X_OFFSET."
  #endif

  #if EXTRUDERS == 1
    #error "DEPENDENCY ERROR: DUAL_X_CARRIAGE requires 2 (or more) extruders."
  #elif MECH(COREXY) || MECH(COREXZ)
    #error "DEPENDENCY ERROR: DUAL_X_CARRIAGE cannot be used with COREXY or COREXZ."
  #elif !HAS_X2_ENABLE || !HAS_X2_STEP || !HAS_X2_DIR
    #error "DEPENDENCY ERROR: DUAL_X_CARRIAGE requires X2 stepper pins to be defined."
  #elif !HAS_X_MAX
    #error "DEPENDENCY ERROR: DUAL_X_CARRIAGE requires use a X Max Endstop."
  #elif DISABLED(X2_HOME_POS) || DISABLED(X2_MIN_POS) || DISABLED(X2_MAX_POS)
    #error "DEPENDENCY ERROR: DUAL_X_CARRIAGE requires X2_HOME_POS, X2_MIN_POS, and X2_MAX_POS."
  #elif X_HOME_DIR != -1 || X2_HOME_DIR != 1
    #error "DEPENDENCY ERROR: DUAL_X_CARRIAGE requires X_HOME_DIR -1 and X2_HOME_DIR 1."
  #endif

  #if ENABLED(X_TWO_STEPPER_DRIVERS)
    #error "DEPENDENCY ERROR: DUAL_X_CARRIAGE or X_TWO_STEPPER_DRIVERS can be set"
  #endif
#endif


// Accuracy settings
#if DISABLED(MIN_STEPS_PER_SEGMENT)
  #error "DEPENDENCY ERROR: Missing setting MIN_STEPS_PER_SEGMENT."
#endif
#if DISABLED(DEFAULT_MIN_SEGMENT_TIME)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_MIN_SEGMENT_TIME."
#endif
#if DISABLED(MM_PER_ARC_SEGMENT)
  #error "DEPENDENCY ERROR: Missing setting MM_PER_ARC_SEGMENT."
#endif
#if DISABLED(N_ARC_CORRECTION)
  #error "DEPENDENCY ERROR: Missing setting N_ARC_CORRECTION."
#endif
#if DISABLED(DEFAULT_AXIS_STEPS_PER_UNIT)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_AXIS_STEPS_PER_UNIT."
#endif


// Velocity and data.acceleration
#if DISABLED(DEFAULT_MIN_TRAVEL_FEEDRATE)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_MIN_TRAVEL_FEEDRATE."
#endif
#if DISABLED(DEFAULT_MAX_ACCELERATION)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_MAX_ACCELERATION."
#endif
#if DISABLED(DEFAULT_RETRACT_ACCELERATION)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_RETRACT_ACCELERATION."
#endif
#if DISABLED(DEFAULT_ACCELERATION)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_ACCELERATION."
#endif
#if DISABLED(DEFAULT_TRAVEL_ACCELERATION)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_TRAVEL_ACCELERATION."
#endif
#if DISABLED(DEFAULT_XJERK)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_XJERK."
#endif
#if DISABLED(DEFAULT_YJERK)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_YJERK."
#endif
#if DISABLED(DEFAULT_ZJERK)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_ZJERK."
#endif

// Two X steppers
#if ENABLED(X_TWO_STEPPER_DRIVERS)
  #if DISABLED(INVERT_X2_VS_X_DIR)
    #error "DEPENDENCY ERROR: Missing setting INVERT_X2_VS_X_DIR."
  #endif
#endif

// Two Y steppers
#if ENABLED(Y_TWO_STEPPER_DRIVERS)
  #if DISABLED(INVERT_Y2_VS_Y_DIR)
    #error "DEPENDENCY ERROR: Missing setting INVERT_Y2_VS_Y_DIR."
  #endif
#endif

// Two Z steppers
#if ENABLED(Z_TWO_STEPPER_DRIVERS)
  #if DISABLED(INVERT_Z2_VS_Z_DIR)
    #error "DEPENDENCY ERROR: Missing setting INVERT_Z2_VS_Z_DIR."
  #endif
#endif

// Pin definitions
#if !HAS_X_STEP
  #error "DEPENDENCY ERROR: X_STEP_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_X_DIR
  #error "DEPENDENCY ERROR: X_DIR_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_X_ENABLE
  #error "DEPENDENCY ERROR: X_ENABLE_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_Y_STEP
  #error "DEPENDENCY ERROR: Y_STEP_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_Y_DIR
  #error "DEPENDENCY ERROR: Y_DIR_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_Y_ENABLE
  #error "DEPENDENCY ERROR: Y_ENABLE_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_Z_STEP
  #error "DEPENDENCY ERROR: Z_STEP_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_Z_DIR
  #error "DEPENDENCY ERROR: Z_DIR_PIN is not defined for your board. You have to define it yourself."
#endif
#if !HAS_Z_ENABLE
  #error "DEPENDENCY ERROR: Z_ENABLE_PIN is not defined for your board. You have to define it yourself."
#endif

#if DRIVER_EXTRUDERS > 0
  #if !HAS_E0_STEP
    #error "DEPENDENCY ERROR: E0_STEP_PIN is not defined for your board. You have to define it yourself."
  #endif
  #if !HAS_E0_DIR
    #error "DEPENDENCY ERROR: E0_DIR_PIN is not defined for your board. You have to define it yourself."
  #endif
  #if !HAS_E0_ENABLE
    #error "DEPENDENCY ERROR: E0_ENABLE_PIN is not defined for your board. You have to define it yourself."
  #endif
  #if DRIVER_EXTRUDERS > 1
    #if !HAS_E1_STEP
      #error "DEPENDENCY ERROR: E1_STEP_PIN is not defined for your board. You have to define it yourself."
    #endif
    #if !HAS_E1_DIR
      #error "DEPENDENCY ERROR: E1_DIR_PIN is not defined for your board. You have to define it yourself."
    #endif
    #if !HAS_E1_ENABLE
      #error "DEPENDENCY ERROR: E1_ENABLE_PIN is not defined for your board. You have to define it yourself."
    #endif
    #if DRIVER_EXTRUDERS > 2
      #if !HAS_E2_STEP
        #error "DEPENDENCY ERROR: E2_STEP_PIN is not defined for your board. You have to define it yourself."
      #endif
      #if !HAS_E2_DIR
        #error "DEPENDENCY ERROR: E2_DIR_PIN is not defined for your board. You have to define it yourself."
      #endif
      #if !HAS_E2_ENABLE
        #error "DEPENDENCY ERROR: E2_ENABLE_PIN is not defined for your board. You have to define it yourself."
      #endif
      #if DRIVER_EXTRUDERS > 3
        #if !HAS_E3_STEP
          #error "DEPENDENCY ERROR: E3_STEP_PIN is not defined for your board. You have to define it yourself."
        #endif
        #if !HAS_E3_DIR
          #error "DEPENDENCY ERROR: E3_DIR_PIN is not defined for your board. You have to define it yourself."
        #endif
        #if !HAS_E3_ENABLE
          #error "DEPENDENCY ERROR: E3_ENABLE_PIN is not defined for your board. You have to define it yourself."
        #endif
        #if DRIVER_EXTRUDERS > 4
          #if !HAS_E4_STEP
            #error "DEPENDENCY ERROR: E4_STEP_PIN is not defined for your board. You have to define it yourself."
          #endif
          #if !HAS_E4_DIR
            #error "DEPENDENCY ERROR: E4_DIR_PIN is not defined for your board. You have to define it yourself."
          #endif
          #if !HAS_E4_ENABLE
            #error "DEPENDENCY ERROR: E4_ENABLE_PIN is not defined for your board. You have to define it yourself."
          #endif
          #if DRIVER_EXTRUDERS > 5
            #if !HAS_E5_STEP
              #error "DEPENDENCY ERROR: E5_STEP_PIN is not defined for your board. You have to define it yourself."
            #endif
            #if !HAS_E5_DIR
              #error "DEPENDENCY ERROR: E5_DIR_PIN is not defined for your board. You have to define it yourself."
            #endif
            #if !HAS_E5_ENABLE
              #error "DEPENDENCY ERROR: E5_ENABLE_PIN is not defined for your board. You have to define it yourself."
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif
#if X2_HAS_DRV(TMC26X) && (!PIN_EXISTS(X2_ENABLE) || !HAS_X2_STEP || !PIN_EXISTS(X2_DIR))
  #error "DEPENDENCY ERROR: You have to set X2_ENABLE_PIN, X2_STEP_PIN and X2_DIR_PIN to a valid pin if you enable X2_IS_TMC."
#endif

/**
 * Linear Advance 1.5 - Check K value range
 */
#if ENABLED(LIN_ADVANCE)
  static_assert(
    WITHIN(LIN_ADVANCE_K, 0, 10),
    "DEPENDENCY ERROR: LIN_ADVANCE_K must be a value from 0 to 10."
  );
#endif

// Z late enable
#if MECH(COREXZ) && ENABLED(Z_LATE_ENABLE)
  #error "DEPENDENCY ERROR: Z_LATE_ENABLE can't be used with COREXZ."
#endif

// Core factor
#if IS_CORE

  #if DISABLED(CORE_FACTOR)
    #error "DEPENDENCY ERROR: Missing setting CORE_FACTOR."
  #endif

  /**
   * TWO STEPPER DRIVERS
   */
  #if ENABLED(X_TWO_STEPPER_DRIVERS) || ENABLED(Y_TWO_STEPPER_DRIVERS)
    #error "DEPENDENCY ERROR: TWO Stepper Driver XY for Core is imposible"
  #endif

  /**
   * TWO ENDSTOPS
   */
  #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS)
    #error "DEPENDENCY ERROR: TWO ENDSTOPS XY for Core is imposible"
  #endif

#endif // IS_CORE

// Delta requirements
#if MECH(DELTA)

  #if ABL_GRID
    #if (GRID_MAX_POINTS_X & 1) == 0  || (GRID_MAX_POINTS_Y & 1) == 0
      #error "DEPENDENCY ERROR: DELTA requires GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y to be odd numbers."
    #elif GRID_MAX_POINTS_X < 3 || GRID_MAX_POINTS_Y < 3
      #error "DEPENDENCY ERROR: DELTA requires GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y to be 3 or higher."
    #endif
  #endif

  static_assert(1 >= 0
    #if ENABLED(DELTA_AUTO_CALIBRATION_1)
      +1
    #endif
    #if ENABLED(DELTA_AUTO_CALIBRATION_2)
      +1
    #endif
    , "DEPENDENCY ERROR: Select only one between DELTA_AUTO_CALIBRATION_1 and DELTA_AUTO_CALIBRATION_2."
  );

  #if DISABLED(DELTA_DIAGONAL_ROD)
    #error "DEPENDENCY ERROR: Missing setting DELTA_DIAGONAL_ROD."
  #endif
  #if DISABLED(DELTA_SMOOTH_ROD_OFFSET)
    #error "DEPENDENCY ERROR: Missing setting DELTA_SMOOTH_ROD_OFFSET."
  #endif
  #if DISABLED(DELTA_CARRIAGE_OFFSET)
    #error "DEPENDENCY ERROR: Missing setting DELTA_CARRIAGE_OFFSET."
  #endif
  #if DISABLED(DELTA_PRINTABLE_RADIUS)
    #error "DEPENDENCY ERROR: Missing setting DELTA_PRINTABLE_RADIUS."
  #endif
  #if DISABLED(TOWER_A_ENDSTOP_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_A_ENDSTOP_ADJ."
  #endif
  #if DISABLED(TOWER_B_ENDSTOP_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_B_ENDSTOP_ADJ."
  #endif
  #if DISABLED(TOWER_C_ENDSTOP_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_C_ENDSTOP_ADJ."
  #endif
  #if DISABLED(TOWER_A_RADIUS_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_A_RADIUS_ADJ."
  #endif
  #if DISABLED(TOWER_B_RADIUS_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_B_RADIUS_ADJ."
  #endif
  #if DISABLED(TOWER_C_RADIUS_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_C_RADIUS_ADJ."
  #endif
  #if DISABLED(TOWER_A_ANGLE_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_A_ANGLE_ADJ."
  #endif
  #if DISABLED(TOWER_B_ANGLE_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_B_ANGLE_ADJ."
  #endif
  #if DISABLED(TOWER_C_ANGLE_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_C_ANGLE_ADJ."
  #endif
  #if DISABLED(TOWER_A_DIAGROD_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_A_DIAGROD_ADJ."
  #endif
  #if DISABLED(TOWER_B_DIAGROD_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_B_DIAGROD_ADJ."
  #endif
  #if DISABLED(TOWER_C_DIAGROD_ADJ)
    #error "DEPENDENCY ERROR: Missing setting TOWER_C_DIAGROD_ADJ."
  #endif

  #if HAS_BED_PROBE
    #if DISABLED(XY_PROBE_SPEED)
      #error "DEPENDENCY ERROR: Missing setting XY_PROBE_SPEED."
    #endif
    #if DISABLED(X_PROBE_OFFSET_FROM_NOZZLE)
      #error "DEPENDENCY ERROR: Missing setting X_PROBE_OFFSET_FROM_NOZZLE."
    #endif
    #if DISABLED(Y_PROBE_OFFSET_FROM_NOZZLE)
      #error "DEPENDENCY ERROR: Missing setting Y_PROBE_OFFSET_FROM_NOZZLE."
    #endif
    #if DISABLED(Z_PROBE_OFFSET_FROM_NOZZLE)
      #error "DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET_FROM_NOZZLE."
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_START_LOCATION)
      #error "DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_START_LOCATION."
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_END_LOCATION)
      #error "DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_END_LOCATION."
    #endif
    #if DISABLED(Z_PROBE_RETRACT_START_LOCATION)
      #error "DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_START_LOCATION."
    #endif
    #if DISABLED(Z_PROBE_RETRACT_END_LOCATION)
      #error "DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_END_LOCATION."
    #endif
    #if DISABLED(Z_PROBE_BETWEEN_HEIGHT)
      #error "DEPENDENCY ERROR: Missing setting Z_PROBE_BETWEEN_HEIGHT."
    #endif
  #endif

  /**
   * Babystepping
   */
  #if ENABLED(BABYSTEPPING) && ENABLED(BABYSTEP_XY)
    #error "DEPENDENCY ERROR: BABYSTEPPING only implemented for Z axis on deltabots."
  #endif

  /**
   * TMC2130
   */
  #if HAVE_DRV(TMC2130)
    #if !(X_HAS_DRV(TMC2130) && Y_HAS_DRV(TMC2130) && Z_HAS_DRV(TMC2130))
      #error "DEPENDENCY ERROR: For delta there must be all three XYZ TMC2130 drivers"
    #endif
  #endif

  /**
   * TWO STEPPER DRIVERS
   */
  #if ENABLED(X_TWO_STEPPER_DRIVERS) || ENABLED(Y_TWO_STEPPER_DRIVERS) || ENABLED(Z_TWO_STEPPER_DRIVERS)
    #error "DEPENDENCY ERROR: TWO Stepper Driver for Delta is imposible"
  #endif

  /**
   * TWO ENDSTOPS
   */
  #if ENABLED(X_TWO_ENDSTOPS) || ENABLED(Y_TWO_ENDSTOPS) || ENABLED(Z_TWO_ENDSTOPS)
    #error "DEPENDENCY ERROR: TWO ENDSTOPS for Delta is imposible"
  #endif

#endif // MECH(DELTA)

// Scara settings
#if IS_SCARA

  #if DISABLED(SCARA_LINKAGE_1)
    #error "DEPENDENCY ERROR: Missing setting SCARA_LINKAGE_1."
  #endif
  #if DISABLED(SCARA_LINKAGE_2)
    #error "DEPENDENCY ERROR: Missing setting SCARA_LINKAGE_2."
  #endif
  #if DISABLED(SCARA_OFFSET_X)
    #error "DEPENDENCY ERROR: Missing setting SCARA_OFFSET_X."
  #endif
  #if DISABLED(SCARA_OFFSET_Y)
    #error "DEPENDENCY ERROR: Missing setting SCARA_OFFSET_Y."
  #endif
  #if DISABLED(THETA_HOMING_OFFSET)
    #error "DEPENDENCY ERROR: Missing setting THETA_HOMING_OFFSET."
  #endif
  #if DISABLED(PSI_HOMING_OFFSET)
    #error "DEPENDENCY ERROR: Missing setting PSI_HOMING_OFFSET."
  #endif

  /**
   * Babystepping
   */
  #if ENABLED(BABYSTEPPING)
    #error "DEPENDENCY ERROR: BABYSTEPPING is not implemented for SCARA yet."
  #endif

#endif // IS_SCARA

#endif /* _MECH_SANITYCHECK_H_ */
