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

#ifndef _MECH_SANITYCHECK_H_
#define _MECH_SANITYCHECK_H_

// Mechanism
#if DISABLED(MECHANISM)
  #error DEPENDENCY ERROR: Missing setting MECHANISM
#endif
#if DISABLED(KNOWN_MECH)
  #error DEPENDENCY ERROR: You have to set a valid MECHANICS.
#endif

#if DISABLED(NUM_POSITON_SLOTS)
  #error DEPENDENCY ERROR: Missing setting NUM_POSITON_SLOTS
#endif

#if DISABLED(AXIS_RELATIVE_MODES)
  #error DEPENDENCY ERROR: Missing setting AXIS_RELATIVE_MODES
#endif

#if DISABLED(MINIMUM_PLANNER_SPEED)
  #error DEPENDENCY ERROR: Missing setting MINIMUM_PLANNER_SPEED
#endif

// Homing Bump
#if DISABLED(X_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting X_HOME_BUMP_MM
#endif
#if DISABLED(Y_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting Y_HOME_BUMP_MM
#endif
#if DISABLED(Z_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting Z_HOME_BUMP_MM
#endif
#if X_HOME_BUMP_MM < 0 || Y_HOME_BUMP_MM < 0 || Z_HOME_BUMP_MM < 0
  #error "[XYZ]_HOME_BUMP_MM must be greater than or equal to 0."
#endif

#if DISABLED(HOMING_BUMP_DIVISOR)
  #error DEPENDENCY ERROR: Missing setting HOMING_BUMP_DIVISOR
#endif


// Home direction
#if DISABLED(X_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting X_HOME_DIR
#endif
#if DISABLED(Y_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting Y_HOME_DIR
#endif
#if DISABLED(Z_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting Z_HOME_DIR
#endif
#if DISABLED(E_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting E_HOME_DIR
#endif


// Manual home position
#if ENABLED(MANUAL_HOME_POSITIONS)
  #if DISABLED(MANUAL_X_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_X_HOME_POS
  #endif
  #if DISABLED(MANUAL_Y_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_Y_HOME_POS
  #endif
  #if DISABLED(MANUAL_Z_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_Z_HOME_POS
  #endif
#endif


// Enable on
#if DISABLED(X_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting X_ENABLE_ON
#endif
#if DISABLED(Y_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting Y_ENABLE_ON
#endif
#if DISABLED(Z_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting Z_ENABLE_ON
#endif
#if DISABLED(E_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting E_ENABLE_ON
#endif


// Invert STEP pin
#if DISABLED(INVERT_X_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_X_STEP_PIN
#endif
#if DISABLED(INVERT_Y_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_Y_STEP_PIN
#endif
#if DISABLED(INVERT_Z_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_Z_STEP_PIN
#endif
#if DISABLED(INVERT_E_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_E_STEP_PIN
#endif


// Invert direction
#if DISABLED(INVERT_X_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_X_DIR
#endif
#if DISABLED(INVERT_Y_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_Y_DIR
#endif
#if DISABLED(INVERT_Z_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_Z_DIR
#endif
#if DISABLED(INVERT_E0_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E0_DIR
#endif
#if DISABLED(INVERT_E1_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E1_DIR
#endif
#if DISABLED(INVERT_E2_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E2_DIR
#endif
#if DISABLED(INVERT_E3_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E3_DIR
#endif


// Disable axis
#if DISABLED(DISABLE_X)
  #error DEPENDENCY ERROR: Missing setting DISABLE_X
#endif
#if DISABLED(DISABLE_Y)
  #error DEPENDENCY ERROR: Missing setting DISABLE_Y
#endif
#if DISABLED(DISABLE_Z)
  #error DEPENDENCY ERROR: Missing setting DISABLE_Z
#endif
#if DISABLED(DISABLE_E)
  #error DEPENDENCY ERROR: Missing setting DISABLE_E
#endif


// Max/Min position
#if DISABLED(X_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting X_MAX_POS
#endif
#if DISABLED(X_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting X_MIN_POS
#endif
#if DISABLED(Y_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting Y_MAX_POS
#endif
#if DISABLED(Y_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting Y_MIN_POS
#endif
#if DISABLED(Z_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting Z_MAX_POS
#endif
#if DISABLED(Z_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting Z_MIN_POS
#endif
#if DISABLED(E_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting E_MIN_POS
#endif


// Dual X carriage
#if ENABLED(DUAL_X_CARRIAGE)
  #if DISABLED(X2_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting X2_MIN_POS
  #endif
  #if DISABLED(X2_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting X2_MAX_POS
  #endif
  #if DISABLED(X2_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting X2_HOME_DIR
  #endif
  #if DISABLED(X2_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting X2_HOME_POS
  #endif
  #if DISABLED(DEFAULT_DUAL_X_CARRIAGE_MODE)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DUAL_X_CARRIAGE_MODE
  #endif
  #if DISABLED(TOOLCHANGE_PARK_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_PARK_ZLIFT
  #endif
  #if DISABLED(TOOLCHANGE_UNPARK_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_UNPARK_ZLIFT
  #endif
  #if DISABLED(DEFAULT_DUPLICATION_X_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DUPLICATION_X_OFFSET
  #endif

  #if EXTRUDERS == 1
    #error "DUAL_X_CARRIAGE requires 2 (or more) extruders."
  #elif MECH(COREXY) || MECH(COREXZ)
    #error "DUAL_X_CARRIAGE cannot be used with COREXY or COREXZ."
  #elif !HAS_X2_ENABLE || !HAS_X2_STEP || !HAS_X2_DIR
    #error "DUAL_X_CARRIAGE requires X2 stepper pins to be defined."
  #elif !HAS_X_MAX
    #error "DUAL_X_CARRIAGE requires use a X Max Endstop."
  #elif DISABLED(X2_HOME_POS) || DISABLED(X2_MIN_POS) || DISABLED(X2_MAX_POS)
    #error "DUAL_X_CARRIAGE requires X2_HOME_POS, X2_MIN_POS, and X2_MAX_POS."
  #elif X_HOME_DIR != -1 || X2_HOME_DIR != 1
    #error "DUAL_X_CARRIAGE requires X_HOME_DIR -1 and X2_HOME_DIR 1."
  #endif
#endif


// Accuracy settings
#if DISABLED(MIN_STEPS_PER_SEGMENT)
  #error DEPENDENCY ERROR: Missing setting MIN_STEPS_PER_SEGMENT
#endif
#if DISABLED(DEFAULT_MINSEGMENTTIME)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MINSEGMENTTIME
#endif
#if DISABLED(MM_PER_ARC_SEGMENT)
  #error DEPENDENCY ERROR: Missing setting MM_PER_ARC_SEGMENT
#endif
#if DISABLED(N_ARC_CORRECTION)
  #error DEPENDENCY ERROR: Missing setting N_ARC_CORRECTION
#endif
#if DISABLED(DEFAULT_AXIS_STEPS_PER_UNIT)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_AXIS_STEPS_PER_UNIT
#endif


// Velocity and acceleration
#if DISABLED(DEFAULT_MINTRAVELFEEDRATE)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MINTRAVELFEEDRATE
#endif
#if DISABLED(DEFAULT_MAX_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MAX_ACCELERATION
#endif
#if DISABLED(DEFAULT_RETRACT_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_RETRACT_ACCELERATION
#endif
#if DISABLED(DEFAULT_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_ACCELERATION
#endif
#if DISABLED(DEFAULT_TRAVEL_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_TRAVEL_ACCELERATION
#endif
#if DISABLED(DEFAULT_XJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_XJERK
#endif
#if DISABLED(DEFAULT_YJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_YJERK
#endif
#if DISABLED(DEFAULT_ZJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_ZJERK
#endif


// Two or more Z steppers
#if ENABLED(Z_TWO_STEPPER)
  #if ENABLED(Z_THREE_STEPPER) || ENABLED(Z_FOUR_STEPPER)
    #error "CONFLICT ERROR: You cannot have two Z stepper and three or four drivers".
  #endif
#elif ENABLED(Z_THREE_STEPPER) && ENABLED(Z_FOUR_STEPPER)
  #error "CONFLICT ERROR: You cannot have three Z stepper and four drivers".
#endif


// Two Y steppers
#if ENABLED(Y_TWO_STEPPER)
  #if DISABLED(INVERT_Y2_VS_Y_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_Y2_VS_Y_DIR
  #endif
#endif

// Pin definitions
#if !PIN_EXISTS(X_STEP)
  #error DEPENDENCY ERROR: X_STEP_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(X_DIR)
  #error DEPENDENCY ERROR: X_DIR_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(X_ENABLE)
  #error DEPENDENCY ERROR: X_ENABLE_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Y_STEP)
  #error DEPENDENCY ERROR: Y_STEP_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Y_DIR)
  #error DEPENDENCY ERROR: Y_DIR_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Y_ENABLE)
  #error DEPENDENCY ERROR: Y_ENABLE_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Z_STEP)
  #error DEPENDENCY ERROR: Z_STEP_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Z_DIR)
  #error DEPENDENCY ERROR: Z_DIR_PIN is not defined for your board. You have to define it yourself.
#endif
#if !PIN_EXISTS(Z_ENABLE)
  #error DEPENDENCY ERROR: Z_ENABLE_PIN is not defined for your board. You have to define it yourself.
#endif

#if DRIVER_EXTRUDERS > 0
  #if !PIN_EXISTS(E0_STEP)
    #error DEPENDENCY ERROR: E0_STEP_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(E0_DIR)
    #error DEPENDENCY ERROR: E0_DIR_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(E0_ENABLE)
    #error DEPENDENCY ERROR: E0_ENABLE_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if DRIVER_EXTRUDERS > 1
    #if !PIN_EXISTS(E1_STEP)
      #error DEPENDENCY ERROR: E1_STEP_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if !PIN_EXISTS(E1_DIR)
      #error DEPENDENCY ERROR: E1_DIR_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if !PIN_EXISTS(E1_ENABLE)
      #error DEPENDENCY ERROR: E1_ENABLE_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if DRIVER_EXTRUDERS > 2
      #if !PIN_EXISTS(E2_STEP)
        #error DEPENDENCY ERROR: E2_STEP_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if !PIN_EXISTS(E2_DIR)
        #error DEPENDENCY ERROR: E2_DIR_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if !PIN_EXISTS(E2_ENABLE)
        #error DEPENDENCY ERROR: E2_ENABLE_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if DRIVER_EXTRUDERS > 3
        #if !PIN_EXISTS(E3_STEP)
          #error DEPENDENCY ERROR: E3_STEP_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if !PIN_EXISTS(E3_DIR)
          #error DEPENDENCY ERROR: E3_DIR_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if !PIN_EXISTS(E3_ENABLE)
          #error DEPENDENCY ERROR: E3_ENABLE_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if DRIVER_EXTRUDERS > 4
          #if !PIN_EXISTS(E4_STEP)
            #error DEPENDENCY ERROR: E4_STEP_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if !PIN_EXISTS(E4_DIR)
            #error DEPENDENCY ERROR: E4_DIR_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if !PIN_EXISTS(E4_ENABLE)
            #error DEPENDENCY ERROR: E4_ENABLE_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if DRIVER_EXTRUDERS > 5
            #if !PIN_EXISTS(E5_STEP)
              #error DEPENDENCY ERROR: E5_STEP_PIN is not defined for your board. You have to define it yourself.
            #endif
            #if !PIN_EXISTS(E5_DIR)
              #error DEPENDENCY ERROR: E5_DIR_PIN is not defined for your board. You have to define it yourself.
            #endif
            #if !PIN_EXISTS(E5_ENABLE)
              #error DEPENDENCY ERROR: E5_ENABLE_PIN is not defined for your board. You have to define it yourself.
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif
#if ENABLED(X2_IS_TMC) && (!PIN_EXISTS(X2_ENABLE) || !PIN_EXISTS(X2_STEP) || !PIN_EXISTS(X2_DIR))
  #error DEPENDENCY ERROR: You have to set X2_ENABLE_PIN, X2_STEP_PIN and X2_DIR_PIN to a valid pin if you enable X2_IS_TMC
#endif

#endif /* _MECH_SANITYCHECK_H_ */
