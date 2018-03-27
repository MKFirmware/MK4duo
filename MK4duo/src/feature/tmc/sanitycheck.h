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

#ifndef _TMC_SANITYCHECK_H_
#define _TMC_SANITYCHECK_H_

/**
 * Make sure HAVE_TMC26X is warranted
 */
#if ENABLED(HAVE_TMC26X) && !( \
         ENABLED( X_IS_TMC26X) \
      || ENABLED(X2_IS_TMC26X) \
      || ENABLED( Y_IS_TMC26X) \
      || ENABLED(Y2_IS_TMC26X) \
      || ENABLED( Z_IS_TMC26X) \
      || ENABLED(Z2_IS_TMC26X) \
      || ENABLED(E0_IS_TMC26X) \
      || ENABLED(E1_IS_TMC26X) \
      || ENABLED(E2_IS_TMC26X) \
      || ENABLED(E3_IS_TMC26X) \
      || ENABLED(E4_IS_TMC26X) \
      || ENABLED(E5_IS_TMC26X) \
  )
  #error "DEPENDENCY ERROR: HAVE_TMC26X requires at least one TMC26X stepper to be set."
#endif

/**
 * TMC2130 Requirements
 */
#if ENABLED(HAVE_TMC2130)
  #if !( ENABLED( X_IS_TMC2130) \
      || ENABLED(X2_IS_TMC2130) \
      || ENABLED( Y_IS_TMC2130) \
      || ENABLED(Y2_IS_TMC2130) \
      || ENABLED( Z_IS_TMC2130) \
      || ENABLED(Z2_IS_TMC2130) \
      || ENABLED(E0_IS_TMC2130) \
      || ENABLED(E1_IS_TMC2130) \
      || ENABLED(E2_IS_TMC2130) \
      || ENABLED(E3_IS_TMC2130) \
      || ENABLED(E4_IS_TMC2130) \
      || ENABLED(E5_IS_TMC2130) \
    )
    #error "DEPENDENCY ERROR: HAVE_TMC2130 requires at least one TMC2130 stepper to be set."
  #elif TMC2130STEPPER_VERSION < 0x020201
    #error "DEPENDENCY ERROR: Update TMC2130Stepper library to 2.2.1 or newer."
  #elif ENABLED(HYBRID_THRESHOLD) && DISABLED(STEALTHCHOP)
    #error "DEPENDENCY ERROR: Enable STEALTHCHOP to use HYBRID_THRESHOLD."
  #endif

  #if ENABLED(X_IS_TMC2130) && !PIN_EXISTS(X_CS)
    #error "DEPENDENCY ERROR: X_CS_PIN is required for X_IS_TMC2130. Define X_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(X2_IS_TMC2130) && !PIN_EXISTS(X2_CS)
    #error "DEPENDENCY ERROR: X2_CS_PIN is required for X2_IS_TMC2130. Define X2_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(Y_IS_TMC2130) && !PIN_EXISTS(Y_CS)
    #error "DEPENDENCY ERROR: Y_CS_PIN is required for Y_IS_TMC2130. Define Y_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(Y2_IS_TMC2130) && !PIN_EXISTS(Y2_CS)
    #error "DEPENDENCY ERROR: Y2_CS_PIN is required for Y2_IS_TMC2130. Define Y2_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(Z_IS_TMC2130) && !PIN_EXISTS(Z_CS)
    #error "DEPENDENCY ERROR: Z_CS_PIN is required for Z_IS_TMC2130. Define Z_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(Z2_IS_TMC2130) && !PIN_EXISTS(Z2_CS)
    #error "DEPENDENCY ERROR: Z2_CS_PIN is required for Z2_IS_TMC2130. Define Z2_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(E0_IS_TMC2130) && !PIN_EXISTS(E0_CS)
    #error "DEPENDENCY ERROR: E0_CS_PIN is required for E0_IS_TMC2130. Define E0_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(E1_IS_TMC2130) && !PIN_EXISTS(E1_CS)
    #error "DEPENDENCY ERROR: E1_CS_PIN is required for E1_IS_TMC2130. Define E1_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(E2_IS_TMC2130) && !PIN_EXISTS(E2_CS)
    #error "DEPENDENCY ERROR: E2_CS_PIN is required for E2_IS_TMC2130. Define E2_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(E3_IS_TMC2130) && !PIN_EXISTS(E3_CS)
    #error "DEPENDENCY ERROR: E3_CS_PIN is required for E3_IS_TMC2130. Define E3_CS_PIN in Configuration_Pins.h."
  #elif ENABLED(E4_IS_TMC2130) && !PIN_EXISTS(E4_CS)
    #error "DEPENDENCY ERROR: E4_CS_PIN is required for E4_IS_TMC2130. Define E4_CS_PIN in Configuration_Pins.h."
  #endif

  // Require STEALTHCHOP for SENSORLESS_HOMING on DELTA as the transition from spreadCycle to stealthChop
  // is necessary in order to reset the stallGuard indication between the initial movement of all three
  // towers to +Z and the individual homing of each tower. This restriction can be removed once a means of
  // clearing the stallGuard activated status is found.
  #if ENABLED(SENSORLESS_HOMING) && ENABLED(DELTA) && !ENABLED(STEALTHCHOP)
    #error "DEPENDENCY ERROR: SENSORLESS_HOMING on DELTA currently requires STEALTHCHOP."
  #endif

  // Sensorless homing is required for both combined steppers in an H-bot
  #if CORE_IS_XY && X_SENSORLESS != Y_SENSORLESS
    #error "DEPENDENCY ERROR: CoreXY requires both X and Y to use sensorless homing if either does."
  #elif CORE_IS_XZ && X_SENSORLESS != Z_SENSORLESS
    #error "DEPENDENCY ERROR: CoreXZ requires both X and Z to use sensorless homing if either does."
  #elif CORE_IS_YZ && Y_SENSORLESS != Z_SENSORLESS
    #error "DEPENDENCY ERROR: CoreYZ requires both Y and Z to use sensorless homing if either does."
  #endif

#elif ENABLED(SENSORLESS_HOMING)

  #error "DEPENDENCY ERROR: SENSORLESS_HOMING requires TMC2130 stepper drivers."

#endif

/**
 * TMC2208 Requirements
 */
#if ENABLED(HAVE_TMC2208)
  #if !( ENABLED( X_IS_TMC2208) \
      || ENABLED(X2_IS_TMC2208) \
      || ENABLED( Y_IS_TMC2208) \
      || ENABLED(Y2_IS_TMC2208) \
      || ENABLED( Z_IS_TMC2208) \
      || ENABLED(Z2_IS_TMC2208) \
      || ENABLED(E0_IS_TMC2208) \
      || ENABLED(E1_IS_TMC2208) \
      || ENABLED(E2_IS_TMC2208) \
      || ENABLED(E3_IS_TMC2208) \
      || ENABLED(E4_IS_TMC2208) \
      || ENABLED(E5_IS_TMC2208) \
    )
    #error "DEPENDENCY ERROR: HAVE_TMC2208 requires at least one TMC2208 stepper to be set."
  #elif ENABLED(ENDSTOP_INTERRUPTS_FEATURE) && \
      !( defined( X_HARDWARE_SERIAL) \
      || defined(X2_HARDWARE_SERIAL) \
      || defined( Y_HARDWARE_SERIAL) \
      || defined(Y2_HARDWARE_SERIAL) \
      || defined( Z_HARDWARE_SERIAL) \
      || defined(Z2_HARDWARE_SERIAL) \
      || defined(E0_HARDWARE_SERIAL) \
      || defined(E1_HARDWARE_SERIAL) \
      || defined(E2_HARDWARE_SERIAL) \
      || defined(E3_HARDWARE_SERIAL) \
      || defined(E4_HARDWARE_SERIAL) \
      || defined(E5_HARDWARE_SERIAL) \
    )
    #error "DEPENDENCY ERROR: Select *_HARDWARE_SERIAL to use both TMC2208 and ENDSTOP_INTERRUPTS_FEATURE."
  #elif TMC2208STEPPER_VERSION < 0x000101
    #error "DEPENDENCY ERROR: Update TMC2130Stepper library to 0.1.1 or newer."
  #endif
#endif

#if ENABLED(HYBRID_THRESHOLD) && DISABLED(STEALTHCHOP)
  #error "DEPENDENCY ERROR: Enable STEALTHCHOP to use HYBRID_THRESHOLD."
#endif

#if ENABLED(TMC_Z_CALIBRATION) && !Z_IS_TRINAMIC && !Z2_IS_TRINAMIC
  #error "DEPENDENCY ERROR: TMC_Z_CALIBRATION requires at least one TMC driver on Z axis"
#endif

/**
 * Make sure HAVE_L6470DRIVER is warranted
 */
#if ENABLED(HAVE_L6470DRIVER) && !( \
       ENABLED( X_IS_L6470) \
    || ENABLED(X2_IS_L6470) \
    || ENABLED( Y_IS_L6470) \
    || ENABLED(Y2_IS_L6470) \
    || ENABLED( Z_IS_L6470) \
    || ENABLED(Z2_IS_L6470) \
    || ENABLED(E0_IS_L6470) \
    || ENABLED(E1_IS_L6470) \
    || ENABLED(E2_IS_L6470) \
    || ENABLED(E3_IS_L6470) \
    || ENABLED(E4_IS_L6470) \
    || ENABLED(E5_IS_L6470) \
  )
  #error "DEPENDENCY ERROR: HAVE_L6470DRIVER requires at least one L6470 stepper to be set."
#endif

/**
 * Check that each axis has only one driver selected
 */
#if 1 < 0 \
  + ENABLED(X_IS_TMC26X) \
  + ENABLED(X_IS_TMC2130) \
  + ENABLED(X_IS_TMC2208) \
  + ENABLED(X_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the X axis."
#endif
#if 1 < 0 \
  + ENABLED(X2_IS_TMC26X) \
  + ENABLED(X2_IS_TMC2130) \
  + ENABLED(X2_IS_TMC2208) \
  + ENABLED(X2_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the X2 axis."
#endif
#if 1 < 0 \
  + ENABLED(Y_IS_TMC26X) \
  + ENABLED(Y_IS_TMC2130) \
  + ENABLED(Y_IS_TMC2208) \
  + ENABLED(Y_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the Y axis."
#endif
#if 1 < 0 \
  + ENABLED(Y2_IS_TMC26X) \
  + ENABLED(Y2_IS_TMC2130) \
  + ENABLED(Y2_IS_TMC2208) \
  + ENABLED(Y2_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the Y2 axis."
#endif
#if 1 < 0 \
  + ENABLED(Z_IS_TMC26X) \
  + ENABLED(Z_IS_TMC2130) \
  + ENABLED(Z_IS_TMC2208) \
  + ENABLED(Z_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the Z axis."
#endif
#if 1 < 0 \
  + ENABLED(Z2_IS_TMC26X) \
  + ENABLED(Z2_IS_TMC2130) \
  + ENABLED(Z2_IS_TMC2208) \
  + ENABLED(Z2_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the Z2 axis."
#endif
#if 1 < 0 \
  + ENABLED(E0_IS_TMC26X) \
  + ENABLED(E0_IS_TMC2130) \
  + ENABLED(E0_IS_TMC2208) \
  + ENABLED(E0_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the E0 axis."
#endif
#if 1 < 0 \
  + ENABLED(E1_IS_TMC26X) \
  + ENABLED(E1_IS_TMC2130) \
  + ENABLED(E1_IS_TMC2208) \
  + ENABLED(E1_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the E1 axis."
#endif
#if 1 < 0 \
  + ENABLED(E2_IS_TMC26X) \
  + ENABLED(E2_IS_TMC2130) \
  + ENABLED(E2_IS_TMC2208) \
  + ENABLED(E2_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the E2 axis."
#endif
#if 1 < 0 \
  + ENABLED(E3_IS_TMC26X) \
  + ENABLED(E3_IS_TMC2130) \
  + ENABLED(E3_IS_TMC2208) \
  + ENABLED(E3_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the E3 axis."
#endif
#if 1 < 0 \
  + ENABLED(E4_IS_TMC26X) \
  + ENABLED(E4_IS_TMC2130) \
  + ENABLED(E4_IS_TMC2208) \
  + ENABLED(E4_IS_L6470)
  #error "DEPENDENCY ERROR: Please enable only one stepper driver for the E4 axis."
#endif

#endif /* _TMC_SANITYCHECK_H_ */
