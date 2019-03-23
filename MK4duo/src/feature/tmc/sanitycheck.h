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

#ifndef _TMC_SANITYCHECK_H_
#define _TMC_SANITYCHECK_H_

/**
 * TMC2130 Requirements
 */
#if HAVE_DRV(TMC2130)

  #if ENABLED(HYBRID_THRESHOLD) && !TMC_HAS_STEALTHCHOP
    #error "DEPENDENCY ERROR: Enable STEALTHCHOP on axis to use HYBRID_THRESHOLD."
  #endif

  #if X_HAS_DRV(TMC2130) && !PIN_EXISTS(X_CS)
    #error "DEPENDENCY ERROR: X_CS_PIN is required for X_IS_TMC2130. Define X_CS_PIN in Configuration_Pins.h."
  #elif X2_HAS_DRV(TMC2130) && !PIN_EXISTS(X2_CS)
    #error "DEPENDENCY ERROR: X2_CS_PIN is required for X2_IS_TMC2130. Define X2_CS_PIN in Configuration_Pins.h."
  #elif Y_HAS_DRV(TMC2130) && !PIN_EXISTS(Y_CS)
    #error "DEPENDENCY ERROR: Y_CS_PIN is required for Y_IS_TMC2130. Define Y_CS_PIN in Configuration_Pins.h."
  #elif Y2_HAS_DRV(TMC2130) && !PIN_EXISTS(Y2_CS)
    #error "DEPENDENCY ERROR: Y2_CS_PIN is required for Y2_IS_TMC2130. Define Y2_CS_PIN in Configuration_Pins.h."
  #elif Z_HAS_DRV(TMC2130) && !PIN_EXISTS(Z_CS)
    #error "DEPENDENCY ERROR: Z_CS_PIN is required for Z_IS_TMC2130. Define Z_CS_PIN in Configuration_Pins.h."
  #elif Z2_HAS_DRV(TMC2130) && !PIN_EXISTS(Z2_CS)
    #error "DEPENDENCY ERROR: Z2_CS_PIN is required for Z2_IS_TMC2130. Define Z2_CS_PIN in Configuration_Pins.h."
  #elif E0_HAS_DRV(TMC2130) && !PIN_EXISTS(E0_CS)
    #error "DEPENDENCY ERROR: E0_CS_PIN is required for E0_IS_TMC2130. Define E0_CS_PIN in Configuration_Pins.h."
  #elif E1_HAS_DRV(TMC2130) && !PIN_EXISTS(E1_CS)
    #error "DEPENDENCY ERROR: E1_CS_PIN is required for E1_IS_TMC2130. Define E1_CS_PIN in Configuration_Pins.h."
  #elif E2_HAS_DRV(TMC2130) && !PIN_EXISTS(E2_CS)
    #error "DEPENDENCY ERROR: E2_CS_PIN is required for E2_IS_TMC2130. Define E2_CS_PIN in Configuration_Pins.h."
  #elif E3_HAS_DRV(TMC2130) && !PIN_EXISTS(E3_CS)
    #error "DEPENDENCY ERROR: E3_CS_PIN is required for E3_IS_TMC2130. Define E3_CS_PIN in Configuration_Pins.h."
  #elif E4_HAS_DRV(TMC2130) && !PIN_EXISTS(E4_CS)
    #error "DEPENDENCY ERROR: E4_CS_PIN is required for E4_IS_TMC2130. Define E4_CS_PIN in Configuration_Pins.h."
  #elif E5_HAS_DRV(TMC2130) && !PIN_EXISTS(E5_CS)
    #error "DEPENDENCY ERROR: E5_CS_PIN is required for E5_IS_TMC2130. Define E5_CS_PIN in Configuration_Pins.h."
  #endif

  // Require STEALTHCHOP for SENSORLESS_HOMING on DELTA as the transition from spreadCycle to stealthChop
  // is necessary in order to reset the stallGuard indication between the initial movement of all three
  // towers to +Z and the individual homing of each tower. This restriction can be removed once a means of
  // clearing the stallGuard activated status is found.
  #if ENABLED(SENSORLESS_HOMING) && MECH(DELTA) && (!AXIS_HAS_STALLGUARD(X) || !AXIS_HAS_STALLGUARD(Y) || !AXIS_HAS_STALLGUARD(Z))
    #error "DEPENDENCY ERROR: SENSORLESS_HOMING on DELTA currently requires STEALTHCHOP on all axis."
  #endif

  // Sensorless homing is required for both combined steppers in an H-bot
  #if CORE_IS_XY && X_HAS_SENSORLESS != Y_HAS_SENSORLESS
    #error "DEPENDENCY ERROR: CoreXY requires both X and Y to use sensorless homing if either does."
  #elif CORE_IS_XZ && X_HAS_SENSORLESS != Z_HAS_SENSORLESS
    #error "DEPENDENCY ERROR: CoreXZ requires both X and Z to use sensorless homing if either does."
  #elif CORE_IS_YZ && Y_HAS_SENSORLESS != Z_HAS_SENSORLESS
    #error "DEPENDENCY ERROR: CoreYZ requires both Y and Z to use sensorless homing if either does."
  #endif

#elif ENABLED(SENSORLESS_HOMING)

  #error "DEPENDENCY ERROR: SENSORLESS_HOMING requires TMC2130 stepper drivers."

#endif

/**
 * TMC2208 Requirements
 */
#if HAVE_DRV(TMC2208)

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE) && \
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
    #error "DEPENDENCY ERROR: Select hardware UART for TMC2208 to use both TMC2208 and ENDSTOP_INTERRUPTS_FEATURE."
  #endif
#endif

#if ENABLED(TMC_Z_CALIBRATION) && !AXIS_HAS_TMC(Z) && !AXIS_HAS_TMC(Z2)
  #error "DEPENDENCY ERROR: TMC_Z_CALIBRATION requires at least one TMC driver on Z axis"
#endif

#if !HAS_TRINAMIC
  #if ENABLED(MONITOR_DRIVER_STATUS)
    #error "DEPENDENCY ERROR: MONITOR_DRIVER_STATUS requires at least one TMC driver"
  #elif ENABLED(TMC_DEBUG)
    #error "DEPENDENCY ERROR: TMC_DEBUG requires at least one TMC driver"
  #endif
#endif

#endif /* _TMC_SANITYCHECK_H_ */
