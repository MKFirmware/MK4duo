/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

/**
 * Allow only one probe option to be defined
 */
#if 1 < 0 \
  + ENABLED(PROBE_MANUALLY)                   \
  + ENABLED(PROBE_FIX_MOUNTED)              \
  + (HAS_Z_SERVO_PROBE && DISABLED(BLTOUCH))  \
  + ENABLED(BLTOUCH)                          \
  + ENABLED(PROBE_ALLEN_KEY)                \
  + ENABLED(PROBE_SLED)                     \
  + ENABLED(PROBE_SENSORLESS)
  #error "DEPENDENCY ERROR: Please enable only one probe: PROBE_MANUALLY, PROBE_FIX_MOUNTED, Z Servo, BLTOUCH, PROBE_ALLEN_KEY, PROBE_SLED, or PROBE_SENSORLESS."
#endif

#if HAS_BED_PROBE

  // PROBE_SLED is incompatible with DELTA
  #if ENABLED(PROBE_SLED) && MECH(DELTA)
    #error "DEPENDENCY ERROR: You cannot use PROBE_SLED with DELTA."
  #endif

  // NUM_SERVOS is required for a Z servo probe
  #if HAS_Z_SERVO_PROBE
    #ifndef NUM_SERVOS
      #error "DEPENDENCY ERROR: You must set NUM_SERVOS for a servo probe (PROBE_SERVO_NR)."
    #elif PROBE_SERVO_NR >= NUM_SERVOS
      #error "DEPENDENCY ERROR: PROBE_SERVO_NR must be less than NUM_SERVOS."
    #endif
  #endif

  // Require pin options and pins to be defined
  #if ENABLED(PROBE_SENSORLESS)
    #if MECH(DELTA) && (!AXIS_HAS_STALLGUARD(X) || !AXIS_HAS_STALLGUARD(Y) || !AXIS_HAS_STALLGUARD(Z))
      #error "PROBE_SENSORLESS requires TMC2130 drivers on X, Y, and Z."
    #elif !AXIS_HAS_STALLGUARD(Z)
      #error "PROBE_SENSORLESS requires a TMC2130 driver on Z."
    #endif
  #else
    #if DISABLED(PROBE_MANUALLY) && !PROBE_PIN_CONFIGURED
      #error "DEPENDENCY ERROR: A probe needs a pin! Use Z_MIN_PIN or Z_PROBE_PIN."
    #endif
  #endif

  // Make sure Z raise values are set
  #if DISABLED(Z_PROBE_DEPLOY_HEIGHT)
    #error "DEPENDENCY ERROR: You must define Z_PROBE_DEPLOY_HEIGHT in your configuration."
  #elif DISABLED(Z_PROBE_BETWEEN_HEIGHT)
    #error "DEPENDENCY ERROR: You must define Z_PROBE_BETWEEN_HEIGHT in your configuration."
  #elif Z_PROBE_DEPLOY_HEIGHT < 0
    #error "DEPENDENCY ERROR: Probes need Z_PROBE_DEPLOY_HEIGHT >= 0."
  #elif Z_PROBE_BETWEEN_HEIGHT < 0
    #error "DEPENDENCY ERROR: Probes need Z_PROBE_BETWEEN_HEIGHT >= 0."
  #endif

#elif !PROBE_SELECTED

  // Require some kind of probe for bed leveling and probe testing
  #if OLD_ABL
    #error "DEPENDENCY ERROR: Auto Bed Leveling requires a probe! Define a PROBE_MANUALLY, Z Servo, BLTOUCH, PROBE_ALLEN_KEY, PROBE_SLED, or PROBE_FIX_MOUNTED."
  #elif ENABLED(DELTA_AUTO_CALIBRATION_1)
    #error "DEPENDENCY ERROR: DELTA_AUTO_CALIBRATION_1 requires a probe! Define a Z PROBE_MANUALLY, Servo, BLTOUCH, PROBE_ALLEN_KEY, PROBE_SLED, or PROBE_FIX_MOUNTED."
  #elif ENABLED(DELTA_AUTO_CALIBRATION_2)
    #error "DEPENDENCY ERROR: DELTA_AUTO_CALIBRATION_2 requires a probe! Define a Z PROBE_MANUALLY, Servo, BLTOUCH, PROBE_ALLEN_KEY, PROBE_SLED, or PROBE_FIX_MOUNTED."
  #endif

#endif

#if (!HAS_BED_PROBE || ENABLED(PROBE_MANUALLY)) && ENABLED(PROBE_REPEATABILITY_TEST)
  #error "DEPENDENCY ERROR: PROBE_REPEATABILITY_TEST requires a probe! Define a Probe Servo, BLTOUCH, PROBE_ALLEN_KEY, PROBE_SLED, or PROBE_FIX_MOUNTED."
#endif

#if ENABLED(PROBE_SLED) && !PIN_EXISTS(SLED)
  #error "DEPENDENCY ERROR: You have to set SLED_PIN to a valid pin if you enable PROBE_SLED."
#endif

// G38 Probe Target
#if ENABLED(G38_PROBE_TARGET)
  #if !HAS_BED_PROBE
    #error "DEPENDENCY ERROR: G38_PROBE_TARGET requires a bed probe."
  #elif IS_KINEMATIC
    #error "DEPENDENCY ERROR: G38_PROBE_TARGET requires a Cartesian or Core machine."
  #endif
#endif

#if HOMING_Z_WITH_PROBE && DISABLED(Z_SAFE_HOMING)
  #error "Z_SAFE_HOMING is recommended when homing with a probe. Enable it or comment out this line to continue."
#endif
