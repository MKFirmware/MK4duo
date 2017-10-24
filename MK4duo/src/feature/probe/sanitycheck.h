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

#ifndef _PROBE_SANITYCHECK_H_
#define _PROBE_SANITYCHECK_H_

// Probes
#if PROBE_SELECTED

  // Allow only one probe option to be defined
  static_assert(1 >= 0
    #if ENABLED(PROBE_MANUALLY)
      + 1
    #endif
    #if ENABLED(FIX_MOUNTED_PROBE)
      + 1
    #endif
    #if HAS_Z_SERVO_PROBE && DISABLED(BLTOUCH)
      + 1
    #endif
    #if ENABLED(BLTOUCH)
      + 1
    #endif
    #if ENABLED(Z_PROBE_ALLEN_KEY)
      + 1
    #endif
    #if ENABLED(Z_PROBE_SLED)
      + 1
    #endif
    , "Please enable only one probe: FIX_MOUNTED_PROBE, Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, or Z_PROBE_SLED."
  );

  // Z_PROBE_SLED is incompatible with DELTA
  #if ENABLED(Z_PROBE_SLED) && MECH(DELTA)
    #error "You cannot use Z_PROBE_SLED with DELTA."
  #endif

  // NUM_SERVOS is required for a Z servo probe
  #if HAS_Z_SERVO_PROBE
    #ifndef NUM_SERVOS
      #error "You must set NUM_SERVOS for a Z servo probe (Z_ENDSTOP_SERVO_NR)."
    #elif Z_ENDSTOP_SERVO_NR >= NUM_SERVOS
      #error "Z_ENDSTOP_SERVO_NR must be less than NUM_SERVOS."
    #endif
  #endif

  // A probe needs a pin
  #if DISABLED(PROBE_MANUALLY) && !PROBE_PIN_CONFIGURED
    #error "A probe needs a pin! Use Z_MIN_PIN or Z_PROBE_PIN."
  #endif

  // Make sure Z raise values are set
  #if DISABLED(Z_PROBE_DEPLOY_HEIGHT)
    #error "You must define Z_PROBE_DEPLOY_HEIGHT in your configuration."
  #elif DISABLED(Z_PROBE_BETWEEN_HEIGHT)
    #error "You must define Z_PROBE_BETWEEN_HEIGHT in your configuration."
  #elif Z_PROBE_DEPLOY_HEIGHT < 0
    #error "Probes need Z_PROBE_DEPLOY_HEIGHT >= 0."
  #elif Z_PROBE_BETWEEN_HEIGHT < 0
    #error "Probes need Z_PROBE_BETWEEN_HEIGHT >= 0."
  #endif

#else

  // Require some kind of probe for bed leveling and probe testing
  #if HAS_ABL && DISABLED(AUTO_BED_LEVELING_UBL)
    #error "Auto Bed Leveling requires a probe! Define a PROBE_MANUALLY, Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #elif ENABLED(DELTA_AUTO_CALIBRATION_1)
    #error "DELTA_AUTO_CALIBRATION_1 requires a probe! Define a Z PROBE_MANUALLY, Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #elif ENABLED(DELTA_AUTO_CALIBRATION_2)
    #error "DELTA_AUTO_CALIBRATION_2 requires a probe! Define a Z PROBE_MANUALLY, Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
  #endif

#endif

#if (!HAS_BED_PROBE || ENABLED(PROBE_MANUALLY)) && ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
  #error "Z_MIN_PROBE_REPEATABILITY_TEST requires a probe! Define a Z Servo, BLTOUCH, Z_PROBE_ALLEN_KEY, Z_PROBE_SLED, or Z_PROBE_FIX_MOUNTED."
#endif

#if ENABLED(Z_PROBE_SLED) && !PIN_EXISTS(SLED)
  #error DEPENDENCY ERROR: You have to set SLED_PIN to a valid pin if you enable Z_PROBE_SLED
#endif

// Check auto bed leveling sub-options, especially probe points
#if ABL_GRID

  #if DISABLED(DELTA_PROBEABLE_RADIUS)
    // Be sure points are in the right order
    #if LEFT_PROBE_BED_POSITION > RIGHT_PROBE_BED_POSITION
      #error "LEFT_PROBE_BED_POSITION must be less than RIGHT_PROBE_BED_POSITION."
    #elif FRONT_PROBE_BED_POSITION > BACK_PROBE_BED_POSITION
      #error "FRONT_PROBE_BED_POSITION must be less than BACK_PROBE_BED_POSITION."
    #endif
  #endif
#endif

// G38 Probe Target
#if ENABLED(G38_PROBE_TARGET)
  #if !HAS_BED_PROBE
    #error "G38_PROBE_TARGET requires a bed probe."
  #elif !IS_CARTESIAN
    #error "G38_PROBE_TARGET requires a Cartesian machine."
  #endif
#endif

#endif /* _PROBE_SANITYCHECK_H_ */
