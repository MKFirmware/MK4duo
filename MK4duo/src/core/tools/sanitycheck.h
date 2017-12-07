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

#ifndef _TOOLS_SANITYCHECK_H_
#define _TOOLS_SANITYCHECK_H_

// Extruders
#if DISABLED(EXTRUDERS)
  #error DEPENDENCY ERROR: Missing setting EXTRUDERS
#endif
#if DISABLED(DRIVER_EXTRUDERS)
  #error DEPENDENCY ERROR: Missing setting DRIVER_EXTRUDERS
#endif

#if ENABLED(PREVENT_COLD_EXTRUSION)
  #if DISABLED(EXTRUDE_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting EXTRUDE_MINTEMP
  #endif
  #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
    #if DISABLED(EXTRUDE_MAXLENGTH)
      #error DEPENDENCY ERROR: Missing setting EXTRUDE_MAXLENGTH
    #endif
  #endif
#endif

// Idle oozing prevent
#if ENABLED(IDLE_OOZING_PREVENT)
  #if DISABLED(IDLE_OOZING_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_MINTEMP
  #endif
  #if DISABLED(IDLE_OOZING_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_FEEDRATE
  #endif
  #if DISABLED(IDLE_OOZING_SECONDS)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_SECONDS
  #endif
  #if DISABLED(IDLE_OOZING_LENGTH)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_LENGTH
  #endif
  #if DISABLED(IDLE_OOZING_RECOVER_LENGTH)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_LENGTH
  #endif
  #if DISABLED(IDLE_OOZING_RECOVER_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_FEEDRATE
  #endif
  #if DISABLED(PREVENT_COLD_EXTRUSION)
    #error DEPENDENCY ERROR: IDLE_OOZING_MINTEMP needs PREVENT_COLD_EXTRUSION
  #endif
  #if IDLE_OOZING_MINTEMP < EXTRUDE_MINTEMP
    #error CONFLICT ERROR: IDLE_OOZING_MINTEMP have to be greater than EXTRUDE_MINTEMP
  #endif
#endif

// Extruder runout prevent
#if ENABLED(EXTRUDER_RUNOUT_PREVENT)
  #if DISABLED(EXTRUDER_RUNOUT_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_MINTEMP
  #endif
  #if DISABLED(EXTRUDER_RUNOUT_SECONDS)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SECONDS
  #endif
  #if DISABLED(EXTRUDER_RUNOUT_SPEED)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SPEED
  #endif
  #if DISABLED(EXTRUDER_RUNOUT_EXTRUDE)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_EXTRUDE
  #endif
  #if DISABLED(PREVENT_COLD_EXTRUSION)
    #error DEPENDENCY ERROR: EXTRUDER_RUNOUT_PREVENT needs PREVENT_COLD_EXTRUSION
  #endif
  #if EXTRUDER_RUNOUT_MINTEMP < EXTRUDE_MINTEMP
    #error CONFLICT ERROR: EXTRUDER_RUNOUT_MINTEMP have to be greater than EXTRUDE_MINTEMP
  #endif
#endif

// Extruder runout prevent is incompatible with idle oozing prevent
#if ENABLED(EXTRUDER_RUNOUT_PREVENT) && ENABLED(IDLE_OOZING_PREVENT)
  #error CONFLICT ERROR: EXTRUDER_RUNOUT_PREVENT and IDLE_OOZING_PREVENT are incopatible. Please comment one of them.
#endif

/**
 * Multi tool options
 */

// Allow only multy tools option to be defined
static_assert(1 >= 0
  #if ENABLED(NPR2)
    + 1
  #endif
  #if ENABLED(MKR4)
    + 1
  #endif
  #if ENABLED(MKR6)
    + 1
  #endif
  #if ENABLED(MKR12)
    + 1
  #endif
  #if ENABLED(MKSE6)
    + 1
  #endif
  , "Please enable only one Multy tools function: NPR2, MKR4, MKR6, MKR12 or MKSE6."
);

#if ENABLED(MKR4)
  #if   (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(E0E1_CHOICE)
    #error DEPENDENCY ERROR: You must set E0E1_CHOICE_PIN to a valid pin if you enable MKR4 with 2 extruder and 1 driver
  #elif (EXTRUDERS > 2) && (DRIVER_EXTRUDERS == 1)
    #error DEPENDENCY ERROR: For 3 or more extruder you must set 2 DRIVER_EXTRUDERS for MKR4 system
  #elif (EXTRUDERS > 2) && PIN_EXISTS(E0E1_CHOICE)
    #error DEPENDENCY ERROR: For 3 or more extruder you must not E0E1_CHOICE_PIN for MKR4 system
  #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 2) && !PIN_EXISTS(E0E2_CHOICE)
    #error DEPENDENCY ERROR: You must set E0E2_CHOICE_PIN to a valid pin if you enable MKR4 with 3 extruder and 1 driver
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(E0E2_CHOICE) || !PIN_EXISTS(E1E3_CHOICE))
    #error DEPENDENCY ERROR: You must set E0E2_CHOICE_PIN and E1E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 2 driver
  #elif (EXTRUDERS > 4)
    #error DEPENDENCY ERROR: MKR4 support only max 4 extruder
  #elif DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKR4 MULTI EXTRUDER
  #endif
#elif ENABLED(MKR6)
  #if   (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(EX1_CHOICE)
    #error DEPENDENCY ERROR: You must to set EX1_CHOICE_PIN to a valid pin if you enable MKR6 with 2 extruder and 1 driver
  #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 3 extruder and 1 driver
  #elif (EXTRUDERS >= 4) && (DRIVER_EXTRUDERS == 1)
    #error DEPENDENCY ERROR: For 4 or more extruder you must set 2 DRIVER_EXTRUDERS for MKR6 system
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 4 extruder and 2 driver
  #elif (EXTRUDERS == 5) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 5 extruder and 2 driver
  #elif (EXTRUDERS == 6) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR6 with 6 extruder and 2 driver
  #elif (EXTRUDERS > 6)
    #error DEPENDENCY ERROR: MKR6 support only max 6 extruder
  #elif DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKR6 MULTI EXTRUDER
  #endif
#elif ENABLED(MKR12)
  #if   (EXTRUDERS >= 4) && (DRIVER_EXTRUDERS == 1)
    #error DEPENDENCY ERROR: For 4 or more extruder you must set more DRIVER_EXTRUDERS for MKR12 system
  #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 4 extruder and 2 driver
  #elif (EXTRUDERS == 5) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 5 extruder and 2 driver
  #elif (EXTRUDERS == 6) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 6 extruder and 2 driver
  #elif (EXTRUDERS >= 7) && (DRIVER_EXTRUDERS == 2)
    #error DEPENDENCY ERROR: For 7 or more extruder you must set more DRIVER_EXTRUDERS for MKR12 system
  #elif (EXTRUDERS == 7) && (DRIVER_EXTRUDERS == 3) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 7 extruder and 3 driver
  #elif (EXTRUDERS == 8) && (DRIVER_EXTRUDERS == 3) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 8 extruder and 3 driver
  #elif (EXTRUDERS == 9) && (DRIVER_EXTRUDERS == 3) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 9 extruder and 3 driver
  #elif (EXTRUDERS >= 10) && (DRIVER_EXTRUDERS == 3)
    #error DEPENDENCY ERROR: For 10 or more extruder you must set more DRIVER_EXTRUDERS for MKR12 system
  #elif (EXTRUDERS == 10) && (DRIVER_EXTRUDERS == 4) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 10 extruder and 4 driver
  #elif (EXTRUDERS == 11) && (DRIVER_EXTRUDERS == 4) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 11 extruder and 4 driver
  #elif (EXTRUDERS == 12) && (DRIVER_EXTRUDERS == 4) && (!PIN_EXISTS(EX1_CHOICE) || !PIN_EXISTS(EX2_CHOICE))
    #error DEPENDENCY ERROR: You have to set EX1_CHOICE_PIN and EX2_CHOICE_PIN to a valid pin if you enable MKR12 with 12 extruder and 4 driver
  #elif (EXTRUDERS > 12)
    #error DEPENDENCY ERROR: MKR12 support only max 12 extruder
  #elif DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKR12 MULTI EXTRUDER
  #endif
#elif ENABLED(MKSE6)
  #if EXTRUDERS < 2
    #error DEPENDENCY ERROR: You must set EXTRUDERS > 1 for MKSE6 MULTI EXTRUDER
  #endif
  #if DRIVER_EXTRUDERS > 1
    #error DEPENDENCY ERROR: You must set DRIVER_EXTRUDERS = 1 for MKSE6 MULTI EXTRUDER
  #endif
  #if !HAS_SERVOS
    #error DEPENDENCY ERROR: You must enabled ENABLE_SERVOS and set NUM_SERVOS > 0 for MKSE6 MULTI EXTRUDER
  #endif
  #if DISABLED(SINGLENOZZLE)
    #error DEPENDENCY ERROR: You must enabled SINGLENOZZLE for MKSE6 MULTI EXTRUDER
  #endif
#endif

#if ENABLED(NPR2) && !PIN_EXISTS(E_MIN)
  #error DEPENDENCY ERROR: You have to set E_MIN_PIN to a valid pin if you enable NPR2
#endif

#if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && !HAS_SERVOS
  #error DEPENDENCY ERROR: You must enabled ENABLE_SERVOS and set NUM_SERVOS > 0 for DONDOLO MULTI EXTRUDER
#endif

#if ENABLED(DONDOLO_SINGLE_MOTOR) && ENABLED(DONDOLO_DUAL_MOTOR)
  #error DEPENDENCY ERROR: You must enabled only one for DONDOLO_SINGLE_MOTOR and DONDOLO_DUAL_MOTOR
#endif

#if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && EXTRUDERS != 2
  #error DEPENDENCY ERROR: You must set EXTRUDERS = 2 for DONDOLO
#endif

#endif /* _TOOLS_SANITYCHECK_H_ */
