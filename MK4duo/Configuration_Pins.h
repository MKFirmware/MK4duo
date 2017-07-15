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
 * Configuration_Pins.h
 *
 * This configuration file contains all Pins.
 *
 */

#ifndef _CONFIGURATION_PINS_H_
#define _CONFIGURATION_PINS_H_

//=================================== BASIC ==================================

// X axis pins
#define X_STEP_PIN          ORIG_X_STEP_PIN
#define X_DIR_PIN           ORIG_X_DIR_PIN
#define X_ENABLE_PIN        ORIG_X_ENABLE_PIN
#define X_CS_PIN            ORIG_X_CS_PIN

// Y axis pins
#define Y_STEP_PIN          ORIG_Y_STEP_PIN
#define Y_DIR_PIN           ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN        ORIG_Y_ENABLE_PIN
#define Y_CS_PIN            ORIG_Y_CS_PIN

// Z axis pins
#define Z_STEP_PIN          ORIG_Z_STEP_PIN
#define Z_DIR_PIN           ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN        ORIG_Z_ENABLE_PIN
#define Z_CS_PIN            ORIG_Z_CS_PIN


// Assign next available extruder slot
#define __EPIN(p,q) ORIG_E##p##_##q##_PIN
#define _EPIN(p,q) __EPIN(p,q)


// Macros for adding
#define INC_0 1
#define INC_1 2
#define INC_2 3
#define INC_3 4
#define INC_4 5
#define INC_5 6
#define INC_6 7
#define INC_7 8
#define INC_8 9

#define INCREMENT_(n) INC_ ##n
#define INCREMENT(n) INCREMENT_(n)


// The X2 axis, if any, should be the next open extruder port
#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER)
  #ifndef X2_STEP_PIN
    #define X2_STEP_PIN   _EPIN(DRIVER_EXTRUDERS, STEP)
    #define X2_DIR_PIN    _EPIN(DRIVER_EXTRUDERS, DIR)
    #define X2_ENABLE_PIN _EPIN(DRIVER_EXTRUDERS, ENABLE)
    #define X2_CS_PIN     _EPIN(DRIVER_EXTRUDERS, CS)
    #if X2_ENABLE_PIN == 0
      #error "No E stepper plug left for X2!"
    #endif
  #endif
  #define Y2_E_INDEX INCREMENT(DRIVER_EXTRUDERS)
#else
  #define Y2_E_INDEX DRIVER_EXTRUDERS
#endif



// Y2 axis pins
#if ENABLED(Y_TWO_STEPPER)
  #ifndef Y2_STEP_PIN
    #define Y2_STEP_PIN   _EPIN(Y2_E_INDEX, STEP)
    #define Y2_DIR_PIN    _EPIN(Y2_E_INDEX, DIR)
    #define Y2_ENABLE_PIN _EPIN(Y2_E_INDEX, ENABLE)
    #define Y2_CS_PIN     _EPIN(Y2_E_INDEX, CS)
    #if Y2_ENABLE_PIN == 0
      #error "No E stepper plug left for Y2!"
    #endif
  #endif
  #define Z2_E_INDEX INCREMENT(Y2_E_INDEX)
#else
  #define Z2_E_INDEX Y2_E_INDEX
#endif
  
// Z2 axis pins
#if ENABLED(Z_TWO_STEPPER)
  #ifndef Z2_STEP_PIN
    #define Z2_STEP_PIN   _EPIN(Z2_E_INDEX, STEP)
    #define Z2_DIR_PIN    _EPIN(Z2_E_INDEX, DIR)
    #define Z2_ENABLE_PIN _EPIN(Z2_E_INDEX, ENABLE)
    #define Z2_CS_PIN     _EPIN(Z2_E_INDEX, CS)
    #if Z2_ENABLE_PIN == 0
      #error "No E stepper plug left for Z2!"
    #endif
  #endif
  #define Z3_E_INDEX INCREMENT(Z2_E_INDEX)
#else
  #define Z3_E_INDEX Z2_E_INDEX
#endif

// Z3 axis pins
#if ENABLED(Z_THREE_STEPPER)
  #ifndef Z3_STEP_PIN
    #define Z3_STEP_PIN   _EPIN(Z3_E_INDEX, STEP)
    #define Z3_DIR_PIN    _EPIN(Z3_E_INDEX, DIR)
    #define Z3_ENABLE_PIN _EPIN(Z3_E_INDEX, ENABLE)
    #define Z3_CS_PIN     _EPIN(Z3_E_INDEX, CS)
    #if Z3_ENABLE_PIN == 0
      #error "No E stepper plug left for Z3!"
    #endif
  #endif
  #define Z4_E_INDEX INCREMENT(Z3_E_INDEX)
#else
  #define Z4_E_INDEX Z3_E_INDEX
#endif

// Z4 axis pins
#if ENABLED(Z_FOUR_STEPPER)
  #ifndef Z4_STEP_PIN
    #define Z4_STEP_PIN   _EPIN(Z4_E_INDEX, STEP)
    #define Z4_DIR_PIN    _EPIN(Z4_E_INDEX, DIR)
    #define Z4_ENABLE_PIN _EPIN(Z4_E_INDEX, ENABLE)
    #define Z4_CS_PIN     _EPIN(Z4_E_INDEX, CS)
    #if Z4_ENABLE_PIN == 0
      #error "No E stepper plug left for Z4!"
    #endif
  #endif
#endif

// E axis pins
#if DRIVER_EXTRUDERS > 0
  #define E0_STEP_PIN       ORIG_E0_STEP_PIN
  #define E0_DIR_PIN        ORIG_E0_DIR_PIN
  #define E0_ENABLE_PIN     ORIG_E0_ENABLE_PIN
  #define E0_CS_PIN         ORIG_E0_CS_PIN
  #define SOL0_PIN          ORIG_SOL0_PIN
  #define E0_ENC_PIN        -1
#endif

#if DRIVER_EXTRUDERS > 1
  #define E1_STEP_PIN       ORIG_E1_STEP_PIN
  #define E1_DIR_PIN        ORIG_E1_DIR_PIN
  #define E1_ENABLE_PIN     ORIG_E1_ENABLE_PIN
  #define E1_CS_PIN         ORIG_E1_CS_PIN
  #define SOL1_PIN          ORIG_SOL1_PIN
  #define E1_ENC_PIN        -1
#endif

#if DRIVER_EXTRUDERS > 2
  #define E2_STEP_PIN       ORIG_E2_STEP_PIN
  #define E2_DIR_PIN        ORIG_E2_DIR_PIN
  #define E2_ENABLE_PIN     ORIG_E2_ENABLE_PIN
  #define E2_CS_PIN         ORIG_E2_CS_PIN
  #define SOL2_PIN          ORIG_SOL2_PIN
  #define E2_ENC_PIN        -1
#endif

#if DRIVER_EXTRUDERS > 3
  #define E3_STEP_PIN       ORIG_E3_STEP_PIN
  #define E3_DIR_PIN        ORIG_E3_DIR_PIN
  #define E3_ENABLE_PIN     ORIG_E3_ENABLE_PIN
  #define E3_CS_PIN         ORIG_E3_CS_PIN
  #define SOL3_PIN          ORIG_SOL3_PIN
  #define E3_ENC_PIN        -1
#endif

#if DRIVER_EXTRUDERS > 4
  #define E4_STEP_PIN       ORIG_E4_STEP_PIN
  #define E4_DIR_PIN        ORIG_E4_DIR_PIN
  #define E4_ENABLE_PIN     ORIG_E4_ENABLE_PIN
  #define E4_CS_PIN         ORIG_E4_CS_PIN
  #define SOL4_PIN          ORIG_SOL4_PIN
  #define E4_ENC_PIN        -1
#endif

#if DRIVER_EXTRUDERS > 5
  #define E5_STEP_PIN       ORIG_E5_STEP_PIN
  #define E5_DIR_PIN        ORIG_E5_DIR_PIN
  #define E5_ENABLE_PIN     ORIG_E5_ENABLE_PIN
  #define E5_CS_PIN         ORIG_E5_CS_PIN
  #define SOL5_PIN          ORIG_SOL5_PIN
  #define E5_ENC_PIN        -1
#endif

// ENDSTOP pin
#define X_MIN_PIN           ORIG_X_MIN_PIN
#define X_MAX_PIN           ORIG_X_MAX_PIN
#define Y_MIN_PIN           ORIG_Y_MIN_PIN
#define Y_MAX_PIN           ORIG_Y_MAX_PIN
#define Z_MIN_PIN           ORIG_Z_MIN_PIN
#define Z_MAX_PIN           ORIG_Z_MAX_PIN
#define Z2_MIN_PIN          -1
#define Z2_MAX_PIN          -1
#define Z3_MIN_PIN          -1
#define Z3_MAX_PIN          -1
#define Z4_MIN_PIN          -1
#define Z4_MAX_PIN          -1
#define E_MIN_PIN           -1
#define Z_PROBE_PIN         -1

// HEATER pin
#define HEATER_0_PIN        ORIG_HEATER_0_PIN
#define HEATER_1_PIN        ORIG_HEATER_1_PIN
#define HEATER_2_PIN        ORIG_HEATER_2_PIN
#define HEATER_3_PIN        ORIG_HEATER_3_PIN
#define HEATER_BED_PIN      ORIG_HEATER_BED_PIN
#define HEATER_CHAMBER_PIN  -1
#define COOLER_PIN          -1

// TEMP pin
#define TEMP_0_PIN          ORIG_TEMP_0_PIN
#define TEMP_1_PIN          ORIG_TEMP_1_PIN
#define TEMP_2_PIN          ORIG_TEMP_2_PIN
#define TEMP_3_PIN          ORIG_TEMP_3_PIN
#define TEMP_BED_PIN        ORIG_TEMP_BED_PIN
#define TEMP_CHAMBER_PIN    -1
#define TEMP_COOLER_PIN     -1

// FAN pin
#define FAN_PIN             ORIG_FAN_PIN
#define FAN1_PIN            ORIG_FAN1_PIN
#define FAN2_PIN            ORIG_FAN2_PIN
#define FAN3_PIN            ORIG_FAN3_PIN

// PS ON pin
#define PS_ON_PIN           ORIG_PS_ON_PIN

// BEEPER pin
#define BEEPER_PIN          ORIG_BEEPER_PIN

//============================================================================

//================================= FEATURE ==================================

#if ENABLED(MKR4)
  #define E0E1_CHOICE_PIN -1
  #define E0E2_CHOICE_PIN -1
  #define E1E3_CHOICE_PIN -1
#elif ENABLED(MKR6) || ENABLED(MKR12)
  #define EX1_CHOICE_PIN  -1
  #define EX2_CHOICE_PIN  -1
#endif

#if ENABLED(LASER)
  #define LASER_PWR_PIN                   ORIG_LASER_PWR_PIN
  #define LASER_PWM_PIN                   ORIG_LASER_PWM_PIN
  #if ENABLED(LASER_PERIPHERALS)
    #define LASER_PERIPHERALS_PIN         -1
    #define LASER_PERIPHERALS_STATUS_PIN  -1
  #endif
#endif

#if ENABLED(CNCROUTER)
  #define CNCROUTER_PIN -1
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_PIN -1
  #define FIL_RUNOUT_DAV_PIN -1
#endif

#if ENABLED(FILAMENT_SENSOR)
  #define FILWIDTH_PIN -1
#endif

#if ENABLED(FLOWMETER_SENSOR)
  #define FLOWMETER_PIN -1
#endif

#if ENABLED(POWER_CONSUMPTION)
  #define POWER_CONSUMPTION_PIN -1
#endif

#if ENABLED(PHOTOGRAPH)
  #define PHOTOGRAPH_PIN -1
#endif

#if ENABLED(CHDK)
  #define CHDK_PIN -1
#endif

#if ENABLED(CASE_LIGHT)
  #define CASE_LIGHT_PIN -1
#endif

#if ENABLED(DOOR_OPEN)
  #define DOOR_PIN -1
#endif

#if ENABLED(POWER_CHECK)
  #define POWER_CHECK_PIN -1
#endif

#if ENABLED(CONTROLLERFAN)
  #define CONTROLLERFAN_PIN -1
#endif

#if ENABLED(HOTEND_AUTO_FAN)
  #define H0_AUTO_FAN_PIN -1
  #define H1_AUTO_FAN_PIN -1
  #define H2_AUTO_FAN_PIN -1
  #define H3_AUTO_FAN_PIN -1
#endif

#if ENABLED(Z_PROBE_SLED)
  #define SLED_PIN -1
#endif

#if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
  #define RGB_LED_R_PIN -1
  #define RGB_LED_G_PIN -1
  #define RGB_LED_B_PIN -1
  #define RGB_LED_W_PIN -1
#endif

//============================================================================

#endif /* _CONFIGURATION_PINS_H_ */
