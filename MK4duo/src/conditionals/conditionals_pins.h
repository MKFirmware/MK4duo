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

#define AS_QUOTED_STRING(S) #S
#define INCLUDE_BY_MB(M)    AS_QUOTED_STRING(../boards/M.h)
#include INCLUDE_BY_MB(MOTHERBOARD)

#if DISABLED(BOARD_NAME)
  #define BOARD_NAME "Unknow"
#endif
/****************************************************************************************/


/****************************************************************************************
 ********************************* END MOTHERBOARD **************************************
 ****************************************************************************************/

#if DISABLED(ORIG_X_CS_PIN)
  #define ORIG_X_CS_PIN       NoPin
#endif
#if DISABLED(ORIG_Y_CS_PIN)
  #define ORIG_Y_CS_PIN       NoPin
#endif
#if DISABLED(ORIG_Z_CS_PIN)
  #define ORIG_Z_CS_PIN       NoPin
#endif
#if DISABLED(ORIG_E0_CS_PIN)
  #define ORIG_E0_CS_PIN      NoPin
#endif
#if DISABLED(ORIG_E1_CS_PIN)
  #define ORIG_E1_CS_PIN      NoPin
#endif
#if DISABLED(ORIG_E2_CS_PIN)
  #define ORIG_E2_CS_PIN      NoPin
#endif
#if DISABLED(ORIG_E3_CS_PIN)
  #define ORIG_E3_CS_PIN      NoPin
#endif
#if DISABLED(ORIG_E4_CS_PIN)
  #define ORIG_E4_CS_PIN      NoPin
#endif
#if DISABLED(ORIG_E5_CS_PIN)
  #define ORIG_E5_CS_PIN      NoPin
#endif

#if DISABLED(ORIG_E0_DIR_PIN)
  #define ORIG_E0_DIR_PIN     NoPin
  #define ORIG_E0_ENABLE_PIN  NoPin
  #define ORIG_E0_STEP_PIN    NoPin
#endif
#if DISABLED(ORIG_E1_DIR_PIN)
  #define ORIG_E1_DIR_PIN     NoPin
  #define ORIG_E1_ENABLE_PIN  NoPin
  #define ORIG_E1_STEP_PIN    NoPin
#endif
#if DISABLED(ORIG_E2_DIR_PIN)
  #define ORIG_E2_DIR_PIN     NoPin
  #define ORIG_E2_ENABLE_PIN  NoPin
  #define ORIG_E2_STEP_PIN    NoPin
#endif
#if DISABLED(ORIG_E3_DIR_PIN)
  #define ORIG_E3_DIR_PIN     NoPin
  #define ORIG_E3_ENABLE_PIN  NoPin
  #define ORIG_E3_STEP_PIN    NoPin
#endif
#if DISABLED(ORIG_E4_DIR_PIN)
  #define ORIG_E4_DIR_PIN     NoPin
  #define ORIG_E4_ENABLE_PIN  NoPin
  #define ORIG_E4_STEP_PIN    NoPin
#endif
#if DISABLED(ORIG_E5_DIR_PIN)
  #define ORIG_E5_DIR_PIN     NoPin
  #define ORIG_E5_ENABLE_PIN  NoPin
  #define ORIG_E5_STEP_PIN    NoPin
#endif

#if DISABLED(ORIG_HEATER_HE1_PIN)
  #define ORIG_HEATER_HE1_PIN   NoPin
#endif
#if DISABLED(ORIG_TEMP_HE1_PIN)
  #define ORIG_TEMP_HE1_PIN     NoPin
#endif
#if DISABLED(ORIG_HEATER_HE2_PIN)
  #define ORIG_HEATER_HE2_PIN   NoPin
#endif
#if DISABLED(ORIG_TEMP_HE2_PIN)
  #define ORIG_TEMP_HE2_PIN     NoPin
#endif
#if DISABLED(ORIG_HEATER_HE3_PIN)
  #define ORIG_HEATER_HE3_PIN   NoPin
#endif
#if DISABLED(ORIG_TEMP_HE3_PIN)
  #define ORIG_TEMP_HE3_PIN     NoPin
#endif
#if DISABLED(ORIG_HEATER_HE4_PIN)
  #define ORIG_HEATER_HE4_PIN   NoPin
#endif
#if DISABLED(ORIG_TEMP_HE4_PIN)
  #define ORIG_TEMP_HE4_PIN     NoPin
#endif
#if DISABLED(ORIG_HEATER_HE5_PIN)
  #define ORIG_HEATER_HE5_PIN   NoPin
#endif
#if DISABLED(ORIG_TEMP_HE5_PIN)
  #define ORIG_TEMP_HE5_PIN     NoPin
#endif

#if DISABLED(ORIG_HEATER_BED0_PIN)
  #define ORIG_HEATER_BED0_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_BED0_PIN)
  #define ORIG_TEMP_BED0_PIN    NoPin
#endif
#if DISABLED(ORIG_HEATER_BED1_PIN)
  #define ORIG_HEATER_BED1_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_BED1_PIN)
  #define ORIG_TEMP_BED1_PIN    NoPin
#endif
#if DISABLED(ORIG_HEATER_BED2_PIN)
  #define ORIG_HEATER_BED2_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_BED2_PIN)
  #define ORIG_TEMP_BED2_PIN    NoPin
#endif
#if DISABLED(ORIG_HEATER_BED3_PIN)
  #define ORIG_HEATER_BED3_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_BED3_PIN)
  #define ORIG_TEMP_BED3_PIN    NoPin
#endif

#if DISABLED(ORIG_HEATER_CHAMBER0_PIN)
  #define ORIG_HEATER_CHAMBER0_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_CHAMBER0_PIN)
  #define ORIG_TEMP_CHAMBER0_PIN    NoPin
#endif
#if DISABLED(ORIG_HEATER_CHAMBER1_PIN)
  #define ORIG_HEATER_CHAMBER1_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_CHAMBER1_PIN)
  #define ORIG_TEMP_CHAMBER1_PIN    NoPin
#endif
#if DISABLED(ORIG_HEATER_CHAMBER2_PIN)
  #define ORIG_HEATER_CHAMBER2_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_CHAMBER2_PIN)
  #define ORIG_TEMP_CHAMBER2_PIN    NoPin
#endif
#if DISABLED(ORIG_HEATER_CHAMBER3_PIN)
  #define ORIG_HEATER_CHAMBER3_PIN  NoPin
#endif
#if DISABLED(ORIG_TEMP_CHAMBER3_PIN)
  #define ORIG_TEMP_CHAMBER3_PIN    NoPin
#endif

#if DISABLED(ORIG_HEATER_COOLER_PIN)
  #define ORIG_HEATER_COOLER_PIN    NoPin
#endif
#if DISABLED(ORIG_TEMP_COOLER_PIN)
  #define ORIG_TEMP_COOLER_PIN      NoPin
#endif

#if PIN_EXISTS(X_STOP)
  #undef ORIG_X_MIN_PIN
  #undef ORIG_X_MAX_PIN
  #if X_HOME_DIR < 0
    #define ORIG_X_MIN_PIN X_STOP_PIN
    #define ORIG_X_MAX_PIN NoPin
  #else
    #define ORIG_X_MIN_PIN NoPin
    #define ORIG_X_MAX_PIN X_STOP_PIN
  #endif
#endif

#if PIN_EXISTS(Y_STOP)
  #undef ORIG_Y_MIN_PIN
  #undef ORIG_Y_MAX_PIN
  #if Y_HOME_DIR < 0
    #define ORIG_Y_MIN_PIN Y_STOP_PIN
    #define ORIG_Y_MAX_PIN NoPin
  #else
    #define ORIG_Y_MIN_PIN NoPin
    #define ORIG_Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#if PIN_EXISTS(Z_STOP)
  #undef ORIG_Z_MIN_PIN
  #undef ORIG_Z_MAX_PIN
  #if Z_HOME_DIR < 0
    #define ORIG_Z_MIN_PIN Z_STOP_PIN
    #define ORIG_Z_MAX_PIN NoPin
  #else
    #define ORIG_Z_MIN_PIN NoPin
    #define ORIG_Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#if DISABLED(X_MS1_PIN)
  #define X_MS1_PIN     NoPin
#endif
#if DISABLED(X_MS2_PIN)
  #define X_MS2_PIN     NoPin
#endif
#if DISABLED(Y_MS1_PIN)
  #define Y_MS1_PIN     NoPin
#endif
#if DISABLED(Y_MS2_PIN)
  #define Y_MS2_PIN     NoPin
#endif
#if DISABLED(Z_MS1_PIN)
  #define Z_MS1_PIN     NoPin
#endif
#if DISABLED(Z_MS2_PIN)
  #define Z_MS2_PIN     NoPin
#endif
#if DISABLED(E0_MS1_PIN)
  #define E0_MS1_PIN    NoPin
#endif
#if DISABLED(E0_MS2_PIN)
  #define E0_MS2_PIN    NoPin
#endif
#if DISABLED(E1_MS1_PIN)
  #define E1_MS1_PIN    NoPin
#endif
#if DISABLED(E1_MS2_PIN)
  #define E1_MS2_PIN    NoPin
#endif
#if DISABLED(DIGIPOTSS_PIN)
  #define DIGIPOTSS_PIN NoPin
#endif
#if DISABLED(LCD_CONTRAST)
  #define LCD_CONTRAST  NoPin
#endif

#if DISABLED(ORIG_FAN0_PIN)
  #define ORIG_FAN0_PIN  NoPin
#endif
#if DISABLED(ORIG_FAN1_PIN)
  #define ORIG_FAN1_PIN NoPin
#endif
#if DISABLED(ORIG_FAN2_PIN)
  #define ORIG_FAN2_PIN NoPin
#endif
#if DISABLED(ORIG_FAN3_PIN)
  #define ORIG_FAN3_PIN NoPin
#endif
#if DISABLED(ORIG_FAN4_PIN)
  #define ORIG_FAN4_PIN NoPin
#endif
#if DISABLED(ORIG_FAN5_PIN)
  #define ORIG_FAN5_PIN NoPin
#endif

// Misc Pins
#if DISABLED(ORIG_PS_ON_PIN)
  #define ORIG_PS_ON_PIN NoPin
#endif
#if DISABLED(ORIG_BEEPER_PIN)
  #define ORIG_BEEPER_PIN NoPin
#endif
#if DISABLED(LED_PIN)
  #define LED_PIN NoPin
#endif
#if DISABLED(SD_DETECT_PIN)
  #define SD_DETECT_PIN NoPin
#endif
#if DISABLED(SDSS)
  #define SDSS NoPin
#endif
#if DISABLED(KILL_PIN)
  #define KILL_PIN NoPin
#endif
#if DISABLED(SUICIDE_PIN)
  #define SUICIDE_PIN NoPin
#endif

/****************************************************************************************/
#include "../../Configuration_Pins.h"
/****************************************************************************************/

#if DISABLED(X2_STEP_PIN)
  #define X2_STEP_PIN       NoPin
  #define X2_DIR_PIN        NoPin
  #define X2_ENABLE_PIN     NoPin
  #define X2_CS_PIN         NoPin
#endif
#if DISABLED(Y2_STEP_PIN)
  #define Y2_STEP_PIN       NoPin
  #define Y2_DIR_PIN        NoPin
  #define Y2_ENABLE_PIN     NoPin
  #define Y2_CS_PIN         NoPin
#endif
#if DISABLED(Z2_STEP_PIN)
  #define Z2_STEP_PIN       NoPin
  #define Z2_DIR_PIN        NoPin
  #define Z2_ENABLE_PIN     NoPin
  #define Z2_CS_PIN         NoPin
#endif
#if DISABLED(Z3_STEP_PIN)
  #define Z3_STEP_PIN       NoPin
  #define Z3_DIR_PIN        NoPin
  #define Z3_ENABLE_PIN     NoPin
  #define Z3_CS_PIN         NoPin
#endif

#if DISABLED(E0_DIR_PIN)
  #define E0_DIR_PIN        NoPin
  #define E0_ENABLE_PIN     NoPin
  #define E0_STEP_PIN       NoPin
#endif
#if DISABLED(E1_DIR_PIN)
  #define E1_DIR_PIN        NoPin
  #define E1_ENABLE_PIN     NoPin
  #define E1_STEP_PIN       NoPin
#endif
#if DISABLED(E2_DIR_PIN)
  #define E2_DIR_PIN        NoPin
  #define E2_ENABLE_PIN     NoPin
  #define E2_STEP_PIN       NoPin
#endif
#if DISABLED(E3_DIR_PIN)
  #define E3_DIR_PIN        NoPin
  #define E3_ENABLE_PIN     NoPin
  #define E3_STEP_PIN       NoPin
#endif
#if DISABLED(E4_DIR_PIN)
  #define E4_DIR_PIN        NoPin
  #define E4_ENABLE_PIN     NoPin
  #define E4_STEP_PIN       NoPin
#endif
#if DISABLED(E5_DIR_PIN)
  #define E5_DIR_PIN        NoPin
  #define E5_ENABLE_PIN     NoPin
  #define E5_STEP_PIN       NoPin
#endif

// Disabled MIN or MAX endstop if not used
#if DISABLED(ENABLED_ALL_SIX_ENDSTOP)

  #if DISABLED(DUAL_X_CARRIAGE)
    #if X_HOME_DIR > 0    // Home X to MAX
      #undef X_MIN_PIN
      #define X_MIN_PIN NoPin
    #elif X_HOME_DIR < 0  // Home X to MIN
      #undef X_MAX_PIN
      #define X_MAX_PIN NoPin
    #endif // X_HOME_DIR > 0
  #endif // DISABLED(DUAL_X_CARRIAGE)

  #if Y_HOME_DIR > 0    // Home Y to MAX
    #undef Y_MIN_PIN
    #define Y_MIN_PIN NoPin
  #elif Y_HOME_DIR < 0  // Home Y to MIN
    #undef Y_MAX_PIN
    #define Y_MAX_PIN NoPin
  #endif // Y_HOME_DIR > 0

  #if Z_HOME_DIR > 0    // Home Z to MAX
    #undef Z_MIN_PIN
    #define Z_MIN_PIN NoPin
  #elif Z_HOME_DIR < 0  // Home Z to MIN
    #undef Z_MAX_PIN
    #define Z_MAX_PIN NoPin
  #endif // Z_HOME_DIR > 0

#endif

/****************************************************************************************/
#define SENSITIVE_PINS { 0, 1, \
                        X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, \
                        Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, \
                        Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, \
                        E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN \
                       }
