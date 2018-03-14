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

// X2 axis pins
#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(X_TWO_STEPPER_DRIVERS)
  #define X2_STEP_PIN       ORIG_E1_STEP_PIN
  #define X2_DIR_PIN        ORIG_E1_DIR_PIN
  #define X2_ENABLE_PIN     ORIG_E1_ENABLE_PIN
  #define X2_CS_PIN         ORIG_E1_CS_PIN
#endif

// Y axis pins
#define Y_STEP_PIN          ORIG_Y_STEP_PIN
#define Y_DIR_PIN           ORIG_Y_DIR_PIN
#define Y_ENABLE_PIN        ORIG_Y_ENABLE_PIN
#define Y_CS_PIN            ORIG_Y_CS_PIN

// Y2 axis pins
#if ENABLED(Y_TWO_STEPPER_DRIVERS)
  #define Y2_STEP_PIN       ORIG_E1_STEP_PIN
  #define Y2_DIR_PIN        ORIG_E1_DIR_PIN
  #define Y2_ENABLE_PIN     ORIG_E1_ENABLE_PIN
  #define Y2_CS_PIN         ORIG_E1_CS_PIN
#endif

// Z axis pins
#define Z_STEP_PIN          ORIG_Z_STEP_PIN
#define Z_DIR_PIN           ORIG_Z_DIR_PIN
#define Z_ENABLE_PIN        ORIG_Z_ENABLE_PIN
#define Z_CS_PIN            ORIG_Z_CS_PIN

// Z2 axis pins
#if ENABLED(Z_TWO_STEPPER_DRIVERS)
  #define Z2_STEP_PIN       ORIG_E1_STEP_PIN
  #define Z2_DIR_PIN        ORIG_E1_DIR_PIN
  #define Z2_ENABLE_PIN     ORIG_E1_ENABLE_PIN
  #define Z2_CS_PIN         ORIG_E1_CS_PIN
#endif

// E axis pins
#if DRIVER_EXTRUDERS > 0
  #define E0_STEP_PIN       ORIG_E0_STEP_PIN
  #define E0_DIR_PIN        ORIG_E0_DIR_PIN
  #define E0_ENABLE_PIN     ORIG_E0_ENABLE_PIN
  #define E0_CS_PIN         ORIG_E0_CS_PIN
  #define SOL0_PIN          ORIG_SOL0_PIN
  #define E0_ENC_PIN        NoPin
#endif

#if DRIVER_EXTRUDERS > 1
  #define E1_STEP_PIN       ORIG_E1_STEP_PIN
  #define E1_DIR_PIN        ORIG_E1_DIR_PIN
  #define E1_ENABLE_PIN     ORIG_E1_ENABLE_PIN
  #define E1_CS_PIN         ORIG_E1_CS_PIN
  #define SOL1_PIN          ORIG_SOL1_PIN
  #define E1_ENC_PIN        NoPin
#endif

#if DRIVER_EXTRUDERS > 2
  #define E2_STEP_PIN       ORIG_E2_STEP_PIN
  #define E2_DIR_PIN        ORIG_E2_DIR_PIN
  #define E2_ENABLE_PIN     ORIG_E2_ENABLE_PIN
  #define E2_CS_PIN         ORIG_E2_CS_PIN
  #define SOL2_PIN          ORIG_SOL2_PIN
  #define E2_ENC_PIN        NoPin
#endif

#if DRIVER_EXTRUDERS > 3
  #define E3_STEP_PIN       ORIG_E3_STEP_PIN
  #define E3_DIR_PIN        ORIG_E3_DIR_PIN
  #define E3_ENABLE_PIN     ORIG_E3_ENABLE_PIN
  #define E3_CS_PIN         ORIG_E3_CS_PIN
  #define SOL3_PIN          ORIG_SOL3_PIN
  #define E3_ENC_PIN        NoPin
#endif

#if DRIVER_EXTRUDERS > 4
  #define E4_STEP_PIN       ORIG_E4_STEP_PIN
  #define E4_DIR_PIN        ORIG_E4_DIR_PIN
  #define E4_ENABLE_PIN     ORIG_E4_ENABLE_PIN
  #define E4_CS_PIN         ORIG_E4_CS_PIN
  #define SOL4_PIN          ORIG_SOL4_PIN
  #define E4_ENC_PIN        NoPin
#endif

#if DRIVER_EXTRUDERS > 5
  #define E5_STEP_PIN       ORIG_E5_STEP_PIN
  #define E5_DIR_PIN        ORIG_E5_DIR_PIN
  #define E5_ENABLE_PIN     ORIG_E5_ENABLE_PIN
  #define E5_CS_PIN         ORIG_E5_CS_PIN
  #define SOL5_PIN          ORIG_SOL5_PIN
  #define E5_ENC_PIN        NoPin
#endif

// ENDSTOP pin
#define X_MIN_PIN           ORIG_X_MIN_PIN
#define X_MAX_PIN           ORIG_X_MAX_PIN
#define Y_MIN_PIN           ORIG_Y_MIN_PIN
#define Y_MAX_PIN           ORIG_Y_MAX_PIN
#define Z_MIN_PIN           ORIG_Z_MIN_PIN
#define Z_MAX_PIN           ORIG_Z_MAX_PIN
#define X2_MIN_PIN          NoPin
#define Y2_MIN_PIN          NoPin
#define Z2_MIN_PIN          NoPin
#define X2_MAX_PIN          NoPin
#define Y2_MAX_PIN          NoPin
#define Z2_MAX_PIN          NoPin
#define Z_PROBE_PIN         NoPin

// HEATER pin
#define HEATER_0_PIN        ORIG_HEATER_0_PIN
#define HEATER_1_PIN        ORIG_HEATER_1_PIN
#define HEATER_2_PIN        ORIG_HEATER_2_PIN
#define HEATER_3_PIN        ORIG_HEATER_3_PIN
#define HEATER_BED_PIN      ORIG_HEATER_BED_PIN
#define HEATER_CHAMBER_PIN  NoPin
#define HEATER_COOLER_PIN   ORIG_COOLER_PIN

// TEMP pin
#define TEMP_0_PIN          ORIG_TEMP_0_PIN
#define TEMP_1_PIN          ORIG_TEMP_1_PIN
#define TEMP_2_PIN          ORIG_TEMP_2_PIN
#define TEMP_3_PIN          ORIG_TEMP_3_PIN
#define TEMP_BED_PIN        ORIG_TEMP_BED_PIN
#define TEMP_CHAMBER_PIN    NoPin
#define TEMP_COOLER_PIN     NoPin

// FAN pin
#define FAN0_PIN            ORIG_FAN0_PIN
#define FAN1_PIN            ORIG_FAN1_PIN
#define FAN2_PIN            ORIG_FAN2_PIN
#define FAN3_PIN            ORIG_FAN3_PIN
#define FAN4_PIN            ORIG_FAN4_PIN
#define FAN5_PIN            ORIG_FAN5_PIN

// PS ON pin
#define PS_ON_PIN           ORIG_PS_ON_PIN

// BEEPER pin
#define BEEPER_PIN          ORIG_BEEPER_PIN

//============================================================================

//================================= FEATURE ==================================

#if ENABLED(MKR4)
  #define E0E1_CHOICE_PIN NoPin
  #define E0E2_CHOICE_PIN NoPin
  #define E1E3_CHOICE_PIN NoPin
#elif ENABLED(MKR6) || ENABLED(MKR12)
  #define EX1_CHOICE_PIN  NoPin
  #define EX2_CHOICE_PIN  NoPin
#endif

#if ENABLED(LASER)
  #define LASER_PWR_PIN                   ORIG_LASER_PWR_PIN
  #define LASER_PWM_PIN                   ORIG_LASER_PWM_PIN
  #if ENABLED(LASER_PERIPHERALS)
    #define LASER_PERIPHERALS_PIN         NoPin
    #define LASER_PERIPHERALS_STATUS_PIN  NoPin
  #endif
#endif

#if ENABLED(CNCROUTER)
  #define CNCROUTER_PIN NoPin
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT0_PIN     NoPin
  #define FIL_RUNOUT1_PIN     NoPin
  #define FIL_RUNOUT2_PIN     NoPin
  #define FIL_RUNOUT3_PIN     NoPin
  #define FIL_RUNOUT4_PIN     NoPin
  #define FIL_RUNOUT5_PIN     NoPin
  #define FIL_RUNOUT_DAV_PIN  NoPin
#endif

#if ENABLED(FILAMENT_SENSOR)
  #define FILWIDTH_PIN NoPin
#endif

#if ENABLED(FLOWMETER_SENSOR)
  #define FLOWMETER_PIN  ORIG_FLOWMETER_PIN
#endif

#if ENABLED(POWER_CONSUMPTION)
  #define POWER_CONSUMPTION_PIN NoPin
#endif

#if ENABLED(PHOTOGRAPH)
  #define PHOTOGRAPH_PIN NoPin
#endif

#if ENABLED(CHDK)
  #define CHDK_PIN NoPin
#endif

#if ENABLED(CASE_LIGHT)
  #define CASE_LIGHT_PIN NoPin
#endif

#if ENABLED(DOOR_OPEN)
  #define DOOR_OPEN_PIN NoPin
#endif

#if ENABLED(POWER_CHECK)
  #define POWER_CHECK_PIN NoPin
#endif

#if ENABLED(Z_PROBE_SLED)
  #define SLED_PIN NoPin
#endif

#if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
  #define RGB_LED_R_PIN NoPin
  #define RGB_LED_G_PIN NoPin
  #define RGB_LED_B_PIN NoPin
  #define RGB_LED_W_PIN NoPin
#endif

#if ENABLED(NEOPIXEL_LED)
  #define NEOPIXEL_PIN NoPin
#endif

#if ENABLED(DHT_SENSOR)
  #define DHT_DATA_PIN NoPin
#endif

#if ENABLED(HAVE_TMC2130) && ENABLED(SOFT_SPI_TMC2130)
  #define SOFT_MOSI_PIN 51
  #define SOFT_MISO_PIN 50
  #define SOFT_SCK_PIN  52
#endif

//============================================================================

#endif /* _CONFIGURATION_PINS_H_ */
