/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#if ENABLED(HAVE_TMC2130)

  #define STEALTHCHOP

  /**
   * Let MK4duo automatically control stepper current.
   * This is still an experimental feature.
   * Increase current every 5s by CURRENT_STEP until stepper temperature prewarn gets triggered,
   * then decrease current by CURRENT_STEP until temperature prewarn is cleared.
   * Adjusting starts from X/Y/Z/E_MAX_CURRENT but will not increase over AUTO_ADJUST_MAX
   */
  //#define AUTOMATIC_CURRENT_CONTROL
  #define CURRENT_STEP          50  // [mA]
  #define AUTO_ADJUST_MAX     1300  // [mA], 1300mA_rms = 1840mA_peak

  // CHOOSE YOUR MOTORS HERE, THIS IS MANDATORY
  //#define X_IS_TMC2130
  //#define X2_IS_TMC2130
  //#define Y_IS_TMC2130
  //#define Y2_IS_TMC2130
  //#define Z_IS_TMC2130
  //#define Z2_IS_TMC2130
  //#define E0_IS_TMC2130
  //#define E1_IS_TMC2130
  //#define E2_IS_TMC2130
  //#define E3_IS_TMC2130

  /**
   * Stepper driver settings
   */

  #define R_SENSE           0.11  // R_sense resistor for SilentStepStick2130
  #define HOLD_MULTIPLIER    0.5  // Scales down the holding current from run current
  #define INTERPOLATE          1  // Interpolate X/Y/Z_MICROSTEPS to 256

  #define X_MAX_CURRENT     1000  // rms current in mA
  #define X_MICROSTEPS        16  // FULLSTEP..256
  #define X_CHIP_SELECT       40  // Pin

  #define Y_MAX_CURRENT     1000
  #define Y_MICROSTEPS        16
  #define Y_CHIP_SELECT       42

  #define Z_MAX_CURRENT     1000
  #define Z_MICROSTEPS        16
  #define Z_CHIP_SELECT       65

  //#define X2_MAX_CURRENT  1000
  //#define X2_MICROSTEPS     16
  //#define X2_CHIP_SELECT    -1

  //#define Y2_MAX_CURRENT  1000
  //#define Y2_MICROSTEPS     16
  //#define Y2_CHIP_SELECT    -1

  //#define Z2_MAX_CURRENT  1000
  //#define Z2_MICROSTEPS     16
  //#define Z2_CHIP_SELECT    -1

  //#define E0_MAX_CURRENT  1000
  //#define E0_MICROSTEPS     16
  //#define E0_CHIP_SELECT    -1

  //#define E1_MAX_CURRENT  1000
  //#define E1_MICROSTEPS     16
  //#define E1_CHIP_SELECT    -1

  //#define E2_MAX_CURRENT  1000
  //#define E2_MICROSTEPS     16
  //#define E2_CHIP_SELECT    -1

  //#define E3_MAX_CURRENT  1000
  //#define E3_MICROSTEPS     16
  //#define E3_CHIP_SELECT    -1

  /**
   * You can set your own advanced settings by filling in predefined functions.
   * A list of available functions can be found on the library github page
   * https://github.com/teemuatlut/TMC2130Stepper
   *
   * Example:
   * #define TMC2130_ADV() { \
   *   stepperX.diag0_temp_prewarn(1); \
   *   stepperX.interpolate(0); \
   * }
   */
  #define  TMC2130_ADV() {  }

#endif // ENABLED(HAVE_TMC2130)
