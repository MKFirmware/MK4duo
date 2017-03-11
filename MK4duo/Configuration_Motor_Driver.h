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

/**********************************************************************************
 **************************** TMC26X motor drivers ********************************
 **********************************************************************************
 *                                                                                *
 * Support for TMC26X motor drivers                                               *
 *                                                                                *
 **********************************************************************************/
#if ENABLED(HAVE_TMCDRIVER)

  //#define X_IS_TMC
  //#define X2_IS_TMC
  //#define Y_IS_TMC
  //#define Y2_IS_TMC
  //#define Z_IS_TMC
  //#define Z2_IS_TMC
  //#define E0_IS_TMC
  //#define E1_IS_TMC
  //#define E2_IS_TMC
  //#define E3_IS_TMC

  #define X_MAX_CURRENT     1000 // in mA
  #define X_SENSE_RESISTOR    91 // in mOhms
  #define X_MICROSTEPS        16 // number of microsteps

  #define X2_MAX_CURRENT    1000
  #define X2_SENSE_RESISTOR   91
  #define X2_MICROSTEPS       16

  #define Y_MAX_CURRENT     1000
  #define Y_SENSE_RESISTOR    91
  #define Y_MICROSTEPS        16

  #define Y2_MAX_CURRENT    1000
  #define Y2_SENSE_RESISTOR   91
  #define Y2_MICROSTEPS       16

  #define Z_MAX_CURRENT     1000
  #define Z_SENSE_RESISTOR    91
  #define Z_MICROSTEPS        16

  #define Z2_MAX_CURRENT    1000
  #define Z2_SENSE_RESISTOR   91
  #define Z2_MICROSTEPS       16

  #define E0_MAX_CURRENT    1000
  #define E0_SENSE_RESISTOR   91
  #define E0_MICROSTEPS       16

  #define E1_MAX_CURRENT    1000
  #define E1_SENSE_RESISTOR   91
  #define E1_MICROSTEPS       16

  #define E2_MAX_CURRENT    1000
  #define E2_SENSE_RESISTOR   91
  #define E2_MICROSTEPS       16

  #define E3_MAX_CURRENT    1000
  #define E3_SENSE_RESISTOR   91
  #define E3_MICROSTEPS       16

#endif


/**********************************************************************************
 *********************** Trinamic TMC2130 motor drivers ***************************
 **********************************************************************************
 *                                                                                *
 * Enable this for SilentStepStick Trinamic TMC2130 SPI-configurable stepper      *
 * drivers.                                                                       *
 *                                                                                *
 * You'll also need the TMC2130Stepper Arduino library                            *
 * (https://github.com/teemuatlut/TMC2130Stepper).                                *
 *                                                                                *
 * To use TMC2130 stepper drivers in SPI mode connect your SPI2130 pins to        *
 * the hardware SPI interface on your board and define the required CS pins       *
 * in your `MYBOARD.h` file. (e.g., RAMPS 1.4 uses AUX3 pins `X_CS_PIN 53`,       *
 * Y_CS_PIN 49`, etc.).                                                           *
 *                                                                                *
 **********************************************************************************/
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


/**********************************************************************************
 ****************************** L6470 motor drivers *******************************
 **********************************************************************************
 *                                                                                *
 * Support for L6470 motor drivers                                                *
 * You need to import the L6470 library into the arduino IDE for this.            *
 *                                                                                *
 **********************************************************************************/
#if ENABLED(HAVE_L6470DRIVER)

  //#define X_IS_L6470
  //#define X2_IS_L6470
  //#define Y_IS_L6470
  //#define Y2_IS_L6470
  //#define Z_IS_L6470
  //#define Z2_IS_L6470
  //#define E0_IS_L6470
  //#define E1_IS_L6470
  //#define E2_IS_L6470
  //#define E3_IS_L6470

  #define X_MICROSTEPS      16 // number of microsteps
  #define X_K_VAL           50 // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define X_OVERCURRENT   2000 // maxc current in mA. If the current goes over this value, the driver will switch off
  #define X_STALLCURRENT  1500 // current in mA where the driver will detect a stall

  #define X2_MICROSTEPS     16
  #define X2_K_VAL          50
  #define X2_OVERCURRENT  2000
  #define X2_STALLCURRENT 1500

  #define Y_MICROSTEPS      16
  #define Y_K_VAL           50
  #define Y_OVERCURRENT   2000
  #define Y_STALLCURRENT  1500

  #define Y2_MICROSTEPS     16
  #define Y2_K_VAL          50
  #define Y2_OVERCURRENT  2000
  #define Y2_STALLCURRENT 1500

  #define Z_MICROSTEPS      16
  #define Z_K_VAL           50
  #define Z_OVERCURRENT   2000
  #define Z_STALLCURRENT  1500

  #define Z2_MICROSTEPS     16
  #define Z2_K_VAL          50
  #define Z2_OVERCURRENT  2000
  #define Z2_STALLCURRENT 1500

  #define E0_MICROSTEPS     16
  #define E0_K_VAL          50
  #define E0_OVERCURRENT  2000
  #define E0_STALLCURRENT 1500

  #define E1_MICROSTEPS     16
  #define E1_K_VAL          50
  #define E1_OVERCURRENT  2000
  #define E1_STALLCURRENT 1500

  #define E2_MICROSTEPS     16
  #define E2_K_VAL          50
  #define E2_OVERCURRENT  2000
  #define E2_STALLCURRENT 1500

  #define E3_MICROSTEPS     16
  #define E3_K_VAL          50
  #define E3_OVERCURRENT  2000
  #define E3_STALLCURRENT 1500

#endif
