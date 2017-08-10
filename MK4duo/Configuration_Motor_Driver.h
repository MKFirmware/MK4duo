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

#ifndef _CONFIGURATION_MOTOR_DRIVER_H_
#define _CONFIGURATION_MOTOR_DRIVER_H_

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

  #define X_CURRENT         1000 // in mA
  #define X_SENSE_RESISTOR    91 // in mOhms
  #define X_MICROSTEPS        16 // number of microsteps

  #define X2_CURRENT        1000
  #define X2_SENSE_RESISTOR   91
  #define X2_MICROSTEPS       16

  #define Y_CURRENT         1000
  #define Y_SENSE_RESISTOR    91
  #define Y_MICROSTEPS        16

  #define Y2_CURRENT        1000
  #define Y2_SENSE_RESISTOR   91
  #define Y2_MICROSTEPS       16

  #define Z_CURRENT         1000
  #define Z_SENSE_RESISTOR    91
  #define Z_MICROSTEPS        16

  #define Z2_MAX_CURRENT    1000
  #define Z2_SENSE_RESISTOR   91
  #define Z2_MICROSTEPS       16

  #define E0_CURRENT        1000
  #define E0_SENSE_RESISTOR   91
  #define E0_MICROSTEPS       16

  #define E1_CURRENT        1000
  #define E1_SENSE_RESISTOR   91
  #define E1_MICROSTEPS       16

  #define E2_CURRENT        1000
  #define E2_SENSE_RESISTOR   91
  #define E2_MICROSTEPS       16

  #define E3_CURRENT        1000
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
  //#define E4_IS_TMC2130
  //#define E5_IS_TMC2130

  /**
   * Stepper driver settings
   */

  #define R_SENSE           0.11  // R_sense resistor for SilentStepStick2130
  #define HOLD_MULTIPLIER    0.5  // Scales down the holding current from run current
  #define INTERPOLATE          1  // Interpolate X/Y/Z_MICROSTEPS to 256

  #define X_CURRENT         1000  // rms current in mA
  #define X_MICROSTEPS        16  // FULLSTEP..256

  #define Y_CURRENT         1000
  #define Y_MICROSTEPS        16

  #define Z_CURRENT         1000
  #define Z_MICROSTEPS        16

  //#define X2_CURRENT      1000
  //#define X2_MICROSTEPS     16

  //#define Y2_CURRENT      1000
  //#define Y2_MICROSTEPS     16

  //#define Z2_CURRENT      1000
  //#define Z2_MICROSTEPS     16

  //#define E0_CURRENT      1000
  //#define E0_MICROSTEPS     16

  //#define E1_CURRENT      1000
  //#define E1_MICROSTEPS     16

  //#define E2_CURRENT      1000
  //#define E2_MICROSTEPS     16

  //#define E3_CURRENT      1000
  //#define E3_MICROSTEPS     16

  //#define E4_CURRENT      1000
  //#define E4_MICROSTEPS     16

  //#define E5_CURRENT      1000
  //#define E5_MICROSTEPS     16

  /**
   * Use Trinamic's ultra quiet stepping mode.
   * When disabled, MK4duo will use spreadCycle stepping mode.
   */
  #define STEALTHCHOP

  /**
   * Let MK4duo automatically control stepper current.
   * This is still an experimental feature.
   * Increase current every 5s by CURRENT_STEP until stepper temperature prewarn gets triggered,
   * then decrease current by CURRENT_STEP until temperature prewarn is cleared.
   * Adjusting starts from X/Y/Z/E_CURRENT but will not increase over AUTO_ADJUST_MAX
   * Relevant g-codes:
   * M906 - Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given.
   * M906 S1 - Start adjusting current
   * M906 S0 - Stop adjusting current
   * M911 - Report stepper driver overtemperature pre-warn condition.
   * M912 - Clear stepper driver overtemperature pre-warn condition flag.
   */
  //#define AUTOMATIC_CURRENT_CONTROL

  #if ENABLED(AUTOMATIC_CURRENT_CONTROL)
    #define CURRENT_STEP          50  // [mA]
    #define AUTO_ADJUST_MAX     1300  // [mA], 1300mA_rms = 1840mA_peak
    #define REPORT_CURRENT_CHANGE
  #endif

  /**
   * The driver will switch to spreadCycle when stepper speed is over HYBRID_THRESHOLD.
   * This mode allows for faster movements at the expense of higher noise levels.
   * STEALTHCHOP needs to be enabled.
   * M913 X/Y/Z/E to live tune the setting
   */
  //#define HYBRID_THRESHOLD

  #define X_HYBRID_THRESHOLD     100  // [mm/s]
  #define X2_HYBRID_THRESHOLD    100
  #define Y_HYBRID_THRESHOLD     100
  #define Y2_HYBRID_THRESHOLD    100
  #define Z_HYBRID_THRESHOLD       4
  #define Z2_HYBRID_THRESHOLD      4
  #define E0_HYBRID_THRESHOLD     30
  #define E1_HYBRID_THRESHOLD     30
  #define E2_HYBRID_THRESHOLD     30
  #define E3_HYBRID_THRESHOLD     30
  #define E4_HYBRID_THRESHOLD     30
  #define E5_HYBRID_THRESHOLD     30

  /**
   * Use stallGuard2 to sense an obstacle and trigger an endstop.
   * You need to place a wire from the driver's DIAG1 pin to the X/Y endstop pin.
   * If used along with STEALTHCHOP, the movement will be louder when homing. This is normal.
   *
   * X/Y_HOMING_SENSITIVITY is used for tuning the trigger sensitivity.
   * Higher values make the system LESS sensitive.
   * Lower value make the system MORE sensitive.
   * Too low values can lead to false positives, while too high values will collide the axis without triggering.
   * It is advised to set X/Y_HOME_BUMP_MM to 0.
   * M914 X/Y to live tune the setting
   */
  //#define SENSORLESS_HOMING

  #if ENABLED(SENSORLESS_HOMING)
    #define X_HOMING_SENSITIVITY  19
    #define Y_HOMING_SENSITIVITY  19
  #endif

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

#endif /* _CONFIGURATION_MOTOR_DRIVER_H_ */
