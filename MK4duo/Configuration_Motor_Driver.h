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

#ifndef _CONFIGURATION_MOTOR_DRIVER_H_
#define _CONFIGURATION_MOTOR_DRIVER_H_


/**********************************************************************************
 ************************** Trinamic TMC26x motor drivers *************************
 **********************************************************************************
 *                                                                                *
 * The TMC26XStepper library is required for this stepper driver.                 *
 * https://github.com/trinamic/TMC26XStepper                                      *
 *                                                                                *
 **********************************************************************************/
#define X_SENSE_RESISTOR  91
#define X2_SENSE_RESISTOR 91
#define Y_SENSE_RESISTOR  91
#define Y2_SENSE_RESISTOR 91
#define Z_SENSE_RESISTOR  91
#define Z2_SENSE_RESISTOR 91
#define Z3_SENSE_RESISTOR 91
#define E0_SENSE_RESISTOR 91
#define E1_SENSE_RESISTOR 91
#define E2_SENSE_RESISTOR 91
#define E3_SENSE_RESISTOR 91
#define E4_SENSE_RESISTOR 91
#define E5_SENSE_RESISTOR 91


/**********************************************************************************
 ********************* Trinamic TMC2130 - TMC2208 motor drivers *******************
 **********************************************************************************
 *                                                                                *
 * Enable this for SilentStepStick Trinamic TMC2130 SPI-configurable stepper      *
 * drivers.                                                                       *
 *                                                                                *
 * You'll also need the TMC2130 and TMC2208 Stepper Arduino library               *
 * (https://github.com/teemuatlut/TMCStepper).                                    *
 *                                                                                *
 * To use TMC2130 stepper drivers in SPI mode connect your SPI2130 pins to        *
 * the hardware SPI interface on your board and define the required CS pins       *
 * in configuration_pins.h                                                        *
 *                                                                                *
 * Enable this for SilentStepStick Trinamic TMC2208 UART-configurable stepper     *
 * drivers.                                                                       *
 * Connect #_SERIAL_TX_PIN to the driver side PDN_UART pin.                       *
 * To use the reading capabilities, also connect #_SERIAL_RX_PIN                  *
 * to #_SERIAL_TX_PIN with a 1K resistor.                                         *
 * The drivers can also be used with hardware serial.                             *
 *                                                                                *
 **********************************************************************************/
#define R_SENSE           0.11  // R_sense resistor
#define INTERPOLATE       true  // Interpolate X/Y/Z_MICROSTEPS to 256

// Select this if use software SPI. Choose pins in Configuration_pins.h
//#define SOFT_SPI_TMC2130

// Use StallGuard2 to sense an obstacle and trigger an endstop.
// Connect the stepper driver's DIAG1 pin to the X/Y/Z endstop pin.
// X, Y, and Z homing will always be done in spreadCycle mode.
//
// X/Y/Z_STALL_SENSITIVITY is used for tuning the trigger sensitivity.
// Higher values make the system LESS sensitive.
// Lower value make the system MORE sensitive.
// Too low values can lead to false positives, while too high values will collide the axis without triggering.
// It is advised to set X/Y/Z_HOME_BUMP_MM to 0.
// M914 X/Y/Z to live tune the setting
// TMC2130 or TMC5130 only
//#define SENSORLESS_HOMING
#define X_STALL_SENSITIVITY 8
#define Y_STALL_SENSITIVITY 8
#define Z_STALL_SENSITIVITY 8

// M922 S0/1 will enable continous reporting.
//#define TMC_DEBUG

// M915 Z Axis Calibration
// - Adjust Z stepper current,
// - Drive the Z axis to its physical maximum, and
// - Home Z to account for the lost steps.
// Use M915 Snn to specify the current.
// Use M925 Znn to add extra Z height to Z_MAX_POS.
//#define TMC_Z_CALIBRATION
#define CALIBRATION_CURRENT 250
#define CALIBRATION_EXTRA_HEIGHT 10

// Software enable
// Use for drivers that do not use a dedicated enable pin, but rather handle the same
// function through a communication line such as SPI or UART.
//#define TMC_SOFTWARE_DRIVER_ENABLE

// Use Trinamic's ultra quiet stepping mode.
// When disabled, MK4duo will use spreadCycle stepping mode.
#define X_STEALTHCHOP   false
#define Y_STEALTHCHOP   false
#define Z_STEALTHCHOP   false
#define E0_STEALTHCHOP  false
#define E1_STEALTHCHOP  false
#define E2_STEALTHCHOP  false
#define E3_STEALTHCHOP  false
#define E4_STEALTHCHOP  false
#define E5_STEALTHCHOP  false

// Optimize spreadCycle chopper parameters by using predefined parameter sets
// or with the help of an example included in the library.
// Provided parameter sets are
#define CHOPPER_DEFAULT_12V  { 3, -1, 1 }
#define CHOPPER_DEFAULT_19V  { 4,  1, 1 }
#define CHOPPER_DEFAULT_24V  { 4,  2, 1 }
#define CHOPPER_DEFAULT_36V  { 5,  2, 4 }
#define CHOPPER_PRUSAMK3_24V { 4,  1, 4 } // Imported parameters from the official Prusa firmware for MK3 (24V)
#define CHOPPER_MK4DUO_436   { 5,  2, 3 } // Old defaults from MK4duo v4.3.6
//
// Define you own with
// { <off_time[1..15]>, <hysteresis_end[-3..12]>, hysteresis_start[1..8] }
#define CHOPPER_TIMING CHOPPER_DEFAULT_12V

// Monitor Trinamic TMC2130 and TMC2208 drivers for error conditions,
// like overtemperature and short to ground. TMC2208 requires hardware serial.
// In the case of overtemperature MK4duo can decrease the driver current until error condition clears.
// Other detected conditions can be used to stop the current print.
// Relevant g-codes:
// M906 - Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given.
// M911 - Report stepper driver overtemperature pre-warn condition.
// M912 - Clear stepper driver overtemperature pre-warn condition flag.
// M922 - Report driver parameters. (Requires TMC_DEBUG)
//#define MONITOR_DRIVER_STATUS
//#define CURRENT_STEP_DOWN     50  // [mA]
//#define REPORT_CURRENT_CHANGE
//#define STOP_ON_ERROR

// The driver will switch to spreadCycle when stepper speed is over HYBRID_THRESHOLD.
// This mode allows for faster movements at the expense of higher noise levels.
// STEALTHCHOP for axis needs to be enabled.
// M913 X/Y/Z/E to live tune the setting [mm/s]
//#define HYBRID_THRESHOLD
#define X_HYBRID_THRESHOLD     100
#define Y_HYBRID_THRESHOLD     100
#define Z_HYBRID_THRESHOLD       2
#define E0_HYBRID_THRESHOLD     30
#define E1_HYBRID_THRESHOLD     30
#define E2_HYBRID_THRESHOLD     30
#define E3_HYBRID_THRESHOLD     30
#define E4_HYBRID_THRESHOLD     30
#define E5_HYBRID_THRESHOLD     30

/**
 * You can set your own advanced settings by filling in predefined functions.
 * A list of available functions can be found on the library github page
 * https://github.com/teemuatlut/TMC2130Stepper
 *
 * Example:
 * #define TMC_ADV() { \
 *   stepperX->diag0_temp_prewarn(1); \
 *   stepperY->interpolate(0); \
 * }
 */
#define  TMC_ADV() {  }


// Hardware serial communication ports for TMC2208
// If undefined software serial is used according to the pins below
//#define X_HARDWARE_SERIAL  Serial1
//#define X2_HARDWARE_SERIAL Serial1
//#define Y_HARDWARE_SERIAL  Serial1
//#define Y2_HARDWARE_SERIAL Serial1
//#define Z_HARDWARE_SERIAL  Serial1
//#define Z2_HARDWARE_SERIAL Serial1
//#define Z3_HARDWARE_SERIAL Serial1
//#define E0_HARDWARE_SERIAL Serial1
//#define E1_HARDWARE_SERIAL Serial1
//#define E2_HARDWARE_SERIAL Serial1
//#define E3_HARDWARE_SERIAL Serial1
//#define E4_HARDWARE_SERIAL Serial1
//#define E5_HARDWARE_SERIAL Serial1

#define X_SERIAL_TX_PIN   NoPin
#define X_SERIAL_RX_PIN   NoPin
#define X2_SERIAL_TX_PIN  NoPin
#define X2_SERIAL_RX_PIN  NoPin

#define Y_SERIAL_TX_PIN   NoPin
#define Y_SERIAL_RX_PIN   NoPin
#define Y2_SERIAL_TX_PIN  NoPin
#define Y2_SERIAL_RX_PIN  NoPin

#define Z_SERIAL_TX_PIN   NoPin
#define Z_SERIAL_RX_PIN   NoPin
#define Z2_SERIAL_TX_PIN  NoPin
#define Z2_SERIAL_RX_PIN  NoPin
#define Z3_SERIAL_TX_PIN  NoPin
#define Z3_SERIAL_RX_PIN  NoPin

#define E0_SERIAL_TX_PIN  NoPin
#define E0_SERIAL_RX_PIN  NoPin
#define E1_SERIAL_TX_PIN  NoPin
#define E1_SERIAL_RX_PIN  NoPin
#define E2_SERIAL_TX_PIN  NoPin
#define E2_SERIAL_RX_PIN  NoPin
#define E3_SERIAL_TX_PIN  NoPin
#define E3_SERIAL_RX_PIN  NoPin
#define E4_SERIAL_TX_PIN  NoPin
#define E4_SERIAL_RX_PIN  NoPin
#define E5_SERIAL_TX_PIN  NoPin
#define E5_SERIAL_RX_PIN  NoPin
/**********************************************************************************/


/**********************************************************************************
 ****************************** L6470 motor drivers *******************************
 **********************************************************************************
 *                                                                                *
 * Support for L6470 motor drivers                                                *
 * You need to import the L6470 library into the arduino IDE for this.            *
 *                                                                                *
 **********************************************************************************/
#define X_K_VAL           50 // 0 - 255, Higher values, are higher power. Be careful not to go too high
#define X_OVERCURRENT   2000 // maxc current in mA. If the current goes over this value, the driver will switch off
#define X_STALLCURRENT  1500 // current in mA where the driver will detect a stall

#define X2_K_VAL          50
#define X2_OVERCURRENT  2000
#define X2_STALLCURRENT 1500

#define Y_K_VAL           50
#define Y_OVERCURRENT   2000
#define Y_STALLCURRENT  1500

#define Y2_K_VAL          50
#define Y2_OVERCURRENT  2000
#define Y2_STALLCURRENT 1500

#define Z_K_VAL           50
#define Z_OVERCURRENT   2000
#define Z_STALLCURRENT  1500

#define Z2_K_VAL          50
#define Z2_OVERCURRENT  2000
#define Z2_STALLCURRENT 1500

#define Z3_K_VAL          50
#define Z3_OVERCURRENT  2000
#define Z3_STALLCURRENT 1500

#define E0_K_VAL          50
#define E0_OVERCURRENT  2000
#define E0_STALLCURRENT 1500

#define E1_K_VAL          50
#define E1_OVERCURRENT  2000
#define E1_STALLCURRENT 1500

#define E2_K_VAL          50
#define E2_OVERCURRENT  2000
#define E2_STALLCURRENT 1500

#define E3_K_VAL          50
#define E3_OVERCURRENT  2000
#define E3_STALLCURRENT 1500

#define E4_K_VAL          50
#define E4_OVERCURRENT  2000
#define E4_STALLCURRENT 1500

#define E5_K_VAL          50
#define E5_OVERCURRENT  2000
#define E5_STALLCURRENT 1500
/**********************************************************************************/

#endif /* _CONFIGURATION_MOTOR_DRIVER_H_ */
