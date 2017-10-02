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
 * Configuration_Basic.h
 *
 * This configuration file contains basic settings.
 *
 * - Serial comunication type
 * - Board type
 * - Mechanism type
 * - Extruders number
 *
 * Mechanisms-settings can be found in Configuration_Xxxxxx.h (where Xxxxxx can be: Cartesian - Delta - Core - Scara)
 * Temperature settings can be found in Configuration_Temperature.h
 * Feature-settings can be found in Configuration_Feature.h
 * Pins-settings can be found in "Configuration_Pins.h"
 */

#ifndef _CONFIGURATION_BASIC_H_
#define _CONFIGURATION_BASIC_H_

/***********************************************************************
 ********************** Serial comunication type ***********************
 ***********************************************************************/
/**
 * Select which serial port on the board will be used for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default port pins.
 * Serial port 0 is always used by the Arduino bootloader regardless of this setting.
 *
 * Valid values are 0-3 for Serial, Serial1, Serial2, Serial3 and -1 for SerialUSB
 */
#define SERIAL_PORT 0

/**
 * This setting determines the communication speed of the printer.
 *
 * 250000 works in most cases, but you might try a lower speed if
 * you commonly experience drop-outs during host printing.
 *
 * 2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000
 */
#define BAUDRATE 115200

/**
 * Enable the Bluetooth serial interface
 */
//#define BLUETOOTH
#define BLUETOOTH_PORT 1
#define BLUETOOTH_BAUD 115200

/**
 * User-specified version info of this build to display in [Pronterface, etc] terminal window during
 * startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
 * build by the user have been successfully uploaded into firmware.
 */
#define STRING_CONFIG_H_AUTHOR "(none, default config)"   // Who made the changes.

/**
 * Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
 * You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
 */
#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

/**
 * Kill Method
 *  0 - Disable heaters, wait forever
 *  1 - Reset controller. Will not reset separate communication chips!
 */
#define KILL_METHOD 0

/**
 * Some particular clients re-start sending commands only after receiving a 'wait' when there is a bad serial-connection.
 * Milliseconds
 */
#define NO_TIMEOUTS 1000
// Uncomment to include more info in ok command
//#define ADVANCED_OK

/**
 * Enable an emergency-command parser to intercept certain commands as they
 * enter the serial receive buffer, so they cannot be blocked.
 * Currently handles M108, M112, M410
 * Does not work on boards using Arduino DUE processors!
 */
//#define EMERGENCY_PARSER

/**
 * Spend 28 bytes of SRAM to optimize the GCode parser
 */
//#define FASTER_GCODE_PARSER

/**
 * Host Keepalive
 *
 * When enabled MK4duo will send a busy status message to the host
 * every couple of seconds when it can't accept commands.
 * Disable this if your host doesn't like keepalive messages
 */
//#define HOST_KEEPALIVE_FEATURE
// Number of seconds between "busy" messages. Set with M113.
#define DEFAULT_KEEPALIVE_INTERVAL 2
// Some hosts require "busy" messages even during heating
#define BUSY_WHILE_HEATING
/***********************************************************************/


/*****************************************************************************************
 *************************************** Board type **************************************
 *****************************************************************************************
 *                                                                                       *
 * Either an numeric ID or name defined in boards.h is valid.                            *
 * See: https://github.com/MagoKimbra/MK4duo/blob/master/Documentation/Hardware.md *
 *                                                                                       *
 *****************************************************************************************/
#define MOTHERBOARD BOARD_RAMPS_13_HFB
/*****************************************************************************************/


/***********************************************************************
 *************************** Mechanism type ****************************
 ***********************************************************************
 *                                                                     *
 * CARTESIAN      - Prusa, Mendel, etc                                 *
 * COREXY         - H-Bot/Core XY (x_motor = x+y, y_motor = x-y)       *
 * COREYX         - H-Bot/Core YX (x_motor = y+x, y_motor = y-x)       *
 * COREXZ         - H-Bot/Core XZ (x_motor = x+z, z_motor = x-z)       *
 * COREZX         - H-Bot/Core ZX (x_motor = z+x, z_motor = z-x)       *
 * COREYZ         - H-Bot/Core YZ (y_motor = y+z, z_motor = y-z)       *
 * COREZY         - H-Bot/Core ZY (y_motor = z+y, z_motor = z-y)       *
 * DELTA          - Rostock, Kossel, RostockMax, Cerberus, etc         *
 * MORGAN_SCARA   - SCARA classic                                      *
 * MAKERARM_SCARA - SCARA Makerfarm                                    *
 *                                                                     *
 ***********************************************************************/
#define MECHANISM MECH_CARTESIAN
//#define MECHANISM MECH_COREXY
//#define MECHANISM MECH_COREYX
//#define MECHANISM MECH_COREXZ
//#define MECHANISM MECH_COREZX
//#define MECHANISM MECH_COREYZ
//#define MECHANISM MECH_COREZY
//#define MECHANISM MECH_DELTA
//#define MECHANISM MECH_MORGAN_SCARA
//#define MECHANISM MECH_MAKERARM_SCARA
//#define MECHANISM MECH_MUVE3D
/***********************************************************************/


/*************************************************************************************
 ************************************ Power supply ***********************************
 *************************************************************************************
 *                                                                                   *
 * The following define selects which power supply you have.                         *
 * Please choose the one that matches your setup and set to POWER_SUPPLY:            *
 * 0 Normal power                                                                    *
 * 1 ATX                                                                             *
 * 2 X-Box 360 203 Watts (the blue wire connected to PS_ON and the red wire to VCC)  *
 *                                                                                   *
 *************************************************************************************/
#define POWER_SUPPLY 0

// Define this to have the electronics keep the power supply off on startup.
// If you don't know what this is leave it.
//#define PS_DEFAULT_OFF
// Define delay after power on in seconds
#define DELAY_AFTER_POWER_ON 5
/*************************************************************************************/


/***********************************************************************
 ************************** Extruders number ***************************
 ***********************************************************************/
// This defines the number of extruder real or virtual
// 0,1,2,3,4,5,6
#define EXTRUDERS 1

// This defines the number of Driver extruder you have and use
// 0,1,2,3,4,5,6
#define DRIVER_EXTRUDERS 1
/***********************************************************************/

#endif /* _CONFIGURATION_BASIC_H_ */
