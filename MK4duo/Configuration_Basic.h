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

/***********************************************************************
 ********************** Serial comunication type ***********************
 ***********************************************************************/
/**
 * Select a primary serial port on the board will be used for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default port pins.
 * Serial port 0 is always used by the Arduino bootloader regardless of this setting.
 *
 * Valid values are -1 to 3 for Serial, Serial1, Serial2, Serial3 and -1 for SerialUSB
 */
#define SERIAL_PORT_1 0

/**
 * This setting determines the communication speed of the printer on primary port.
 *
 * 250000 works in most cases, but you might try a lower speed if
 * you commonly experience drop-outs during host printing.
 * You may try up to 1000000 to speed up SD file transfer.
 *
 * 2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000
 */
#define BAUDRATE_1 250000

/**
 * Select a secondary serial port on the board to use for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default port pins.
 *
 * Valid values are -1 to 3 for Serial, Serial1, Serial2, Serial3 and -1 for SerialUSB
 * -2 not used
 */
#define SERIAL_PORT_2 -2

/**
 * This setting determines the communication speed of the printer on secondary port.
 *
 * 250000 works in most cases, but you might try a lower speed if
 * you commonly experience drop-outs during host printing.
 * You may try up to 1000000 to speed up SD file transfer.
 *
 * 2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000
 */
#define BAUDRATE_2 250000

/**
 * The number of linear motions that can be in the plan at any give time.
 * THE BLOCK BUFFER SIZE NEEDS TO BE A POWER OF 2 (i.g. 8, 16, 32) because shifts
 * and ors are used to do the ring-buffering.
 * For Arduino DUE setting to 32.
 */
#define BLOCK_BUFFER_SIZE 16

/**
 * The ASCII buffer for receiving from the serial:
 * For Arduino DUE setting bufsize to 8.
 */
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

/**
 * Transmission to Host Buffer Size
 * To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
 * To buffer a simple "ok" you need 4 bytes.
 * For ADVANCED OK (M105) you need 32 bytes.
 * For debug-echo: 128 bytes for the optimal speed.
 * Other output doesn't need to be that speedy.
 * 0, 2, 4, 8, 16, 32, 64, 128, 256
 */
#define TX_BUFFER_SIZE 0

/**
 * Host Receive Buffer Size
 * Without XON/XOFF flow control (see SERIAL XON XOFF below) 32 bytes should be enough.
 * To use flow control, set this buffer size to at least 1024 bytes.
 * 0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048
 */
#define RX_BUFFER_SIZE 128

/**
 * Enable to have the controller send XON/XOFF control characters to
 * the host to signal the RX buffer is becoming full.
 */
//#define SERIAL_XON_XOFF

/**
 * Enable this option to collect and display the maximum
 * RX queue usage after transferring a file to SD.
 */
//#define SERIAL_STATS_MAX_RX_QUEUED

/**
 * Enable this option to collect and display the number
 * of dropped bytes after a file transfer to SD.
 */
//#define SERIAL_STATS_DROPPED_RX

/**
 * User-specified version info of this build to display in [Pronterface, etc] terminal window during
 * startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
 * build by the user have been successfully uploaded into firmware.
 */
#define STRING_CONFIG_AUTHOR "(none, default config)"   // Who made the changes.

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
 */
//#define EMERGENCY_PARSER

/**
 * Spend 28 bytes of SRAM to optimize the GCode parser
 */
//#define FASTER_GCODE_PARSER

/**
 * Spend more bytes of SRAM to optimize the GCode execute
 */
//#define FASTER_GCODE_EXECUTE

/**
 * Host Keepalive
 *
 * When enabled MK4duo will send a busy status message to the host
 * every couple of seconds when it can't accept commands.
 */
// Disable this if your host doesn't like keepalive messages
#define HOST_KEEPALIVE_FEATURE
// Number of seconds between "busy" messages. Set with M113.
#define DEFAULT_KEEPALIVE_INTERVAL 2
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
 * MECH_CARTESIAN       - Prusa, Mendel, etc                           *
 * MECH_COREXY          - H-Bot/Core XY (x_motor = x+y, y_motor = x-y) *
 * MECH_COREYX          - H-Bot/Core YX (x_motor = y+x, y_motor = y-x) *
 * MECH_COREXZ          - H-Bot/Core XZ (x_motor = x+z, z_motor = x-z) *
 * MECH_COREZX          - H-Bot/Core ZX (x_motor = z+x, z_motor = z-x) *
 * MECH_COREYZ          - H-Bot/Core YZ (y_motor = y+z, z_motor = y-z) *
 * MECH_COREZY          - H-Bot/Core ZY (y_motor = z+y, z_motor = z-y) *
 * MECH_DELTA           - Rostock, Kossel, RostockMax, Cerberus, etc   *
 * MECH_MORGAN_SCARA    - SCARA classic                                *
 * MECH_MAKERARM_SCARA  - SCARA Makerfarm                              *
 * MECH_MUVE3D          - Muve 3D with serial projector                *
 *                                                                     *
 ***********************************************************************/
#define MECHANISM MECH_CARTESIAN
/***********************************************************************/


/*************************************************************************************
 ************************************ Power supply ***********************************
 *************************************************************************************
 *                                                                                   *
 * The following define selects which power supply you have.                         *
 * Please choose the one that matches your setup and set to POWER SUPPLY:            *
 * 0 Normal power                                                                    *
 * 1 POWER WITH PS-ON TO GND (ATX)                                                   *
 * 2 POWER WITH PS-ON TO VCC (X-BOX 360)                                             *
 *                                                                                   *
 *************************************************************************************/
#define POWER_SUPPLY 0
//#define POWER_NAME "Generic"

// Define this to have the electronics keep the power supply off on startup.
// If you don't know what this is leave it.
#define PS_DEFAULT_OFF false
// Define delay after power on in seconds
#define DELAY_AFTER_POWER_ON 5
// Define time for automatic power off if not needed in second
#define POWER_TIMEOUT 30
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
