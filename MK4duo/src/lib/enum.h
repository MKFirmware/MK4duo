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
 * Axis indices as enumerated constants
 *
 *  - X_AXIS, Y_AXIS, and Z_AXIS should be used for axes in Cartesian space
 *  - A_AXIS, B_AXIS, and C_AXIS should be used for Steppers, corresponding to XYZ on Cartesians
 *  - X_HEAD, Y_HEAD, and Z_HEAD should be used for Steppers on Core kinematics
 */
enum AxisEnum : uint8_t {
  X_AXIS   = 0, A_AXIS = 0,
  Y_AXIS   = 1, B_AXIS = 1,
  Z_AXIS   = 2, C_AXIS = 2,
  E_AXIS   = 3,
  X_HEAD   = 4, Y_HEAD = 5, Z_HEAD = 6,
  E0_AXIS  = 3,
  E1_AXIS  = 4,
  E2_AXIS  = 5,
  E3_AXIS  = 6,
  E4_AXIS  = 7,
  E5_AXIS  = 8,
  ALL_AXES = 0xFE, NO_AXIS = 0xFF
};

/**
 * Driver indices as enumerated constants
 */
enum DriverEnum : uint8_t {
  X_DRV   = 0,
  Y_DRV   = 1,
  Z_DRV   = 2,
  X2_DRV  = 3,
  Y2_DRV  = 4,
  Z2_DRV  = 5,
  Z3_DRV  = 6,
  E0_DRV  = 0,
  E1_DRV  = 1,
  E2_DRV  = 2,
  E3_DRV  = 3,
  E4_DRV  = 4,
  E5_DRV  = 5
};

/**
 * Printer Mode
 */
enum PrinterModeEnum : uint8_t {
  PRINTER_MODE_FFF,           // M450 S0 or M451
  PRINTER_MODE_LASER,         // M450 S1 or M452
  PRINTER_MODE_CNC,           // M450 S2 or M453
  PRINTER_MODE_PICKER,        // M450 S3 or M454
  PRINTER_MODE_SOLDER,        // M450 S4
  PRINTER_MODE_PLOTTER,
  PRINTER_MODE_COUNT
};

/**
 * States for managing MK4duo and host communication
 * MK4duo sends messages if blocked or busy
 */
enum BusyStateEnum : uint8_t {
  NotBusy,          // Not in a handler
  InHandler,        // Processing a GCode
  InProcess,        // Known to be blocking command input (as in G29)
  PausedforUser,    // Blocking pending any input
  PausedforInput,   // Blocking pending text input
  DoorOpen          // Door open
};

/**
 * Emergency Parser
 *  Currently looking for: M108, M112, M410, M876
 */
enum EmergencyStateEnum : uint8_t {
  EP_RESET,
  EP_N,
  EP_M,
  EP_M1,
  EP_M10,
  EP_M108,
  EP_M11,
  EP_M112,
  EP_M4,
  EP_M41,
  EP_M410,
  EP_M8,
  EP_M87,
  EP_M876,
  EP_M876S,
  EP_M876SN,
  EP_IGNORE // to '\n'
};

/**
 * Prompt Reason
 *  For M876 command
 */
enum HostPromptEnum : uint8_t {
  PROMPT_NOT_DEFINED,
  PROMPT_FILAMENT_RUNOUT,
  PROMPT_FILAMENT_RUNOUT_REHEAT,
  PROMPT_USER_CONTINUE,
  PROMPT_PAUSE_RESUME,
  PROMPT_INFO
};

/**
 * Planner
 */
enum BlockFlagBitEnum : uint8_t {
  // Recalculate trapezoids on entry junction. For optimization.
  BLOCK_BIT_RECALCULATE,

  // Nominal speed always reached.
  // i.e., The segment is long enough, so the nominal speed is reachable if accelerating
  // from a safe speed (in consideration of jerking from zero speed).
  BLOCK_BIT_NOMINAL_LENGTH,

  // Sync the stepper counts from the block
  BLOCK_BIT_SYNC_POSITION
};

enum BlockFlagEnum : uint8_t {
  BLOCK_FLAG_RECALCULATE    = _BV(BLOCK_BIT_RECALCULATE),
  BLOCK_FLAG_NOMINAL_LENGTH = _BV(BLOCK_BIT_NOMINAL_LENGTH),
  BLOCK_FLAG_SYNC_POSITION  = _BV(BLOCK_BIT_SYNC_POSITION)
};

/**
 * DUAL X CARRIAGE
 */
enum DualXModeEnum : uint8_t {
  DXC_FULL_CONTROL_MODE,
  DXC_AUTO_PARK_MODE,
  DXC_DUPLICATION_MODE,
  DXC_MIRRORED_MODE
};

/**
 * Work Space Plane
 */
enum WorkspacePlaneEnum : uint8_t {
  PLANE_XY,
  PLANE_ZX,
  PLANE_YZ
};

/**
 * Endstop
 */
enum EndstopEnum : uint8_t {
  X_MIN,        // Bit 0
  Y_MIN,        // Bit 1
  Z_MIN,        // Bit 2
  Z_PROBE,      // Bit 3
  X_MAX,        // Bit 4
  Y_MAX,        // Bit 5
  Z_MAX,        // Bit 6
  X2_MIN,       // Bit 7
  X2_MAX,       // Bit 8
  Y2_MIN,       // Bit 9
  Y2_MAX,       // Bit 10
  Z2_MIN,       // Bit 11
  Z2_MAX,       // Bit 12
  Z3_MIN,       // Bit 13
  Z3_MAX,       // Bit 14
  DOOR_OPEN     // Bit 15
};

/**
 * Temperature
 */
enum TempUnitEnum : uint8_t {
  TEMPUNIT_C,
  TEMPUNIT_K,
  TEMPUNIT_F
};

/**
 * LCD
 */
enum LCDViewActionEnum : uint8_t {
  LCDVIEW_NONE,
  LCDVIEW_REDRAW_NOW,
  LCDVIEW_CALL_REDRAW_NEXT,
  LCDVIEW_CLEAR_CALL_REDRAW,
  LCDVIEW_CALL_NO_REDRAW
};
enum MK4duoFontEnum : uint8_t {
  FONT_STATUSMENU = 1,
  FONT_EDIT,
  FONT_MENU
};
enum HD44780CharSetEnum : uint8_t {
  CHARSET_MENU,
  CHARSET_INFO,
  CHARSET_BOOT
};

/**
 * Probe raise
 */
enum ProbePtRaiseEnum : uint8_t {
  PROBE_PT_NONE,  // No raise or stow after run_probing
  PROBE_PT_STOW,  // Do a complete stow after run_probing
  PROBE_PT_RAISE  // Raise to "between" clearance after run_probing
};

/**
 * Filament runout
 */
enum FilRunoutEnum : uint8_t {
  FIL_RUNOUT_0, // Bit 0
  FIL_RUNOUT_1, // Bit 1
  FIL_RUNOUT_2, // Bit 2
  FIL_RUNOUT_3, // Bit 3
  FIL_RUNOUT_4, // Bit 4
  FIL_RUNOUT_5  // Bit 5
};

/**
 * SD Settings
 */
enum CfgSDEnum : uint8_t {   // This need to be in the same order as cfgSD_KEY
  SD_CFG_CPR,
  SD_CFG_FIL,
  SD_CFG_NPR,
  SD_CFG_PWR,
  SD_CFG_TME,
  SD_CFG_TPR,
  SD_CFG_END // Leave this always as the last
};

enum LsActionEnum : uint8_t {
  LS_Count,
  LS_GetFilename
};

/**
 * Sound
 */
enum SoundModeEnum : uint8_t {
  SOUND_MODE_ON,
  SOUND_MODE_SILENT,
  SOUND_MODE_MUTE
};

/**
 * Advanced Pause
 */
enum PauseModeEnum : uint8_t {
  PAUSE_MODE_SAME,
  PAUSE_MODE_PAUSE_PRINT,
  PAUSE_MODE_CHANGE_FILAMENT,
  PAUSE_MODE_LOAD_FILAMENT,
  PAUSE_MODE_UNLOAD_FILAMENT
};

enum PauseMessageEnum : uint8_t {
  PAUSE_MESSAGE_PAUSING,
  PAUSE_MESSAGE_CHANGING,
  PAUSE_MESSAGE_WAITING,
  PAUSE_MESSAGE_UNLOAD,
  PAUSE_MESSAGE_INSERT,
  PAUSE_MESSAGE_LOAD,
  PAUSE_MESSAGE_PURGE,
  PAUSE_MESSAGE_OPTION,
  PAUSE_MESSAGE_RESUME,
  PAUSE_MESSAGE_STATUS,
  PAUSE_MESSAGE_HEAT,
  PAUSE_MESSAGE_PRINTER_OFF,
  PAUSE_MESSAGE_HEATING
};

enum PauseMenuResponseEnum : uint8_t {
  PAUSE_RESPONSE_WAIT_FOR,
  PAUSE_RESPONSE_EXTRUDE_MORE,
  PAUSE_RESPONSE_RESUME_PRINT
};

/**
 * Mesh Bed Level
 */
enum MeshLevelingStateEnum : uint8_t {
  MeshReport,     // G29 S0
  MeshStart,      // G29 S1
  MeshNext,       // G29 S2
  MeshSet,        // G29 S3
  MeshSetZOffset, // G29 S4
  MeshReset       // G29 S5
};

/**
 * Trinamic Debug
 */
enum TMCdebugEnum : char {
  TMC_CODES,
  TMC_ENABLED,
  TMC_CURRENT,
  TMC_RMS_CURRENT,
  TMC_MAX_CURRENT,
  TMC_IRUN,
  TMC_IHOLD,
  TMC_GLOBAL_SCALER,
  TMC_CS_ACTUAL,
  TMC_PWM_SCALE,
  TMC_VSENSE,
  TMC_STEALTHCHOP,
  TMC_MICROSTEPS,
  TMC_TSTEP,
  TMC_TPWMTHRS,
  TMC_TPWMTHRS_MMS,
  TMC_OTPW,
  TMC_OTPW_TRIGGERED,
  TMC_TOFF,
  TMC_TBL,
  TMC_HEND,
  TMC_HSTRT,
  TMC_SGT
};
enum TMCdrvStatusEnum : char {
  TMC_DRV_CODES,
  TMC_STST,
  TMC_OLB,
  TMC_OLA,
  TMC_S2GB,
  TMC_S2GA,
  TMC_DRV_OTPW,
  TMC_OT,
  TMC_STALLGUARD,
  TMC_DRV_CS_ACTUAL,
  TMC_FSACTIVE,
  TMC_SG_RESULT,
  TMC_DRV_STATUS_HEX,
  TMC_T157,
  TMC_T150,
  TMC_T143,
  TMC_T120,
  TMC_STEALTH,
  TMC_S2VSB,
  TMC_S2VSA
};
enum TMCgetRegistersEnum : char {
  TMC_AXIS_CODES,
  TMC_GET_GCONF,
  TMC_GET_IHOLD_IRUN,
  TMC_GET_GSTAT,
  TMC_GET_IOIN,
  TMC_GET_TPOWERDOWN,
  TMC_GET_TSTEP,
  TMC_GET_TPWMTHRS,
  TMC_GET_TCOOLTHRS,
  TMC_GET_THIGH,
  TMC_GET_CHOPCONF,
  TMC_GET_COOLCONF,
  TMC_GET_PWMCONF,
  TMC_GET_PWM_SCALE,
  TMC_GET_DRV_STATUS,
  TMC_GET_DRVCONF,
  TMC_GET_DRVCTRL,
  TMC_GET_DRVSTATUS,
  TMC_GET_SGCSCONF,
  TMC_GET_SMARTEN
};
