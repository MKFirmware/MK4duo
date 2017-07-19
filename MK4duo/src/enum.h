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

#ifndef __ENUM_H__
#define __ENUM_H__

/**
 * Axis indices as enumerated constants
 *
 * Special axis:
 *  A_AXIS and B_AXIS are used by COREXY or COREYX printers
 *  A_AXIS and C_AXIS are used by COREXZ or COREZX printers
 *  X_HEAD and Y_HEAD and Z_HEAD is used for systems that don't have a 1:1 relationship between X_AXIS and X Head movement, like CoreXY bots.
 *  A_AXIS and B_AXIS and C_AXIS is used for DELTA system.
 */
enum AxisEnum {
  NO_AXIS = -1,
  X_AXIS  = 0,
  A_AXIS  = 0,
  Y_AXIS  = 1,
  B_AXIS  = 1,
  Z_AXIS  = 2,
  C_AXIS  = 2,
  E_AXIS  = 3,
  X_HEAD  = 4,
  Y_HEAD  = 5,
  Z_HEAD  = 6,
  ALL_AXES = 100
};

typedef enum {
  LINEARUNIT_MM,
  LINEARUNIT_INCH
} LinearUnit;

typedef enum {
  TEMPUNIT_C,
  TEMPUNIT_K,
  TEMPUNIT_F
} TempUnit;

/**
 * Debug flags
 * Not yet widely applied
 */
enum DebugFlags {
  DEBUG_NONE          = 0,
  DEBUG_ECHO          = _BV(0), ///< Echo commands in order as they are processed
  DEBUG_INFO          = _BV(1), ///< Print messages for code that has debug output
  DEBUG_ERRORS        = _BV(2), ///< Not implemented
  DEBUG_DRYRUN        = _BV(3), ///< Ignore temperature setting and E movement commands
  DEBUG_COMMUNICATION = _BV(4), ///< Not implemented
  DEBUG_LEVELING      = _BV(5), ///< Print detailed output for homing and leveling
  DEBUG_ALL           = 0xFF
};

enum EndstopEnum {
  X_MIN,
  Y_MIN,
  Z_MIN,
  Z_PROBE,
  X_MAX,
  Y_MAX,
  Z_MAX,
  Z2_MIN,
  Z2_MAX,
  Z3_MIN,
  Z3_MAX,
  Z4_MIN,
  Z4_MAX,
  E_MIN
};

#if ENABLED(EMERGENCY_PARSER)
  enum e_parser_state {
    state_RESET,
    state_N,
    state_M,
    state_M1,
    state_M10,
    state_M108,
    state_M11,
    state_M112,
    state_M4,
    state_M41,
    state_M410,
    state_IGNORE // to '\n'
  };
#endif

/**
 * Interrupt Event
 */
enum MK4duoInterruptEvent {
  INTERRUPT_EVENT_NONE,
  INTERRUPT_EVENT_FIL_RUNOUT,
  INTERRUPT_EVENT_DAV_SYSTEM,
  INTERRUPT_EVENT_ENC_DETECT
};

/**
 * States for managing MK4duo and host communication
 * MK4duo sends messages if blocked or busy
 */
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  enum MK4duoBusyState {
    NOT_BUSY,           // Not in a handler
    IN_HANDLER,         // Processing a GCode
    IN_PROCESS,         // Known to be blocking command input (as in G29)
    WAIT_HEATER,        // Wait heater
    PAUSED_FOR_USER,    // Blocking pending any input
    PAUSED_FOR_INPUT,   // Blocking pending text input (concept)
    DOOR_OPEN           // Door open
  };
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  enum AdvancedPauseMenuResponse {
    ADVANCED_PAUSE_RESPONSE_WAIT_FOR,
    ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE,
    ADVANCED_PAUSE_RESPONSE_RESUME_PRINT
  };
  
  #if HAS_LCD
    enum AdvancedPauseMessage {
      ADVANCED_PAUSE_MESSAGE_INIT,
      ADVANCED_PAUSE_MESSAGE_COOLDOWN,
      ADVANCED_PAUSE_MESSAGE_UNLOAD,
      ADVANCED_PAUSE_MESSAGE_INSERT,
      ADVANCED_PAUSE_MESSAGE_LOAD,
      ADVANCED_PAUSE_MESSAGE_EXTRUDE,
      ADVANCED_PAUSE_MESSAGE_OPTION,
      ADVANCED_PAUSE_MESSAGE_RESUME,
      ADVANCED_PAUSE_MESSAGE_STATUS,
      ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE,
      ADVANCED_PAUSE_MESSAGE_PRINTER_OFF,
      ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT
    };
  #endif
#endif

enum PrinterMode {
  PRINTER_MODE_FFF,           // M450 S0 or M451
  PRINTER_MODE_LASER,         // M450 S1 or M452
  PRINTER_MODE_CNC,           // M450 S2 or M453
  PRINTER_MODE_PICKER,        // M450 S3 or M454
  PRINTER_MODE_SOLDER,        // M450 S4
  PRINTER_MODE_PLOTTER,
  PRINTER_MODE_COUNT
};

/**
 * Ultra LCD
 */
enum LCDViewAction {
  LCDVIEW_NONE,
  LCDVIEW_REDRAW_NOW,
  LCDVIEW_CALL_REDRAW_NEXT,
  LCDVIEW_CLEAR_CALL_REDRAW,
  LCDVIEW_CALL_NO_REDRAW
};

/**
 * SD Settings
 */
enum cfgSD_ENUM {   // This need to be in the same order as cfgSD_KEY
  SD_CFG_CPR,
  SD_CFG_FIL,
  SD_CFG_NPR,
#if HAS_POWER_CONSUMPTION_SENSOR
  SD_CFG_PWR,
#endif
  SD_CFG_TME,
  SD_CFG_TPR,
  SD_CFG_END // Leave this always as the last
};

/**
 * DUAL X CARRIAGE
 */
#if ENABLED(DUAL_X_CARRIAGE)
  enum DualXMode {
    DXC_FULL_CONTROL_MODE,
    DXC_AUTO_PARK_MODE,
    DXC_DUPLICATION_MODE
  };
#endif

/**
 * Workspace planes only apply to G2/G3 moves
 * (and "canned cycles" - not a current feature)
 */
#if ENABLED(CNC_WORKSPACE_PLANES)
  enum WorkspacePlane { PLANE_XY, PLANE_ZX, PLANE_YZ };
#endif

#endif /* __ENUM_H__ */
