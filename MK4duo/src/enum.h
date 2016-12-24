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
  E_MIN
};

/**
 * Temperature
 * Stages in the ISR loop
 */
enum TempState {
  PrepareTemp_0,
  MeasureTemp_0,
  PrepareTemp_BED,
  MeasureTemp_BED,
  PrepareTemp_1,
  MeasureTemp_1,
  PrepareTemp_2,
  MeasureTemp_2,
  PrepareTemp_3,
  MeasureTemp_3,
  PrepareTemp_CHAMBER,
  MeasureTemp_CHAMBER,
  PrepareTemp_COOLER,
  MeasureTemp_COOLER,
  Prepare_FILWIDTH,
  Measure_FILWIDTH,
  Prepare_POWCONSUMPTION,
  Measure_POWCONSUMPTION,
  StartupDelay // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

/**
 * States for managing MK4duo and host communication
 * MK4duo sends messages if blocked or busy
 */
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  enum FirmwareState {
    NOT_BUSY,           // Not in a handler
    IN_HANDLER,         // Processing a GCode
    IN_PROCESS,         // Known to be blocking command input (as in G29)
    WAIT_HEATER,        // Wait heater
    DOOR_OPEN,          // Door open
    PAUSED_FOR_USER,    // Blocking pending any input
    PAUSED_FOR_INPUT    // Blocking pending text input (concept)
  };
#endif

#if ENABLED(FILAMENT_CHANGE_FEATURE)
  enum FilamentChangeMenuResponse {
    FILAMENT_CHANGE_RESPONSE_WAIT_FOR,
    FILAMENT_CHANGE_RESPONSE_EXTRUDE_MORE,
    FILAMENT_CHANGE_RESPONSE_RESUME_PRINT
  };
  
  #if HAS(LCD)
    enum FilamentChangeMessage {
      FILAMENT_CHANGE_MESSAGE_INIT,
      FILAMENT_CHANGE_MESSAGE_UNLOAD,
      FILAMENT_CHANGE_MESSAGE_INSERT,
      FILAMENT_CHANGE_MESSAGE_LOAD,
      FILAMENT_CHANGE_MESSAGE_EXTRUDE,
      FILAMENT_CHANGE_MESSAGE_OPTION,
      FILAMENT_CHANGE_MESSAGE_RESUME,
      FILAMENT_CHANGE_MESSAGE_STATUS
    };
  #endif
#endif

#if ENABLED(MESH_BED_LEVELING) && NOMECH(DELTA)
  enum MeshLevelingState {
    MeshReport,
    MeshStart,
    MeshNext,
    MeshSet,
    MeshSetZOffset,
    MeshReset
  };

  enum MBLStatus {
    MBL_STATUS_NONE = 0,
    MBL_STATUS_HAS_MESH_BIT = 0,
    MBL_STATUS_ACTIVE_BIT = 1
  };
#endif

/**
 * Ultra LCD
 */
enum LCDViewAction {
  LCDVIEW_NONE,
  LCDVIEW_REDRAW_NOW,
  LCDVIEW_CALL_REDRAW_NEXT,
  LCDVIEW_CLEAR_CALL_REDRAW,
  LCDVIEW_CALL_NO_REDRAW,
  LCDVIEW_U8G_CONTINUE
};

/**
 * SD Settings
 */
enum cfgSD_ENUM {   // This need to be in the same order as cfgSD_KEY
  SD_CFG_CPR,
  SD_CFG_FIL,
  SD_CFG_NPR,
#if HAS(POWER_CONSUMPTION_SENSOR)
  SD_CFG_PWR,
#endif
  SD_CFG_TME,
  SD_CFG_TPR,
  SD_CFG_END // Leave this always as the last
};

#endif //__ENUM_H__
