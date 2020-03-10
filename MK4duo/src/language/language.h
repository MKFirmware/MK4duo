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

#define _UxGT(a) a

// Fallback if no language is set. DON'T CHANGE
#if DISABLED(LCD_LANGUAGE)
  #define LCD_LANGUAGE en
#endif

#define PROTOCOL_VERSION "2.0"

// For character-based LCD controllers (LCD_CHARSET_HD44780)
#define JAPANESE 1
#define WESTERN  2
#define CYRILLIC 3

//Machine name
#if ENABLED(CUSTOM_MACHINE_NAME)
  #define MACHINE_NAME CUSTOM_MACHINE_NAME
#else
  #define MACHINE_NAME BOARD_NAME
#endif

// LCD Menu Messages
#if DISABLED(DISPLAY_CHARSET_HD44780_JAPAN) && DISABLED(DISPLAY_CHARSET_HD44780_WESTERN) && DISABLED(DISPLAY_CHARSET_HD44780_CYRILLIC)
  #define DISPLAY_CHARSET_HD44780_JAPAN
#endif

// Serial Console Messages (do not translate those!)
#if MECH(CARTESIAN)
  #define MACHINE_TYPE                      "Cartesian"
#elif MECH(COREXY)
  #define MACHINE_TYPE                      "CoreXY"
#elif MECH(COREYX)
  #define MACHINE_TYPE                      "CoreYX"
#elif MECH(COREXZ)
  #define MACHINE_TYPE                      "CoreXZ"
#elif MECH(COREZX)
  #define MACHINE_TYPE                      "CoreZX"
#elif MECH(COREYZ)
  #define MACHINE_TYPE                      "CoreYZ"
#elif MECH(COREZY)
  #define MACHINE_TYPE                      "CoreZY"
#elif MECH(DELTA)
  #define MACHINE_TYPE                      "Delta"
#elif MECH(MORGAN_SCARA)
  #define MACHINE_TYPE                      "Morgan Scara"
#elif MECH(MAKERARM_SCARA)
  #define MACHINE_TYPE                      "MakerArm Scara"
#elif MECH(MUVE3D)
  #define MACHINE_TYPE                      "Muve3D"
#endif  

#define MSG_1_LINE(A)         A "\0"   "\0"
#define MSG_2_LINE(A,B)       A "\0" B "\0"
#define MSG_3_LINE(A,B,C)     A "\0" B "\0" C
#define MSG_4_LINE(A,B,C,D)   A "\0" B "\0" C "\0" D

#if HAS_CHARACTER_LCD

  // Custom characters defined in the first 8 characters of the LCD
  #define LCD_STR_BEDTEMP     "\x00"
  #define LCD_STR_DEGREE      "\x01"
  #define LCD_STR_THERMOMETER "\x02"
  #define LCD_STR_UPLEVEL     "\x03"
  #define LCD_STR_REFRESH     "\x04"
  #define LCD_STR_FOLDER      "\x05"
  #define LCD_STR_FEEDRATE    "\x06"
  #define LCD_STR_CLOCK       "\x07"
  #define LCD_STR_ARROW_RIGHT ">"  /* from the default character set */

#elif HAS_NEXTION_LCD

  #define LCD_STR_BEDTEMP     ""
  #define LCD_STR_DEGREE      "°"
  #define LCD_STR_THERMOMETER ""
  #define LCD_STR_UPLEVEL     "^"
  #define LCD_STR_REFRESH     ""
  #define LCD_STR_FOLDER      "| "
  #define LCD_STR_FEEDRATE    ""
  #define LCD_STR_CLOCK       ""
  #define LCD_STR_ARROW_RIGHT ">"

#else

  //
  // Custom characters from MK4duo_symbols.fon which was merged into ISO10646-0-3.bdf
  // \x00 intentionally skipped to avoid problems in strings
  //
  #define LCD_STR_REFRESH     "\x01"
  #define LCD_STR_FOLDER      "\x02"
  #define LCD_STR_ARROW_RIGHT "\x03"
  #define LCD_STR_UPLEVEL     "\x04"
  #define LCD_STR_CLOCK       "\x05"
  #define LCD_STR_FEEDRATE    "\x06"
  #define LCD_STR_BEDTEMP     "\x07"
  #define LCD_STR_THERMOMETER "\x08"
  #define LCD_STR_DEGREE      "\x09"

  #define LCD_STR_SPECIAL_MAX '\x09'
  // Maximum here is 0x1F because 0x20 is ' ' (space) and the normal charsets begin.
  // Better stay below 0x10 because DISPLAY_CHARSET_HD44780_WESTERN begins here.

  // Symbol characters
  #define LCD_STR_FILAM_DIA   "\xF8"
  #define LCD_STR_FILAM_MUL   "\xA4"

#endif

#define MSG_HOST_ENQUEUEING                     "enqueueing \""
#define MSG_HOST_POWERUP                        "PowerUp"
#define MSG_HOST_EXTERNAL_RESET                 "External Reset"
#define MSG_HOST_BROWNOUT_RESET                 "Brown out Reset"
#define MSG_HOST_WATCHDOG_RESET                 "Watchdog Reset"
#define MSG_HOST_SOFTWARE_RESET                 "Software Reset"
#define MSG_HOST_AUTHOR                         " | Author: "
#define MSG_HOST_CONFIGURATION_VER              "Last Updated: "
#define MSG_HOST_COMPILED                       "Compiled: "
#define MSG_HOST_FREE_MEMORY                    "Free Memory: "
#define MSG_HOST_PLANNER_BUFFER_BYTES           " PlannerBufferBytes: "
#define MSG_HOST_STATS                          "Stats: "
#define MSG_HOST_SERVICE                        "Service: "
#define MSG_HOST_ERR_LINE_NO                    "Line Number is not Last Line Number+1, Last Line: "
#define MSG_HOST_ERR_CHECKSUM_MISMATCH          "checksum mismatch, Last Line: "
#define MSG_HOST_ERR_NO_CHECKSUM                "No Checksum with line number, Last Line: "
#define MSG_HOST_FILE_PRINTED                   "Done printing file"
#define MSG_HOST_BEGIN_FILE_LIST                "Begin file list"
#define MSG_HOST_END_FILE_LIST                  "End file list"
#define MSG_HOST_INVALID_EXTRUDER               "Invalid extruder"
#define MSG_HOST_INVALID_DRIVER                 "Invalid driver"
#define MSG_HOST_INVALID_HOTEND                 "Invalid hotend"
#define MSG_HOST_INVALID_HEATER                 "Invalid heater"
#define MSG_HOST_INVALID_SOLENOID               "Invalid solenoid"
#define MSG_HOST_ERR_NO_THERMISTORS             "No thermistors - no temperature"
#define MSG_HOST_M115_REPORT                    "FIRMWARE_NAME:MK4duo " SHORT_BUILD_VERSION " FIRMWARE_URL:" MK4DUO_FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:" MACHINE_TYPE " EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#define MSG_HOST_COUNT_X                        " Count X:"
#define MSG_HOST_COUNT_A                        " Count A:"
#define MSG_HOST_COUNT_ALPHA                    " Count Alpha:"
#define MSG_HOST_ERR_KILLED                     "Printer halted. kill() called!"
#define MSG_HOST_ERR_STOPPED                    "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
#define MSG_HOST_BUSY_PROCESSING                "processing"
#define MSG_HOST_BUSY_WAIT_HEATER               "heating"
#define MSG_HOST_BUSY_DOOR_OPEN                 "door open"
#define MSG_HOST_BUSY_PAUSED_FOR_USER           "paused for user"
#define MSG_HOST_BUSY_PAUSED_FOR_INPUT          "paused for input"
#define MSG_HOST_UNKNOWN_COMMAND                "Unknown command: \""
#define MSG_HOST_ACTIVE_DRIVER                  "Active Driver: "
#define MSG_HOST_ACTIVE_EXTRUDER                "Active Extruder: "
#define MSG_HOST_ACTIVE_COLOR                   "Active Color: "
#define MSG_HOST_X_MIN                          "x_min"
#define MSG_HOST_X_MAX                          "x_max"
#define MSG_HOST_X2_MIN                         "x2_min"
#define MSG_HOST_X2_MAX                         "x2_max"
#define MSG_HOST_Y_MIN                          "y_min"
#define MSG_HOST_Y_MAX                          "y_max"
#define MSG_HOST_Y2_MIN                         "y2_min"
#define MSG_HOST_Y2_MAX                         "y2_max"
#define MSG_HOST_Z_MIN                          "z_min"
#define MSG_HOST_Z_MAX                          "z_max"
#define MSG_HOST_Z2_MIN                         "z2_min"
#define MSG_HOST_Z2_MAX                         "z2_max"
#define MSG_HOST_Z3_MIN                         "z3_min"
#define MSG_HOST_Z3_MAX                         "z3_max"
#define MSG_HOST_Z_PROBE                        "z_probe"
#define MSG_HOST_E_MIN                          "e_min"
#define MSG_HOST_PROBE_Z_OFFSET                 "Probe Z Offset"
#define MSG_HOST_FILAMENT_RUNOUT                "filament"
#define MSG_HOST_DOOR_OPEN                      "door"
#define MSG_HOST_POWER_CHECK                    "power check"
#define MSG_HOST_ERR_MATERIAL_INDEX             "M145 S<index> out of range (0-2)"
#define MSG_HOST_ERR_M421_PARAMETERS            "M421 incorrect parameter usage"
#define MSG_HOST_ERR_MESH_XY                    "Mesh point cannot be resolved"
#define MSG_HOST_ERR_ARC_ARGS                   "G2/G3 bad parameters"
#define MSG_HOST_ERR_PROTECTED_PIN              "Protected Pin"
#define MSG_HOST_ERR_M420_FAILED                "Failed to enable Bed Leveling"
#define MSG_HOST_ERR_M428_TOO_FAR               "Too far from reference point"
#define MSG_HOST_HOTEND_TOO_COLD                "Hotend too cold"
#define MSG_HOST_M119_REPORT                    "Reporting endstop status"
#define MSG_HOST_ON                             "ON"
#define MSG_HOST_OFF                            "OFF"
#define MSG_HOST_ENDSTOP_HIT                    "TRIGGERED"
#define MSG_HOST_ENDSTOP_OPEN                   "NOT TRIGGERED"
#define MSG_HOST_HOTEND_OFFSET                  "Hotend offsets:"
#define MSG_HOST_DUPLICATION_MODE               "Duplication mode: "
#define MSG_HOST_SOFT_ENDSTOPS                  "Soft endstops"
#define MSG_HOST_SOFT_MIN                       "  Min:"
#define MSG_HOST_SOFT_MAX                       "  Max:"
#define MSG_HOST_ZPROBE_OUT                     "Z Probe Past Bed"

// Slot
#define MSG_HOST_SAVED_POS                      "Saved position"
#define MSG_HOST_RESTORING_POS                  "Restoring position"
#define MSG_HOST_INVALID_POS_SLOT               "Invalid slot, total slots:"

// SD Card
#define MSG_HOST_SD_CANT_OPEN_SUBDIR            "Cannot open subdir "
#define MSG_HOST_SD_ERRORCODE                   "SD errorCode:"
#define MSG_HOST_SD_INIT_FAIL                   "SD init fail"
#define MSG_HOST_SD_CARD_OK                     "SD card ok"
#define MSG_HOST_SD_OPEN_FILE_FAIL              "open failed, File:"
#define MSG_HOST_SD_FILE_OPENED                 "File opened:"
#define MSG_HOST_SD_SIZE                        " Size:"
#define MSG_HOST_SD_FILE_SELECTED               "File selected"
#define MSG_HOST_SD_WRITE_TO_FILE               "Writing to file:"
#define MSG_HOST_SD_FILE_SAVED                  "Done saving file."
#define MSG_HOST_SD_PRINTING_BYTE               "SD printing byte "
#define MSG_HOST_SD_NOT_PRINTING                "Not SD printing"
#define MSG_HOST_SD_ERR_WRITE_TO_FILE           "error writing to file"
#define MSG_HOST_SD_ERR_READ                    "SD read error"
#define MSG_HOST_SD_CANT_ENTER_SUBDIR           "Cannot enter subdir:"
#define MSG_HOST_SD_FILE_DELETED                "File deleted"
#define MSG_HOST_SD_FILE_DELETION_ERR           "Deletion failed"
#define MSG_HOST_SD_DIRECTORY_CREATED           "Directory created"
#define MSG_HOST_SD_CREATION_FAILED             "Creation failed"
#define MSG_HOST_SD_SLASH                       "/"
#define MSG_HOST_SD_MAX_DEPTH                   "trying to call sub-gcode files with too many levels. MAX level is:"

#define MSG_HOST_ENDSTOPS_HIT                   "endstops hit: "
#define MSG_HOST_ERR_COLD_EXTRUDE_STOP          "cold extrusion prevented"
#define MSG_HOST_ERR_LONG_EXTRUDE_STOP          "too long extrusion prevented"
#define MSG_HOST_MAX_INACTIVITY_TIME            "Heating disabled by safety timer."

#define MSG_HOST_FILAMENT_CHANGE_HEAT           "Press button (or M108) to heat nozzle"
#define MSG_HOST_FILAMENT_CHANGE_INSERT         "Insert filament and press button (or M108)"
#define MSG_HOST_FILAMENT_CHANGE_WAIT           "Press button (or M108) to resume"
#define MSG_HOST_FILAMENT_CHANGE_HEAT_LCD       "Press button to heat nozzle"
#define MSG_HOST_FILAMENT_CHANGE_INSERT_LCD     "Insert filament and press button"
#define MSG_HOST_FILAMENT_CHANGE_WAIT_LCD       "Press button to resume"
#define MSG_HOST_FILAMENT_CHANGE_HEAT_M108      "Send M108 to heat nozzle"
#define MSG_HOST_FILAMENT_CHANGE_INSERT_M108    "Insert filament and send M108"
#define MSG_HOST_FILAMENT_CHANGE_WAIT_M108      "Send M108 to resume"

#define MSG_HOST_ERR_EEPROM_WRITE               "Error writing to EEPROM!"

#define MSG_HOST_STOP_BLTOUCH                   "STOP called because of BLTouch error - restart with M999"
#define MSG_HOST_STOP_UNHOMED                   "STOP called because of unhomed error - restart with M999"
#define MSG_HOST_KILL_INACTIVE_TIME             "KILL caused by too much inactive time - current command: "
#define MSG_HOST_KILL_BUTTON                    "KILL caused by KILL button/pin"

#define MSG_HOST_MICROSTEP_MS1_MS2              "MS1,MS2 Pins"
#define MSG_HOST_MICROSTEP_X                    "X:"
#define MSG_HOST_MICROSTEP_Y                    "Y:"
#define MSG_HOST_MICROSTEP_Z                    "Z:"
#define MSG_HOST_MICROSTEP_E                    "E"

// temperature.cpp strings
#define MSG_HOST_PID_AUTOTUNE_PREFIX            "PID Autotune"
#define MSG_HOST_PID_AUTOTUNE_START             MSG_HOST_PID_AUTOTUNE_PREFIX " start"
#define MSG_HOST_PID_AUTOTUNE_FAILED            MSG_HOST_PID_AUTOTUNE_PREFIX " failed!"
#define MSG_HOST_PID_AUTOTUNE_FINISHED          MSG_HOST_PID_AUTOTUNE_PREFIX " finished! Put the last Kp, Ki and Kd constants from below into Configuration!"
#define MSG_HOST_PID_TEMP_TOO_HIGH              MSG_HOST_PID_AUTOTUNE_FAILED " Temperature too high"
#define MSG_HOST_PID_TEMP_TOO_LOW               MSG_HOST_PID_AUTOTUNE_FAILED " Temperature too low"
#define MSG_HOST_PID_TIMEOUT                    MSG_HOST_PID_AUTOTUNE_FAILED " timeout"
#define MSG_HOST_BIAS                           " bias:"
#define MSG_HOST_D                              " d:"
#define MSG_HOST_T_MIN                          " min:"
#define MSG_HOST_T_MAX                          " max:"
#define MSG_HOST_KU                             " Ku:"
#define MSG_HOST_TU                             " Tu:"
#define MSG_HOST_CLASSIC_PID                    " Classic PID:"
#define MSG_HOST_SOME_OVERSHOOT_PID             " Some Overshoot PID:"
#define MSG_HOST_NO_OVERSHOOT_PID               " No Overshoot PID:"
#define MSG_HOST_PESSEN_PID                     " Pessen Integral Rule PID:"
#define MSG_HOST_TYREUS_LYBEN_PID               " Tyreus-Lyben PID:"
#define MSG_HOST_KP                             " Kp:"
#define MSG_HOST_KI                             " Ki:"
#define MSG_HOST_KD                             " Kd:"
#define MSG_HOST_T                              " T:"
#define MSG_HOST_AT                             " @"
#define MSG_HOST_BAT                            " B@"
#define MSG_HOST_CAT                            " C@"
#define MSG_HOST_PID_DEBUG                      " PID_DEBUG "
#define MSG_HOST_PID_DEBUG_INPUT                ": Input:"
#define MSG_HOST_PID_DEBUG_OUTPUT               " Output:"
#define MSG_HOST_PID_DEBUG_PTERM                " pTerm:"
#define MSG_HOST_PID_DEBUG_ITERM                " iTerm:"
#define MSG_HOST_PID_DEBUG_DTERM                " dTerm:"
#define MSG_HOST_INVALID_EXTRUDER_NUM           " - Invalid extruder number !"

#define MSG_HOST_HEATER_STOPPED                 ", heater stopped! "
#define MSG_HOST_HEATER_HOTEND                  "Hotend"
#define MSG_HOST_HEATER_BED                     "Bed"
#define MSG_HOST_HEATER_CHAMBER                 "Chamber"
#define MSG_HOST_HEATER_COOLER                  "Cooler"
#define MSG_HOST_REDUNDANCY                     "Heater switched off. Temperature difference between temp sensors is too high !"
#define MSG_HOST_HEATING_FAILED                 "Heating failed"
#define MSG_HOST_T_THERMAL_RUNAWAY              "Thermal Runaway"
#define MSG_HOST_T_MAXTEMP                      "MAXTEMP triggered"
#define MSG_HOST_T_MINTEMP                      "MINTEMP triggered"

// other
#define MSG_HOST_BED_LEVELING_X                 " X:"
#define MSG_HOST_BED_LEVELING_Y                 " Y:"
#define MSG_HOST_BED_LEVELING_Z                 "Z-probe:"
#define MSG_HOST_ERR_PROBING_FAILED             "Probing Failed"
#define MSG_HOST_ERR_Z_HOMING                   "Home XY First"

// Pins
#define MSG_HOST_CHANGE_PIN                     "For change pin please write in EEPROM and reset the printer."

// Never translate these strings
#define MSG_HOST_X "X"
#define MSG_HOST_Y "Y"
#define MSG_HOST_Z "Z"
#define MSG_HOST_E "E"
#if IS_KINEMATIC
  #define MSG_HOST_A "A"
  #define MSG_HOST_B "B"
  #define MSG_HOST_C "C"
#else
  #define MSG_HOST_A "X"
  #define MSG_HOST_B "Y"
  #define MSG_HOST_C "Z"
#endif
#define MSG_HOST_X2 "X2"
#define MSG_HOST_Y2 "Y2"
#define MSG_HOST_Z2 "Z2"
#define MSG_HOST_Z3 "Z3"

#define LCD_STR_A MSG_HOST_A
#define LCD_STR_B MSG_HOST_B
#define LCD_STR_C MSG_HOST_C
#define LCD_STR_E MSG_HOST_E

#define LANGUAGE_INCL(M)        STRINGIFY(language_##M.h)
#define INCLUDE_BY_LANGUAGE(M)  LANGUAGE_INCL(M)
#define NO_LANGUAGE             false

#if ENABLED(LCD_LANGUAGE_4) && LCD_LANGUAGE_4
  #define NUM_LANGUAGES 5
#elif ENABLED(LCD_LANGUAGE_3) && LCD_LANGUAGE_3
  #define NUM_LANGUAGES 4
#elif ENABLED(LCD_LANGUAGE_2) && LCD_LANGUAGE_2
  #define NUM_LANGUAGES 3
#elif ENABLED(LCD_LANGUAGE_1) && LCD_LANGUAGE_1
  #define NUM_LANGUAGES 2
#else
  #define NUM_LANGUAGES 1
#endif

#if DISABLED(LCD_LANGUAGE_0)
  #define LCD_LANGUAGE_0  LCD_LANGUAGE
#endif

#if DISABLED(LCD_LANGUAGE_1) || !LCD_LANGUAGE_1
  #undef  LCD_LANGUAGE_1
  #define LCD_LANGUAGE_1  LCD_LANGUAGE_0
#endif

#if DISABLED(LCD_LANGUAGE_2) || !LCD_LANGUAGE_2
  #undef  LCD_LANGUAGE_2
  #define LCD_LANGUAGE_2  LCD_LANGUAGE_1
#endif

#if DISABLED(LCD_LANGUAGE_3) || !LCD_LANGUAGE_3
  #undef  LCD_LANGUAGE_3
  #define LCD_LANGUAGE_3  LCD_LANGUAGE_2
#endif

#if DISABLED(LCD_LANGUAGE_4) || !LCD_LANGUAGE_4
  #undef  LCD_LANGUAGE_4
  #define LCD_LANGUAGE_4  LCD_LANGUAGE_3
#endif

#define _GET_LANG(LANG)   language_##LANG
#define GET_LANG(LANG)    _GET_LANG(LANG)

#if NUM_LANGUAGES > 1
  #define GET_TEXT(MSG) ( \
    lcdui.lang == 0 ? GET_LANG(LCD_LANGUAGE_0)::MSG :   \
    lcdui.lang == 1 ? GET_LANG(LCD_LANGUAGE_1)::MSG : \
    lcdui.lang == 2 ? GET_LANG(LCD_LANGUAGE_2)::MSG : \
    lcdui.lang == 3 ? GET_LANG(LCD_LANGUAGE_3)::MSG : \
                      GET_LANG(LCD_LANGUAGE_4)::MSG   \
    )
  #define MAX_LANG_CHARSIZE MAX(GET_LANG(LCD_LANGUAGE_0)::CHARSIZE, \
                                GET_LANG(LCD_LANGUAGE_1)::CHARSIZE, \
                                GET_LANG(LCD_LANGUAGE_2)::CHARSIZE, \
                                GET_LANG(LCD_LANGUAGE_3)::CHARSIZE, \
                                GET_LANG(LCD_LANGUAGE_4)::CHARSIZE)
  #define GET_LANGUAGE_NAME(N)  GET_LANG(LCD_LANGUAGE_##N)::LANGUAGE
#else
  #define GET_TEXT(MSG)         GET_LANG(LCD_LANGUAGE_0)::MSG
  #define MAX_LANG_CHARSIZE     GET_LANG(LCD_LANGUAGE_0)::CHARSIZE
#endif

#include "language_en.h"
#include INCLUDE_BY_LANGUAGE(LCD_LANGUAGE_0)
#include INCLUDE_BY_LANGUAGE(LCD_LANGUAGE_1)
#include INCLUDE_BY_LANGUAGE(LCD_LANGUAGE_2)
#include INCLUDE_BY_LANGUAGE(LCD_LANGUAGE_3)
#include INCLUDE_BY_LANGUAGE(LCD_LANGUAGE_4)

#if     DISABLED(DISPLAY_CHARSET_ISO10646_1)      \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_5)      \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_KANA)   \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_GREEK)  \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_CN)     \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_TR)     \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_PL)     \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_CZ)     \
    &&  DISABLED(DISPLAY_CHARSET_ISO10646_SK)
  #define DISPLAY_CHARSET_ISO10646_1  // use the better font on full graphic displays.
#endif
