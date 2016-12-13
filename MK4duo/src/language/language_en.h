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

/**
 * English
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_EN_H
#define LANGUAGE_EN_H

#define WELCOME_MSG                         MACHINE_NAME " ready."
#define MSG_SD                              "SD"
#define MSG_SD_INSERTED                     MSG_SD " inserted"
#define MSG_SD_REMOVED                      MSG_SD " removed"
#define MSG_MAIN                            "Main"
#define MSG_AUTOSTART                       "Autostart"
#define MSG_DISABLE_STEPPERS                "Disable steppers"
#define MSG_AUTO_HOME                       "Auto home"
#define MSG_AUTO_HOME_X                     "Home X"
#define MSG_AUTO_HOME_Y                     "Home Y"
#define MSG_AUTO_HOME_Z                     "Home Z"
#define MSG_LEVEL_BED_HOMING                "Homing XYZ"
#define MSG_LEVEL_BED_WAITING               "Click to Begin"
#define MSG_LEVEL_BED_NEXT_POINT            "Next Point"
#define MSG_LEVEL_BED_DONE                  "Leveling Done!"
#define MSG_LEVEL_BED_CANCEL                "Cancel"
#define MSG_SET_HOME_OFFSETS                "Set home offsets"
#define MSG_HOME_OFFSETS_APPLIED            "Offsets applied"
#define MSG_SET_ORIGIN                      "Set origin"
#define MSG_PREHEAT                         "Preheat"
#define MSG_PREHEAT_1                       MSG_PREHEAT " PLA"
#define MSG_PREHEAT_1_N                     MSG_PREHEAT_1 " "
#define MSG_PREHEAT_1_ALL                   MSG_PREHEAT_1 " All"
#define MSG_PREHEAT_1_BEDONLY               MSG_PREHEAT_1 " Bed"
#define MSG_PREHEAT_1_SETTINGS              MSG_PREHEAT_1 " conf"
#define MSG_PREHEAT_2                       MSG_PREHEAT " ABS"
#define MSG_PREHEAT_2_N                     MSG_PREHEAT_2 " "
#define MSG_PREHEAT_2_ALL                   MSG_PREHEAT_2 " All"
#define MSG_PREHEAT_2_BEDONLY               MSG_PREHEAT_2 " Bed"
#define MSG_PREHEAT_2_SETTINGS              MSG_PREHEAT_2 " conf"
#define MSG_PREHEAT_3                       MSG_PREHEAT " GUM"
#define MSG_PREHEAT_3_N                     MSG_PREHEAT_3 " "
#define MSG_PREHEAT_3_ALL                   MSG_PREHEAT_3 " All"
#define MSG_PREHEAT_3_BEDONLY               MSG_PREHEAT_3 " Bed"
#define MSG_PREHEAT_3_SETTINGS              MSG_PREHEAT_3 " conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "Cooldown"
#define MSG_SWITCH_PS_ON                    "Switch power on"
#define MSG_SWITCH_PS_OFF                   "Switch power off"
#define MSG_EXTRUDE                         "Extrude"
#define MSG_RETRACT                         "Retract"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "Speed"
#define MSG_NOZZLE                          "Nozzle"
#define MSG_BED                             "Bed"
#define MSG_CHAMBER                         "Chamber"
#define MSG_COOLER                          "Cooler"
#define MSG_BED_Z                           "Bed Z"
#define MSG_FAN_SPEED                       "Fan speed"
#define MSG_FLOW                            "Flow"
#define MSG_CONTROL                         "Control"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Autotemp"
#define MSG_ON                              "On "
#define MSG_OFF                             "Off"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_PID_C                           "PID-C"
#define MSG_SELECT                          "Select"
#define MSG_ACC                             "Accel"
#define MSG_VX_JERK                         "Vx-jerk"
#define MSG_VY_JERK                         "Vy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_MOVE                            "Move"
#define MSG_MOVE_AXIS                       MSG_MOVE " axis"
#define MSG_MOVE_X                          MSG_MOVE " " MSG_X
#define MSG_MOVE_Y                          MSG_MOVE " " MSG_Y
#define MSG_MOVE_Z                          MSG_MOVE " " MSG_Z
#define MSG_MOVE_01MM                       MSG_MOVE " 0.1mm"
#define MSG_MOVE_1MM                        MSG_MOVE " 1mm"
#define MSG_MOVE_10MM                       MSG_MOVE " 10mm"
#define MSG_MOVE_E                          "Extruder"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          MSG_X " steps/mm"
#define MSG_YSTEPS                          MSG_Y " steps/mm"
#define MSG_ZSTEPS                          MSG_Z " steps/mm"
#define MSG_E0STEPS                         MSG_E "0 steps/mm"
#define MSG_E1STEPS                         MSG_E "1 steps/mm"
#define MSG_E2STEPS                         MSG_E "2 steps/mm"
#define MSG_E3STEPS                         MSG_E "3 steps/mm"
#define MSG_E4STEPS                         MSG_E "4 steps/mm"
#define MSG_E5STEPS                         MSG_E "5 steps/mm"
#define MSG_TEMPERATURE                     "Temperature"
#define MSG_MOTION                          "Motion"
#define MSG_FILAMENT                        "Filament"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD contrast"
#define MSG_STORE_EPROM                     "Store memory"
#define MSG_LOAD_EPROM                      "Load memory"
#define MSG_RESTORE_FAILSAFE                "Restore failsafe"
#define MSG_REFRESH                         "Refresh"
#define MSG_WATCH                           "Info screen"
#define MSG_PREPARE                         "Prepare"
#define MSG_TUNE                            "Tune"
#define MSG_PAUSE_PRINT                     "Pause print"
#define MSG_RESUME_PRINT                    "Resume print"
#define MSG_STOP_PRINT                      "Stop print"
#define MSG_STOP_SAVE_PRINT                 "Stop and Save"
#define MSG_CARD_MENU                       "Print from " MSG_SD
#define MSG_NO_CARD                         "No " MSG_SD
#define MSG_DWELL                           "Sleep..."
#define MSG_USERWAIT                        "Wait for user..."
#define MSG_RESUMING                        "Resuming print"
#define MSG_PRINT_ABORTED                   "Print aborted"
#define MSG_NO_MOVE                         "No move."
#define MSG_KILLED                          "KILLED. "
#define MSG_STOPPED                         "STOPPED. "
#define MSG_CONTROL_RETRACT                 "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            "Swap Re. mm"
#define MSG_CONTROL_RETRACTF                "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Swap UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENTCHANGE                  "Change filament"
#define MSG_INIT_SDCARD                     "Init. " MSG_SD
#define MSG_CNG_SDCARD                      "Change " MSG_SD
#define MSG_ZPROBE_OUT                      "Z probe out. bed"
#define MSG_BLTOUCH_SELFTEST                "BLTouch Self-Test"
#define MSG_BLTOUCH_RESET                   "Reset BLTouch"
#define MSG_HOME                            "Home"
#define MSG_FIRST                           "first"
#define MSG_ZPROBE_ZOFFSET                  "ZProbe ZOffset"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Endstop abort"
#define MSG_HEATING_FAILED_LCD              "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              "REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_AD595                           "AD595 Offset & Gain"
#define MSG_ERR_MAXTEMP                     "MAXTEMP ERROR"
#define MSG_ERR_MINTEMP                     "MINTEMP ERROR"
#define MSG_ERR_MAXTEMP_BED                 "MAXTEMP BED ERROR"
#define MSG_ERR_MINTEMP_BED                 "MINTEMP BED ERROR"
#define MSG_ERR_Z_HOMING                    "G28 Z Error"
#define MSG_ERR_MAXTEMP_CHAMBER             "MAXTEMP CHAMBER ERROR"
#define MSG_ERR_MINTEMP_CHAMBER             "MINTEMP CHAMBER ERROR"
#define MSG_ERR_MAXTEMP_COOLER              "MAXTEMP COOLER ERROR"
#define MSG_ERR_MINTEMP_COOLER              "MINTEMP COOLER ERROR"
#define MSG_HALTED                          "PRINTER HALTED"
#define MSG_PLEASE_RESET                    "Please reset"
#define MSG_END_DAY                         "days"
#define MSG_END_HOUR                        "hours"
#define MSG_END_MINUTE                      "minutes"
#define MSG_PRINT_TIME                      "Print time "

// Calibrate Delta
#define MSG_DELTA_CALIBRATE                 "Delta Calibration"
#define MSG_DELTA_CALIBRATE_X               "Calibrate " MSG_X
#define MSG_DELTA_CALIBRATE_Y               "Calibrate " MSG_Y
#define MSG_DELTA_CALIBRATE_Z               "Calibrate " MSG_Z
#define MSG_DELTA_CALIBRATE_CENTER          "Calibrate Center"

// Info printers
#define MSG_INFO_MENU                       "About Printer"
#define MSG_INFO_FIRMWARE_MENU              "Firmware Info"
#define MSG_INFO_STATS_MENU                 "Printer Stats"
#define MSG_INFO_BOARD_MENU                 "Board Info"
#define MSG_INFO_THERMISTOR_MENU            "Thermistors"
#define MSG_INFO_EXTRUDERS                  "Extruders"
#define MSG_INFO_HOTENDS                    "Hotends"
#define MSG_INFO_BED                        "Bed"
#define MSG_INFO_CHAMBER                    "Hot Chamber"
#define MSG_INFO_COOLER                     "Cooler"
#define MSG_INFO_BAUDRATE                   "Baud"
#define MSG_INFO_PROTOCOL                   "Protocol"
#define MSG_INFO_TOTAL_PRINTS               "Total Prints"
#define MSG_INFO_FINISHED_PRINTS            "Fin. Prints"
#define MSG_INFO_ON_TIME                    "On x"
#define MSG_INFO_PRINT_TIME                 "Pr x"
#define MSG_INFO_FILAMENT_USAGE             "Fil"
#define MSG_INFO_PWRCONSUMED                "PWR"
#define MSG_INFO_MIN_TEMP                   "Min Temp"
#define MSG_INFO_MAX_TEMP                   "Max Temp"
#define MSG_INFO_PSU                        "Power Supply"

// CASE LIGHT
#define MSG_LIGHTS_ON                       "Case light on"
#define MSG_LIGHTS_OFF                      "Case light off"

// FILAMENT_CHANGE_FEATURE
#define MSG_FILAMENT_CHANGE_HEADER          "CHANGE FILAMENT"
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   "CHANGE OPTIONS:"
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE  "Extrude more"
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   "Resume print"
#if LCD_HEIGHT >= 4
  #define MSG_FILAMENT_CHANGE_INIT_1          "Wait for start"
  #define MSG_FILAMENT_CHANGE_INIT_2          "of the filament"
  #define MSG_FILAMENT_CHANGE_INIT_3          "change"
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        "Wait for"
  #define MSG_FILAMENT_CHANGE_UNLOAD_2        "filament unload"
  #define MSG_FILAMENT_CHANGE_UNLOAD_3        ""
  #define MSG_FILAMENT_CHANGE_INSERT_1        "Insert filament"
  #define MSG_FILAMENT_CHANGE_INSERT_2        "and press button"
  #define MSG_FILAMENT_CHANGE_INSERT_3        "to continue..."
  #define MSG_FILAMENT_CHANGE_LOAD_1          "Wait for"
  #define MSG_FILAMENT_CHANGE_LOAD_2          "filament load"
  #define MSG_FILAMENT_CHANGE_LOAD_3          ""
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       "Wait for"
  #define MSG_FILAMENT_CHANGE_EXTRUDE_2       "filament extrude"
  #define MSG_FILAMENT_CHANGE_EXTRUDE_3       ""
  #define MSG_FILAMENT_CHANGE_RESUME_1        "Wait for print"
  #define MSG_FILAMENT_CHANGE_RESUME_2        "resume"
  #define MSG_FILAMENT_CHANGE_RESUME_3        ""
#else // LCD_HEIGHT < 4
  #define MSG_FILAMENT_CHANGE_INIT_1          "Please wait..."
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        "Ejecting..."
  #define MSG_FILAMENT_CHANGE_INSERT_1        "Insert and Click"
  #define MSG_FILAMENT_CHANGE_LOAD_1          "Loading..."
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       "Extruding..."
  #define MSG_FILAMENT_CHANGE_RESUME_1        "Resuming..."
#endif // LCD_HEIGHT < 4

// Heater
#define MSG_HEATING                         "Heating..."
#define MSG_HEATING_COMPLETE                "Heating done."
#define MSG_BED_HEATING                     "Bed Heating."
#define MSG_BED_DONE                        "Bed done."
#define MSG_CHAMBER_HEATING                 "Chamber Heating."
#define MSG_CHAMBER_DONE                    "Chamber done."
#define MSG_COOLER_COOLING                  "Cooling..."
#define MSG_COOLER_DONE                     "Cooling done."

// Extra
#define MSG_LASER                           "Laser Preset"
#define MSG_CONFIG                          "Configuration"
#define MSG_E_BOWDEN_LENGTH                 MSG_EXTRUDE " " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_R_BOWDEN_LENGTH                 MSG_RETRACT " " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_PURGE_XMM                       MSG_PURGE " " STRINGIFY(LCD_PURGE_LENGTH) "mm"
#define MSG_RETRACT_XMM                     MSG_RETRACT " " STRINGIFY(LCD_RETRACT_LENGTH) "mm"
#define MSG_SAVED_POS                       "Saved position"
#define MSG_RESTORING_POS                   "Restoring position"
#define MSG_INVALID_POS_SLOT                "Invalid slot, total slots: "

// Rfid module
#define MSG_RFID_SPOOL                      "Spool on E"
#define MSG_RFID_BRAND                      "Brand: "
#define MSG_RFID_TYPE                       "Type: "
#define MSG_RFID_COLOR                      "Color: "
#define MSG_RFID_SIZE                       "Size: "
#define MSG_RFID_TEMP_HOTEND                "Temperature Hotend: "
#define MSG_RFID_TEMP_BED                   "Temperature Bed: "
#define MSG_RFID_TEMP_USER_HOTEND           "User temperature Hotend: "
#define MSG_RFID_TEMP_USER_BED              "User temperatura Bed: "
#define MSG_RFID_DENSITY                    "Density: "
#define MSG_RFID_SPOOL_LENGHT               "Spool Lenght: "


#endif // LANGUAGE_EN_H
