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
 * Japanese (Kana)
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_KANA_H
#define LANGUAGE_KANA_H

#define MAPPER_NON
#define SIMULATE_ROMFONT
#define DISPLAY_CHARSET_ISO10646_KANA


// 片仮名表示定義
#define WELCOME_MSG                         MACHINE_NAME " ready."
#define MSG_SD                              "SD"
#define MSG_SD_INSERTED                     "\xb6\xb0\xc4\xde\x20\xbf\xb3\xc6\xad\xb3\xbb\xda\xcf\xbc\xc0" // "Card inserted"
#define MSG_SD_REMOVED                      "\xb6\xb0\xc4\xde\xb6\xde\xb1\xd8\xcf\xbe\xdd"                  // "Card removed"
#define MSG_MAIN                            "\xd2\xb2\xdd"                                                 // "Main"
#define MSG_AUTOSTART                       "\xbc\xde\xc4\xde\xb3\xb6\xb2\xbc"                             // "Autostart"
#define MSG_DISABLE_STEPPERS                "\xd3\xb0\xc0\xb0\xc3\xde\xdd\xb9\xde\xdd\x20\xb5\xcc"         // "Disable steppers"
#define MSG_AUTO_HOME                       "\xb9\xde\xdd\xc3\xdd\xc6\xb2\xc4\xde\xb3"                     // "Auto home"
#define MSG_AUTO_HOME_X                     "Home X"
#define MSG_AUTO_HOME_Y                     "Home Y"
#define MSG_AUTO_HOME_Z                     "Home Z"
#define MSG_LEVEL_BED_HOMING                "Homing XYZ"
#define MSG_LEVEL_BED_WAITING               "Click to Begin"
#define MSG_LEVEL_BED_NEXT_POINT            "Next Point"
#define MSG_LEVEL_BED_DONE                  "Leveling Done!"
#define MSG_LEVEL_BED_CANCEL                "Cancel"
#define MSG_SET_HOME_OFFSETS                "\xb7\xbc\xde\xad\xdd\xb5\xcc\xbe\xaf\xc4\xbe\xaf\xc3\xb2"     // "Set home offsets"
#define MSG_SET_ORIGIN                      "\xb7\xbc\xde\xad\xdd\xbe\xaf\xc4"                             // "Set origin"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"
#define MSG_FILCONSUMED                     "F:"
#define MSG_PREHEAT                         "\xd6\xc8\xc2"                                                 // "Preheat"
#define MSG_PREHEAT_PLA                     "PLA \xd6\xc8\xc2"                                             // "Preheat PLA"
#define MSG_PREHEAT_PLA_ALL                 MSG_PREHEAT_PLA " \xbd\xcd\xde\xc3"                            // " All"
#define MSG_PREHEAT_PLA_BEDONLY             MSG_PREHEAT_PLA " \xcd\xde\xaf\xc4\xde"                        // "Bed"
#define MSG_PREHEAT_PLA_SETTINGS            MSG_PREHEAT_PLA " \xbe\xaf\xc3\xb2"                            // "conf"
#define MSG_PREHEAT_ABS                     "ABS \xd6\xc8\xc2"                                             // "Preheat ABS"
#define MSG_PREHEAT_ABS_ALL                 MSG_PREHEAT_ABS " \xbd\xcd\xde\xc3"                            // " All"
#define MSG_PREHEAT_ABS_BEDONLY             MSG_PREHEAT_ABS " \xcd\xde\xaf\xc4\xde"                        // "Bed"
#define MSG_PREHEAT_ABS_SETTINGS            MSG_PREHEAT_ABS " \xbe\xaf\xc3\xb2"                            // "conf"
#define MSG_PREHEAT_GUM                     "GUM \xd6\xc8\xc2"                                             // "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 MSG_PREHEAT_GUM " \xbd\xcd\xde\xc3"                            // " All"
#define MSG_PREHEAT_GUM_BEDONLY             MSG_PREHEAT_GUM " \xcd\xde\xaf\xc4\xde"                        // "Bed"
#define MSG_PREHEAT_GUM_SETTINGS            MSG_PREHEAT_GUM " \xbe\xaf\xc3\xb2"                            // "conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "\xb6\xc8\xc2\xc3\xb2\xbc"                                     // "Cooldown"
#define MSG_SWITCH_PS_ON                    "\xc3\xde\xdd\xb9\xde\xdd\x20\xb5\xdd"                         // "Switch power on"
#define MSG_SWITCH_PS_OFF                   "\xc3\xde\xdd\xb9\xde\xdd\x20\xb5\xcc"                         // "Switch power off"
#define MSG_EXTRUDE                         "\xb5\xbc\xc0\xde\xbc"                                         // "Extrude"
#define MSG_RETRACT                         "\xd8\xc4\xd7\xb8\xc4"                                         // "Retract"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "\xbd\xcb\xdf\xb0\xc4\xde"                                     // "Speed"
#define MSG_NOZZLE                          "\xc9\xbd\xde\xd9"                                             // "Nozzle"
#define MSG_BED                             "\xcd\xde\xaf\xc4\xde"                                         // "Bed"
#define MSG_CHAMBER                         "Chamber"
#define MSG_COOLER                          "Cooler"
#define MSG_BED_Z                           "Bed Z"
#define MSG_FAN_SPEED                       "\xcc\xa7\xdd\xbf\xb8\xc4\xde"                                 // "Fan speed"
#define MSG_FLOW                            "\xb5\xb8\xd8\xd8\xae\xb3"                                     // "Flow"
#define MSG_CONTROL                         "\xba\xdd\xc4\xdb\xb0\xd9"                                     // "Control"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "\xbc\xde\xc4\xde\xb3\xb5\xdd\xc4\xde"                         // "Autotemp"
#define MSG_ON                              "ON "
#define MSG_OFF                             "OFF"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_PID_C                           "PID-C"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "\xb6\xbf\xb8\xc4\xde"                                         // "Accel"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"
#define MSG_MOVE                            "\xb2\xc4\xde\xb3"                                             // "Move"
#define MSG_MOVE_AXIS                       "\xbc\xde\xb8\xb2\xc4\xde\xb3"                                 // "Move axis"
#define MSG_MOVE_X                          "X\xbc\xde\xb8\x20\xb2\xc4\xde\xb3"                            // "Move X"
#define MSG_MOVE_Y                          "Y\xbc\xde\xb8\x20\xb2\xc4\xde\xb3"                            // "Move Y"
#define MSG_MOVE_Z                          "Z\xbc\xde\xb8\x20\xb2\xc4\xde\xb3"                            // "Move Z"
#define MSG_MOVE_01MM                       "0.1mm \xb2\xc4\xde\xb3"                                       // "Move 0.1mm"
#define MSG_MOVE_1MM                        "  1mm \xb2\xc4\xde\xb3"                                       // "Move 1mm"
#define MSG_MOVE_10MM                       " 10mm \xb2\xc4\xde\xb3"                                       // "Move 10mm"
#define MSG_MOVE_E                          "\xb4\xb8\xbd\xc4\xd9\xb0\xc0\xde\xb0"                         // "Extruder"
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
#define MSG_TEMPERATURE                     "\xb5\xdd\xc4\xde"                                             // "Temperature"
#define MSG_MOTION                          "\xb3\xba\xde\xb7\xbe\xaf\xc3\xb2"                             // "Motion"
#define MSG_FILAMENT                        "\xcc\xa8\xd7\xd2\xdd\xc4"                                     // "Filament"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD\xba\xdd\xc4\xd7\xbd\xc4"                                  // "LCD contrast"
#define MSG_STORE_EPROM                     "\xd2\xd3\xd8\xcd\xb6\xb8\xc9\xb3"                             // "Store memory"
#define MSG_LOAD_EPROM                      "\xd2\xd3\xd8\xb6\xd7\xd6\xd0\xba\xd0"                          // "Load memory"
#define MSG_RESTORE_FAILSAFE                "\xbe\xaf\xc3\xb2\xd8\xbe\xaf\xc4"                             // "Restore failsafe"
#define MSG_REFRESH                         "\xd8\xcc\xda\xaf\xbc\xad"                                     // "Refresh"
#define MSG_WATCH                           "\xb2\xdd\xcc\xab"                                             // "Info screen"
#define MSG_PREPARE                         "\xbc\xde\xad\xdd\xcb\xde\xbe\xaf\xc3\xb2"                     // "Prepare"
#define MSG_TUNE                            "\xc1\xae\xb3\xbe\xb2"                                         // "Tune"
#define MSG_PAUSE_PRINT                     "\xb2\xc1\xbc\xde\xc3\xb2\xbc"                                 // "Pause print"
#define MSG_RESUME_PRINT                    "\xcc\xdf\xd8\xdd\xc4\xbb\xb2\xb6\xb2"                         // "Resume print"
#define MSG_STOP_PRINT                      "\xcc\xdf\xd8\xdd\xc4\xc3\xb2\xbc"                             // "Stop print"
#define MSG_STOP_SAVE_PRINT                 "Stop and Save"
#define MSG_CARD_MENU                       "SD\xb6\xb0\xc4\xde\xb6\xd7\xcc\xdf\xd8\xdd\xc4"               // "Print from SD"
#define MSG_NO_CARD                         "SD\xb6\xb0\xc4\xde\xb6\xde\xb1\xd8\xcf\xbe\xdd"               // "No SD card"
#define MSG_DWELL                           "\xbd\xd8\xb0\xcc\xdf"                                         // "Sleep..."
#define MSG_USERWAIT                        "\xbc\xca\xde\xd7\xb9\xb5\xcf\xc1\xb8\xc0\xde\xbb\xb2"         // "Wait for user..."
#define MSG_RESUMING                        "\xcc\xdf\xd8\xdd\xc4\xbb\xb2\xb6\xb2"                         // "Resuming print"
#define MSG_PRINT_ABORTED                   "\xcc\xdf\xd8\xdd\xc4\xc1\xad\xb3\xbc\xbb\xda\xcf\xbc\xc0"     // "Print aborted"
#define MSG_NO_MOVE                         "\xb3\xba\xde\xb7\xcf\xbe\xdd"                                 // "No move."
#define MSG_KILLED                          "\xbc\xae\xb3\xb7\xae"                                         // "KILLED. "
#define MSG_STOPPED                         "\xc3\xb2\xbc\xbc\xcf\xbc\xc0"                                 // "STOPPED. "
#define MSG_CONTROL_RETRACT                 "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            "Swap Re.mm"
#define MSG_CONTROL_RETRACTF                "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "S UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENT_CHANGE                 "\xcc\xa8\xd7\xd2\xdd\xc4\xba\xb3\xb6\xdd"                     // "Change filament"
#define MSG_INIT_SDCARD                     "SD\xb6\xb0\xc4\xde\xbb\xb2\xd6\xd0\xba\xd0"                   // "Init. SD card"
#define MSG_CNG_SDCARD                      "SD\xb6\xb0\xc4\xde\xba\xb3\xb6\xdd"                           // "Change SD card"
#define MSG_ZPROBE_OUT                      "Z\xcc\xdf\xdb\xb0\xcc\xde \xcd\xde\xaf\xc4\xee\xb6\xde\xb2"   // "Z probe out. bed"
#define MSG_HOME                            "Home"
#define MSG_FIRST                           "first"
#define MSG_ZPROBE_ZOFFSET                  "ZProbe ZOffset"
#define MSG_BABYSTEP                        "\xcb\xde\xc4\xde\xb3"
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
#define MSG_ERR_MAXTEMP_CHAMBER             "MAXTEMP CHAMBER ERROR"
#define MSG_ERR_MINTEMP_CHAMBER             "MINTEMP CHAMBER ERROR"
#define MSG_ERR_MAXTEMP_COOLER              "MAXTEMP COOLER ERROR"
#define MSG_ERR_MINTEMP_COOLER              "MINTEMP COOLER ERROR"
#define MSG_END_DAY                         "days"
#define MSG_END_HOUR                        "hours"
#define MSG_END_MINUTE                      "minutes"

#define MSG_ENDSTOPS_HIT                    "endstops hit: "
#define MSG_BABYSTEPPING                    "Babystepping"
#define MSG_BABYSTEPPING_X                  MSG_BABYSTEPPING " " MSG_X
#define MSG_BABYSTEPPING_Y                  MSG_BABYSTEPPING " " MSG_Y
#define MSG_BABYSTEPPING_Z                  MSG_BABYSTEPPING " " MSG_Z

#define MSG_ENDSTOP_XS                      MSG_X
#define MSG_ENDSTOP_YS                      MSG_Y
#define MSG_ENDSTOP_ZS                      MSG_Z
#define MSG_ENDSTOP_ZPS                     MSG_Z "P"
#define MSG_ENDSTOP_ES                      MSG_E

// Calibrate Delta
#if MECH(DELTA)
  #define MSG_DELTA_CALIBRATE               "Delta Calibration"
  #define MSG_DELTA_CALIBRATE_X             "Calibrate " MSG_X
  #define MSG_DELTA_CALIBRATE_Y             "Calibrate " MSG_Y
  #define MSG_DELTA_CALIBRATE_Z             "Calibrate " MSG_Z
  #define MSG_DELTA_CALIBRATE_CENTER        "Calibrate Center"
#endif // DELTA

// FILAMENT_CHANGE_FEATURE
#define MSG_FILAMENT_CHANGE_HEADER          "CHANGE FILAMENT"
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
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   "WHAT NEXT?"
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE  "Extrude more"
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   "Resume print"
#define MSG_FILAMENT_CHANGE_RESUME_1        "Wait for print"
#define MSG_FILAMENT_CHANGE_RESUME_2        "resume"
#define MSG_FILAMENT_CHANGE_RESUME_3        ""

// Scara
#if MECH(SCARA)
  #define MSG_SCALE                         "Scale"
  #define MSG_XSCALE                        MSG_X " " MSG_SCALE
  #define MSG_YSCALE                        MSG_Y " " MSG_SCALE
#endif

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
#if ENABLED(RFID_MODULE)
  #define MSG_RFID_SPOOL                    "Spool on E"
  #define MSG_RFID_BRAND                    "Brand: "
  #define MSG_RFID_TYPE                     "Type: "
  #define MSG_RFID_COLOR                    "Color: "
  #define MSG_RFID_SIZE                     "Size: "
  #define MSG_RFID_TEMP_HOTEND              "Temperature Hotend: "
  #define MSG_RFID_TEMP_BED                 "Temperature Bed: "
  #define MSG_RFID_TEMP_USER_HOTEND         "User temperature Hotend: "
  #define MSG_RFID_TEMP_USER_BED            "User temperatura Bed: "
  #define MSG_RFID_DENSITY                  "Density: "
  #define MSG_RFID_SPOOL_LENGHT             "Spool Lenght: "
#endif

// Firmware Test
#if ENABLED(FIRMWARE_TEST)
  #define MSG_FWTEST_YES                    "Put the Y command to go next"
  #define MSG_FWTEST_NO                     "Put the N command to go next"
  #define MSG_FWTEST_YES_NO                 "Put the Y or N command to go next"
  #define MSG_FWTEST_ENDSTOP_ERR            "ENDSTOP ERROR! Check wire and connection"
  #define MSG_FWTEST_PRESS                  "Press and hold the endstop "
  #define MSG_FWTEST_INVERT                 "Reverse value of "
  #define MSG_FWTEST_XAXIS                  "Has the nozzle moved to the right?"
  #define MSG_FWTEST_YAXIS                  "Has the nozzle moved forward?"
  #define MSG_FWTEST_ZAXIS                  "Has the nozzle moved up?"
  #define MSG_FWTEST_01                     "Manually move the axes X, Y and Z away from the endstop"
  #define MSG_FWTEST_02                     "Do you want check ENDSTOP?"
  #define MSG_FWTEST_03                     "Start check ENDSTOP"
  #define MSG_FWTEST_04                     "Start check MOTOR"
  #define MSG_FWTEST_ATTENTION              "ATTENTION! Check that the three axes are more than 5 mm from the endstop!"
  #define MSG_FWTEST_END                    "Finish Test. Disable FIRMWARE_TEST and recompile."
  #define MSG_FWTEST_INTO                   "into "
  #define MSG_FWTEST_ERROR                  "ERROR"
  #define MSG_FWTEST_OK                     "OK"
  #define MSG_FWTEST_NDEF                   "not defined"
#endif // FIRMWARE_TEST

#endif // LANGUAGE_KANA_H
