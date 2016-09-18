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
 * German
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_DE_H
#define LANGUAGE_DE_H

#define MAPPER_C2C3
#define DISPLAY_CHARSET_ISO10646_1


#define WELCOME_MSG                         MACHINE_NAME " Bereit."
#define MSG_SD                              "SDKarte"
#define MSG_SD_INSERTED                     MSG_SD " erkannt."
#define MSG_SD_REMOVED                      MSG_SD " entfernt."
#define MSG_MAIN                            "Hauptmenü"
#define MSG_AUTOSTART                       "Autostart"
#define MSG_DISABLE_STEPPERS                "Motoren Aus"
#define MSG_AUTO_HOME                       "Auto home"
#define MSG_AUTO_HOME_X                     "Home X"
#define MSG_AUTO_HOME_Y                     "Home Y"
#define MSG_AUTO_HOME_Z                     "Home Z"
#define MSG_LEVEL_BED_HOMING                "Homing XYZ"
#define MSG_LEVEL_BED_WAITING               "Click to Begin"
#define MSG_LEVEL_BED_NEXT_POINT            "Next Point"
#define MSG_LEVEL_BED_DONE                  "Leveling Done!"
#define MSG_LEVEL_BED_CANCEL                "Cancel"
#define MSG_SET_HOME_OFFSETS                "Setze Home hier"
#define MSG_SET_ORIGIN                      "Setze Null hier"
#define MSG_PREHEAT                         "Vorwärmen"
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
#define MSG_COOLDOWN                        "Abkühlen"
#define MSG_SWITCH_PS_ON                    "Netzteil Ein"
#define MSG_SWITCH_PS_OFF                   "Netzteil Aus"
#define MSG_EXTRUDE                         "Extrude"
#define MSG_RETRACT                         "Retract"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "Geschw."
#define MSG_NOZZLE                          "Düse"
#define MSG_BED                             "Bett"
#define MSG_CHAMBER                         "Chamber"
#define MSG_COOLER                          "Cooler"
#define MSG_BED_Z                           "Bed Z"
#define MSG_FAN_SPEED                       "Lüftergeschw."
#define MSG_FLOW                            "Fluss"
#define MSG_CONTROL                         "Einstellungen"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Faktor"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "AutoTemp"
#define MSG_ON                              "Ein"
#define MSG_OFF                             "Aus"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_PID_C                           "PID-C"
#define MSG_SELECT                          "Select"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "Accel"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"
#define MSG_MOVE                            "Bewegen"
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
#define MSG_TEMPERATURE                     "Temperatur"
#define MSG_MOTION                          "Bewegung"
#define MSG_FILAMENT                        "Filament"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD contrast"
#define MSG_STORE_EPROM                     "EPROM speichern"
#define MSG_LOAD_EPROM                      "EPROM laden"
#define MSG_RESTORE_FAILSAFE                "Standardkonfig."
#define MSG_REFRESH                         "Aktualisieren"
#define MSG_WATCH                           "Info"
#define MSG_PREPARE                         "Vorbereitung"
#define MSG_TUNE                            "Justierung"
#define MSG_PAUSE_PRINT                     "SD-Druck Pause"
#define MSG_RESUME_PRINT                    "SD-Druck Weiter"
#define MSG_STOP_PRINT                      "SD-Druck Abbruch"
#define MSG_STOP_SAVE_PRINT                 "Stop and Save"
#define MSG_CARD_MENU                       "SDKarte"
#define MSG_NO_CARD                         "Keine SDKarte"
#define MSG_DWELL                           "Warten..."
#define MSG_USERWAIT                        "Warte auf Nutzer."
#define MSG_RESUMING                        "Druck geht weiter"
#define MSG_PRINT_ABORTED                   "Druck abgebrochen"
#define MSG_NO_MOVE                         "Motoren Eingesch."
#define MSG_KILLED                          "KILLED."
#define MSG_STOPPED                         "ANGEHALTEN."
#define MSG_CONTROL_RETRACT                 "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            "Wechs. Retract mm"
#define MSG_CONTROL_RETRACTF                "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Wechs. UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENT_CHANGE                 "Filament wechseln"
#define MSG_INIT_SDCARD                     "SDKarte erkennen"
#define MSG_CNG_SDCARD                      "SDKarte erkennen"
#define MSG_ZPROBE_OUT                      "Sensor ausserhalb"
#define MSG_HOME                            "Home"
#define MSG_FIRST                           "first"
#define MSG_ZPROBE_ZOFFSET                  "ZProbe ZOffset"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Endstop Abbr. Ein"
#define MSG_HEATING_FAILED_LCD              "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              "Err: REDUNDANT TEMP ERROR"
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
#define MSG_END_DAY                         "tage"
#define MSG_END_HOUR                        "uur"
#define MSG_END_MINUTE                      "minuten"
#define MSG_PRINT_TIME                      "Print time "

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
#endif

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

// Firmware Test
#define MSG_FWTEST_YES                      "Put the Y command to go next"
#define MSG_FWTEST_NO                       "Put the N command to go next"
#define MSG_FWTEST_YES_NO                   "Put the Y or N command to go next"
#define MSG_FWTEST_ENDSTOP_ERR              "ENDSTOP ERROR! Check wire and connection"
#define MSG_FWTEST_PRESS                    "Press and hold the endstop "
#define MSG_FWTEST_INVERT                   "Reverse value of "
#define MSG_FWTEST_XAXIS                    "Has the nozzle moved to the right?"
#define MSG_FWTEST_YAXIS                    "Has the nozzle moved forward?"
#define MSG_FWTEST_ZAXIS                    "Has the nozzle moved up?"
#define MSG_FWTEST_01                       "Manually move the axes X, Y and Z away from the endstop"
#define MSG_FWTEST_02                       "Do you want check ENDSTOP?"
#define MSG_FWTEST_03                       "Start check ENDSTOP"
#define MSG_FWTEST_04                       "Start check MOTOR"
#define MSG_FWTEST_ATTENTION                "ATTENTION! Check that the three axes are more than 5 mm from the endstop!"
#define MSG_FWTEST_END                      "Finish Test. Disable FIRMWARE_TEST and recompile."
#define MSG_FWTEST_INTO                     "into "
#define MSG_FWTEST_ERROR                    "ERROR"
#define MSG_FWTEST_OK                       "OK"
#define MSG_FWTEST_NDEF                     "not defined"

#endif // LANGUAGE_DE_H
