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
 * Polish
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_PL_H
#define LANGUAGE_PL_H

#define MAPPER_NON                  // For direct asci codes
#define DISPLAY_CHARSET_ISO10646_1  // use the better font on full graphic displays.


#define WELCOME_MSG                         MACHINE_NAME " gotowy."
#define MSG_SD                              "SD"
#define MSG_SD_INSERTED                     MSG_SD " wlozona"
#define MSG_SD_REMOVED                      MSG_SD " usunieta"
#define MSG_MAIN                            "Menu glowne"
#define MSG_AUTOSTART                       "Autostart"
#define MSG_DISABLE_STEPPERS                "Wylacz silniki"
#define MSG_AUTO_HOME                       "Auto. poz. zerowa"
#define MSG_MBL_SETTING                     "Manual Bed Leveling"
#define MSG_MBL_BUTTON                      " Press the button   "
#define MSG_MBL_INTRO                       " Leveling bed...    "
#define MSG_MBL_1                           " Adjust first point "
#define MSG_MBL_2                           " Adjust second point"
#define MSG_MBL_3                           " Adjust third point "
#define MSG_MBL_4                           " Adjust fourth point"
#define MSG_MBL_5                           "    Is it ok?       "
#define MSG_MBL_6                           " BED leveled!       "
#define MSG_SET_HOME_OFFSETS                "Set home offsets"
#define MSG_SET_ORIGIN                      "Ustaw punkt zero"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"
#define MSG_FILCONSUMED                     "F:"
#define MSG_PREHEAT                         "Preheat"
#define MSG_PREHEAT_PLA                     "Rozgrzej PLA"
#define MSG_PREHEAT_PLA_ALL                 "Roz. PLA Wszystko"
#define MSG_PREHEAT_PLA_BEDONLY             "Rozgrzej PLA Loze"
#define MSG_PREHEAT_PLA_SETTINGS            "Ustaw. rozg. PLA"
#define MSG_PREHEAT_ABS                     "Rozgrzej ABS"
#define MSG_PREHEAT_ABS_ALL                 "Roz. ABS Wszystko"
#define MSG_PREHEAT_ABS_BEDONLY             "Rozgrzej ABS Loze"
#define MSG_PREHEAT_ABS_SETTINGS            "Ustaw. rozg. ABS"
#define MSG_PREHEAT_GUM                     "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 "Preheat GUM All"
#define MSG_PREHEAT_GUM_BEDONLY             "Preheat GUM Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "Preheat GUM conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "Chlodzenie"
#define MSG_SWITCH_PS_ON                    "Wlacz zasilacz"
#define MSG_SWITCH_PS_OFF                   "Wylacz zasilacz"
#define MSG_EXTRUDE                         "Ekstruzja"
#define MSG_RETRACT                         "Cofanie"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "Predkosc"
#define MSG_NOZZLE                          "Dysza"
#define MSG_BED                             "Loze"
#define MSG_FAN_SPEED                       "Obroty wiatraka"
#define MSG_FLOW                            "Przeplyw"
#define MSG_CONTROL                         "Ustawienia"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Auto. temperatura"
#define MSG_ON                              "Wl. "
#define MSG_OFF                             "Wyl."
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_PID_C                           "PID-C"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "Przyspieszenie"
#define MSG_VXY_JERK                        "Zryw Vxy"
#define MSG_VZ_JERK                         "Zryw Vz"
#define MSG_VE_JERK                         "Zryw Ve"
#define MSG_VMAX                            "Vmax"
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"
#define MSG_MOVE                            "Move"
#define MSG_MOVE_AXIS                       "Ruch osi"
#define MSG_MOVE_X                          "Przesun w " MSG_X
#define MSG_MOVE_Y                          "Przesun w " MSG_Y
#define MSG_MOVE_Z                          "Przesun w " MSG_Z
#define MSG_MOVE_01MM                       "Przesuwaj co .1mm"
#define MSG_MOVE_1MM                        "Przesuwaj co 1mm"
#define MSG_MOVE_10MM                       "Przesuwaj co 10mm"
#define MSG_MOVE_E                          "Ekstruzja"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "Vskok min"
#define MSG_AMAX                            "Amax"
#define MSG_A_RETRACT                       "A-wycofanie"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          MSG_X "roki/mm"
#define MSG_YSTEPS                          MSG_Y "roki/mm"
#define MSG_ZSTEPS                          MSG_Z "roki/mm"
#define MSG_E0STEPS                         MSG_E "0 kroki/mm"
#define MSG_E1STEPS                         MSG_E "1 kroki/mm"
#define MSG_E2STEPS                         MSG_E "2 kroki/mm"
#define MSG_E3STEPS                         MSG_E "3 kroki/mm"
#define MSG_TEMPERATURE                     "Temperatura"
#define MSG_MOTION                          "Ruch"
#define MSG_FILAMENT                        "Filament"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "Kontrast LCD"
#define MSG_STORE_EPROM                     "Zapisz w pamieci"
#define MSG_LOAD_EPROM                      "Wczytaj z pamieci"
#define MSG_RESTORE_FAILSAFE                "Ustaw. fabryczne"
#define MSG_REFRESH                         "Refresh"
#define MSG_WATCH                           "Ekran glowny"
#define MSG_PREPARE                         "Przygotuj"
#define MSG_TUNE                            "Strojenie"
#define MSG_PAUSE_PRINT                     "Pauza"
#define MSG_RESUME_PRINT                    "Wznowienie"
#define MSG_STOP_PRINT                      "Stop"
#define MSG_CARD_MENU                       "Menu " MSG_SD
#define MSG_NO_CARD                         "Brak " MSG_SD
#define MSG_DWELL                           "Uspij..."
#define MSG_USERWAIT                        "Oczekiwanie..."
#define MSG_RESUMING                        "Wznawianie druku"
#define MSG_PRINT_ABORTED                   "Print aborted"
#define MSG_NO_MOVE                         "Brak ruchu"
#define MSG_KILLED                          "Ubity. "
#define MSG_STOPPED                         "Zatrzymany. "
#define MSG_CONTROL_RETRACT                 "Wycofaj mm"
#define MSG_CONTROL_RETRACT_SWAP            "Z Wycof. mm"
#define MSG_CONTROL_RETRACTF                "Wycofaj  V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Skok Z mm:"
#define MSG_CONTROL_RETRACT_RECOVER         "Cof. wycof. +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Z Cof. wyc. +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "Cof. wycof.  V"
#define MSG_AUTORETRACT                     "Auto. wycofanie"
#define MSG_FILAMENTCHANGE                  "Zmien filament"
#define MSG_INIT_SDCARD                     "Inicjal. " MSG_SD
#define MSG_CNG_SDCARD                      "Zmiana " MSG_SD
#define MSG_ZPROBE_OUT                      "Sonda Z za lozem"
#define MSG_POSITION_UNKNOWN                "Wroc w XY przed Z"
#define MSG_ZPROBE_ZOFFSET                  "Offset Z"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Blad wyl. kranc."
#define MSG_HEATING_FAILED_LCD              "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              "REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_HOTEND_AD595                    "HOTEND AD595 Offset & Gain"
#define MSG_ERR_MAXTEMP                     "MAXTEMP ERROR"
#define MSG_ERR_MINTEMP                     "MINTEMP ERROR"
#define MSG_ERR_MAXTEMP_BED                 "MAXTEMP BED ERROR"
#define MSG_ERR_MINTEMP_BED                 "MINTEMP BED ERROR"
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
  #define MSG_RFID_COLOR                    "Color: "
  #define MSG_RFID_SIZE                     "Size: "
  #define MSG_RFID_TEMPERATURE              "Temperature: "
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

#endif // LANGUAGE_PL_H
