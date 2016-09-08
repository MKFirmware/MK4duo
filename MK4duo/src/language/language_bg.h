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
 * Bulgarian
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_BG_H
#define LANGUAGE_BG_H

#define MAPPER_D0D1                // For Cyrillic
#define DISPLAY_CHARSET_ISO10646_5


#define WELCOME_MSG                         MACHINE_NAME " Готов."
#define MSG_SD_INSERTED                     "Картата е поставена"
#define MSG_SD_REMOVED                      "Картата е извадена"
#define MSG_MAIN                            "Меню"
#define MSG_AUTOSTART                       "Автостарт"
#define MSG_DISABLE_STEPPERS                "Изкл. двигатели"
#define MSG_AUTO_HOME                       "Паркиране"
#define MSG_AUTO_HOME_X                     "Home X"
#define MSG_AUTO_HOME_Y                     "Home Y"
#define MSG_AUTO_HOME_Z                     "Home Z"
#define MSG_LEVEL_BED_HOMING                "Homing XYZ"
#define MSG_LEVEL_BED_WAITING               "Click to Begin"
#define MSG_LEVEL_BED_NEXT_POINT            "Next Point"
#define MSG_LEVEL_BED_DONE                  "Leveling Done!"
#define MSG_LEVEL_BED_CANCEL                "Cancel"
#define MSG_SET_HOME_OFFSETS                "Задай Начало"
#define MSG_SET_ORIGIN                      "Изходна точка"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"
#define MSG_FILCONSUMED                     "F:"
#define MSG_PREHEAT                         "Подгр."
#define MSG_PREHEAT_PLA                     MSG_PREHEAT " PLA"
#define MSG_PREHEAT_PLA_ALL                 MSG_PREHEAT_PLA " All"
#define MSG_PREHEAT_PLA_BEDONLY             MSG_PREHEAT_PLA " Bed"
#define MSG_PREHEAT_PLA_SETTINGS            "PLA Настр."
#define MSG_PREHEAT_ABS                     MSG_PREHEAT " ABS"
#define MSG_PREHEAT_ABS_ALL                 MSG_PREHEAT_ABS " All"
#define MSG_PREHEAT_ABS_BEDONLY             MSG_PREHEAT_ABS " Bed"
#define MSG_PREHEAT_ABS_SETTINGS            "ABS Настр."
#define MSG_PREHEAT_GUM                     MSG_PREHEAT " GUM"
#define MSG_PREHEAT_GUM_ALL                 MSG_PREHEAT_GUM " All"
#define MSG_PREHEAT_GUM_BEDONLY             MSG_PREHEAT_GUM " Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "GUM Настр." 
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "Охлаждане"
#define MSG_SWITCH_PS_ON                    "Вкл. захранване"
#define MSG_SWITCH_PS_OFF                   "Изкл. захранване"
#define MSG_EXTRUDE                         "Екструзия"
#define MSG_RETRACT                         "Откат"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Нивелиране"
#define MSG_SPEED                           "Скорост"
#define MSG_NOZZLE                          "Дюза"
#define MSG_BED                             "Легло"
#define MSG_CHAMBER                         "Chamber"
#define MSG_COOLER                          "Cooler"
#define MSG_BED_Z                           "Bed Z"
#define MSG_FAN_SPEED                       "Вентилатор"
#define MSG_FLOW                            "Поток"
#define MSG_CONTROL                         "Управление"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Минимум"
#define MSG_MAX                             LCD_STR_THERMOMETER " Максимум"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Фактор"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Авто-темп."
#define MSG_ON                              "Вкл. "
#define MSG_OFF                             "Изкл. "
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_PID_C                           "PID-C"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "Acc"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"
#define MSG_MOVE                            "Move"
#define MSG_MOVE_AXIS                       MSG_MOVE " axis"
#define MSG_MOVE_X                          "Движение по " MSG_X
#define MSG_MOVE_Y                          "Движение по " MSG_Y
#define MSG_MOVE_Z                          "Движение по " MSG_Z
#define MSG_MOVE_01MM                       "Премести с 0.1mm"
#define MSG_MOVE_1MM                        "Премести с 1mm"
#define MSG_MOVE_10MM                       "Премести с 10mm"
#define MSG_MOVE_E                          "Екструдер"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-откат"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          MSG_X " стъпки/mm"
#define MSG_YSTEPS                          MSG_Y " стъпки/mm"
#define MSG_ZSTEPS                          MSG_Z " стъпки/mm"
#define MSG_E0STEPS                         MSG_E "0 стъпки/mm"
#define MSG_E1STEPS                         MSG_E "1 стъпки/mm"
#define MSG_E2STEPS                         MSG_E "2 стъпки/mm"
#define MSG_E3STEPS                         MSG_E "3 стъпки/mm"
#define MSG_TEMPERATURE                     "Температура"
#define MSG_MOTION                          "Движение"
#define MSG_FILAMENT                        "Нишка"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Диам. нишка"
#define MSG_CONTRAST                        "LCD контраст"
#define MSG_STORE_EPROM                     "Запази в EPROM"
#define MSG_LOAD_EPROM                      "Зареди от EPROM"
#define MSG_RESTORE_FAILSAFE                "Фабрични настройки"
#define MSG_REFRESH                         LCD_STR_REFRESH "Обнови"
#define MSG_WATCH                           "Преглед"
#define MSG_PREPARE                         "Действия"
#define MSG_TUNE                            "Настройка"
#define MSG_PAUSE_PRINT                     "Пауза"
#define MSG_RESUME_PRINT                    "Възобнови печата"
#define MSG_STOP_PRINT                      "Спри печата"
#define MSG_STOP_SAVE_PRINT                 "Stop and Save"
#define MSG_CARD_MENU                       "Меню " MSG_SD
#define MSG_NO_CARD                         "Няма " MSG_SD
#define MSG_DWELL                           "Почивка..."
#define MSG_USERWAIT                        "Изчакване"
#define MSG_RESUMING                        "Продълж. печата"
#define MSG_PRINT_ABORTED                   "Печатът е прекъснат"
#define MSG_NO_MOVE                         "Няма движение"
#define MSG_KILLED                          "УБИТО."
#define MSG_STOPPED                         "СПРЯНО."
#define MSG_CONTROL_RETRACT                 "Откат mm"
#define MSG_CONTROL_RETRACT_SWAP            "Смяна Откат mm"
#define MSG_CONTROL_RETRACTF                "Откат  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Скок mm"
#define MSG_CONTROL_RETRACT_RECOVER         "Възврат +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Смяна Възврат +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "Възврат  F"
#define MSG_AUTORETRACT                     "Автоoткат"
#define MSG_FILAMENT_CHANGE                 "Смяна нишка"
#define MSG_INIT_SDCARD                     "Иниц. " MSG_SD
#define MSG_CNG_SDCARD                      "Смяна " MSG_SD
#define MSG_ZPROBE_OUT                      "Z-сондата е извадена"
#define MSG_HOME                            "Home"
#define MSG_FIRST                           "first"
#define MSG_ZPROBE_ZOFFSET                  "Z Отстояние"
#define MSG_BABYSTEP                        "Министъпка"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Стоп Кр.Изключватели"
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
#define MSG_END_HOUR                        "часа"
#define MSG_END_MINUTE                      "минути"

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
  #define MSG_DELTA_CALIBRATE               "Делта Калибровка"
  #define MSG_DELTA_CALIBRATE_X             "Калибровка " MSG_X
  #define MSG_DELTA_CALIBRATE_Y             "Калибровка " MSG_Y
  #define MSG_DELTA_CALIBRATE_Z             "Калибровка " MSG_Z
  #define MSG_DELTA_CALIBRATE_CENTER        "Калибровка Център"
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

#endif // LANGUAGE_BG_H
