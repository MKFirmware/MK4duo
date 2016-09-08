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
 * Japanese (Kana UTF8 version)
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_KANA_UTF_H
#define LANGUAGE_KANA_UTF_H

#define MAPPER_E382E383
#define DISPLAY_CHARSET_ISO10646_KANA
// This is very crude replacement of the codes used in language_kana.h from somebody who really does not know what he is doing.
// Just to show the potential benefit of unicode. 
// This translation can be improved by using the full charset of unicode codeblock U+30A0 to U+30FF.
// 片仮名表示定義



#define WELCOME_MSG                         MACHINE_NAME " ready."
#define MSG_SD                              "SD"
#define MSG_SD_INSERTED                     "カード ソウニュウサレマシタ"          // "Card inserted"
#define MSG_SD_REMOVED                      "カードガアリマセン"               // "Card removed"
#define MSG_MAIN                            "メイン"                        // "Main"
#define MSG_AUTOSTART                       "ジドウカイシ"                   // "Autostart"
#define MSG_DISABLE_STEPPERS                "モーターデンゲン オフ"             // "Disable steppers"
#define MSG_AUTO_HOME                       "ゲンテンニイドウ"                // "Auto home"
#define MSG_MBL_SETTING                     "Manual Bed Leveling"
#define MSG_MBL_BUTTON                      " Press the button   "
#define MSG_MBL_INTRO                       " Leveling bed...    "
#define MSG_MBL_1                           " Adjust first point "
#define MSG_MBL_2                           " Adjust second point"
#define MSG_MBL_3                           " Adjust third point "
#define MSG_MBL_4                           " Adjust fourth point"
#define MSG_MBL_5                           "    Is it ok?       "
#define MSG_MBL_6                           " BED leveled!       "
#define MSG_SET_HOME_OFFSETS                "キジュンオフセットセッテイ"         // "Set home offsets"
#define MSG_SET_ORIGIN                      "キジュンセット"                 // "Set origin"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"
#define MSG_FILCONSUMED                     "F:"
#define MSG_PREHEAT                         "Preheat"
#define MSG_PREHEAT_PLA                     "PLA ヨネツ"                    // "Preheat PLA"
#define MSG_PREHEAT_PLA_ALL                 MSG_PREHEAT_PLA " スベテ"      // " All"
#define MSG_PREHEAT_PLA_BEDONLY             MSG_PREHEAT_PLA " ベッド"    // "Bed"
#define MSG_PREHEAT_PLA_SETTINGS            MSG_PREHEAT_PLA " セッテイ"     // "conf"
#define MSG_PREHEAT_ABS                     "ABS ヨネツ"                    // "Preheat ABS"
#define MSG_PREHEAT_ABS_ALL                 MSG_PREHEAT_ABS " スベテ"      // " All"
#define MSG_PREHEAT_ABS_BEDONLY             MSG_PREHEAT_ABS " ベッド"    // "Bed"
#define MSG_PREHEAT_ABS_SETTINGS            MSG_PREHEAT_ABS " セッテイ"    // "conf"
#define MSG_PREHEAT_GUM                     "GUM ヨネツ"                    // "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 MSG_PREHEAT_GUM " スベテ"      // " All"
#define MSG_PREHEAT_GUM_BEDONLY             MSG_PREHEAT_GUM " ベッド"    // "Bed"
#define MSG_PREHEAT_GUM_SETTINGS            MSG_PREHEAT_GUM " セッテイ"    // "conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "カネツテイシ"                    // "Cooldown"
#define MSG_SWITCH_PS_ON                    "デンゲン オン"                 // "Switch power on"
#define MSG_SWITCH_PS_OFF                   "デンゲン オフ"                 // "Switch power off"
#define MSG_EXTRUDE                         "オシダシ"                     // "Extrude"
#define MSG_RETRACT                         "リトラクト"                     // "Retract"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "スピード"                     // "Speed"
#define MSG_NOZZLE                          "ノズル"                       // "Nozzle"
#define MSG_BED                             "ベッド"                     // "Bed"
#define MSG_FAN_SPEED                       "ファンソクド"                    // "Fan speed"
#define MSG_FLOW                            "オクリリョウ"                     // "Flow"
#define MSG_CONTROL                         "コントロール"                    // "Control"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "ジドウオンド"                  // "Autotemp"
#define MSG_ON                              "ON "
#define MSG_OFF                             "OFF"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_PID_C                           "PID-C"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "カソクド"                     // "Accel"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"
#define MSG_MOVE                            "イドウ"                      // "Move"
#define MSG_MOVE_AXIS                       "ジクイドウ"                   // "Move axis"
#define MSG_MOVE_X                          "Xジク イドウ"                 // "Move X"
#define MSG_MOVE_Y                          "Yジク イドウ"                 // "Move Y"
#define MSG_MOVE_Z                          "Zジク イドウ"                 // "Move Z"
#define MSG_MOVE_01MM                       "0.1mm イドウ"                 // "Move 0.1mm"
#define MSG_MOVE_1MM                        "  1mm イドウ"                 // "Move 1mm"
#define MSG_MOVE_10MM                       " 10mm イドウ"                 // "Move 10mm"
#define MSG_MOVE_E                          "エクストルーダー"                // "Extruder"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "Xsteps/mm"
#define MSG_YSTEPS                          "Ysteps/mm"
#define MSG_ZSTEPS                          "Zsteps/mm"
#define MSG_E0STEPS                         "E0steps/mm"
#define MSG_E1STEPS                         "E1steps/mm"
#define MSG_E2STEPS                         "E2steps/mm"
#define MSG_E3STEPS                         "E3steps/mm"
#define MSG_TEMPERATURE                     "オンド"                      // "Temperature"
#define MSG_MOTION                          "ウゴキセッテイ"                // "Motion"
#define MSG_FILAMENT                        "フィラメント"                    // "Filament"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCDコントラスト"                 // "LCD contrast"
#define MSG_STORE_EPROM                     "メモリヘカクノウ"                 // "Store memory"
#define MSG_LOAD_EPROM                      "メモリカラヨミコミ"               // "Load memory"
#define MSG_RESTORE_FAILSAFE                "セッテイリセット"               // "Restore failsafe"
#define MSG_REFRESH                         "リフレッシュ"                  // "Refresh"
#define MSG_WATCH                           "インフォ"                     // "Info screen"
#define MSG_PREPARE                         "ジュンビセッテイ"             //"Prepare"
#define MSG_TUNE                            "チョウセイ"                    // "Tune"
#define MSG_PAUSE_PRINT                     "イチジテイシ"                  // "Pause print"
#define MSG_RESUME_PRINT                    "プリントサイカイ"                // "Resume print"
#define MSG_STOP_PRINT                      "プリントテイシ"                 // "Stop print"
#define MSG_CARD_MENU                       "SDカードカラプリント"            // "Print from SD"
#define MSG_NO_CARD                         "SDカードガアリマセン"            // "No SD card"
#define MSG_DWELL                           "スリープ"                     // "Sleep..."
#define MSG_USERWAIT                        "シバラクオマチクダサイ"           // "Wait for user..."
#define MSG_RESUMING                        "プリントサイカイ"                // "Resuming print"
#define MSG_PRINT_ABORTED                   "プリントチュウシサレマシタ"          // "Print aborted"
#define MSG_NO_MOVE                         "ウゴキマセン"                  // "No move."
#define MSG_KILLED                          "ショウキョ"                     // "KILLED. "
#define MSG_STOPPED                         "テイシシマシタ"                  // "STOPPED. "
#define MSG_CONTROL_RETRACT                 "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            "Swap Re.mm"
#define MSG_CONTROL_RETRACTF                "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "S UnRet+mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENTCHANGE                  "フィラメントコウカン"               // "Change filament"
#define MSG_INIT_SDCARD                     "SDカードサイヨミコミ"              // "Init. SD card"
#define MSG_CNG_SDCARD                      "SDカードコウカン"                // "Change SD card"
#define MSG_ZPROBE_OUT                      "Zプローブ ベッドガイ"         // "Z probe out. bed"
#define MSG_POSITION_UNKNOWN                "ゲンテンハXYイドウゴZ"           // "Home X/Y before Z"
#define MSG_ZPROBE_ZOFFSET                  "Zオフセット"                   // "Z Offset"
#define MSG_BABYSTEP                        "ビドウ"
#define MSG_BABYSTEP_X                      "ビドウ X"                    // "Babystep X"
#define MSG_BABYSTEP_Y                      "ビドウ Y"                    // "Babystep Y"
#define MSG_BABYSTEP_Z                      "ビドウ Z"                    // "Babystep Z"
#define MSG_ENDSTOP_ABORT                   "Endstop abort"
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

#endif // LANGUAGE_KANA_UTF_H
