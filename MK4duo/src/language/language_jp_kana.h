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
 * Japanese (Kana)
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 *
 */

//#define DISPLAY_CHARSET_ISO10646_KANA

namespace language_jp_kana {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 3;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Japanese"));

  // This is just to show the potential benefit of Unicode.
  // This translation can be improved by using the full charset of unicode codeblock U+30A0 to U+30FF.

  // 片仮名表示定義
  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" ジュンビカンリョウ"));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("メディアガソウニュウサレマシタ"));        // "Card inserted"
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("メディアガアリマセン"));               // "Card removed"
  FSTRINGVALUE(MSG_RELEASE_MEDIA                    , _UxGT("メディアノトリダシ"));
  FSTRINGVALUE(MSG_MEDIA_RELEASED                   , _UxGT("メディアガアリマセン"));               // "Card removed"
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("エンドストップ"));                  // "Endstops" // Max length 8 characters
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("メイン"));                       // "Main"
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("ジドウカイシ"));                   // "Autostart"
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("モーターデンゲン オフ"));            // "Disable steppers"
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("デバッグメニュー"));                // "Debug Menu"
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("プログレスバー テスト"));            // "Progress Bar Test"
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("ゲンテンフッキ"));                  // "Auto home"
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Xジク ゲンテンフッキ"));             // "Home X"
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Yジク ゲンテンフッキ"));             // "Home Y"
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Zジク ゲンテンフッキ"));             // "Home Z"
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("ゲンテンフッキチュウ"));              // "Homing XYZ"
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("レベリングカイシ"));                // "Click to Begin"
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("ツギノソクテイテンヘ"));             // "Next Point"
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("レベリングカンリョウ"));              // "Leveling Done!"
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("キジュンオフセットセッテイ"));         // "Set home offsets"
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("オフセットガテキヨウサレマシタ"));       // "Offsets applied"
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("キジュンセット"));                 // "Set origin"
  FSTRINGVALUE(MSG_PREHEAT_1                        , PREHEAT_1_LABEL _UxGT(" ヨネツ"));       // "Preheat " PREHEAT_1_LABEL
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , PREHEAT_1_LABEL _UxGT(" ヨネツ ~"));       // "Preheat " PREHEAT_1_LABEL
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , PREHEAT_1_LABEL _UxGT(" ヨネツノズル"));  // " Nozzle"
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , PREHEAT_1_LABEL _UxGT(" ヨネツノズル ~"));  // " Nozzle"
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , PREHEAT_1_LABEL _UxGT(" スベテヨネツ"));  // " All"
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , PREHEAT_1_LABEL _UxGT(" ベッドヨネツ"));  // " Bed"
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , PREHEAT_1_LABEL _UxGT(" ヨネツセッテイ"));  // " conf"
  FSTRINGVALUE(MSG_PREHEAT_2                        , PREHEAT_2_LABEL _UxGT(" ヨネツ"));       // "Preheat " PREHEAT_1_LABEL
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , PREHEAT_2_LABEL _UxGT(" ヨネツ ~"));       // "Preheat " PREHEAT_1_LABEL
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , PREHEAT_2_LABEL _UxGT(" ヨネツノズル"));  // " Nozzle"
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , PREHEAT_2_LABEL _UxGT(" ヨネツノズル ~"));  // " Nozzle"
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , PREHEAT_2_LABEL _UxGT(" スベテヨネツ"));  // " All"
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , PREHEAT_2_LABEL _UxGT(" ベッドヨネツ"));  // " Bed"
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , PREHEAT_2_LABEL _UxGT(" ヨネツセッテイ"));  // " conf"
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("カネツテイシ"));                  // "Cooldown"
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("デンゲン オン"));                 // "Switch power on"
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("デンゲン オフ"));                 // "Switch power off"
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("オシダシ"));                     // "Extrude"
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("ヒキコミセッテイ"));                // "Retract"
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("ジクイドウ"));                    // "Move axis"
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("ベッドレベリング"));                // "Bed leveling"
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("ベッドレベリング"));                // "Level bed"

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("イドウチュウ"));                   // "Moving..."
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("XYジク カイホウ"));                // "Free XY"
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Xジク イドウ"));                  // "Move X"
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Yジク イドウ"));                  // "Move Y"
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Zジク イドウ"));                  // "Move Z"
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("エクストルーダー"));                // "Extruder"
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("%smm イドウ"));                  // "Move 0.025mm"
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("0.1mm イドウ"));                 // "Move 0.1mm"
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("  1mm イドウ"));                 // "Move 1mm"
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT(" 10mm イドウ"));                 // "Move 10mm"
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("ソクド"));                       // "Speed"
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Zオフセット"));                   // "Bed Z"
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("ノズル"));                       // "Nozzle"
  FSTRINGVALUE(MSG_BED                              , _UxGT("ベッド"));                       // "Bed"
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("ファンソクド"));                    // "Fan speed"
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("トシュツリョウ"));                   // "Flow"
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("セイギョ"));                      // "Control"
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" サイテイ")); // " Min"
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" サイコウ")); // " Max"
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" ファクター")); // " Fact"
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("ジドウオンドセイギョ"));               // "Autotemp"
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("オン"));                         // "On"
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("オフ"));                         // "Off"
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID-P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID-I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID-D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID-C *"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("センタク"));                     // "Select"
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("センタク *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("カソクド mm/s²"));               // "Accel"
  FSTRINGVALUE(MSG_JERK                             , _UxGT("ヤクドウ mm/s"));                  // "Jerk"
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("ジク ヤクドウ mm/s") LCD_STR_A);             // "Va-jerk"
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("ジク ヤクドウ mm/s") LCD_STR_B);             // "Vb-jerk"
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("ジク ヤクドウ mm/s") LCD_STR_C);             // "Vc-jerk"
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT("ステップ/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT("ステップ/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT("ステップ/mm"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("エクストルーダー ヤクド"));          // "Ve-jerk"
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("サイダイオクリソクド ") LCD_STR_A);  // "Vmax A"
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("サイダイオクリソクド ") LCD_STR_A);  // "Vmax B"
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("サイダイオクリソクド ") LCD_STR_A);  // "Vmax C"
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("サイダイオクリソクド ") LCD_STR_A);  // "Vmax E"
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("サイダイオクリソクド *"));  // "Vmax E1"
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("サイショウオクリソクド"));           // "Vmin"
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("サイショウイドウソクド"));           // "VTrav min"
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("カソクド mm/s²"));               // "Accel"
  FSTRINGVALUE(MSG_AMAX                             , _UxGT("サイダイカソクド "));              // "Amax "
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("ヒキコミカソクド"));               // "A-retract"
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("イドウカソクド"));                // "A-travel"
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("オンド"));                      // "Temperature"
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("ウゴキセッテイ"));                // "Motion"
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("フィラメント"));                   // "Filament"
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm³"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("フィラメントチョッケイ"));            // "Fil. Dia."
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("フィラメントチョッケイ *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCDコントラスト"));               // "LCD contrast"
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("EEPROMヘホゾン"));               // "Store memory"
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("EEPROMカラヨミコミ"));               // "Load memory"
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("セッテイリセット"));               // "Restore failsafe"
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("リフレッシュ"));                  // "Refresh"
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("ジョウホウガメン"));               // "Info screen"
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("ジュンビセッテイ"));               // "Prepare"
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("チョウセイ"));                    // "Tune"
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("イチジテイシ"));                  // "Pause print"
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("プリントサイカイ"));                // "Resume print"
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("プリントテイシ"));                 // "Stop print"
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("メディアカラプリント"));            // "Print from SD"
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("メディアガアリマセン"));               // "Card removed"
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("キュウシ"));                     // "Sleep..."
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("ツヅケルニハクリックシテクダサイ"));  // "Wait for user..."
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("プリントガチュウシサレマシタ"));       // "Print aborted"
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("ウゴキマセン"));                  // "No move."
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("ヒジョウテイシ"));                  // "KILLED. "
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("テイシシマシタ"));                  // "STOPPED. "
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("ヒキコミリョウ mm"));                // "Retract mm"
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("ヒキコミリョウS mm"));               // "Swap Re.mm"
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("ヒキコミソクド mm/s"));             // "Retract  V"
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("ノズルタイヒ mm"));                // "Hop mm"
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("ホショウリョウ mm"));               // "UnRet mm"
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("ホショウリョウS mm"));              // "S UnRet mm"
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("ホショウソクド mm/s"));            // "UnRet  V"
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("ジドウヒキコミ"));                 // "AutoRetr."
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("フィラメントコウカン"));              // "Change filament"
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("メディアサイヨミコミ"));             // "Init. SD card"
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("メディアコウカン"));               // "Change SD card"
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Zプローブ ベッドガイ"));            // "Z probe out. bed"
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch ジコシンダン"));          // "BLTouch Self-Test"
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("BLTouch リセット"));             // "Reset BLTouch"
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("サキニ %s%s%s ヲフッキサセテクダサイ")); // "Home ... first"
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Zオフセット"));                   // "Z Offset"
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Xジク ビドウ"));                  // "Babystep X"
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Yジク ビドウ"));                  // "Babystep Y"
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Zジク ビドウ"));                  // "Babystep Z"
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("イドウゲンカイケンチキノウ"));         // "Endstop abort"
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("カネツシッパイ"));                 // "Heating failed"
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("エラー:ジョウチョウサーミスターキノウ"));  // "Err: REDUNDANT TEMP"
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("ネツボウソウ"));                   // "THERMAL RUNAWAY"
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("エラー:サイコウオンチョウカ"));         // "Err: MAXTEMP"
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("エラー:サイテイオンミマン"));          // "Err: MINTEMP"
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("エラー:ベッド サイコウオンチョウカ"));    // "Err: MAXTEMP BED"
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("エラー:ベッド サイテイオンミマン"));     // "Err: MINTEMP BED"
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("サキニ XY ヲフッキサセテクダサイ"));     // "Home XY first"
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("プリンターハテイシシマシタ"));         // "PRINTER HALTED"
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("リセットシテクダサイ"));              // "Please reset"
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d"));                          // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h"));                          // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m"));                          // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("カネツチュウ"));                   // "Heating..."
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("ベッド カネツチュウ"));              // "Bed Heating..."
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("デルタ コウセイ"));                // "Delta Calibration"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Xジク コウセイ"));                 // "Calibrate X"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Yジク コウセイ"));                 // "Calibrate Y"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Zジク コウセイ"));                 // "Calibrate Z"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("チュウシン コウセイ"));              // "Calibrate Center"
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("コノプリンターニツイテ"));             // "About Printer"
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("プリンタージョウホウ"));              // "Printer Info"
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("プリントジョウキョウ"));              // "Printer Stats"
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("セイギョケイジョウホウ"));            // "Board Info"
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("サーミスター"));                   // "Thermistors"
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("エクストルーダースウ"));             // "Extruders"
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("ボーレート"));                    // "Baud"
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("プロトコル"));                    // "Protocol"
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("キョウタイナイショウメイ"));       // "Case light"
  FSTRINGVALUE(MSG_INFO_PRINT_COUNT                 , _UxGT("プリントスウ "));                  // "Print Count"
  FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS            , _UxGT("カンリョウスウ"));                  // "Completed"
  FSTRINGVALUE(MSG_INFO_PRINT_TIME                  , _UxGT("プリントジカンルイケイ"));            // "Total print time"
  FSTRINGVALUE(MSG_INFO_PRINT_LONGEST               , _UxGT("サイチョウプリントジカン"));           // "Longest job time"
  FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT              , _UxGT("フィラメントシヨウリョウルイケイ"));       // "Extruded total"
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("セッテイサイテイオン"));              // "Min Temp"
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("セッテイサイコウオン"));              // "Max Temp"
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("デンゲンシュベツ"));                // "Power Supply"
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("モータークドウリョク"));              // "Drive Strength"
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("DACシュツリョク %"));               // "Driver %"
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("EEPROMヘホゾン"));               // "Store memory"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("イチジテイシ"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("プリントサイカイ"));                // "Resume print"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT             , _UxGT(MSG_2_LINE("コウカンヲカイシシマス", "シバラクオマチクダサイ")));   // "Wait for start of the filament"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD           , _UxGT(MSG_2_LINE("フィラメントヌキダシチュウ", "シバラクオマチクダサイ")));   // "Wait for filament unload"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT           , _UxGT(MSG_2_LINE("フィラメントヲソウニュウシ,", "クリックスルトゾッコウシマス")));   // "Insert filament and press button"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD             , _UxGT(MSG_2_LINE("フィラメントソウテンチュウ", "シバラクオマチクダサイ")));   // "Wait for filament load"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME           , _UxGT(MSG_2_LINE("プリントヲサイカイシマス", "シバラクオマチクダサイ")));   // "Wait for print to resume"

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("マチガッタプリンター"));               // "Wrong printer"

  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("セッテイカンリ"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("ショウサイセッテイ"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("コショカイフク"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("EEPROMショキカ"));

  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("ツギヘ"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("ショキカ"));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("ストップ"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("プリント"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("リセット"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("キャンセル"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("カンリョウ"));

  FSTRINGVALUE(MSG_YES                              , _UxGT("ハイ"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("イイエ"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("モドリ"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("ソクド"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("ステップ/mm"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("ユーザーコマンド"));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("プリントガイチジテイシサレマシタ"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("プリントチュウ..."));
}
