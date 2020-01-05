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
 * Traditional Chinese
 *
 * LCD Menu Messages
 *
 */

namespace language_zh_TW {
  using namespace language_en;  // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 3;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Traditional Chinese"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT("已就緒."));     //" ready."
  FSTRINGVALUE(MSG_BACK                             , _UxGT("返回"));     // ”Back“
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("記憶卡已插入"));     //"Card inserted"
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("記憶卡被拔出"));     //"Card removed"
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("擋塊"));     //"Endstops" // Max length 8 characters
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("主選單"));     //"Main"
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("自動開始"));     //"Autostart"
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("關閉步進馬達"));     //"Disable steppers"
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("除錯選單"));     // "Debug Menu"
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("進度條測試"));     // "Progress Bar Test"
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("自動回原點"));     //"Auto home"
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("回X原點"));     //"Home X"
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("回Y原點"));     //"Home Y"
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("回Z原點"));     //"Home Z"
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("平台調平XYZ歸原點"));     //"Homing XYZ"
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("單擊開始熱床調平"));     //"Click to Begin"
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("下個熱床調平點"));     //"Next Point"
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("完成熱床調平"));     //"Leveling Done!"
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("淡出高度"));     // "Fade Height"
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("設置原點偏移"));     //"Set home offsets"
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("偏移已啟用"));     //"Offsets applied"
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("設置原點"));     //"Set origin"
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("預熱 ") PREHEAT_1_LABEL);     //"Preheat PREHEAT_1_LABEL"
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("預熱 ") PREHEAT_1_LABEL " ~");     //"Preheat PREHEAT_1_LABEL"
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("預熱 ") PREHEAT_1_LABEL _UxGT(" 噴嘴"));     //MSG_PREHEAT_1 " "
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("預熱 ") PREHEAT_1_LABEL _UxGT(" 噴嘴 ~"));     //MSG_PREHEAT_1 " "
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("預熱 ") PREHEAT_1_LABEL _UxGT(" 全部"));     //MSG_PREHEAT_1 " All"
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("預熱 ") PREHEAT_1_LABEL _UxGT(" 熱床"));     //MSG_PREHEAT_1 " Bed"
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("預熱 ") PREHEAT_1_LABEL _UxGT(" 設置"));     //MSG_PREHEAT_1 " conf"
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("預熱 ") PREHEAT_2_LABEL);     //"Preheat PREHEAT_2_LABEL"
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("預熱 ") PREHEAT_2_LABEL " ~");     //"Preheat PREHEAT_2_LABEL"
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("預熱 ") PREHEAT_2_LABEL _UxGT(" 噴嘴"));     //MSG_PREHEAT_2 " "
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("預熱 ") PREHEAT_2_LABEL _UxGT(" 噴嘴 ~"));     //MSG_PREHEAT_2 " "
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("預熱 ") PREHEAT_2_LABEL _UxGT(" 全部"));     //MSG_PREHEAT_2 " All"
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("預熱 ") PREHEAT_2_LABEL _UxGT(" 熱床"));     //MSG_PREHEAT_2 " Bed"
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("預熱 ") PREHEAT_2_LABEL _UxGT(" 設置"));     //MSG_PREHEAT_2 " conf"
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("降溫"));     //"Cooldown"
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("電源打開"));     //"Switch power on"
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("電源關閉"));     //"Switch power off"
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("擠出"));     //"Extrude"
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("回縮"));     //"Retract"
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("移動軸"));     //"Move axis"
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("調平熱床"));     //"Bed leveling"
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("調平熱床"));     //"Level bed"
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("調平邊角"));     // "Level corners"

  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("下個邊角"));     // "Next corner"
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("編輯網格"));     // "Edit Mesh"
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("網格編輯已停止"));     // "Mesh Editing Stopped"
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("客制命令"));     // "Custom Commands"

  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("執行G29"));     // "Doing G29"
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("UBL工具"));     // "UBL Tools"
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("統一熱床調平(UBL)"));     // "Unified Bed Leveling"
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("手工建網"));     // "Manually Build Mesh"
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("放置墊片並測量"));     // "Place shim & measure"
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("測量"));     // "Measure"
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("移除並測量熱床"));     // "Remove & measure bed"
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("移動到下一個"));     // "Moving to next"
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("啟動UBL"));     // "Activate UBL"
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("關閉UBL"));     // "Deactivate UBL"
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("設置熱床溫度"));     // "Bed Temp"
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("熱端溫度"));     // "Hotend Temp"
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("網格編輯"));     // "Mesh Edit"
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("編輯客戶網格"));     // "Edit Custom Mesh"
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("細調網格"));     // "Fine Tuning Mesh"
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("完成編輯網格"));     // "Done Editing Mesh"
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("創設客戶網格"));     // "Build Custom Mesh"
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("創設網格"));     // "Build Mesh"
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("創設 ") PREHEAT_1_LABEL _UxGT(" 網格"));     // "Build PREHEAT_1_LABEL Mesh"
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("創設 ") PREHEAT_2_LABEL _UxGT(" 網格"));     // "Build PREHEAT_2_LABEL Mesh"
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("創設冷網格"));     // "Build Cold Mesh"
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("調整網格高度"));     // "Adjust Mesh Height"
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("高度合計"));     // "Height Amount"
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("批准網格"));     // "Validate Mesh"
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("批准 ") PREHEAT_1_LABEL _UxGT(" 網格"));     // "Validate PREHEAT_1_LABEL Mesh"
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("批准 ") PREHEAT_2_LABEL _UxGT(" 網格"));     // "Validate PREHEAT_2_LABEL Mesh"
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("批准客戶網格"));     // "Validate Custom Mesh"
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("繼續熱床網格"));     // "Continue Bed Mesh"
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("網格調平"));     // "Mesh Leveling"
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("三點調平"));     // "3-Point Leveling"
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("格子網格調平"));     // "Grid Mesh Leveling"
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("調平網格"));     // "Level Mesh"
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("邊點"));     // "Side Points"
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("圖類型"));     // "Map Type"
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("輸出網格圖"));     // "Output Mesh Map"
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("輸出到主機"));     // "Output for Host"
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("輸出到CSV"));     // "Output for CSV"
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("輸出到備份"));     // "Off Printer Backup"
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("輸出UBL信息"));     // "Output UBL Info"
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("填充合計"));     // "Fill-in Amount"
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("手工填充"));     // "Manual Fill-in"
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("聰明填充"));     // "Smart Fill-in"
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("填充網格"));     // "Fill-in Mesh"
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("作廢所有的"));     // "Invalidate All"
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("作廢最近的"));     // "Invalidate Closest"
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("細調所有的"));     // "Fine Tune All"
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("細調最近的"));     // "Fine Tune Closest"
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("網格存儲"));     // "Mesh Storage"
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("存儲槽"));     // "Memory Slot"
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("裝載熱床網格"));     // "Load Bed Mesh"
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("保存熱床網格"));     // "Save Bed Mesh"
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 網格 %i 已裝載"));     // "Mesh %i loaded"
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 網格 %i 已保存"));     // "Mesh %i saved"
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("沒有存儲"));     // "No storage"
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("錯誤: UBL保存"));     // "Err: UBL Save"
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("錯誤: UBL還原"));     // "Err: UBL Restore"
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Z偏移已停止"));     // "Z-Offset Stopped"
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("一步步UBL"));     // "Step-By-Step UBL"
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1. 創設冷網格"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2. 聰明填充"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3. 批准網格"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4. 細調所有的"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5. 批准網格"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6. 細調所有的"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7. 保存熱床網格"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("燈管控制"));     // "LED Control")
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("燈"));     // "Lights")
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("燈預置"));     // "Light Presets")
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("红"));     // "Red")
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("橙"));     // "Orange")
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("黃"));     // "Yellow")
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("綠"));     // "Green")
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("藍"));     // "Blue")
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("青"));     // "Indigo")
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("紫"));     // "Violet")
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("白"));     // "White")
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("缺省"));     // "Default")
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("定制燈"));     // "Custom Lights")
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("紅飽和度"));     // "Red Intensity")
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("綠飽和度"));     // "Green Intensity")
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("藍飽和度"));     // "Blue Intensity")
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("白飽和度"));     // "White Intensity")
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("亮度"));     // "Brightness")

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("移動 ..."));     // "Moving...")
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("釋放 XY"));     // "Free XY")
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("移動X"));     //"Move X"
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("移動Y"));     //"Move Y"
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("移動Z"));     //"Move Z"
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("擠出機"));     //"Extruder"
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("擠出機 *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("移動 %s mm"));     //"Move 0.025mm"
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("移動 0.1 mm"));     //"Move 0.1mm"
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("移動 1 mm"));     //"Move 1mm"
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("移動 10 mm"));     //"Move 10mm"
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("速率"));     //"Speed"
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("熱床Z"));     //"Bed Z"
  FSTRINGVALUE(MSG_NOZZLE                           , " " LCD_STR_THERMOMETER _UxGT(" 噴嘴"));     //"Nozzle" 噴嘴
  FSTRINGVALUE(MSG_NOZZLE_N                         , " " LCD_STR_THERMOMETER _UxGT(" 噴嘴 ~"));
  FSTRINGVALUE(MSG_BED                              , " " LCD_STR_THERMOMETER _UxGT(" 熱床"));     //"Bed"
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("風扇速率"));     //"Fan speed"
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("風扇速率 ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("額外風扇速率"));     // "Extra fan speed"
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("額外風扇速率 ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("擠出速率"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("擠出速率 ~"));     //"Flow"
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("控制"));     //"Control"
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" 最小"));     //" " LCD_STR_THERMOMETER " Min"
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" 最大"));     //" " LCD_STR_THERMOMETER " Max"
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" 系數"));     //" " LCD_STR_THERMOMETER " Fact"
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("自動控溫"));     //"Autotemp"
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("開"));     //"On"
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("關"));     //"Off"
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));     //"PID-P"
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID-P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));     //"PID-I"
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID-I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));     //"PID-D"
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID-D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));     //"PID-C"
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID-C *"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("選擇"));     //"Select"
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("選擇 *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("加速度"));     //"Accel" acceleration
  FSTRINGVALUE(MSG_JERK                             , _UxGT("抖動速率"));     //"Jerk"
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("軸抖動速率") LCD_STR_A);     //"Va-jerk"
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("軸抖動速率") LCD_STR_B);     //"Vb-jerk"
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("軸抖動速率") LCD_STR_C);     //"Vc-jerk"
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("擠出機抖動速率"));     //"Ve-jerk"
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("速度"));     // "Velocity"
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("最大進料速率") LCD_STR_A);     //"Vmax " max_feedrate_mm_s
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("最大進料速率") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("最大進料速率") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("最大進料速率") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("最大進料速率 *"));     //"Vmax " max_feedrate_mm_s
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("最小進料速率"));     //"Vmin"  min_feedrate_mm_s
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("最小移動速率"));     //"VTrav min" min_travel_feedrate_mm_s, (target) speed of the move
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("加速度"));     // "Acceleration"
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("最大列印加速度") LCD_STR_A);     //"Amax " max_acceleration_mm_per_s2, acceleration in units/s^2 for print moves
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("最大列印加速度") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("最大列印加速度") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("最大列印加速度") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("最大列印加速度 *"));     //"Amax " max_acceleration_mm_per_s2, acceleration in units/s^2 for print moves
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("回縮加速度"));     //"A-retract" retract_acceleration, E acceleration in mm/s^2 for retracts
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("非列印移動加速度"));     //"A-travel" travel_acceleration, X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("軸步數/mm"));     //"Steps/mm" axis_steps_per_mm, axis steps-per-unit G92
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT("軸步數/mm"));     //"Asteps/mm" axis_steps_per_mm, axis steps-per-unit G92
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT("軸步數/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT("軸步數/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("擠出機步數/mm"));     //"Esteps/mm"
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("擠出機~步數/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("溫度"));     //"Temperature"
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("運作"));     //"Motion"
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("絲料測容"));     //"Filament" menu_control_volumetric
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("測容積mm³"));     //"E in mm3" volumetric_enabled
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("絲料直徑"));     //"Fil. Dia."
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("絲料直徑 *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("卸載 mm"));     // "Unload mm"
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("装載 mm"));     // "Load mm"
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD對比度"));     //"LCD contrast"
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("保存設置"));     //"Store memory"
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("載入設置"));     //"Load memory"
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("恢復安全值"));     //"Restore failsafe"
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("初始化設置"));     // "Initialize EEPROM"
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("刷新"));     //"Refresh"
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("資訊界面"));     //"Info screen"
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("準備"));     //"Prepare"
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("調整"));     //"Tune"
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("暫停列印"));     //"Pause print"
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("恢復列印"));     //"Resume print"
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("停止列印"));     //"Stop print"
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("從記憶卡上列印"));     //"Print from SD"
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("無記憶卡"));     //"No SD card"
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("休眠 ..."));     //"Sleep..."
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("點擊繼續 ..."));     //"Click to resume..."
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("列印已暫停"));     // "Print paused"
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("已取消列印"));     //"Print aborted"
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("無移動"));     //"No move."
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("已砍掉"));     //"KILLED. "
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("已停止"));     //"STOPPED. "
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("回縮長度mm"));     //"Retract mm" retract_length, retract length (positive mm)
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("換手回抽長度mm"));     //"Swap Re.mm" swap_retract_length, swap retract length (positive mm), for extruder change
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("回縮速率mm/s"));     //"Retract  V" retract_feedrate_mm_s, feedrate for retracting (mm/s)
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Hop mm"));     //"Hop mm" retract_zraise, retract Z-lift
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("回縮恢復長度mm"));     //"UnRet +mm" retract_recover_extra, additional recover length (mm, added to retract length when recovering)
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("換手回縮恢復長度mm"));     //"S UnRet+mm" swap_retract_recover_extra, additional swap recover length (mm, added to retract length when recovering from extruder change)
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("回縮恢復後進料速率mm/s"));     //"UnRet V" retract_recover_feedrate_mm_s, feedrate for recovering from retraction (mm/s)
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));     // "S UnRet V"
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("自動回縮"));     //"AutoRetr." autoretract_enabled,
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("更換絲料"));     //"Change filament"
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("更換絲料 *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("裝載絲料"));     // "Load filament"
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("裝載絲料 *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("卸載絲料"));     // "Unload filament"
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("卸載絲料 *"));     // "Unload filament"
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("卸載全部"));     // "Unload All"
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("初始化記憶卡"));     //"Init. SD card"
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("更換記憶卡"));     //"Change SD card"
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z探針在熱床之外"));     //"Z probe out. bed" Z probe is not within the physical limits
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("偏斜因數"));     // "Skew Factor"
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));     // "BLTouch"
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch 自檢"));     // "BLTouch Self-Test"
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("重置BLTouch"));     // "Reset BLTouch"
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("部署BLTouch"));     // "Deploy BLTouch"
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("裝載BLTouch"));     // "Stow BLTouch"
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("歸位 %s%s%s 先"));     //"Home ... first"
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Z偏移"));     //"Z Offset"
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("微量調整X軸"));     //"Babystep X" lcd_babystep_x, Babystepping enables the user to control the axis in tiny amounts
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("微量調整Y軸"));     //"Babystep Y"
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("微量調整Z軸"));     //"Babystep Z"
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("擋塊終止"));     //"Endstop abort"
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("加熱失敗"));     //"Heating failed"
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("錯誤：REDUNDANT TEMP"));     //"Err: REDUNDANT TEMP"
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("溫控失控"));     //"THERMAL RUNAWAY"
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("錯誤：最高溫度"));     //"Err: MAXTEMP"
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("錯誤：最低溫度"));     //"Err: MINTEMP"
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("錯誤：最高熱床溫度"));     //"Err: MAXTEMP BED"
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("錯誤：最低熱床溫度"));     //"Err: MINTEMP BED"
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("歸位 XY 先"));     //"Home XY First"
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("印表機停機"));     //"PRINTER HALTED"
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("請重置"));     //"Please reset"
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("天"));     //"d" // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("時"));     //"h" // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("分"));     //"m" // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("加熱中 ..."));     //"Heating..."
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("加熱熱床中 ..."));     //"Bed Heating..."
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("⊿校準"));     //"Delta Calibration"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("⊿校準X"));     //"Calibrate X"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("⊿校準Y"));     //"Calibrate Y"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("⊿校準Z"));     //"Calibrate Z"
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("⊿校準中心"));     //"Calibrate Center"
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("⊿設置"));     // "Delta Settings"
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("⊿自動校準"));     // "Auto Calibration"
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("設置⊿高度"));     // "Set Delta Height"
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("⊿斜柱"));     // "Diag Rod"
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("⊿高度"));     // "Height"
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("⊿半徑"));     // "Radius"
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("關於印表機"));     //"About Printer"
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("印表機訊息"));     //"Printer Info"
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("三點調平"));     // "3-Point Leveling"
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("線性調平"));     // "Linear Leveling"
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("雙線性調平"));     // "Bilinear Leveling"
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("統一熱床調平(UBL)"));     // "Unified Bed Leveling"
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("網格調平"));     // "Mesh Leveling"
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("印表機統計"));     //"Printer Stats"
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("主板訊息"));     //"Board Info"
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("溫度計"));     //"Thermistors"
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("擠出機"));     //"Extruders"
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("傳輸率"));     //"Baud"
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("協議"));     //"Protocol"
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("外殼燈"));     // "Case light"
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("燈亮度"));     // "Light BRIGHTNESS"

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("打印機不正確"));     // "The printer is incorrect"

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("列印計數"));     //"Print Count"
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("已完成"));     //"Completed"
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("總列印時間"));     //"Total print time"
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("最長工作時間"));     //"Longest job time"
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("總計擠出"));     //"Extruded total"
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("列印數"));     //"Prints"
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("完成"));     //"Completed"
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("總共"));     //"Total"
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("最長"));     //"Longest"
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("已擠出"));     //"Extruded"
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("最低溫度"));     //"Min Temp"
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("最高溫度"));     //"Max Temp"
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("電源供應"));     //"Power Supply"
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("驅動力度"));     // "Drive Strength"
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("驅動 %"));     // "Driver %"
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("保存驅動設置"));     // "DAC EEPROM Write"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("列印已暫停"));     // "PRINT PAUSED"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("裝載絲料"));     // "LOAD FILAMENT"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("卸載絲料"));     // "UNLOAD FILAMENT"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("恢複選項:"));     // "RESUME OPTIONS:"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("清除更多"));     // "Purge more"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("恢復列印"));     //"Resume print"
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  噴嘴: "));     // "  Nozzle: "
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("歸原位失敗"));     // "Homing failed"
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("探針探測失敗"));     // "Probing failed"
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: 太冷"));     // "M600: Too cold"

  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("等待開始", "絲料", "變更")));     //"Wait for start of the filament change"
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("等待", "卸下絲料")));     //"Wait for filament unload"
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("插入絲料", "並按鍵", "繼續 ...")));     //"Insert filament and press button to continue..."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("按下按鈕來", "加熱噴嘴.")));     // "Press button to heat nozzle."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("加熱噴嘴", "請等待 ...")));     // "Heating nozzle Please wait..."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("等待", "進料")));     //"Wait for filament load"
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_2_LINE("等待", "絲料清除")));     // "Wait for filament purge"
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("等待列印", "恢復")));     //"Wait for print to resume"
  #else // LCD_HEIGHT < 4
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("請等待 ...")));     //"Please wait..."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("退出中 ...")));     //"Ejecting..."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("插入並點擊")));     //"Insert and Click"
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("加熱中 ...")));     // "Heating..."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("載入中 ...")));     //"Loading..."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("清除中 ...")));     // "Purging..."
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("恢復中 ...")));     //"Resuming..."
  #endif // LCD_HEIGHT < 4
}
