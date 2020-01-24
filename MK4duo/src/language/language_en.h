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
 * English
 *
 * LCD Menu Messages
 *
 */

#define en 1234
#if LCD_LANGUAGE == en || HAS_NEXTION_LCD
  #define NOT_EXTENDED_ISO10646_1_5X7
#endif
#undef en

namespace language_en {
  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("English"));

  #ifdef NOT_EXTENDED_ISO10646_1_5X7
    FSTRINGVALUE(MSG_CUBED                          , _UxGT("^3"));
  #else
    FSTRINGVALUE(MSG_CUBED                          , _UxGT("³"));
  #endif

  FSTRINGVALUE(MSG_WELCOME                          , MACHINE_NAME _UxGT(" Ready."));
  FSTRINGVALUE(MSG_LANGUAGE                         , _UxGT("Language"));
  FSTRINGVALUE(MSG_YES                              , _UxGT("YES"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("NO"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Back"));
  FSTRINGVALUE(MSG_MEDIA_ABORTING                   , _UxGT("Aborting..."));
  FSTRINGVALUE(MSG_SD_INSERTED                      , _UxGT("Card Inserted"));
  FSTRINGVALUE(MSG_SD_REMOVED                       , _UxGT("Card Removed"));
  FSTRINGVALUE(MSG_SD_RELEASED                      , _UxGT("Card Released"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstops")); // Max length 8 characters
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Soft Endstops"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Main"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Advanced Settings"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Configuration"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Autostart"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Disable Steppers"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Debug Menu"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Progress Bar Test"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Auto Home"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Home X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Home Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Home Z"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Auto Z-Align"));
  FSTRINGVALUE(MSG_TMC_Z_CALIBRATION                , _UxGT("Calibrate Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Homing XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Click to Begin"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Next Point"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Leveling Done!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Fade Height"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Set Home Offsets"));
  FSTRINGVALUE(MSG_OFFSETS_APPLIED                  , _UxGT("Offsets Applied"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Set Origin"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Preheat ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" H"));
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" End"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" All"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" Conf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Preheat ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" H"));
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" End"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" All"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" Conf"));
  FSTRINGVALUE(MSG_PREHEAT_3                        , _UxGT("Preheat ") PREHEAT_3_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_3_H                      , _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" H"));
  FSTRINGVALUE(MSG_PREHEAT_3_END                    , _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" End"));
  FSTRINGVALUE(MSG_PREHEAT_3_ALL                    , _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" All"));
  FSTRINGVALUE(MSG_PREHEAT_3_SETTINGS               , _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" Conf"));
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Preheat Custom"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Cooldown"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Switch Power On"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Switch Power Off"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrude"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retract"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Move Axis"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Bed Leveling"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Level Bed"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Level Corners"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Next Corner"));
  FSTRINGVALUE(MSG_MESH_EDITOR                      , _UxGT("Mesh Editor"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Edit Mesh"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Mesh Editing Stopped"));
  FSTRINGVALUE(MSG_PROBING_MESH                     , _UxGT("Probing Point"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Index X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Index Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Z Value"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Custom Commands"));
  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("M48 Probe Test"));
  FSTRINGVALUE(MSG_M48_POINT                        , _UxGT("M48 Point"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Deviation"));
  FSTRINGVALUE(MSG_DXC_MENU                         , _UxGT("DXC Mode"));
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Tool Offsets"));
  FSTRINGVALUE(MSG_DXC_MODE_AUTOPARK                , _UxGT("Auto-Park"));
  FSTRINGVALUE(MSG_DXC_MODE_DUPLICATE               , _UxGT("Duplication"));
  FSTRINGVALUE(MSG_DXC_MODE_MIRRORED_COPY           , _UxGT("Mirrored copy"));
  FSTRINGVALUE(MSG_DXC_MODE_FULL_CTRL               , _UxGT("Full control"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2nd Nozzle X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2nd Nozzle Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2nd Nozzle Z"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("Doing G29"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("UBL Tools"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_LCD_TILTING_MESH                 , _UxGT("Tilting Point"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Manually Build Mesh"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Place Shim & Measure"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Measure"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Remove & Measure Bed"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Moving to next"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Activate UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Deactivate UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Bed Temp"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Bed Temp"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Hotend Temp"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Hotend Temp"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Mesh Edit"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Edit Custom Mesh"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Fine Tuning Mesh"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Done Editing Mesh"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Build Custom Mesh"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Build Mesh"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Build Mesh (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Build Mesh (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Build Cold Mesh"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Adjust Mesh Height"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Height Amount"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Validate Mesh"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Validate Mesh (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Validate Mesh (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Validate Custom Mesh"));
  FSTRINGVALUE(MSG_G26_HEATING_BED                  , _UxGT("G26 Heating Bed"));
  FSTRINGVALUE(MSG_G26_HEATING_NOZZLE               , _UxGT("G26 Heating Nozzle"));
  FSTRINGVALUE(MSG_G26_MANUAL_PRIME                 , _UxGT("Manual priming..."));
  FSTRINGVALUE(MSG_G26_FIXED_LENGTH                 , _UxGT("Fixed Length Prime"));
  FSTRINGVALUE(MSG_G26_PRIME_DONE                   , _UxGT("Done Priming"));
  FSTRINGVALUE(MSG_G26_CANCELED                     , _UxGT("G26 Canceled"));
  FSTRINGVALUE(MSG_G26_LEAVING                      , _UxGT("Leaving G26"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Continue Bed Mesh"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Mesh Leveling"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("3-Point Leveling"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Grid Mesh Leveling"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Level Mesh"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Side Points"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Map Type"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Output Mesh Map"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Output for Host"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Output for CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Off Printer Backup"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Output UBL Info"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Fill-in Amount"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Manual Fill-in"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Smart Fill-in"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Fill-in Mesh"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Invalidate All"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Invalidate Closest"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Fine Tune All"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Fine Tune Closest"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Mesh Storage"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Memory Slot"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Load Bed Mesh"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Save Bed Mesh"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Mesh %i Loaded"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Mesh %i Saved"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("No Storage"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Err: UBL Save"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Err: UBL Restore"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET                     , _UxGT("Z-Offset: "));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Z-Offset Stopped"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("Step-By-Step UBL"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1. Build Cold Mesh"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2. Smart Fill-in"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3. Validate Mesh"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4. Fine Tune All"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5. Validate Mesh"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6. Fine Tune All"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7. Save Bed Mesh"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("LED Control"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Lights"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Light Presets"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Red"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Orange"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Yellow"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Green"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Blue"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Indigo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Violet"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("White"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Default"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Custom Lights"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Red Intensity"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Green Intensity"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Blue Intensity"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("White Intensity"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Brightness"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Moving..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Free XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Move X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Move Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Move Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extruder"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extruder *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Hotend too cold"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Move %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Move 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Move 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Move 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Speed"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Bed Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Nozzle"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Bed"));
  FSTRINGVALUE(MSG_CHAMBER                          , _UxGT("Chamber"));
  FSTRINGVALUE(MSG_COOLER                           , _UxGT("Cooler"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Fan Speed"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Extra Fan Speed"));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Flow"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Control"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fact"));
  FSTRINGVALUE(MSG_IDLEOOZING                       , _UxGT("Anti oozing"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Autotemp"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("On"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Off"));
  FSTRINGVALUE(MSG_PID_AUTOTUNE                     , _UxGT("PID Autotune"));
  FSTRINGVALUE(MSG_PID_BED_AUTOTUNE                 , _UxGT("Bed PID Autotune"));
  FSTRINGVALUE(MSG_PID_CHAMBER_AUTOTUNE             , _UxGT("Chamber PID Autotune"));
  FSTRINGVALUE(MSG_PID_AUTOTUNE_START               , _UxGT("PID Autotune start"));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_BED_PID_P                        , _UxGT("Bed PID-P"));
  FSTRINGVALUE(MSG_BED_PID_I                        , _UxGT("Bed PID-I"));
  FSTRINGVALUE(MSG_BED_PID_D                        , _UxGT("Bed PID-D"));
  FSTRINGVALUE(MSG_CHAMBER_PID_P                    , _UxGT("Chamber PID-P"));
  FSTRINGVALUE(MSG_CHAMBER_PID_I                    , _UxGT("Chamber PID-I"));
  FSTRINGVALUE(MSG_CHAMBER_PID_D                    , _UxGT("Chamber PID-D"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Select"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Accel"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Jerk"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("Jerk-V") LCD_STR_A);
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("Jerk-V") LCD_STR_B);
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("Jerk-V") LCD_STR_C);
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Jerk-V") LCD_STR_E);
  FSTRINGVALUE(MSG_JUNCTION_DEVIATION               , _UxGT("Junction Dev"));
  FSTRINGVALUE(MSG_JUNCTION_MM                      , _UxGT("Junction mm"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Velocity"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("Vmax ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("Vmax ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("Vmax ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vmax ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("Vmin"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("VTrav Min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Acceleration"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Amax ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Amax ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Amax ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Amax ") LCD_STR_E);
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-Retract E"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-Travel"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Steps/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , _UxGT("steps/mm ") LCD_STR_A);
  FSTRINGVALUE(MSG_B_STEPS                          , _UxGT("steps/mm ") LCD_STR_B);
  FSTRINGVALUE(MSG_C_STEPS                          , _UxGT("steps/mm ") LCD_STR_C);
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("steps/mm ") LCD_STR_E);
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperature"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Motion"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Fil. Dia."));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Unload mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Load mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("Advance K"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD Contrast"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Store Settings"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Load Settings"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Restore failsafe"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Initialize EEPROM"));
  FSTRINGVALUE(MSG_SD_UPDATE                        , _UxGT("SD Update"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Reset Printer"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Refresh"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Info Screen"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Prepare"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Tune"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Start Print"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Next"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Init"));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Stop"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Print"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Reset"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Cancel"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Done"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pause Print"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Resume Print"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Stop Print"));
  FSTRINGVALUE(MSG_RESTART                          , _UxGT("Restart"));
  FSTRINGVALUE(MSG_CARD_MENU                        , _UxGT("Print from SD"));
  FSTRINGVALUE(MSG_NO_CARD                          , _UxGT("No SD Card"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Sleep..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Click to Resume..."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Print Paused"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Printing..."));
  FSTRINGVALUE(MSG_STOPPING                         , _UxGT("Print finish"));
  FSTRINGVALUE(MSG_RESUMING                         , _UxGT("Resuming print"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Print Aborted"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("No Move."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("KILLED. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("STOPPED. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retract mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Swap Re.mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retract  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Hop mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("S UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("AutoRetr."));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Swap Length"));
  FSTRINGVALUE(MSG_FILAMENT_PURGE_LENGTH            , _UxGT("Purge Length"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Tool Change"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Z Raise"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Prime Speed"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Retract Speed"));
  FSTRINGVALUE(MSG_NOZZLE_STANDBY                   , _UxGT("Nozzle Standby"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Change Filament"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Load Filament"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Unload Filament"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Unload All"));
  FSTRINGVALUE(MSG_INIT_SDCARD                      , _UxGT("Init. SD Card"));
  FSTRINGVALUE(MSG_CHANGE_SDCARD                    , _UxGT("Change SD Card"));
  FSTRINGVALUE(MSG_RELEASE_SDCARD                   , _UxGT("Release SD Card"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z Probe Past Bed"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Skew Factor"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Self-Test"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Reset"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Stow"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Deploy"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_SW                  , _UxGT("Mode SW"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_5V                  , _UxGT("Mode 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_OD                  , _UxGT("Mode OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE               , _UxGT("Mode Store"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_5V            , _UxGT("Set to 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_OD            , _UxGT("Set to OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_ECHO                , _UxGT("Report Drain"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_CHANGE              , _UxGT("DANGER: Bad settings can cause damage! Proceed anyway?"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Deploy Z-Probe"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Stow Z-Probe"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Home %s%s%s First"));
  FSTRINGVALUE(MSG_ZPROBE_OFFSETS                   , _UxGT("Probe Offsets"));
  FSTRINGVALUE(MSG_ZPROBE_XOFFSET                   , _UxGT("Probe X Offset"));
  FSTRINGVALUE(MSG_ZPROBE_YOFFSET                   , _UxGT("Probe Y Offset"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Probe Z Offset"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Babystep X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Babystep Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Babystep Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Endstop Abort"));
  FSTRINGVALUE(MSG_HEATING_FAILED                   , _UxGT("Heating Failed"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Err: REDUNDANT TEMP"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("THERMAL RUNAWAY"));
  FSTRINGVALUE(MSG_AD595                            , _UxGT("AD595 Offset & Gain"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err: MAXTEMP"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err: MINTEMP"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Err: MAXTEMP BED"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Err: MINTEMP BED"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_CHAMBER              , _UxGT("Err: MAXTEMP CHAMBER"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_CHAMBER              , _UxGT("Err: MINTEMP CHAMBER"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_COOLER               , _UxGT("Err: MAXTEMP COOLER"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_COOLER               , _UxGT("Err: MINTEMP COOLER"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Home XY First"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("PRINTER HALTED"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Please Reset"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d"));
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h"));
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m"));
  FSTRINGVALUE(MSG_LONG_DAY                         , _UxGT("days"));
  FSTRINGVALUE(MSG_LONG_HOUR                        , _UxGT("hours"));
  FSTRINGVALUE(MSG_LONG_MINUTE                      , _UxGT("minutes"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Heating..."));
  FSTRINGVALUE(MSG_HEATING_COMPLETE                 , _UxGT("Heating Done."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Cooling..."));
  FSTRINGVALUE(MSG_COOLING_COMPLETE                 , _UxGT("Cooling Done."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Bed Heating..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Bed Cooling..."));
  FSTRINGVALUE(MSG_BED_DONE                         , _UxGT("Bed Done."));
  FSTRINGVALUE(MSG_CHAMBER_HEATING                  , _UxGT("Chamber Heating..."));
  FSTRINGVALUE(MSG_CHAMBER_COOLING                  , _UxGT("Chamber Cooling..."));
  FSTRINGVALUE(MSG_CHAMBER_DONE                     , _UxGT("Chamber Done."));
  FSTRINGVALUE(MSG_COOLER_COOLING                   , _UxGT("Cooler Cooling."));
  FSTRINGVALUE(MSG_COOLER_DONE                      , _UxGT("Cooler Done."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta Calibration"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibrate X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibrate Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibrate Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibrate Center"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Delta Settings"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto Calibration"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Set Delta Height"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Diag Rod"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Height"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Radius"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE_OK          , _UxGT("Calibration OK"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("About Printer"));
  FSTRINGVALUE(MSG_INFO_FIRMWARE_MENU               , _UxGT("Firmware Info"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Printer Info"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("3-Point Leveling"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Linear Leveling"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Bilinear Leveling"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Mesh Leveling"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Printer Stats"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Board Info"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Thermistors"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extruders"));
  FSTRINGVALUE(MSG_INFO_HOTENDS                     , _UxGT("Hotends"));
  FSTRINGVALUE(MSG_INFO_BEDS                        , _UxGT("Beds"));
  FSTRINGVALUE(MSG_INFO_CHAMBERS                    , _UxGT("Chambers"));
  FSTRINGVALUE(MSG_INFO_COOLER                      , _UxGT("Coolers"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baud"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocol"));

  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Case Light"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Light Brightness"));
  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("INCORRECT PRINTER"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Total prints"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completed"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total Print time"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Longest Job Time"));
    FSTRINGVALUE(MSG_INFO_POWER_ON                  , _UxGT("Power on time"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Filament Total"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Prints"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completed"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Longest"));
    FSTRINGVALUE(MSG_INFO_POWER_ON                  , _UxGT("Power on"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Filament"));
  #endif

  FSTRINGVALUE(MSG_INFO_PWRCONSUMED                 , _UxGT("PWR"));
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Min Temp"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Max Temp"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("PSU"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Drive Strength"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driver %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("DAC EEPROM Write"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER           , _UxGT("FILAMENT CHANGE"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("PRINT PAUSED"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("LOAD FILAMENT"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("UNLOAD FILAMENT"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("RESUME OPTIONS:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Purge more"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Continue"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Nozzle: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Runout Sensor"));
  FSTRINGVALUE(MSG_RUNOUT_DISTANCE_MM               , _UxGT("Runout Dist mm"));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Homing Failed"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Probing Failed"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: Too Cold"));

  FSTRINGVALUE(MSG_MMU2_CHOOSE_FILAMENT_HEADER      , _UxGT("CHOOSE FILAMENT"));
  FSTRINGVALUE(MSG_MMU2_MENU                        , _UxGT("MMU"));
  FSTRINGVALUE(MSG_MMU2_WRONG_FIRMWARE              , _UxGT("Update MMU Firmware!"));
  FSTRINGVALUE(MSG_MMU2_NOT_RESPONDING              , _UxGT("MMU Needs Attention."));
  FSTRINGVALUE(MSG_MMU2_RESUME                      , _UxGT("Resume Print"));
  FSTRINGVALUE(MSG_MMU2_RESUMING                    , _UxGT("Resuming..."));
  FSTRINGVALUE(MSG_MMU2_LOAD_FILAMENT               , _UxGT("Load Filament"));
  FSTRINGVALUE(MSG_MMU2_LOAD_ALL                    , _UxGT("Load All"));
  FSTRINGVALUE(MSG_MMU2_LOAD_TO_NOZZLE              , _UxGT("Load to Nozzle"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT              , _UxGT("Eject Filament"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT_N            , _UxGT("Eject Filament E"));
  FSTRINGVALUE(MSG_MMU2_UNLOAD_FILAMENT             , _UxGT("Unload Filament"));
  FSTRINGVALUE(MSG_MMU2_LOADING_FILAMENT            , _UxGT("Loading Fil. %i..."));
  FSTRINGVALUE(MSG_MMU2_EJECTING_FILAMENT           , _UxGT("Ejecting Fil. ..."));
  FSTRINGVALUE(MSG_MMU2_UNLOADING_FILAMENT          , _UxGT("Unloading Fil...."));
  FSTRINGVALUE(MSG_MMU2_ALL                         , _UxGT("All"));
  FSTRINGVALUE(MSG_MMU2_FILAMENT_N                  , _UxGT("Filament E"));
  FSTRINGVALUE(MSG_MMU2_RESET                       , _UxGT("Reset MMU"));
  FSTRINGVALUE(MSG_MMU2_RESETTING                   , _UxGT("Resetting MMU..."));
  FSTRINGVALUE(MSG_MMU2_EJECT_RECOVER               , _UxGT("Remove, click"));

  FSTRINGVALUE(MSG_MIX                              , _UxGT("Mix"));
  FSTRINGVALUE(MSG_MIX_COMPONENT                    , _UxGT("Component"));
  FSTRINGVALUE(MSG_MIXER                            , _UxGT("Mixer"));
  FSTRINGVALUE(MSG_GRADIENT                         , _UxGT("Gradient"));
  FSTRINGVALUE(MSG_FULL_GRADIENT                    , _UxGT("Full Gradient"));
  FSTRINGVALUE(MSG_TOGGLE_MIX                       , _UxGT("Toggle Mix"));
  FSTRINGVALUE(MSG_CYCLE_MIX                        , _UxGT("Cycle Mix"));
  FSTRINGVALUE(MSG_GRADIENT_MIX                     , _UxGT("Gradient Mix"));
  FSTRINGVALUE(MSG_REVERSE_GRADIENT                 , _UxGT("Reverse Gradient"));
  FSTRINGVALUE(MSG_ACTIVE_VTOOL                     , _UxGT("Active V-tool"));
  FSTRINGVALUE(MSG_START_VTOOL                      , _UxGT("Start V-tool"));
  FSTRINGVALUE(MSG_END_VTOOL                        , _UxGT("  End V-tool"));
  FSTRINGVALUE(MSG_GRADIENT_ALIAS                   , _UxGT("Alias V-tool"));
  FSTRINGVALUE(MSG_RESET_VTOOLS                     , _UxGT("Reset V-tools"));
  FSTRINGVALUE(MSG_COMMIT_VTOOL                     , _UxGT("Commit V-tool Mix"));
  FSTRINGVALUE(MSG_VTOOLS_RESET                     , _UxGT("V-tools Were Reset"));
  FSTRINGVALUE(MSG_START_Z                          , _UxGT("Start Z"));
  FSTRINGVALUE(MSG_END_Z                            , _UxGT("  End Z"));

  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Games"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Brickout"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Invaders"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Sn4k3"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Maze"));

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Press Button", "to resume print")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parking...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Wait for", "filament change", "to start")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Insert filament", "and press button", "to continue")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Press button", "to heat nozzle")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_ZZZ            , _UxGT(MSG_2_LINE(" z   z   z", "Z   Z   Z")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Nozzle heating", "Please wait...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Wait for", "filament unload")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Wait for", "filament load")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_2_LINE("Wait for", "filament purge")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_2_LINE("Click to finish", "filament purge")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Wait for print", "to resume...")));
  #else // LCD_HEIGHT < 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Click to continue")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parking...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Please wait...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Insert and Click")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Click to heat")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_ZZZ            , _UxGT(MSG_1_LINE(" Zz   Zz   Zz")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Heating...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Ejecting...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Loading...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Purging...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Click to finish")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Resuming...")));
  #endif  

  FSTRINGVALUE(MSG_TMC_DRIVERS                      , _UxGT("TMC Drivers"));
  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Driver Current"));
  FSTRINGVALUE(MSG_TMC_MICROSTEP                    , _UxGT("Driver Microstep"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Hybrid Threshold"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Sensorless Homing"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Stepping Mode"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("StealthChop Enabled"));

  FSTRINGVALUE(MSG_SERVICE_RESET                    , _UxGT("Reset"));
  FSTRINGVALUE(MSG_SERVICE_IN                       , _UxGT(" in:"));
  FSTRINGVALUE(MSG_MAX_INACTIVITY_TIME              , _UxGT("Heating disabled by safety timer."));

  // Max extruder
  FSTRINGVALUE(MSG_MAX_EXTRUDERS                    , _UxGT("Extruders"));

  // Extra
  FSTRINGVALUE(MSG_RESTART_PRINT                    , _UxGT("Restart print"));
  FSTRINGVALUE(MSG_FIX_LOSE_STEPS                   , _UxGT("Fix axis steps"));
  FSTRINGVALUE(MSG_LASER                            , _UxGT("Laser Preset"));
  FSTRINGVALUE(MSG_NEED_TUNE_PID                    , _UxGT("Need Tune PID"));
  FSTRINGVALUE(MSG_ARE_YOU_SURE                     , _UxGT("Are you sure"));

  // Rfid module
  FSTRINGVALUE(MSG_RFID_SPOOL                       , _UxGT("Spool on E"));
  FSTRINGVALUE(MSG_RFID_BRAND                       , _UxGT("Brand: "));
  FSTRINGVALUE(MSG_RFID_TYPE                        , _UxGT("Type: "));
  FSTRINGVALUE(MSG_RFID_COLOR                       , _UxGT("Color: "));
  FSTRINGVALUE(MSG_RFID_SIZE                        , _UxGT("Size: "));
  FSTRINGVALUE(MSG_RFID_TEMP_HOTEND                 , _UxGT("Temperature Hotend: "));
  FSTRINGVALUE(MSG_RFID_TEMP_BED                    , _UxGT("Temperature Bed: "));
  FSTRINGVALUE(MSG_RFID_TEMP_USER_HOTEND            , _UxGT("User temperature Hotend: "));
  FSTRINGVALUE(MSG_RFID_TEMP_USER_BED               , _UxGT("User temperatura Bed: "));
  FSTRINGVALUE(MSG_RFID_DENSITY                     , _UxGT("Density: "));
  FSTRINGVALUE(MSG_RFID_SPOOL_LENGHT                , _UxGT("Spool Lenght: "));

  // Sound
  FSTRINGVALUE(MSG_SOUND_MODE_ON                    , _UxGT("Sound          [on]"));
  FSTRINGVALUE(MSG_SOUND_MODE_SILENT                , _UxGT("Sound      [silent]"));
  FSTRINGVALUE(MSG_SOUND_MODE_MUTE                  , _UxGT("Sound        [mute]"));

  // EEPROM Allert
  FSTRINGVALUE(MSG_EEPROM_ALLERT                    , _UxGT(MSG_3_LINE("ATTENTION...", "EEPROM Changed.", "Click to continue")));

  // Nextion Allert
  FSTRINGVALUE(MSG_NEXTION_ALLERT                   , _UxGT(MSG_4_LINE("ATTENTION...", "NEXTION FW changed.", "Please upload new FW", "Click to continue")));

  // Nextion M0 M1
  FSTRINGVALUE(MSG_NEXTION_M0_M1_1                  , _UxGT("Press button enter"));
  FSTRINGVALUE(MSG_NEXTION_M0_M1_2                  , _UxGT("to resume print"));

  // DHT
  FSTRINGVALUE(MSG_DHT                              , _UxGT("DHT"));
  FSTRINGVALUE(MSG_DHT_11                           , _UxGT("DHT11"));
  FSTRINGVALUE(MSG_DHT_12                           , _UxGT("DHT12"));
  FSTRINGVALUE(MSG_DHT_21                           , _UxGT("DHT21"));
  FSTRINGVALUE(MSG_DHT_22                           , _UxGT("DHT22"));
  FSTRINGVALUE(MSG_DHT_TEMPERATURE                  , _UxGT("Temperature (C):"));
  FSTRINGVALUE(MSG_DHT_HUMIDITY                     , _UxGT("Humidity (%):"));
  FSTRINGVALUE(MSG_DHT_DEWPOINT                     , _UxGT("Dew Point (C):"));
}
