/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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

namespace Language_en {
  constexpr uint8_t    CHARSIZE                             = 2;
  PROGMEM Language_Str LANGUAGE                             = _UxGT("English");

  #ifdef NOT_EXTENDED_ISO10646_1_5X7
    PROGMEM Language_Str MSG_CUBED                          = _UxGT("^3");
  #else
    PROGMEM Language_Str MSG_CUBED                          = _UxGT("³");
  #endif

  PROGMEM Language_Str MSG_WELCOME                          = MACHINE_NAME _UxGT(" Ready.");
  PROGMEM Language_Str MSG_LANGUAGE                         = _UxGT("Language");
  PROGMEM Language_Str MSG_YES                              = _UxGT("YES");
  PROGMEM Language_Str MSG_NO                               = _UxGT("NO");
  PROGMEM Language_Str MSG_BACK                             = _UxGT("Back");
  PROGMEM Language_Str MSG_MEDIA_ABORTING                   = _UxGT("Aborting...");
  PROGMEM Language_Str MSG_SD_INSERTED                      = _UxGT("Card Inserted");
  PROGMEM Language_Str MSG_SD_REMOVED                       = _UxGT("Card Removed");
  PROGMEM Language_Str MSG_SD_RELEASED                      = _UxGT("Card Released");
  PROGMEM Language_Str MSG_LCD_ENDSTOPS                     = _UxGT("Endstops"); // Max length 8 characters
  PROGMEM Language_Str MSG_LCD_SOFT_ENDSTOPS                = _UxGT("Soft Endstops");
  PROGMEM Language_Str MSG_MAIN                             = _UxGT("Main");
  PROGMEM Language_Str MSG_ADVANCED_SETTINGS                = _UxGT("Advanced Settings");
  PROGMEM Language_Str MSG_CONFIGURATION                    = _UxGT("Configuration");
  PROGMEM Language_Str MSG_AUTOSTART                        = _UxGT("Autostart");
  PROGMEM Language_Str MSG_DISABLE_STEPPERS                 = _UxGT("Disable Steppers");
  PROGMEM Language_Str MSG_DEBUG_MENU                       = _UxGT("Debug Menu");
  PROGMEM Language_Str MSG_PROGRESS_BAR_TEST                = _UxGT("Progress Bar Test");
  PROGMEM Language_Str MSG_AUTO_HOME                        = _UxGT("Auto Home");
  PROGMEM Language_Str MSG_AUTO_HOME_X                      = _UxGT("Home X");
  PROGMEM Language_Str MSG_AUTO_HOME_Y                      = _UxGT("Home Y");
  PROGMEM Language_Str MSG_AUTO_HOME_Z                      = _UxGT("Home Z");
  PROGMEM Language_Str MSG_AUTO_Z_ALIGN                     = _UxGT("Auto Z-Align");
  PROGMEM Language_Str MSG_TMC_Z_CALIBRATION                = _UxGT("Calibrate Z");
  PROGMEM Language_Str MSG_LEVEL_BED_HOMING                 = _UxGT("Homing XYZ");
  PROGMEM Language_Str MSG_LEVEL_BED_WAITING                = _UxGT("Click to Begin");
  PROGMEM Language_Str MSG_LEVEL_BED_NEXT_POINT             = _UxGT("Next Point");
  PROGMEM Language_Str MSG_LEVEL_BED_DONE                   = _UxGT("Leveling Done!");
  PROGMEM Language_Str MSG_Z_FADE_HEIGHT                    = _UxGT("Fade Height");
  PROGMEM Language_Str MSG_SET_HOME_OFFSETS                 = _UxGT("Set Home Offsets");
  PROGMEM Language_Str MSG_OFFSETS_APPLIED                  = _UxGT("Offsets Applied");
  PROGMEM Language_Str MSG_SET_ORIGIN                       = _UxGT("Set Origin");
  PROGMEM Language_Str MSG_PREHEAT_1                        = _UxGT("Preheat ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_1_H                      = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" H");
  PROGMEM Language_Str MSG_PREHEAT_1_END                    = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" End");
  PROGMEM Language_Str MSG_PREHEAT_1_ALL                    = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" All");
  PROGMEM Language_Str MSG_PREHEAT_1_SETTINGS               = _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" Conf");
  PROGMEM Language_Str MSG_PREHEAT_2                        = _UxGT("Preheat ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_2_H                      = _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" H");
  PROGMEM Language_Str MSG_PREHEAT_2_END                    = _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" End");
  PROGMEM Language_Str MSG_PREHEAT_2_ALL                    = _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" All");
  PROGMEM Language_Str MSG_PREHEAT_2_SETTINGS               = _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" Conf");
  PROGMEM Language_Str MSG_PREHEAT_3                        = _UxGT("Preheat ") PREHEAT_3_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_3_H                      = _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" H");
  PROGMEM Language_Str MSG_PREHEAT_3_END                    = _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" End");
  PROGMEM Language_Str MSG_PREHEAT_3_ALL                    = _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" All");
  PROGMEM Language_Str MSG_PREHEAT_3_SETTINGS               = _UxGT("Preheat ") PREHEAT_3_LABEL _UxGT(" Conf");
  PROGMEM Language_Str MSG_PREHEAT_CUSTOM                   = _UxGT("Preheat Custom");
  PROGMEM Language_Str MSG_COOLDOWN                         = _UxGT("Cooldown");
  PROGMEM Language_Str MSG_SWITCH_PS_ON                     = _UxGT("Switch Power On");
  PROGMEM Language_Str MSG_SWITCH_PS_OFF                    = _UxGT("Switch Power Off");
  PROGMEM Language_Str MSG_EXTRUDE                          = _UxGT("Extrude");
  PROGMEM Language_Str MSG_RETRACT                          = _UxGT("Retract");
  PROGMEM Language_Str MSG_MOVE_AXIS                        = _UxGT("Move Axis");
  PROGMEM Language_Str MSG_BED_LEVELING                     = _UxGT("Bed Leveling");
  PROGMEM Language_Str MSG_LEVEL_BED                        = _UxGT("Level Bed");
  PROGMEM Language_Str MSG_LEVEL_CORNERS                    = _UxGT("Level Corners");
  PROGMEM Language_Str MSG_NEXT_CORNER                      = _UxGT("Next Corner");
  PROGMEM Language_Str MSG_MESH_EDITOR                      = _UxGT("Mesh Editor");
  PROGMEM Language_Str MSG_EDIT_MESH                        = _UxGT("Edit Mesh");
  PROGMEM Language_Str MSG_EDITING_STOPPED                  = _UxGT("Mesh Editing Stopped");
  PROGMEM Language_Str MSG_PROBING_MESH                     = _UxGT("Probing Point");
  PROGMEM Language_Str MSG_MESH_X                           = _UxGT("Index X");
  PROGMEM Language_Str MSG_MESH_Y                           = _UxGT("Index Y");
  PROGMEM Language_Str MSG_MESH_EDIT_Z                      = _UxGT("Z Value");
  PROGMEM Language_Str MSG_USER_MENU                        = _UxGT("Custom Commands");
  PROGMEM Language_Str MSG_M48_TEST                         = _UxGT("M48 Probe Test");
  PROGMEM Language_Str MSG_M48_POINT                        = _UxGT("M48 Point");
  PROGMEM Language_Str MSG_M48_DEVIATION                    = _UxGT("Deviation");
  PROGMEM Language_Str MSG_DXC_MENU                         = _UxGT("DXC Mode");
  PROGMEM Language_Str MSG_OFFSETS_MENU                     = _UxGT("Tool Offsets");
  PROGMEM Language_Str MSG_DXC_MODE_AUTOPARK                = _UxGT("Auto-Park");
  PROGMEM Language_Str MSG_DXC_MODE_DUPLICATE               = _UxGT("Duplication");
  PROGMEM Language_Str MSG_DXC_MODE_MIRRORED_COPY           = _UxGT("Mirrored copy");
  PROGMEM Language_Str MSG_DXC_MODE_FULL_CTRL               = _UxGT("Full control");
  PROGMEM Language_Str MSG_X_OFFSET                         = _UxGT("2nd Nozzle X");
  PROGMEM Language_Str MSG_Y_OFFSET                         = _UxGT("2nd Nozzle Y");
  PROGMEM Language_Str MSG_Z_OFFSET                         = _UxGT("2nd Nozzle Z");
  PROGMEM Language_Str MSG_UBL_DOING_G29                    = _UxGT("Doing G29");
  PROGMEM Language_Str MSG_UBL_TOOLS                        = _UxGT("UBL Tools");
  PROGMEM Language_Str MSG_UBL_LEVEL_BED                    = _UxGT("Unified Bed Leveling");
  PROGMEM Language_Str MSG_LCD_TILTING_MESH                 = _UxGT("Tilting Point");
  PROGMEM Language_Str MSG_UBL_MANUAL_MESH                  = _UxGT("Manually Build Mesh");
  PROGMEM Language_Str MSG_UBL_BC_INSERT                    = _UxGT("Place Shim & Measure");
  PROGMEM Language_Str MSG_UBL_BC_INSERT2                   = _UxGT("Measure");
  PROGMEM Language_Str MSG_UBL_BC_REMOVE                    = _UxGT("Remove & Measure Bed");
  PROGMEM Language_Str MSG_UBL_MOVING_TO_NEXT               = _UxGT("Moving to next");
  PROGMEM Language_Str MSG_UBL_ACTIVATE_MESH                = _UxGT("Activate UBL");
  PROGMEM Language_Str MSG_UBL_DEACTIVATE_MESH              = _UxGT("Deactivate UBL");
  PROGMEM Language_Str MSG_UBL_SET_TEMP_BED                 = _UxGT("Bed Temp");
  PROGMEM Language_Str MSG_UBL_BED_TEMP_CUSTOM              = _UxGT("Bed Temp");
  PROGMEM Language_Str MSG_UBL_SET_TEMP_HOTEND              = _UxGT("Hotend Temp");
  PROGMEM Language_Str MSG_UBL_HOTEND_TEMP_CUSTOM           = _UxGT("Hotend Temp");
  PROGMEM Language_Str MSG_UBL_MESH_EDIT                    = _UxGT("Mesh Edit");
  PROGMEM Language_Str MSG_UBL_EDIT_CUSTOM_MESH             = _UxGT("Edit Custom Mesh");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_MESH               = _UxGT("Fine Tuning Mesh");
  PROGMEM Language_Str MSG_UBL_DONE_EDITING_MESH            = _UxGT("Done Editing Mesh");
  PROGMEM Language_Str MSG_UBL_BUILD_CUSTOM_MESH            = _UxGT("Build Custom Mesh");
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_MENU              = _UxGT("Build Mesh");
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_M1                = _UxGT("Build Mesh (") PREHEAT_1_LABEL _UxGT(")");
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_M2                = _UxGT("Build Mesh (") PREHEAT_2_LABEL _UxGT(")");
  PROGMEM Language_Str MSG_UBL_BUILD_COLD_MESH              = _UxGT("Build Cold Mesh");
  PROGMEM Language_Str MSG_UBL_MESH_HEIGHT_ADJUST           = _UxGT("Adjust Mesh Height");
  PROGMEM Language_Str MSG_UBL_MESH_HEIGHT_AMOUNT           = _UxGT("Height Amount");
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_MENU           = _UxGT("Validate Mesh");
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_M1             = _UxGT("Validate Mesh (") PREHEAT_1_LABEL _UxGT(")");
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_M2             = _UxGT("Validate Mesh (") PREHEAT_2_LABEL _UxGT(")");
  PROGMEM Language_Str MSG_UBL_VALIDATE_CUSTOM_MESH         = _UxGT("Validate Custom Mesh");
  PROGMEM Language_Str MSG_G26_HEATING_BED                  = _UxGT("G26 Heating Bed");
  PROGMEM Language_Str MSG_G26_HEATING_NOZZLE               = _UxGT("G26 Heating Nozzle");
  PROGMEM Language_Str MSG_G26_MANUAL_PRIME                 = _UxGT("Manual priming...");
  PROGMEM Language_Str MSG_G26_FIXED_LENGTH                 = _UxGT("Fixed Length Prime");
  PROGMEM Language_Str MSG_G26_PRIME_DONE                   = _UxGT("Done Priming");
  PROGMEM Language_Str MSG_G26_CANCELED                     = _UxGT("G26 Canceled");
  PROGMEM Language_Str MSG_G26_LEAVING                      = _UxGT("Leaving G26");
  PROGMEM Language_Str MSG_UBL_CONTINUE_MESH                = _UxGT("Continue Bed Mesh");
  PROGMEM Language_Str MSG_UBL_MESH_LEVELING                = _UxGT("Mesh Leveling");
  PROGMEM Language_Str MSG_UBL_3POINT_MESH_LEVELING         = _UxGT("3-Point Leveling");
  PROGMEM Language_Str MSG_UBL_GRID_MESH_LEVELING           = _UxGT("Grid Mesh Leveling");
  PROGMEM Language_Str MSG_UBL_MESH_LEVEL                   = _UxGT("Level Mesh");
  PROGMEM Language_Str MSG_UBL_SIDE_POINTS                  = _UxGT("Side Points");
  PROGMEM Language_Str MSG_UBL_MAP_TYPE                     = _UxGT("Map Type");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP                   = _UxGT("Output Mesh Map");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_HOST              = _UxGT("Output for Host");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_CSV               = _UxGT("Output for CSV");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_BACKUP            = _UxGT("Off Printer Backup");
  PROGMEM Language_Str MSG_UBL_INFO_UBL                     = _UxGT("Output UBL Info");
  PROGMEM Language_Str MSG_UBL_FILLIN_AMOUNT                = _UxGT("Fill-in Amount");
  PROGMEM Language_Str MSG_UBL_MANUAL_FILLIN                = _UxGT("Manual Fill-in");
  PROGMEM Language_Str MSG_UBL_SMART_FILLIN                 = _UxGT("Smart Fill-in");
  PROGMEM Language_Str MSG_UBL_FILLIN_MESH                  = _UxGT("Fill-in Mesh");
  PROGMEM Language_Str MSG_UBL_INVALIDATE_ALL               = _UxGT("Invalidate All");
  PROGMEM Language_Str MSG_UBL_INVALIDATE_CLOSEST           = _UxGT("Invalidate Closest");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_ALL                = _UxGT("Fine Tune All");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_CLOSEST            = _UxGT("Fine Tune Closest");
  PROGMEM Language_Str MSG_UBL_STORAGE_MESH_MENU            = _UxGT("Mesh Storage");
  PROGMEM Language_Str MSG_UBL_STORAGE_SLOT                 = _UxGT("Memory Slot");
  PROGMEM Language_Str MSG_UBL_LOAD_MESH                    = _UxGT("Load Bed Mesh");
  PROGMEM Language_Str MSG_UBL_SAVE_MESH                    = _UxGT("Save Bed Mesh");
  PROGMEM Language_Str MSG_MESH_LOADED                      = _UxGT("M117 Mesh %i Loaded");
  PROGMEM Language_Str MSG_MESH_SAVED                       = _UxGT("M117 Mesh %i Saved");
  PROGMEM Language_Str MSG_UBL_NO_STORAGE                   = _UxGT("No Storage");
  PROGMEM Language_Str MSG_UBL_SAVE_ERROR                   = _UxGT("Err: UBL Save");
  PROGMEM Language_Str MSG_UBL_RESTORE_ERROR                = _UxGT("Err: UBL Restore");
  PROGMEM Language_Str MSG_UBL_Z_OFFSET                     = _UxGT("Z-Offset: ");
  PROGMEM Language_Str MSG_UBL_Z_OFFSET_STOPPED             = _UxGT("Z-Offset Stopped");
  PROGMEM Language_Str MSG_UBL_STEP_BY_STEP_MENU            = _UxGT("Step-By-Step UBL");
  PROGMEM Language_Str MSG_UBL_1_BUILD_COLD_MESH            = _UxGT("1. Build Cold Mesh");
  PROGMEM Language_Str MSG_UBL_2_SMART_FILLIN               = _UxGT("2. Smart Fill-in");
  PROGMEM Language_Str MSG_UBL_3_VALIDATE_MESH_MENU         = _UxGT("3. Validate Mesh");
  PROGMEM Language_Str MSG_UBL_4_FINE_TUNE_ALL              = _UxGT("4. Fine Tune All");
  PROGMEM Language_Str MSG_UBL_5_VALIDATE_MESH_MENU         = _UxGT("5. Validate Mesh");
  PROGMEM Language_Str MSG_UBL_6_FINE_TUNE_ALL              = _UxGT("6. Fine Tune All");
  PROGMEM Language_Str MSG_UBL_7_SAVE_MESH                  = _UxGT("7. Save Bed Mesh");

  PROGMEM Language_Str MSG_LED_CONTROL                      = _UxGT("LED Control");
  PROGMEM Language_Str MSG_LEDS                             = _UxGT("Lights");
  PROGMEM Language_Str MSG_LED_PRESETS                      = _UxGT("Light Presets");
  PROGMEM Language_Str MSG_SET_LEDS_RED                     = _UxGT("Red");
  PROGMEM Language_Str MSG_SET_LEDS_ORANGE                  = _UxGT("Orange");
  PROGMEM Language_Str MSG_SET_LEDS_YELLOW                  = _UxGT("Yellow");
  PROGMEM Language_Str MSG_SET_LEDS_GREEN                   = _UxGT("Green");
  PROGMEM Language_Str MSG_SET_LEDS_BLUE                    = _UxGT("Blue");
  PROGMEM Language_Str MSG_SET_LEDS_INDIGO                  = _UxGT("Indigo");
  PROGMEM Language_Str MSG_SET_LEDS_VIOLET                  = _UxGT("Violet");
  PROGMEM Language_Str MSG_SET_LEDS_WHITE                   = _UxGT("White");
  PROGMEM Language_Str MSG_SET_LEDS_DEFAULT                 = _UxGT("Default");
  PROGMEM Language_Str MSG_CUSTOM_LEDS                      = _UxGT("Custom Lights");
  PROGMEM Language_Str MSG_INTENSITY_R                      = _UxGT("Red Intensity");
  PROGMEM Language_Str MSG_INTENSITY_G                      = _UxGT("Green Intensity");
  PROGMEM Language_Str MSG_INTENSITY_B                      = _UxGT("Blue Intensity");
  PROGMEM Language_Str MSG_INTENSITY_W                      = _UxGT("White Intensity");
  PROGMEM Language_Str MSG_LED_BRIGHTNESS                   = _UxGT("Brightness");

  PROGMEM Language_Str MSG_MOVING                           = _UxGT("Moving...");
  PROGMEM Language_Str MSG_FREE_XY                          = _UxGT("Free XY");
  PROGMEM Language_Str MSG_MOVE_X                           = _UxGT("Move X");
  PROGMEM Language_Str MSG_MOVE_Y                           = _UxGT("Move Y");
  PROGMEM Language_Str MSG_MOVE_Z                           = _UxGT("Move Z");
  PROGMEM Language_Str MSG_MOVE_E                           = _UxGT("Extruder");
  PROGMEM Language_Str MSG_MOVE_EN                          = _UxGT("Extruder *");
  PROGMEM Language_Str MSG_HOTEND_TOO_COLD                  = _UxGT("Hotend too cold");
  PROGMEM Language_Str MSG_MOVE_Z_DIST                      = _UxGT("Move %smm");
  PROGMEM Language_Str MSG_MOVE_01MM                        = _UxGT("Move 0.1mm");
  PROGMEM Language_Str MSG_MOVE_1MM                         = _UxGT("Move 1mm");
  PROGMEM Language_Str MSG_MOVE_10MM                        = _UxGT("Move 10mm");
  PROGMEM Language_Str MSG_SPEED                            = _UxGT("Speed");
  PROGMEM Language_Str MSG_BED_Z                            = _UxGT("Bed Z");
  PROGMEM Language_Str MSG_NOZZLE                           = _UxGT("Nozzle");
  PROGMEM Language_Str MSG_BED                              = _UxGT("Bed");
  PROGMEM Language_Str MSG_CHAMBER                          = _UxGT("Chamber");
  PROGMEM Language_Str MSG_COOLER                           = _UxGT("Cooler");
  PROGMEM Language_Str MSG_FAN_SPEED                        = _UxGT("Fan Speed");
  PROGMEM Language_Str MSG_EXTRA_FAN_SPEED                  = _UxGT("Extra Fan Speed");
  PROGMEM Language_Str MSG_FLOW                             = _UxGT("Flow");
  PROGMEM Language_Str MSG_CONTROL                          = _UxGT("Control");
  PROGMEM Language_Str MSG_MIN                              = " " LCD_STR_THERMOMETER _UxGT(" Min");
  PROGMEM Language_Str MSG_MAX                              = " " LCD_STR_THERMOMETER _UxGT(" Max");
  PROGMEM Language_Str MSG_FACTOR                           = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  PROGMEM Language_Str MSG_IDLEOOZING                       = _UxGT("Anti oozing");
  PROGMEM Language_Str MSG_AUTOTEMP                         = _UxGT("Autotemp");
  PROGMEM Language_Str MSG_LCD_ON                           = _UxGT("On");
  PROGMEM Language_Str MSG_LCD_OFF                          = _UxGT("Off");
  PROGMEM Language_Str MSG_PID_AUTOTUNE                     = _UxGT("PID Autotune");
  PROGMEM Language_Str MSG_PID_BED_AUTOTUNE                 = _UxGT("Bed PID Autotune");
  PROGMEM Language_Str MSG_PID_CHAMBER_AUTOTUNE             = _UxGT("Chamber PID Autotune");
  PROGMEM Language_Str MSG_PID_AUTOTUNE_START               = _UxGT("PID Autotune start");
  PROGMEM Language_Str MSG_PID_P                            = _UxGT("PID-P");
  PROGMEM Language_Str MSG_PID_I                            = _UxGT("PID-I");
  PROGMEM Language_Str MSG_PID_D                            = _UxGT("PID-D");
  PROGMEM Language_Str MSG_PID_C                            = _UxGT("PID-C");
  PROGMEM Language_Str MSG_BED_PID_P                        = _UxGT("Bed PID-P");
  PROGMEM Language_Str MSG_BED_PID_I                        = _UxGT("Bed PID-I");
  PROGMEM Language_Str MSG_BED_PID_D                        = _UxGT("Bed PID-D");
  PROGMEM Language_Str MSG_CHAMBER_PID_P                    = _UxGT("Chamber PID-P");
  PROGMEM Language_Str MSG_CHAMBER_PID_I                    = _UxGT("Chamber PID-I");
  PROGMEM Language_Str MSG_CHAMBER_PID_D                    = _UxGT("Chamber PID-D");
  PROGMEM Language_Str MSG_SELECT                           = _UxGT("Select");
  PROGMEM Language_Str MSG_ACC                              = _UxGT("Accel");
  PROGMEM Language_Str MSG_JERK                             = _UxGT("Jerk");
  PROGMEM Language_Str MSG_VA_JERK                          = _UxGT("Jerk-V") LCD_STR_A;
  PROGMEM Language_Str MSG_VB_JERK                          = _UxGT("Jerk-V") LCD_STR_B;
  PROGMEM Language_Str MSG_VC_JERK                          = _UxGT("Jerk-V") LCD_STR_C;
  PROGMEM Language_Str MSG_VE_JERK                          = _UxGT("Jerk-V") LCD_STR_E;
  PROGMEM Language_Str MSG_JUNCTION_DEVIATION               = _UxGT("Junction Dev");
  PROGMEM Language_Str MSG_JUNCTION_MM                      = _UxGT("Junction mm");
  PROGMEM Language_Str MSG_VELOCITY                         = _UxGT("Velocity");
  PROGMEM Language_Str MSG_VMAX_A                           = _UxGT("Vmax ") LCD_STR_A;
  PROGMEM Language_Str MSG_VMAX_B                           = _UxGT("Vmax ") LCD_STR_B;
  PROGMEM Language_Str MSG_VMAX_C                           = _UxGT("Vmax ") LCD_STR_C;
  PROGMEM Language_Str MSG_VMAX_E                           = _UxGT("Vmax ") LCD_STR_E;
  PROGMEM Language_Str MSG_VMIN                             = _UxGT("Vmin");
  PROGMEM Language_Str MSG_VTRAV_MIN                        = _UxGT("VTrav Min");
  PROGMEM Language_Str MSG_ACCELERATION                     = _UxGT("Acceleration");
  PROGMEM Language_Str MSG_AMAX_A                           = _UxGT("Amax ") LCD_STR_A;
  PROGMEM Language_Str MSG_AMAX_B                           = _UxGT("Amax ") LCD_STR_B;
  PROGMEM Language_Str MSG_AMAX_C                           = _UxGT("Amax ") LCD_STR_C;
  PROGMEM Language_Str MSG_AMAX_E                           = _UxGT("Amax ") LCD_STR_E;
  PROGMEM Language_Str MSG_A_RETRACT                        = _UxGT("A-Retract E");
  PROGMEM Language_Str MSG_A_TRAVEL                         = _UxGT("A-Travel");
  PROGMEM Language_Str MSG_STEPS_PER_MM                     = _UxGT("Steps/mm");
  PROGMEM Language_Str MSG_A_STEPS                          = _UxGT("steps/mm ") LCD_STR_A;
  PROGMEM Language_Str MSG_B_STEPS                          = _UxGT("steps/mm ") LCD_STR_B;
  PROGMEM Language_Str MSG_C_STEPS                          = _UxGT("steps/mm ") LCD_STR_C;
  PROGMEM Language_Str MSG_E_STEPS                          = _UxGT("steps/mm ") LCD_STR_E;
  PROGMEM Language_Str MSG_TEMPERATURE                      = _UxGT("Temperature");
  PROGMEM Language_Str MSG_MOTION                           = _UxGT("Motion");
  PROGMEM Language_Str MSG_FILAMENT                         = _UxGT("Filament");
  PROGMEM Language_Str MSG_VOLUMETRIC_ENABLED               = _UxGT("E in mm");
  PROGMEM Language_Str MSG_FILAMENT_DIAM                    = _UxGT("Fil. Dia.");
  PROGMEM Language_Str MSG_FILAMENT_UNLOAD                  = _UxGT("Unload mm");
  PROGMEM Language_Str MSG_FILAMENT_LOAD                    = _UxGT("Load mm");
  PROGMEM Language_Str MSG_ADVANCE_K                        = _UxGT("Advance K");
  PROGMEM Language_Str MSG_CONTRAST                         = _UxGT("LCD Contrast");
  PROGMEM Language_Str MSG_STORE_EEPROM                     = _UxGT("Store Settings");
  PROGMEM Language_Str MSG_LOAD_EEPROM                      = _UxGT("Load Settings");
  PROGMEM Language_Str MSG_RESTORE_FAILSAFE                 = _UxGT("Restore failsafe");
  PROGMEM Language_Str MSG_INIT_EEPROM                      = _UxGT("Initialize EEPROM");
  PROGMEM Language_Str MSG_SD_UPDATE                        = _UxGT("SD Update");
  PROGMEM Language_Str MSG_RESET_PRINTER                    = _UxGT("Reset Printer");
  PROGMEM Language_Str MSG_REFRESH                          = LCD_STR_REFRESH  _UxGT("Refresh");
  PROGMEM Language_Str MSG_WATCH                            = _UxGT("Info Screen");
  PROGMEM Language_Str MSG_PREPARE                          = _UxGT("Prepare");
  PROGMEM Language_Str MSG_TUNE                             = _UxGT("Tune");
  PROGMEM Language_Str MSG_START_PRINT                      = _UxGT("Start Print");
  PROGMEM Language_Str MSG_BUTTON_NEXT                      = _UxGT("Next");
  PROGMEM Language_Str MSG_BUTTON_INIT                      = _UxGT("Init");
  PROGMEM Language_Str MSG_BUTTON_STOP                      = _UxGT("Stop");
  PROGMEM Language_Str MSG_BUTTON_PRINT                     = _UxGT("Print");
  PROGMEM Language_Str MSG_BUTTON_RESET                     = _UxGT("Reset");
  PROGMEM Language_Str MSG_BUTTON_CANCEL                    = _UxGT("Cancel");
  PROGMEM Language_Str MSG_BUTTON_DONE                      = _UxGT("Done");
  PROGMEM Language_Str MSG_PAUSE_PRINT                      = _UxGT("Pause Print");
  PROGMEM Language_Str MSG_RESUME_PRINT                     = _UxGT("Resume Print");
  PROGMEM Language_Str MSG_STOP_PRINT                       = _UxGT("Stop Print");
  PROGMEM Language_Str MSG_RESTART                          = _UxGT("Restart");
  PROGMEM Language_Str MSG_CARD_MENU                        = _UxGT("Print from SD");
  PROGMEM Language_Str MSG_NO_CARD                          = _UxGT("No SD Card");
  PROGMEM Language_Str MSG_DWELL                            = _UxGT("Sleep...");
  PROGMEM Language_Str MSG_USERWAIT                         = _UxGT("Click to Resume...");
  PROGMEM Language_Str MSG_PRINT_PAUSED                     = _UxGT("Print Paused");
  PROGMEM Language_Str MSG_PRINTING                         = _UxGT("Printing...");
  PROGMEM Language_Str MSG_RESUMING                         = _UxGT("Resuming print");
  PROGMEM Language_Str MSG_PRINT_ABORTED                    = _UxGT("Print Aborted");
  PROGMEM Language_Str MSG_NO_MOVE                          = _UxGT("No Move.");
  PROGMEM Language_Str MSG_KILLED                           = _UxGT("KILLED. ");
  PROGMEM Language_Str MSG_STOPPED                          = _UxGT("STOPPED. ");
  PROGMEM Language_Str MSG_CONTROL_RETRACT                  = _UxGT("Retract mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_SWAP             = _UxGT("Swap Re.mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACTF                 = _UxGT("Retract  V");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_ZHOP             = _UxGT("Hop mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER          = _UxGT("UnRet mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAP     = _UxGT("S UnRet mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVERF         = _UxGT("UnRet V");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAPF    = _UxGT("S UnRet V");
  PROGMEM Language_Str MSG_AUTORETRACT                      = _UxGT("AutoRetr.");
  PROGMEM Language_Str MSG_FILAMENT_SWAP_LENGTH             = _UxGT("Swap Length");
  PROGMEM Language_Str MSG_FILAMENT_PURGE_LENGTH            = _UxGT("Purge Length");
  PROGMEM Language_Str MSG_TOOL_CHANGE                      = _UxGT("Tool Change");
  PROGMEM Language_Str MSG_TOOL_CHANGE_ZLIFT                = _UxGT("Z Raise");
  PROGMEM Language_Str MSG_SINGLENOZZLE_PRIME_SPD           = _UxGT("Prime Speed");
  PROGMEM Language_Str MSG_SINGLENOZZLE_RETRACT_SPD         = _UxGT("Retract Speed");
  PROGMEM Language_Str MSG_NOZZLE_STANDBY                   = _UxGT("Nozzle Standby");
  PROGMEM Language_Str MSG_FILAMENTCHANGE                   = _UxGT("Change Filament");
  PROGMEM Language_Str MSG_FILAMENTLOAD                     = _UxGT("Load Filament");
  PROGMEM Language_Str MSG_FILAMENTUNLOAD                   = _UxGT("Unload Filament");
  PROGMEM Language_Str MSG_FILAMENTUNLOAD_ALL               = _UxGT("Unload All");
  PROGMEM Language_Str MSG_INIT_SDCARD                      = _UxGT("Init. SD Card");
  PROGMEM Language_Str MSG_CHANGE_SDCARD                    = _UxGT("Change SD Card");
  PROGMEM Language_Str MSG_RELEASE_SDCARD                   = _UxGT("Release SD Card");
  PROGMEM Language_Str MSG_ZPROBE_OUT                       = _UxGT("Z Probe Past Bed");
  PROGMEM Language_Str MSG_SKEW_FACTOR                      = _UxGT("Skew Factor");
  PROGMEM Language_Str MSG_BLTOUCH                          = _UxGT("BLTouch");
  PROGMEM Language_Str MSG_BLTOUCH_SELFTEST                 = _UxGT("Self-Test");
  PROGMEM Language_Str MSG_BLTOUCH_RESET                    = _UxGT("Reset");
  PROGMEM Language_Str MSG_BLTOUCH_STOW                     = _UxGT("Stow");
  PROGMEM Language_Str MSG_BLTOUCH_DEPLOY                   = _UxGT("Deploy");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_SW                  = _UxGT("Mode SW");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_5V                  = _UxGT("Mode 5V");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_OD                  = _UxGT("Mode OD");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE               = _UxGT("Mode Store");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE_5V            = _UxGT("Set to 5V");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE_OD            = _UxGT("Set to OD");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_ECHO                = _UxGT("Report Drain");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_CHANGE              = _UxGT("DANGER: Bad settings can cause damage! Proceed anyway?");
  PROGMEM Language_Str MSG_MANUAL_DEPLOY                    = _UxGT("Deploy Z-Probe");
  PROGMEM Language_Str MSG_MANUAL_STOW                      = _UxGT("Stow Z-Probe");
  PROGMEM Language_Str MSG_HOME_FIRST                       = _UxGT("Home %s%s%s First");
  PROGMEM Language_Str MSG_ZPROBE_ZOFFSET                   = _UxGT("Z Offset");
  PROGMEM Language_Str MSG_BABYSTEP_X                       = _UxGT("Babystep X");
  PROGMEM Language_Str MSG_BABYSTEP_Y                       = _UxGT("Babystep Y");
  PROGMEM Language_Str MSG_BABYSTEP_Z                       = _UxGT("Babystep Z");
  PROGMEM Language_Str MSG_ENDSTOP_ABORT                    = _UxGT("Endstop Abort");
  PROGMEM Language_Str MSG_HEATING_FAILED                   = _UxGT("Heating Failed");
  PROGMEM Language_Str MSG_ERR_REDUNDANT_TEMP               = _UxGT("Err: REDUNDANT TEMP");
  PROGMEM Language_Str MSG_THERMAL_RUNAWAY                  = _UxGT("THERMAL RUNAWAY");
  PROGMEM Language_Str MSG_AD595                            = _UxGT("AD595 Offset & Gain");
  PROGMEM Language_Str MSG_ERR_MAXTEMP                      = _UxGT("Err: MAXTEMP");
  PROGMEM Language_Str MSG_ERR_MINTEMP                      = _UxGT("Err: MINTEMP");
  PROGMEM Language_Str MSG_ERR_MAXTEMP_BED                  = _UxGT("Err: MAXTEMP BED");
  PROGMEM Language_Str MSG_ERR_MINTEMP_BED                  = _UxGT("Err: MINTEMP BED");
  PROGMEM Language_Str MSG_ERR_MAXTEMP_CHAMBER              = _UxGT("Err: MAXTEMP CHAMBER");
  PROGMEM Language_Str MSG_ERR_MINTEMP_CHAMBER              = _UxGT("Err: MINTEMP CHAMBER");
  PROGMEM Language_Str MSG_ERR_MAXTEMP_COOLER               = _UxGT("Err: MAXTEMP COOLER");
  PROGMEM Language_Str MSG_ERR_MINTEMP_COOLER               = _UxGT("Err: MINTEMP COOLER");
  PROGMEM Language_Str MSG_ERR_Z_HOMING                     = _UxGT("Home XY First");
  PROGMEM Language_Str MSG_HALTED                           = _UxGT("PRINTER HALTED");
  PROGMEM Language_Str MSG_PLEASE_RESET                     = _UxGT("Please Reset");
  PROGMEM Language_Str MSG_SHORT_DAY                        = _UxGT("d"); // One character only
  PROGMEM Language_Str MSG_SHORT_HOUR                       = _UxGT("h"); // One character only
  PROGMEM Language_Str MSG_SHORT_MINUTE                     = _UxGT("m"); // One character only
  PROGMEM Language_Str MSG_LONG_DAY                         = _UxGT("days");
  PROGMEM Language_Str MSG_LONG_HOUR                        = _UxGT("hours");
  PROGMEM Language_Str MSG_LONG_MINUTE                      = _UxGT("minutes");
  PROGMEM Language_Str MSG_HEATING                          = _UxGT("Heating...");
  PROGMEM Language_Str MSG_HEATING_COMPLETE                 = _UxGT("Heating Done.");
  PROGMEM Language_Str MSG_COOLING                          = _UxGT("Cooling...");
  PROGMEM Language_Str MSG_COOLING_COMPLETE                 = _UxGT("Cooling Done.");
  PROGMEM Language_Str MSG_BED_HEATING                      = _UxGT("Bed Heating...");
  PROGMEM Language_Str MSG_BED_COOLING                      = _UxGT("Bed Cooling...");
  PROGMEM Language_Str MSG_BED_DONE                         = _UxGT("Bed Done.");
  PROGMEM Language_Str MSG_CHAMBER_HEATING                  = _UxGT("Chamber Heating...");
  PROGMEM Language_Str MSG_CHAMBER_COOLING                  = _UxGT("Chamber Cooling...");
  PROGMEM Language_Str MSG_CHAMBER_DONE                     = _UxGT("Chamber Done.");
  PROGMEM Language_Str MSG_COOLER_COOLING                   = _UxGT("Cooler Cooling.");
  PROGMEM Language_Str MSG_COOLER_DONE                      = _UxGT("Cooler Done.");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE                  = _UxGT("Delta Calibration");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_X                = _UxGT("Calibrate X");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_Y                = _UxGT("Calibrate Y");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_Z                = _UxGT("Calibrate Z");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_CENTER           = _UxGT("Calibrate Center");
  PROGMEM Language_Str MSG_DELTA_SETTINGS                   = _UxGT("Delta Settings");
  PROGMEM Language_Str MSG_DELTA_AUTO_CALIBRATE             = _UxGT("Auto Calibration");
  PROGMEM Language_Str MSG_DELTA_HEIGHT_CALIBRATE           = _UxGT("Set Delta Height");
  PROGMEM Language_Str MSG_DELTA_DIAG_ROD                   = _UxGT("Diag Rod");
  PROGMEM Language_Str MSG_DELTA_HEIGHT                     = _UxGT("Height");
  PROGMEM Language_Str MSG_DELTA_RADIUS                     = _UxGT("Radius");
  PROGMEM Language_Str MSG_DELTA_AUTO_CALIBRATE_OK          = _UxGT("Calibration OK");
  PROGMEM Language_Str MSG_INFO_MENU                        = _UxGT("About Printer");
  PROGMEM Language_Str MSG_INFO_FIRMWARE_MENU               = _UxGT("Firmware Info");
  PROGMEM Language_Str MSG_INFO_PRINTER_MENU                = _UxGT("Printer Info");
  PROGMEM Language_Str MSG_3POINT_LEVELING                  = _UxGT("3-Point Leveling");
  PROGMEM Language_Str MSG_LINEAR_LEVELING                  = _UxGT("Linear Leveling");
  PROGMEM Language_Str MSG_BILINEAR_LEVELING                = _UxGT("Bilinear Leveling");
  PROGMEM Language_Str MSG_UBL_LEVELING                     = _UxGT("Unified Bed Leveling");
  PROGMEM Language_Str MSG_MESH_LEVELING                    = _UxGT("Mesh Leveling");
  PROGMEM Language_Str MSG_INFO_STATS_MENU                  = _UxGT("Printer Stats");
  PROGMEM Language_Str MSG_INFO_BOARD_MENU                  = _UxGT("Board Info");
  PROGMEM Language_Str MSG_INFO_THERMISTOR_MENU             = _UxGT("Thermistors");
  PROGMEM Language_Str MSG_INFO_EXTRUDERS                   = _UxGT("Extruders");
  PROGMEM Language_Str MSG_INFO_HOTENDS                     = _UxGT("Hotends");
  PROGMEM Language_Str MSG_INFO_BEDS                        = _UxGT("Beds");
  PROGMEM Language_Str MSG_INFO_CHAMBERS                    = _UxGT("Chambers");
  PROGMEM Language_Str MSG_INFO_COOLER                      = _UxGT("Coolers");
  PROGMEM Language_Str MSG_INFO_BAUDRATE                    = _UxGT("Baud");
  PROGMEM Language_Str MSG_INFO_PROTOCOL                    = _UxGT("Protocol");

  PROGMEM Language_Str MSG_CASE_LIGHT                       = _UxGT("Case Light");
  PROGMEM Language_Str MSG_CASE_LIGHT_BRIGHTNESS            = _UxGT("Light Brightness");
  PROGMEM Language_Str MSG_EXPECTED_PRINTER                 = _UxGT("INCORRECT PRINTER");

  #if LCD_WIDTH >= 20
    PROGMEM Language_Str MSG_INFO_PRINT_COUNT               = _UxGT("Total prints");
    PROGMEM Language_Str MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completed");
    PROGMEM Language_Str MSG_INFO_PRINT_TIME                = _UxGT("Total Print time");
    PROGMEM Language_Str MSG_INFO_PRINT_LONGEST             = _UxGT("Longest Job Time");
    PROGMEM Language_Str MSG_INFO_POWER_ON                  = _UxGT("Power on time");
    PROGMEM Language_Str MSG_INFO_PRINT_FILAMENT            = _UxGT("Filament Total");
  #else
    PROGMEM Language_Str MSG_INFO_PRINT_COUNT               = _UxGT("Prints");
    PROGMEM Language_Str MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completed");
    PROGMEM Language_Str MSG_INFO_PRINT_TIME                = _UxGT("Total");
    PROGMEM Language_Str MSG_INFO_PRINT_LONGEST             = _UxGT("Longest");
    PROGMEM Language_Str MSG_INFO_POWER_ON                  = _UxGT("Power on");
    PROGMEM Language_Str MSG_INFO_PRINT_FILAMENT            = _UxGT("Filament");
  #endif

  PROGMEM Language_Str MSG_INFO_PWRCONSUMED                 = _UxGT("PWR");
  PROGMEM Language_Str MSG_INFO_MIN_TEMP                    = _UxGT("Min Temp");
  PROGMEM Language_Str MSG_INFO_MAX_TEMP                    = _UxGT("Max Temp");
  PROGMEM Language_Str MSG_INFO_PSU                         = _UxGT("PSU");
  PROGMEM Language_Str MSG_DRIVE_STRENGTH                   = _UxGT("Drive Strength");
  PROGMEM Language_Str MSG_DAC_PERCENT                      = _UxGT("Driver %");
  PROGMEM Language_Str MSG_DAC_EEPROM_WRITE                 = _UxGT("DAC EEPROM Write");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER           = _UxGT("FILAMENT CHANGE");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_PAUSE     = _UxGT("PRINT PAUSED");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_LOAD      = _UxGT("LOAD FILAMENT");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_UNLOAD    = _UxGT("UNLOAD FILAMENT");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_HEADER    = _UxGT("RESUME OPTIONS:");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_PURGE     = _UxGT("Purge more");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_RESUME    = _UxGT("Continue");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_NOZZLE           = _UxGT("  Nozzle: ");
  PROGMEM Language_Str MSG_RUNOUT_SENSOR                    = _UxGT("Runout Sensor");
  PROGMEM Language_Str MSG_RUNOUT_DISTANCE_MM               = _UxGT("Runout Dist mm");
  PROGMEM Language_Str MSG_LCD_HOMING_FAILED                = _UxGT("Homing Failed");
  PROGMEM Language_Str MSG_LCD_PROBING_FAILED               = _UxGT("Probing Failed");
  PROGMEM Language_Str MSG_M600_TOO_COLD                    = _UxGT("M600: Too Cold");

  PROGMEM Language_Str MSG_MMU2_CHOOSE_FILAMENT_HEADER      = _UxGT("CHOOSE FILAMENT");
  PROGMEM Language_Str MSG_MMU2_MENU                        = _UxGT("MMU");
  PROGMEM Language_Str MSG_MMU2_WRONG_FIRMWARE              = _UxGT("Update MMU Firmware!");
  PROGMEM Language_Str MSG_MMU2_NOT_RESPONDING              = _UxGT("MMU Needs Attention.");
  PROGMEM Language_Str MSG_MMU2_RESUME                      = _UxGT("Resume Print");
  PROGMEM Language_Str MSG_MMU2_RESUMING                    = _UxGT("Resuming...");
  PROGMEM Language_Str MSG_MMU2_LOAD_FILAMENT               = _UxGT("Load Filament");
  PROGMEM Language_Str MSG_MMU2_LOAD_ALL                    = _UxGT("Load All");
  PROGMEM Language_Str MSG_MMU2_LOAD_TO_NOZZLE              = _UxGT("Load to Nozzle");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT              = _UxGT("Eject Filament");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT_N            = _UxGT("Eject Filament E");
  PROGMEM Language_Str MSG_MMU2_UNLOAD_FILAMENT             = _UxGT("Unload Filament");
  PROGMEM Language_Str MSG_MMU2_LOADING_FILAMENT            = _UxGT("Loading Fil. %i...");
  PROGMEM Language_Str MSG_MMU2_EJECTING_FILAMENT           = _UxGT("Ejecting Fil. ...");
  PROGMEM Language_Str MSG_MMU2_UNLOADING_FILAMENT          = _UxGT("Unloading Fil....");
  PROGMEM Language_Str MSG_MMU2_ALL                         = _UxGT("All");
  PROGMEM Language_Str MSG_MMU2_FILAMENT_N                  = _UxGT("Filament E");
  PROGMEM Language_Str MSG_MMU2_RESET                       = _UxGT("Reset MMU");
  PROGMEM Language_Str MSG_MMU2_RESETTING                   = _UxGT("Resetting MMU...");
  PROGMEM Language_Str MSG_MMU2_EJECT_RECOVER               = _UxGT("Remove, click");

  PROGMEM Language_Str MSG_MIX                              = _UxGT("Mix");
  PROGMEM Language_Str MSG_MIX_COMPONENT                    = _UxGT("Component");
  PROGMEM Language_Str MSG_MIXER                            = _UxGT("Mixer");
  PROGMEM Language_Str MSG_GRADIENT                         = _UxGT("Gradient");
  PROGMEM Language_Str MSG_FULL_GRADIENT                    = _UxGT("Full Gradient");
  PROGMEM Language_Str MSG_TOGGLE_MIX                       = _UxGT("Toggle Mix");
  PROGMEM Language_Str MSG_CYCLE_MIX                        = _UxGT("Cycle Mix");
  PROGMEM Language_Str MSG_GRADIENT_MIX                     = _UxGT("Gradient Mix");
  PROGMEM Language_Str MSG_REVERSE_GRADIENT                 = _UxGT("Reverse Gradient");
  PROGMEM Language_Str MSG_ACTIVE_VTOOL                     = _UxGT("Active V-tool");
  PROGMEM Language_Str MSG_START_VTOOL                      = _UxGT("Start V-tool");
  PROGMEM Language_Str MSG_END_VTOOL                        = _UxGT("  End V-tool");
  PROGMEM Language_Str MSG_GRADIENT_ALIAS                   = _UxGT("Alias V-tool");
  PROGMEM Language_Str MSG_RESET_VTOOLS                     = _UxGT("Reset V-tools");
  PROGMEM Language_Str MSG_COMMIT_VTOOL                     = _UxGT("Commit V-tool Mix");
  PROGMEM Language_Str MSG_VTOOLS_RESET                     = _UxGT("V-tools Were Reset");
  PROGMEM Language_Str MSG_START_Z                          = _UxGT("Start Z");
  PROGMEM Language_Str MSG_END_Z                            = _UxGT("  End Z");

  PROGMEM Language_Str MSG_GAMES                            = _UxGT("Games");
  PROGMEM Language_Str MSG_BRICKOUT                         = _UxGT("Brickout");
  PROGMEM Language_Str MSG_INVADERS                         = _UxGT("Invaders");
  PROGMEM Language_Str MSG_SNAKE                            = _UxGT("Sn4k3");
  PROGMEM Language_Str MSG_MAZE                             = _UxGT("Maze");

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING_1       = _UxGT("Press Button");
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING_2       = _UxGT("to resume print");
    PROGMEM Language_Str MSG_PAUSE_PRINT_INIT_1             = _UxGT("Parking...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_1         = _UxGT("Wait for");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_2         = _UxGT("filament change");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_3         = _UxGT("to start");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_1       = _UxGT("Insert filament");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_2       = _UxGT("and press button");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_3       = _UxGT("to continue");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT_1         = _UxGT("Press button");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT_2         = _UxGT("to heat nozzle");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_ZZZ_1          = _UxGT(" z   z   z");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_ZZZ_2          = _UxGT("Z   Z   Z");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING_1      = _UxGT("Nozzle heating");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING_2      = _UxGT("Please wait...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD_1       = _UxGT("Wait for filament");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD_2       = _UxGT("to fully unload");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD_1         = _UxGT("Wait for");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD_2         = _UxGT("filament load");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE_1        = _UxGT("Wait for");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE_2        = _UxGT("filament purge");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE_1   = _UxGT("Click to finish");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE_2   = _UxGT("filament purge");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME_1       = _UxGT("Wait for print");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME_2       = _UxGT("to resume...");
  #else // LCD_HEIGHT < 4
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING_1       = _UxGT("Click to continue");
    PROGMEM Language_Str MSG_PAUSE_PRINT_INIT_1             = _UxGT("Parking...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_1         = _UxGT("Please wait...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_1       = _UxGT("Insert and Click");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT_1         = _UxGT("Click to heat");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING_1      = _UxGT("Heating...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD_1       = _UxGT("Ejecting...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD_1         = _UxGT("Loading...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE_1        = _UxGT("Purging...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE_1   = _UxGT("Click to finish");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME_1       = _UxGT("Resuming...");
  #endif  

  PROGMEM Language_Str MSG_TMC_DRIVERS                      = _UxGT("TMC Drivers");
  PROGMEM Language_Str MSG_TMC_CURRENT                      = _UxGT("Driver Current");
  PROGMEM Language_Str MSG_TMC_MICROSTEP                    = _UxGT("Driver Microstep");
  PROGMEM Language_Str MSG_TMC_HYBRID_THRS                  = _UxGT("Hybrid Threshold");
  PROGMEM Language_Str MSG_TMC_HOMING_THRS                  = _UxGT("Sensorless Homing");
  PROGMEM Language_Str MSG_TMC_STEPPING_MODE                = _UxGT("Stepping Mode");
  PROGMEM Language_Str MSG_TMC_STEALTH_ENABLED              = _UxGT("StealthChop Enabled");

  PROGMEM Language_Str MSG_SERVICE_RESET                    = _UxGT("Reset");
  PROGMEM Language_Str MSG_SERVICE_IN                       = _UxGT(" in:");
  PROGMEM Language_Str MSG_MAX_INACTIVITY_TIME              = _UxGT("Heating disabled by safety timer.");

  // Max extruder
  PROGMEM Language_Str MSG_MAX_EXTRUDERS                    = _UxGT("Extruders");

  // Extra
  PROGMEM Language_Str MSG_RESTART_PRINT                    = _UxGT("Restart print");
  PROGMEM Language_Str MSG_FIX_LOSE_STEPS                   = _UxGT("Fix axis steps");
  PROGMEM Language_Str MSG_LASER                            = _UxGT("Laser Preset");
  PROGMEM Language_Str MSG_NEED_TUNE_PID                    = _UxGT("Need Tune PID");
  PROGMEM Language_Str MSG_ARE_YOU_SURE                     = _UxGT("Are you sure");

  // Rfid module
  PROGMEM Language_Str MSG_RFID_SPOOL                       = _UxGT("Spool on E");
  PROGMEM Language_Str MSG_RFID_BRAND                       = _UxGT("Brand: ");
  PROGMEM Language_Str MSG_RFID_TYPE                        = _UxGT("Type: ");
  PROGMEM Language_Str MSG_RFID_COLOR                       = _UxGT("Color: ");
  PROGMEM Language_Str MSG_RFID_SIZE                        = _UxGT("Size: ");
  PROGMEM Language_Str MSG_RFID_TEMP_HOTEND                 = _UxGT("Temperature Hotend: ");
  PROGMEM Language_Str MSG_RFID_TEMP_BED                    = _UxGT("Temperature Bed: ");
  PROGMEM Language_Str MSG_RFID_TEMP_USER_HOTEND            = _UxGT("User temperature Hotend: ");
  PROGMEM Language_Str MSG_RFID_TEMP_USER_BED               = _UxGT("User temperatura Bed: ");
  PROGMEM Language_Str MSG_RFID_DENSITY                     = _UxGT("Density: ");
  PROGMEM Language_Str MSG_RFID_SPOOL_LENGHT                = _UxGT("Spool Lenght: ");

  // Sound
  PROGMEM Language_Str MSG_SOUND_MODE_ON                    = _UxGT("Sound          [on]");
  PROGMEM Language_Str MSG_SOUND_MODE_SILENT                = _UxGT("Sound      [silent]");
  PROGMEM Language_Str MSG_SOUND_MODE_MUTE                  = _UxGT("Sound        [mute]");

  // EEPROM Allert
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_1          = _UxGT("ATTENTION...");
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_2          = _UxGT("EEPROM Changed.");
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_3          = _UxGT("Press button");
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_4          = _UxGT("to continue...");

  // Nextion Allert
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_1         = _UxGT("ATTENTION...");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_2         = _UxGT("NEXTION FW changed.");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_3         = _UxGT("Please upload new FW");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_4         = _UxGT("Press button");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_5         = _UxGT("to continue...");

  // Nextion M0 M1
  PROGMEM Language_Str MSG_NEXTION_M0_M1_1                  = _UxGT("Press button enter");
  PROGMEM Language_Str MSG_NEXTION_M0_M1_2                  = _UxGT("to resume print");

  // DHT
  PROGMEM Language_Str MSG_DHT                              = _UxGT("DHT");
  PROGMEM Language_Str MSG_DHT_11                           = _UxGT("DHT11");
  PROGMEM Language_Str MSG_DHT_12                           = _UxGT("DHT12");
  PROGMEM Language_Str MSG_DHT_21                           = _UxGT("DHT21");
  PROGMEM Language_Str MSG_DHT_22                           = _UxGT("DHT22");
  PROGMEM Language_Str MSG_DHT_TEMPERATURE                  = _UxGT("Temperature (C):");
  PROGMEM Language_Str MSG_DHT_HUMIDITY                     = _UxGT("Humidity (%):");
  PROGMEM Language_Str MSG_DHT_DEWPOINT                     = _UxGT("Dew Point (C):");
}
