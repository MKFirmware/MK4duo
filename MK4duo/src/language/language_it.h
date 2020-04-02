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
 * Italian
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace language_it {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 1;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Italiano"));

  FSTRINGVALUE(MSG_WELCOME                          , MACHINE_NAME _UxGT(" pronta."));
  FSTRINGVALUE(MSG_LANGUAGE                         , _UxGT("Linguaggio"));
  FSTRINGVALUE(MSG_YES                              , _UxGT("Si"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("No"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Indietro"));
  FSTRINGVALUE(MSG_SD_INSERTED                      , _UxGT("SD Card inserita"));
  FSTRINGVALUE(MSG_SD_REMOVED                       , _UxGT("SD Card rimossa"));
  FSTRINGVALUE(MSG_SD_RELEASED                      , _UxGT("SD Card rilasciata"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Finecor."));
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Finecorsa Soft"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu principale"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Impostaz. avanzate"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Configurazione"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Autostart"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Disabilita Motori"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Menu di debug"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Test barra avanzam."));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Auto Home"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Home asse X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Home asse Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Home asse Z"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Allineam.automat. Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Home assi XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Premi per iniziare"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Punto successivo"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Livel. terminato!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Fade Height"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Imp. offset home"));
  FSTRINGVALUE(MSG_OFFSETS_APPLIED                  , _UxGT("Offset applicato"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Imposta Origine"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Preriscalda ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" H"));
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" Ugello"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" Tutto"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Preriscalda ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Preris. ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Preris. ") PREHEAT_2_LABEL _UxGT(" H"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Preris. ") PREHEAT_2_LABEL _UxGT(" Tutto"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Preris. ") PREHEAT_2_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_PREHEAT_3                        , _UxGT("Preriscalda ") PREHEAT_3_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_3_H                      , _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" H"));
  FSTRINGVALUE(MSG_PREHEAT_3_END                    , _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" Ugello"));
  FSTRINGVALUE(MSG_PREHEAT_3_ALL                    , _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" Tutto"));
  FSTRINGVALUE(MSG_PREHEAT_3_SETTINGS               , _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Prerisc.personal."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Raffredda"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Accendi aliment."));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Spegni aliment."));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Estrudi"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Ritrai"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Muovi Asse"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Livella piano"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Livella piano"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Livella angoli"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Prossimo angolo"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Modifica Mesh"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Modif. Mesh Fermata"));
  FSTRINGVALUE(MSG_PROBING_MESH                     , _UxGT("Punto sondato"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Indice X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Indice Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Valore di Z"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Comandi Utente"));
  FSTRINGVALUE(MSG_LCD_TILTING_MESH                 , _UxGT("Punto inclinaz."));
  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("Test sonda M48"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Deviazione"));
  FSTRINGVALUE(MSG_DXC_MENU                         , _UxGT("Modo DXC"));
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Strumenti Offsets"));
  FSTRINGVALUE(MSG_DXC_MODE_AUTOPARK                , _UxGT("Auto-Park"));
  FSTRINGVALUE(MSG_DXC_MODE_DUPLICATE               , _UxGT("Duplicazione"));
  FSTRINGVALUE(MSG_DXC_MODE_MIRRORED_COPY           , _UxGT("Copia speculare"));
  FSTRINGVALUE(MSG_DXC_MODE_FULL_CTRL               , _UxGT("Pieno controllo"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2° ugello X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2° ugello Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2° ugello Z"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("G29 in corso"));
  FSTRINGVALUE(MSG_UBL_UNHOMED                      , _UxGT("Home XYZ prima"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("Strumenti UBL"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Mesh Manuale"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Metti spes. e misura"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Misura"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Rimuovi e mis.piatto"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Spostamento succes."));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Attiva UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Disattiva UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Temp. Piatto"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Temp. Piatto"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Temp. Ugello"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Temp. Ugello"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Modifica Mesh"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Modif.Mesh personal."));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Ritocca Mesh"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Modif.Mesh fatta"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Crea Mesh personal."));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Crea Mesh"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Crea Mesh ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Crea Mesh ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Crea Mesh a freddo"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Aggiusta Alt. Mesh"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Altezza"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Valida Mesh"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Valida Mesh ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Valida Mesh ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Valida Mesh pers."));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Continua Mesh"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Livell. Mesh"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("Livell. 3 Punti"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Livell. Griglia Mesh"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Livella Mesh"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Punti laterali"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Tipo di Mappa"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Esporta Mappa"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Esporta per Host"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Esporta in CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Backup esterno"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Esporta Info UBL"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Riempimento"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Riempimento Manuale"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Riempimento Smart"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Riempimento Mesh"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Invalida Tutto"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Invalid.Punto Vicino"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Ritocca All"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Ritocca Punto Vicino"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Mesh Salvate"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Slot di memoria"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Carica Mesh Piatto"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Salva Mesh Piatto"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Mesh %i caricata"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Mesh %i salvata"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Nessuna memoria"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Err: Salvataggio UBL"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Err: Ripristino UBL"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Z-Offset Fermato"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("UBL passo passo"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Crea Mesh a freddo"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Riempimento Smart"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Valida Mesh"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Ritocca All"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Valida Mesh"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Ritocca All"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Salva Mesh Piatto"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Controllo LED"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Luci"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Preset luce"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Rosso"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Arancione"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Giallo"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Verde"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Blu"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Indaco"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Viola"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Bianco"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Predefinito"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Luci personalizzate"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Intensita' rosso"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Intensita' verde"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Intensita' blu"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Intensita' bianco"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Luminosita'"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("In movimento..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("XY liberi"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Muovi X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Muovi Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Muovi Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Estrusore"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Ugello freddo"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Muovi di %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Muovi di 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Muovi di 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Muovi di 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Velocita'"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Piatto Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Ugello"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Piatto"));
  FSTRINGVALUE(MSG_CHAMBER                          , _UxGT("Camera"));
  FSTRINGVALUE(MSG_COOLER                           , _UxGT("Raffreddamento"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Vel. ventola"));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Flusso"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Controllo"));
  FSTRINGVALUE(MSG_FIX_LOSE_STEPS                   , _UxGT("Fix axis steps"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fact"));
  FSTRINGVALUE(MSG_IDLEOOZING                       , _UxGT("Anti oozing"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Autotemp"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("On"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Off"));
  FSTRINGVALUE(MSG_PID_AUTOTUNE                     , _UxGT("PID Autotune"));
  FSTRINGVALUE(MSG_PID_BED_AUTOTUNE                 , _UxGT("Letto PID Autotune"));
  FSTRINGVALUE(MSG_PID_CHAMBER_AUTOTUNE             , _UxGT("Camera PID Autotune"));
  FSTRINGVALUE(MSG_PID_AUTOTUNE_START               , _UxGT("PID Autotune partito"));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_BED_PID_P                        , _UxGT("Letto PID-P"));
  FSTRINGVALUE(MSG_BED_PID_I                        , _UxGT("Letto PID-I"));
  FSTRINGVALUE(MSG_BED_PID_D                        , _UxGT("Letto PID-D"));
  FSTRINGVALUE(MSG_CHAMBER_PID_P                    , _UxGT("Camera PID-P"));
  FSTRINGVALUE(MSG_CHAMBER_PID_I                    , _UxGT("Camera PID-I"));
  FSTRINGVALUE(MSG_CHAMBER_PID_D                    , _UxGT("Camera PID-D"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Seleziona"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Accel"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Jerk"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("Jerk-V") LCD_STR_A);
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("Jerk-V") LCD_STR_B);
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("Jerk-V") LCD_STR_C);
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Jerk-V") LCD_STR_E);
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Velocita'"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("Vmax ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("Vmax ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("Vmax ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vmax ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("Vmin"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("VTrav min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Accelerazione"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Amax ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Amax ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Amax ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Amax ") LCD_STR_E);
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-Ritrazione E"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-Spostamento"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Passi/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , _UxGT("passi/mm ") LCD_STR_A);
  FSTRINGVALUE(MSG_B_STEPS                          , _UxGT("passi/mm ") LCD_STR_B);
  FSTRINGVALUE(MSG_C_STEPS                          , _UxGT("passi/mm ") LCD_STR_C);
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("passi/mm ") LCD_STR_E);
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Movimento"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filamento"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Diam. filo"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Rimuovi mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Carica mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("K Avanzamento"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Contrasto LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Salva impostazioni"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Carica impostazioni"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Ripristina imp."));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Inizializza EEPROM"));
  FSTRINGVALUE(MSG_SD_UPDATE                        , _UxGT("Aggiorna SD"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Resetta stampante"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH _UxGT("Aggiorna"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Schermata info"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Prepara"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Regola"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Avvia stampa"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Prossimo"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Inizializza"));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Stop"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Stampa"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Resetta"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Annulla"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Fatto"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausa stampa"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Riprendi stampa"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Arresta stampa"));
  FSTRINGVALUE(MSG_RESTART                          , _UxGT("Restart"));
  FSTRINGVALUE(MSG_CARD_MENU                        , _UxGT("Stampa da SD"));
  FSTRINGVALUE(MSG_NO_CARD                          , _UxGT("SD non presente"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Sospensione..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Premi tasto.."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Stampa sospesa"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Stampa..."));
  FSTRINGVALUE(MSG_RESUMING                         , _UxGT("Ripresa Stampa"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Stampa annullata"));
  FSTRINGVALUE(MSG_PRINT_DONE                       , _UxGT("Stampa finita"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Nessun Movimento"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("UCCISO. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ARRESTATO. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Ritrai mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Scamb. Ritrai mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Ritrai  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Salta mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Scamb. UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("AutoRitrai"));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Dist. swap"));
  FSTRINGVALUE(MSG_FILAMENT_PURGE_LENGTH            , _UxGT("Dist. pulizia"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Cambio utensile"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Risalita Z"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Velocita' pulizia"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Velocita' retrazione"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Cambia filamento"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Carica filamento"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Rimuovi filamento"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Rimuovi tutto"));
  FSTRINGVALUE(MSG_INIT_SDCARD                      , _UxGT("Iniz. SD card"));
  FSTRINGVALUE(MSG_CHANGE_SDCARD                    , _UxGT("Cambia SD card"));
  FSTRINGVALUE(MSG_RELEASE_SDCARD                   , _UxGT("Rilascia SD card"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z probe fuori piatto"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Autotest"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Resetta"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Estendi"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_SW                  , _UxGT("Modo SW"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_5V                  , _UxGT("Modo 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_OD                  , _UxGT("Modo OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE               , _UxGT("Mode Store"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Ritrai"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_5V            , _UxGT("Setta a 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_OD            , _UxGT("Setta a OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_ECHO                , _UxGT("Report Drain"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_CHANGE              , _UxGT("PERICOLO: Impostazioni errate possono causare danni! Procedere comunque?"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Estendi Probe"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Ritrai Probe"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Home %s%s%s prima"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Finecorsa abort"));
  FSTRINGVALUE(MSG_HEATING_FAILED                   , _UxGT("Riscald. Fallito"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Err: TEMP RIDONDANTE"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("TEMP FUORI CONTROLLO"));
  FSTRINGVALUE(MSG_AD595                            , _UxGT("AD595 Offset & Gain"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err: TEMP MASSIMA"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err: TEMP MINIMA"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Err: TEMP MAX PIATTO"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Err: TEMP MIN PIATTO"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_CHAMBER              , _UxGT("Err: TEMP MAX CAMERA"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_CHAMBER              , _UxGT("Err: TEMP MIN CAMERA"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_COOLER               , _UxGT("Err: TEMP MAX COOLER"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_COOLER               , _UxGT("Err: TEMP MIN COOLER"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Home XY prima"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("STAMPANTE FERMATA"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Riavviare prego"));
  FSTRINGVALUE(MSG_LONG_DAY                         , _UxGT("giorni"));
  FSTRINGVALUE(MSG_LONG_HOUR                        , _UxGT("ore"));
  FSTRINGVALUE(MSG_LONG_MINUTE                      , _UxGT("minuti"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Riscaldamento..."));
  FSTRINGVALUE(MSG_HEATING_COMPLETE                 , _UxGT("Risc. completato"));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Raffreddamento.."));
  FSTRINGVALUE(MSG_COOLING_COMPLETE                 , _UxGT("Raff.completato."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Risc. piatto..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Raffr. piatto..."));
  FSTRINGVALUE(MSG_BED_DONE                         , _UxGT("Piatto pronto"));
  FSTRINGVALUE(MSG_CHAMBER_HEATING                  , _UxGT("Risc. camera."));
  FSTRINGVALUE(MSG_CHAMBER_COOLING                  , _UxGT("Raffr. camera."));
  FSTRINGVALUE(MSG_CHAMBER_DONE                     , _UxGT("Camera pronta."));
  FSTRINGVALUE(MSG_COOLER_COOLING                   , _UxGT("Raffreddamento..."));
  FSTRINGVALUE(MSG_COOLER_DONE                      , _UxGT("Raffreddamento finito."));

  // Calibrate Delta
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibraz. Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibra X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibra Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibra Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibra centro"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Impostaz. Delta"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto calibrazione"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Imp. altezza Delta"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Barra Diagonale"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Altezza"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Raggio"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE_OK          , _UxGT("Calibrazione OK"));

  // Info printers
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Info. stampante"));
  FSTRINGVALUE(MSG_INFO_FIRMWARE_MENU               , _UxGT("Info. Firmware"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("Livel. a 3 punti"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Livel. Lineare"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Livel. Bilineare"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Livel.piatto unific."));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Livel. Mesh"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Statistiche"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Info. scheda"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistori"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Estrusori"));
  FSTRINGVALUE(MSG_INFO_HOTENDS                     , _UxGT("Ugelli"));
  FSTRINGVALUE(MSG_INFO_BEDS                        , _UxGT("Letti"));
  FSTRINGVALUE(MSG_INFO_CHAMBERS                    , _UxGT("Camere calde"));
  FSTRINGVALUE(MSG_INFO_COOLER                      , _UxGT("Raffreddamento"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baud"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocollo"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Luci Case"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Luminosita' Luci"));
  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Stampante errata"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Stampe totali"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completate"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tempo totale"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Stampa piu' lunga"));
    FSTRINGVALUE(MSG_INFO_POWER_ON                  , _UxGT("Accesa per tempo"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Filamento usato"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Stampe"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completate"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tempo tot."));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Piu' lunga"));
    FSTRINGVALUE(MSG_INFO_POWER_ON                  , _UxGT("Accesa per"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Fil. usato"));
  #endif

  FSTRINGVALUE(MSG_INFO_PWRCONSUMED                 , _UxGT("PWR"));
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Temp min"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Temp max"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Alimentatore"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Potenza Drive"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driver %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Scrivi DAC EEPROM"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER           , _UxGT("CAMBIO FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("STAMPA IN PAUSA"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("CARICA FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("RIMUOVI FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("OPZIONI RIPRESA:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Spurga ancora"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Riprendi stampa"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Ugello: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Sensore filo"));
  FSTRINGVALUE(MSG_RUNOUT_DISTANCE_MM               , _UxGT("Dist mm filo term."));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Home fallito"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Probe fallito"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600:Troppo freddo"));

  FSTRINGVALUE(MSG_MMU2_CHOOSE_FILAMENT_HEADER      , _UxGT("SCELTA FILAMENTO"));
  FSTRINGVALUE(MSG_MMU2_MENU                        , _UxGT("MMU"));
  FSTRINGVALUE(MSG_MMU2_WRONG_FIRMWARE              , _UxGT("Agg.firmware MMU!"));
  FSTRINGVALUE(MSG_MMU2_NOT_RESPONDING              , _UxGT("MMU chiede attenz."));
  FSTRINGVALUE(MSG_MMU2_RESUME                      , _UxGT("Riprendi stampa"));
  FSTRINGVALUE(MSG_MMU2_RESUMING                    , _UxGT("Ripresa..."));
  FSTRINGVALUE(MSG_MMU2_LOAD_FILAMENT               , _UxGT("Carica filamento"));
  FSTRINGVALUE(MSG_MMU2_LOAD_ALL                    , _UxGT("Carica tutto"));
  FSTRINGVALUE(MSG_MMU2_LOAD_TO_NOZZLE              , _UxGT("Carica ugello"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT              , _UxGT("Espelli filamento"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT_N            , _UxGT("Espelli filam.E"));
  FSTRINGVALUE(MSG_MMU2_UNLOAD_FILAMENT             , _UxGT("Scarica filamento"));
  FSTRINGVALUE(MSG_MMU2_LOADING_FILAMENT            , _UxGT("Caric.fil. %i..."));
  FSTRINGVALUE(MSG_MMU2_EJECTING_FILAMENT           , _UxGT("Esplus.filam. ..."));
  FSTRINGVALUE(MSG_MMU2_UNLOADING_FILAMENT          , _UxGT("Scaric.filam. ..."));
  FSTRINGVALUE(MSG_MMU2_ALL                         , _UxGT("Tutto"));
  FSTRINGVALUE(MSG_MMU2_FILAMENT_N                  , _UxGT("Filamento E"));
  FSTRINGVALUE(MSG_MMU2_RESET                       , _UxGT("Azzera MMU"));
  FSTRINGVALUE(MSG_MMU2_RESETTING                   , _UxGT("Azzeramento MMU..."));
  FSTRINGVALUE(MSG_MMU2_EJECT_RECOVER               , _UxGT("Rimuovi, click"));

  FSTRINGVALUE(MSG_MIX                              , _UxGT("Miscela"));
  FSTRINGVALUE(MSG_MIX_COMPONENT                    , _UxGT("Componente"));
  FSTRINGVALUE(MSG_MIXER                            , _UxGT("Miscelatore"));
  FSTRINGVALUE(MSG_GRADIENT                         , _UxGT("Gradiente"));
  FSTRINGVALUE(MSG_FULL_GRADIENT                    , _UxGT("Gradiente pieno"));
  FSTRINGVALUE(MSG_TOGGLE_MIX                       , _UxGT("Alterna miscela"));
  FSTRINGVALUE(MSG_CYCLE_MIX                        , _UxGT("Ciclo miscela"));
  FSTRINGVALUE(MSG_GRADIENT_MIX                     , _UxGT("Miscela gradiente"));
  FSTRINGVALUE(MSG_REVERSE_GRADIENT                 , _UxGT("Inverti gradiente"));
  FSTRINGVALUE(MSG_ACTIVE_VTOOL                     , _UxGT("V-tool attivo"));
  FSTRINGVALUE(MSG_START_VTOOL                      , _UxGT("V-tool iniziale"));
  FSTRINGVALUE(MSG_END_VTOOL                        , _UxGT("V-tool finale"));
  FSTRINGVALUE(MSG_GRADIENT_ALIAS                   , _UxGT("V-tool alias"));
  FSTRINGVALUE(MSG_RESET_VTOOLS                     , _UxGT("Ripristina V-tools"));
  FSTRINGVALUE(MSG_COMMIT_VTOOL                     , _UxGT("Commit mix V-tool"));
  FSTRINGVALUE(MSG_VTOOLS_RESET                     , _UxGT("V-tools ripristin."));
  FSTRINGVALUE(MSG_START_Z                          , _UxGT("Z inizio"));
  FSTRINGVALUE(MSG_END_Z                            , _UxGT("Z fine"));

  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Giochi"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Brickout"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Invaders"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Sn4k3"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Maze"));

  //
  // Le schermate di Cambio Filamento possono visualizzare fino a 3 linee su un display a 4 righe
  //                                                  ...o fino a 2 linee su un display a 3 righe.
  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_3_LINE("Premi per", "riprendere", "la stampa")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parcheggiando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Attendere avvio", "del cambio", "di filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Inserisci il", "filamento e premi", "per continuare")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Premi per", "riscaldare ugello")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_ZZZ            , _UxGT(MSG_2_LINE(" z   z   z", "Z   Z   Z")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Riscaldam. ugello", "Attendere prego...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_3_LINE("Attendere", "l'espulsione", "del filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_3_LINE("Attendere", "il caricamento", "del filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_3_LINE("Attendere", "lo spurgo", "del filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_3_LINE("Premi per terminare", "lo spurgo", "del filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_3_LINE("Attendere", "la ripresa", "della stampa...")));
  #else // LCD_HEIGHT < 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Premi per continuare")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Attendere...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Inserisci e premi")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Riscalda ugello")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_ZZZ            , _UxGT(MSG_1_LINE(" Zz   Zz   Zz")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Riscaldamento...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Espulsione...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Caricamento...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Spurgo filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Premi per terminare")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Ripresa...")));
  #endif // LCD_HEIGHT < 4

  // TMC Driver
  FSTRINGVALUE(MSG_TMC_DRIVERS                      , _UxGT("Drivers TMC"));
  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Driver in uso"));
  FSTRINGVALUE(MSG_TMC_MICROSTEP                    , _UxGT("Driver microstep"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Soglia modo ibrido"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Azzer. sensorles"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Modo stepping"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("StealthChop abil."));

  FSTRINGVALUE(MSG_SERVICE_RESET                    , _UxGT("Reset Servizi"));
  FSTRINGVALUE(MSG_SERVICE_IN                       , _UxGT(" tra:"));

  // Max Inactivity Time
  FSTRINGVALUE(MSG_MAX_INACTIVITY_TIME              , _UxGT("Riscaldamenti spenti da safety timer."));

  // Max extruder
  FSTRINGVALUE(MSG_MAX_EXTRUDERS                    , _UxGT("Estrusori"));

  // Extra
  FSTRINGVALUE(MSG_LASER                            , _UxGT("Laser Preset"));
  FSTRINGVALUE(MSG_NEED_TUNE_PID                    , _UxGT("Necessita Tune PID"));
  FSTRINGVALUE(MSG_ARE_YOU_SURE                     , _UxGT("Sei sicuro"));

  // Rfid module
  FSTRINGVALUE(MSG_RFID_SPOOL                       , _UxGT("Bobina su E"));
  FSTRINGVALUE(MSG_RFID_BRAND                       , _UxGT("Marca: "));
  FSTRINGVALUE(MSG_RFID_TYPE                        , _UxGT("Tipo: "));
  FSTRINGVALUE(MSG_RFID_COLOR                       , _UxGT("Colore: "));
  FSTRINGVALUE(MSG_RFID_SIZE                        , _UxGT("Size: "));
  FSTRINGVALUE(MSG_RFID_TEMP_HOTEND                 , _UxGT("Temperatura Hotend: "));
  FSTRINGVALUE(MSG_RFID_TEMP_BED                    , _UxGT("Temperatura Bed: "));
  FSTRINGVALUE(MSG_RFID_TEMP_USER_HOTEND            , _UxGT("Temperatura utente Hotend: "));
  FSTRINGVALUE(MSG_RFID_TEMP_USER_BED               , _UxGT("Temperatura utente Bed: "));
  FSTRINGVALUE(MSG_RFID_DENSITY                     , _UxGT("Densita': "));
  FSTRINGVALUE(MSG_RFID_SPOOL_LENGHT                , _UxGT("Lunghezza bobina: "));

  // Sound
  FSTRINGVALUE(MSG_SOUND_MODE_ON                    , _UxGT("Suono          [on]"));
  FSTRINGVALUE(MSG_SOUND_MODE_SILENT                , _UxGT("Suono  [silenzioso]"));
  FSTRINGVALUE(MSG_SOUND_MODE_MUTE                  , _UxGT("Suono        [muto]"));

  // EEPROM Allert
  FSTRINGVALUE(MSG_EEPROM_ALLERT                    , _UxGT(MSG_3_LINE("ATTENZIONE...", "EEPROM Cambiata.", "Premi per continuare")));

  // Nextion Allert
  FSTRINGVALUE(MSG_NEXTION_ALLERT                   , _UxGT(MSG_4_LINE("ATTENTIONE...", "NEXTION FW cambiato.", "Aggiornare con nuovo FW", "Premi per continuare")));

  // Nextion M0 M1
  FSTRINGVALUE(MSG_NEXTION_M0_M1_1                  , _UxGT("Premi enter per"));
  FSTRINGVALUE(MSG_NEXTION_M0_M1_2                  , _UxGT("riprendere la stampa"));

  // DHT
  FSTRINGVALUE(MSG_DHT                               , _UxGT("DHT"));
  FSTRINGVALUE(MSG_DHT_11                            , _UxGT("DHT11"));
  FSTRINGVALUE(MSG_DHT_12                            , _UxGT("DHT12"));
  FSTRINGVALUE(MSG_DHT_21                            , _UxGT("DHT21"));
  FSTRINGVALUE(MSG_DHT_22                            , _UxGT("DHT22"));
  FSTRINGVALUE(MSG_DHT_TEMPERATURE                   , _UxGT("Temperatura (C):"));
  FSTRINGVALUE(MSG_DHT_HUMIDITY                      , _UxGT("Umidita' (%):"));
  FSTRINGVALUE(MSG_DHT_DEWPOINT                      , _UxGT("Dew Point (C):"));
}
