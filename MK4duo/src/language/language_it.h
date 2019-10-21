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
 * Italian
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace Language_it {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t    CHARSIZE                             = 1;
  PROGMEM Language_Str LANGUAGE                             = _UxGT("Italiano");

  PROGMEM Language_Str MSG_WELCOME                          = MACHINE_NAME _UxGT(" pronta.");
  PROGMEM Language_Str MSG_LANGUAGE                         = _UxGT("Linguaggio");
  PROGMEM Language_Str MSG_YES                              = _UxGT("Si");
  PROGMEM Language_Str MSG_NO                               = _UxGT("No");
  PROGMEM Language_Str MSG_BACK                             = _UxGT("Indietro");
  PROGMEM Language_Str MSG_SD_INSERTED                      = _UxGT("SD Card inserita");
  PROGMEM Language_Str MSG_SD_REMOVED                       = _UxGT("SD Card rimossa");
  PROGMEM Language_Str MSG_SD_RELEASED                      = _UxGT("SD Card rilasciata");
  PROGMEM Language_Str MSG_LCD_ENDSTOPS                     = _UxGT("Finecor.");
  PROGMEM Language_Str MSG_LCD_SOFT_ENDSTOPS                = _UxGT("Finecorsa Soft");
  PROGMEM Language_Str MSG_MAIN                             = _UxGT("Menu principale");
  PROGMEM Language_Str MSG_ADVANCED_SETTINGS                = _UxGT("Impostaz. avanzate");
  PROGMEM Language_Str MSG_CONFIGURATION                    = _UxGT("Configurazione");
  PROGMEM Language_Str MSG_AUTOSTART                        = _UxGT("Autostart");
  PROGMEM Language_Str MSG_DISABLE_STEPPERS                 = _UxGT("Disabilita Motori");
  PROGMEM Language_Str MSG_DEBUG_MENU                       = _UxGT("Menu di debug");
  PROGMEM Language_Str MSG_PROGRESS_BAR_TEST                = _UxGT("Test barra avanzam.");
  PROGMEM Language_Str MSG_AUTO_HOME                        = _UxGT("Auto Home");
  PROGMEM Language_Str MSG_AUTO_HOME_X                      = _UxGT("Home asse X");
  PROGMEM Language_Str MSG_AUTO_HOME_Y                      = _UxGT("Home asse Y");
  PROGMEM Language_Str MSG_AUTO_HOME_Z                      = _UxGT("Home asse Z");
  PROGMEM Language_Str MSG_AUTO_Z_ALIGN                     = _UxGT("Allineam.automat. Z");
  PROGMEM Language_Str MSG_LEVEL_BED_HOMING                 = _UxGT("Home assi XYZ");
  PROGMEM Language_Str MSG_LEVEL_BED_WAITING                = _UxGT("Premi per iniziare");
  PROGMEM Language_Str MSG_LEVEL_BED_NEXT_POINT             = _UxGT("Punto successivo");
  PROGMEM Language_Str MSG_LEVEL_BED_DONE                   = _UxGT("Livel. terminato!");
  PROGMEM Language_Str MSG_Z_FADE_HEIGHT                    = _UxGT("Fade Height");
  PROGMEM Language_Str MSG_SET_HOME_OFFSETS                 = _UxGT("Imp. offset home");
  PROGMEM Language_Str MSG_OFFSETS_APPLIED             = _UxGT("Offset applicato");
  PROGMEM Language_Str MSG_SET_ORIGIN                       = _UxGT("Imposta Origine");
  PROGMEM Language_Str MSG_PREHEAT_1                        = _UxGT("Preriscalda ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_1_H                      = _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" H");
  PROGMEM Language_Str MSG_PREHEAT_1_END                    = _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" Ugello");
  PROGMEM Language_Str MSG_PREHEAT_1_ALL                    = _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" Tutto");
  PROGMEM Language_Str MSG_PREHEAT_1_SETTINGS               = _UxGT("Preris. ") PREHEAT_1_LABEL _UxGT(" conf");
  PROGMEM Language_Str MSG_PREHEAT_2                        = _UxGT("Preriscalda ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_2_H                      = _UxGT("Preris. ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_2_END                    = _UxGT("Preris. ") PREHEAT_2_LABEL _UxGT(" H");
  PROGMEM Language_Str MSG_PREHEAT_2_ALL                    = _UxGT("Preris. ") PREHEAT_2_LABEL _UxGT(" Tutto");
  PROGMEM Language_Str MSG_PREHEAT_2_SETTINGS               = _UxGT("Preris. ") PREHEAT_2_LABEL _UxGT(" conf");
  PROGMEM Language_Str MSG_PREHEAT_3                        = _UxGT("Preriscalda ") PREHEAT_3_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_3_H                      = _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" H");
  PROGMEM Language_Str MSG_PREHEAT_3_END                    = _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" Ugello");
  PROGMEM Language_Str MSG_PREHEAT_3_ALL                    = _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" Tutto");
  PROGMEM Language_Str MSG_PREHEAT_3_SETTINGS               = _UxGT("Preris. ") PREHEAT_3_LABEL _UxGT(" conf");
  PROGMEM Language_Str MSG_PREHEAT_CUSTOM                   = _UxGT("Prerisc.personal.");
  PROGMEM Language_Str MSG_COOLDOWN                         = _UxGT("Raffredda");
  PROGMEM Language_Str MSG_SWITCH_PS_ON                     = _UxGT("Accendi aliment.");
  PROGMEM Language_Str MSG_SWITCH_PS_OFF                    = _UxGT("Spegni aliment.");
  PROGMEM Language_Str MSG_EXTRUDE                          = _UxGT("Estrudi");
  PROGMEM Language_Str MSG_RETRACT                          = _UxGT("Ritrai");
  PROGMEM Language_Str MSG_MOVE_AXIS                        = _UxGT("Muovi Asse");
  PROGMEM Language_Str MSG_BED_LEVELING                     = _UxGT("Livella piano");
  PROGMEM Language_Str MSG_LEVEL_BED                        = _UxGT("Livella piano");
  PROGMEM Language_Str MSG_LEVEL_CORNERS                    = _UxGT("Livella angoli");
  PROGMEM Language_Str MSG_NEXT_CORNER                      = _UxGT("Prossimo angolo");
  PROGMEM Language_Str MSG_EDIT_MESH                        = _UxGT("Modifica Mesh");
  PROGMEM Language_Str MSG_EDITING_STOPPED                  = _UxGT("Modif. Mesh Fermata");
  PROGMEM Language_Str MSG_PROBING_MESH                     = _UxGT("Punto sondato");
  PROGMEM Language_Str MSG_MESH_X                           = _UxGT("Indice X");
  PROGMEM Language_Str MSG_MESH_Y                           = _UxGT("Indice Y");
  PROGMEM Language_Str MSG_MESH_EDIT_Z                      = _UxGT("Valore di Z");
  PROGMEM Language_Str MSG_USER_MENU                        = _UxGT("Comandi Utente");
  PROGMEM Language_Str MSG_LCD_TILTING_MESH                 = _UxGT("Punto inclinaz.");
  PROGMEM Language_Str MSG_M48_TEST                         = _UxGT("Test sonda M48");
  PROGMEM Language_Str MSG_M48_DEVIATION                    = _UxGT("Deviazione");
  PROGMEM Language_Str MSG_DXC_MENU                         = _UxGT("Modo DXC");
  PROGMEM Language_Str MSG_OFFSETS_MENU                     = _UxGT("Strumenti Offsets");
  PROGMEM Language_Str MSG_DXC_MODE_AUTOPARK                = _UxGT("Auto-Park");
  PROGMEM Language_Str MSG_DXC_MODE_DUPLICATE               = _UxGT("Duplicazione");
  PROGMEM Language_Str MSG_DXC_MODE_MIRRORED_COPY           = _UxGT("Copia speculare");
  PROGMEM Language_Str MSG_DXC_MODE_FULL_CTRL               = _UxGT("Pieno controllo");
  PROGMEM Language_Str MSG_X_OFFSET                         = _UxGT("2° ugello X");
  PROGMEM Language_Str MSG_Y_OFFSET                         = _UxGT("2° ugello Y");
  PROGMEM Language_Str MSG_Z_OFFSET                         = _UxGT("2° ugello Z");
  PROGMEM Language_Str MSG_UBL_DOING_G29                    = _UxGT("G29 in corso");
  PROGMEM Language_Str MSG_UBL_UNHOMED                      = _UxGT("Home XYZ prima");
  PROGMEM Language_Str MSG_UBL_TOOLS                        = _UxGT("Strumenti UBL");
  PROGMEM Language_Str MSG_UBL_LEVEL_BED                    = _UxGT("Unified Bed Leveling");
  PROGMEM Language_Str MSG_UBL_MANUAL_MESH                  = _UxGT("Mesh Manuale");
  PROGMEM Language_Str MSG_UBL_BC_INSERT                    = _UxGT("Metti spes. e misura");
  PROGMEM Language_Str MSG_UBL_BC_INSERT2                   = _UxGT("Misura");
  PROGMEM Language_Str MSG_UBL_BC_REMOVE                    = _UxGT("Rimuovi e mis.piatto");
  PROGMEM Language_Str MSG_UBL_MOVING_TO_NEXT               = _UxGT("Spostamento succes.");
  PROGMEM Language_Str MSG_UBL_ACTIVATE_MESH                = _UxGT("Attiva UBL");
  PROGMEM Language_Str MSG_UBL_DEACTIVATE_MESH              = _UxGT("Disattiva UBL");
  PROGMEM Language_Str MSG_UBL_SET_TEMP_BED                 = _UxGT("Temp. Piatto");
  PROGMEM Language_Str MSG_UBL_BED_TEMP_CUSTOM              = _UxGT("Temp. Piatto");
  PROGMEM Language_Str MSG_UBL_SET_TEMP_HOTEND              = _UxGT("Temp. Ugello");
  PROGMEM Language_Str MSG_UBL_HOTEND_TEMP_CUSTOM           = _UxGT("Temp. Ugello");
  PROGMEM Language_Str MSG_UBL_MESH_EDIT                    = _UxGT("Modifica Mesh");
  PROGMEM Language_Str MSG_UBL_EDIT_CUSTOM_MESH             = _UxGT("Modif.Mesh personal.");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_MESH               = _UxGT("Ritocca Mesh");
  PROGMEM Language_Str MSG_UBL_DONE_EDITING_MESH            = _UxGT("Modif.Mesh fatta");
  PROGMEM Language_Str MSG_UBL_BUILD_CUSTOM_MESH            = _UxGT("Crea Mesh personal.");
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_MENU              = _UxGT("Crea Mesh");
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_M1                = _UxGT("Crea Mesh ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_M2                = _UxGT("Crea Mesh ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_UBL_BUILD_COLD_MESH              = _UxGT("Crea Mesh a freddo");
  PROGMEM Language_Str MSG_UBL_MESH_HEIGHT_ADJUST           = _UxGT("Aggiusta Alt. Mesh");
  PROGMEM Language_Str MSG_UBL_MESH_HEIGHT_AMOUNT           = _UxGT("Altezza");
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_MENU           = _UxGT("Valida Mesh");
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_M1             = _UxGT("Valida Mesh ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_M2             = _UxGT("Valida Mesh ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_UBL_VALIDATE_CUSTOM_MESH         = _UxGT("Valida Mesh pers.");
  PROGMEM Language_Str MSG_UBL_CONTINUE_MESH                = _UxGT("Continua Mesh");
  PROGMEM Language_Str MSG_UBL_MESH_LEVELING                = _UxGT("Livell. Mesh");
  PROGMEM Language_Str MSG_UBL_3POINT_MESH_LEVELING         = _UxGT("Livell. 3 Punti");
  PROGMEM Language_Str MSG_UBL_GRID_MESH_LEVELING           = _UxGT("Livell. Griglia Mesh");
  PROGMEM Language_Str MSG_UBL_MESH_LEVEL                   = _UxGT("Livella Mesh");
  PROGMEM Language_Str MSG_UBL_SIDE_POINTS                  = _UxGT("Punti laterali");
  PROGMEM Language_Str MSG_UBL_MAP_TYPE                     = _UxGT("Tipo di Mappa");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP                   = _UxGT("Esporta Mappa");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_HOST              = _UxGT("Esporta per Host");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_CSV               = _UxGT("Esporta in CSV");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_BACKUP            = _UxGT("Backup esterno");
  PROGMEM Language_Str MSG_UBL_INFO_UBL                     = _UxGT("Esporta Info UBL");
  PROGMEM Language_Str MSG_UBL_FILLIN_AMOUNT                = _UxGT("Riempimento");
  PROGMEM Language_Str MSG_UBL_MANUAL_FILLIN                = _UxGT("Riempimento Manuale");
  PROGMEM Language_Str MSG_UBL_SMART_FILLIN                 = _UxGT("Riempimento Smart");
  PROGMEM Language_Str MSG_UBL_FILLIN_MESH                  = _UxGT("Riempimento Mesh");
  PROGMEM Language_Str MSG_UBL_INVALIDATE_ALL               = _UxGT("Invalida Tutto");
  PROGMEM Language_Str MSG_UBL_INVALIDATE_CLOSEST           = _UxGT("Invalid.Punto Vicino");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_ALL                = _UxGT("Ritocca All");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_CLOSEST            = _UxGT("Ritocca Punto Vicino");
  PROGMEM Language_Str MSG_UBL_STORAGE_MESH_MENU            = _UxGT("Mesh Salvate");
  PROGMEM Language_Str MSG_UBL_STORAGE_SLOT                 = _UxGT("Slot di memoria");
  PROGMEM Language_Str MSG_UBL_LOAD_MESH                    = _UxGT("Carica Mesh Piatto");
  PROGMEM Language_Str MSG_UBL_SAVE_MESH                    = _UxGT("Salva Mesh Piatto");
  PROGMEM Language_Str MSG_MESH_LOADED                      = _UxGT("Mesh %i caricata");
  PROGMEM Language_Str MSG_MESH_SAVED                       = _UxGT("Mesh %i salvata");
  PROGMEM Language_Str MSG_UBL_NO_STORAGE                   = _UxGT("Nessuna memoria");
  PROGMEM Language_Str MSG_UBL_SAVE_ERROR                   = _UxGT("Err: Salvataggio UBL");
  PROGMEM Language_Str MSG_UBL_RESTORE_ERROR                = _UxGT("Err: Ripristino UBL");
  PROGMEM Language_Str MSG_UBL_Z_OFFSET_STOPPED             = _UxGT("Z-Offset Fermato");
  PROGMEM Language_Str MSG_UBL_STEP_BY_STEP_MENU            = _UxGT("UBL passo passo");
  PROGMEM Language_Str MSG_UBL_1_BUILD_COLD_MESH           = _UxGT("1.Crea Mesh a freddo");
  PROGMEM Language_Str MSG_UBL_2_SMART_FILLIN              = _UxGT("2.Riempimento Smart");
  PROGMEM Language_Str MSG_UBL_3_VALIDATE_MESH_MENU        = _UxGT("3.Valida Mesh");
  PROGMEM Language_Str MSG_UBL_4_FINE_TUNE_ALL             = _UxGT("4.Ritocca All");
  PROGMEM Language_Str MSG_UBL_5_VALIDATE_MESH_MENU        = _UxGT("5.Valida Mesh");
  PROGMEM Language_Str MSG_UBL_6_FINE_TUNE_ALL             = _UxGT("6.Ritocca All");
  PROGMEM Language_Str MSG_UBL_7_SAVE_MESH                 = _UxGT("7.Salva Mesh Piatto");

  PROGMEM Language_Str MSG_LED_CONTROL                      = _UxGT("Controllo LED");
  PROGMEM Language_Str MSG_LEDS                             = _UxGT("Luci");
  PROGMEM Language_Str MSG_LED_PRESETS                      = _UxGT("Preset luce");
  PROGMEM Language_Str MSG_SET_LEDS_RED                     = _UxGT("Rosso");
  PROGMEM Language_Str MSG_SET_LEDS_ORANGE                  = _UxGT("Arancione");
  PROGMEM Language_Str MSG_SET_LEDS_YELLOW                  = _UxGT("Giallo");
  PROGMEM Language_Str MSG_SET_LEDS_GREEN                   = _UxGT("Verde");
  PROGMEM Language_Str MSG_SET_LEDS_BLUE                    = _UxGT("Blu");
  PROGMEM Language_Str MSG_SET_LEDS_INDIGO                  = _UxGT("Indaco");
  PROGMEM Language_Str MSG_SET_LEDS_VIOLET                  = _UxGT("Viola");
  PROGMEM Language_Str MSG_SET_LEDS_WHITE                   = _UxGT("Bianco");
  PROGMEM Language_Str MSG_SET_LEDS_DEFAULT                 = _UxGT("Predefinito");
  PROGMEM Language_Str MSG_CUSTOM_LEDS                      = _UxGT("Luci personalizzate");
  PROGMEM Language_Str MSG_INTENSITY_R                      = _UxGT("Intensita' rosso");
  PROGMEM Language_Str MSG_INTENSITY_G                      = _UxGT("Intensita' verde");
  PROGMEM Language_Str MSG_INTENSITY_B                      = _UxGT("Intensita' blu");
  PROGMEM Language_Str MSG_INTENSITY_W                      = _UxGT("Intensita' bianco");
  PROGMEM Language_Str MSG_LED_BRIGHTNESS                   = _UxGT("Luminosita'");

  PROGMEM Language_Str MSG_MOVING                           = _UxGT("In movimento...");
  PROGMEM Language_Str MSG_FREE_XY                          = _UxGT("XY liberi");
  PROGMEM Language_Str MSG_MOVE_X                           = _UxGT("Muovi X");
  PROGMEM Language_Str MSG_MOVE_Y                           = _UxGT("Muovi Y");
  PROGMEM Language_Str MSG_MOVE_Z                           = _UxGT("Muovi Z");
  PROGMEM Language_Str MSG_MOVE_E                           = _UxGT("Estrusore");
  PROGMEM Language_Str MSG_HOTEND_TOO_COLD                  = _UxGT("Ugello freddo");
  PROGMEM Language_Str MSG_MOVE_Z_DIST                      = _UxGT("Muovi di %smm");
  PROGMEM Language_Str MSG_MOVE_01MM                        = _UxGT("Muovi di 0.1mm");
  PROGMEM Language_Str MSG_MOVE_1MM                         = _UxGT("Muovi di 1mm");
  PROGMEM Language_Str MSG_MOVE_10MM                        = _UxGT("Muovi di 10mm");
  PROGMEM Language_Str MSG_SPEED                            = _UxGT("Velocita'");
  PROGMEM Language_Str MSG_BED_Z                            = _UxGT("Piatto Z");
  PROGMEM Language_Str MSG_NOZZLE                           = _UxGT("Ugello");
  PROGMEM Language_Str MSG_BED                              = _UxGT("Piatto");
  PROGMEM Language_Str MSG_CHAMBER                          = _UxGT("Camera");
  PROGMEM Language_Str MSG_COOLER                           = _UxGT("Raffreddamento");
  PROGMEM Language_Str MSG_FAN_SPEED                        = _UxGT("Vel. ventola");
  PROGMEM Language_Str MSG_FLOW                             = _UxGT("Flusso");
  PROGMEM Language_Str MSG_CONTROL                          = _UxGT("Controllo");
  PROGMEM Language_Str MSG_FIX_LOSE_STEPS                   = _UxGT("Fix axis steps");
  PROGMEM Language_Str MSG_MIN                              = " " LCD_STR_THERMOMETER _UxGT(" Min");
  PROGMEM Language_Str MSG_MAX                              = " " LCD_STR_THERMOMETER _UxGT(" Max");
  PROGMEM Language_Str MSG_FACTOR                           = " " LCD_STR_THERMOMETER _UxGT(" Fact");
  PROGMEM Language_Str MSG_IDLEOOZING                       = _UxGT("Anti oozing");
  PROGMEM Language_Str MSG_AUTOTEMP                         = _UxGT("Autotemp");
  PROGMEM Language_Str MSG_LCD_ON                           = _UxGT("On");
  PROGMEM Language_Str MSG_LCD_OFF                          = _UxGT("Off");
  PROGMEM Language_Str MSG_PID_AUTOTUNE                     = _UxGT("PID Autotune");
  PROGMEM Language_Str MSG_PID_BED_AUTOTUNE                 = _UxGT("Letto PID Autotune");
  PROGMEM Language_Str MSG_PID_CHAMBER_AUTOTUNE             = _UxGT("Camera PID Autotune");
  PROGMEM Language_Str MSG_PID_AUTOTUNE_START               = _UxGT("PID Autotune partito");
  PROGMEM Language_Str MSG_PID_P                            = _UxGT("PID-P");
  PROGMEM Language_Str MSG_PID_I                            = _UxGT("PID-I");
  PROGMEM Language_Str MSG_PID_D                            = _UxGT("PID-D");
  PROGMEM Language_Str MSG_PID_C                            = _UxGT("PID-C");
  PROGMEM Language_Str MSG_BED_PID_P                        = _UxGT("Letto PID-P");
  PROGMEM Language_Str MSG_BED_PID_I                        = _UxGT("Letto PID-I");
  PROGMEM Language_Str MSG_BED_PID_D                        = _UxGT("Letto PID-D");
  PROGMEM Language_Str MSG_CHAMBER_PID_P                    = _UxGT("Camera PID-P");
  PROGMEM Language_Str MSG_CHAMBER_PID_I                    = _UxGT("Camera PID-I");
  PROGMEM Language_Str MSG_CHAMBER_PID_D                    = _UxGT("Camera PID-D");
  PROGMEM Language_Str MSG_SELECT                           = _UxGT("Seleziona");
  PROGMEM Language_Str MSG_ACC                              = _UxGT("Accel");
  PROGMEM Language_Str MSG_JERK                             = _UxGT("Jerk");
  PROGMEM Language_Str MSG_VA_JERK                          = _UxGT("Jerk-V") LCD_STR_A;
  PROGMEM Language_Str MSG_VB_JERK                          = _UxGT("Jerk-V") LCD_STR_B;
  PROGMEM Language_Str MSG_VC_JERK                          = _UxGT("Jerk-V") LCD_STR_C;
  PROGMEM Language_Str MSG_VE_JERK                          = _UxGT("Jerk-V") LCD_STR_E;
  PROGMEM Language_Str MSG_VELOCITY                         = _UxGT("Velocita'");
  PROGMEM Language_Str MSG_VMAX_A                           = _UxGT("Vmax ") LCD_STR_A;
  PROGMEM Language_Str MSG_VMAX_B                           = _UxGT("Vmax ") LCD_STR_B;
  PROGMEM Language_Str MSG_VMAX_C                           = _UxGT("Vmax ") LCD_STR_C;
  PROGMEM Language_Str MSG_VMAX_E                           = _UxGT("Vmax ") LCD_STR_E;
  PROGMEM Language_Str MSG_VMIN                             = _UxGT("Vmin");
  PROGMEM Language_Str MSG_VTRAV_MIN                        = _UxGT("VTrav min");
  PROGMEM Language_Str MSG_ACCELERATION                     = _UxGT("Accelerazione");
  PROGMEM Language_Str MSG_AMAX_A                           = _UxGT("Amax ") LCD_STR_A;
  PROGMEM Language_Str MSG_AMAX_B                           = _UxGT("Amax ") LCD_STR_B;
  PROGMEM Language_Str MSG_AMAX_C                           = _UxGT("Amax ") LCD_STR_C;
  PROGMEM Language_Str MSG_AMAX_E                           = _UxGT("Amax ") LCD_STR_E;
  PROGMEM Language_Str MSG_A_RETRACT                        = _UxGT("A-Ritrazione E");
  PROGMEM Language_Str MSG_A_TRAVEL                         = _UxGT("A-Spostamento");
  PROGMEM Language_Str MSG_STEPS_PER_MM                     = _UxGT("Passi/mm");
  PROGMEM Language_Str MSG_A_STEPS                          = _UxGT("passi/mm ") LCD_STR_A;
  PROGMEM Language_Str MSG_B_STEPS                          = _UxGT("passi/mm ") LCD_STR_B;
  PROGMEM Language_Str MSG_C_STEPS                          = _UxGT("passi/mm ") LCD_STR_C;
  PROGMEM Language_Str MSG_E_STEPS                          = _UxGT("passi/mm ") LCD_STR_E;
  PROGMEM Language_Str MSG_TEMPERATURE                      = _UxGT("Temperatura");
  PROGMEM Language_Str MSG_MOTION                           = _UxGT("Movimento");
  PROGMEM Language_Str MSG_FILAMENT                         = _UxGT("Filamento");
  PROGMEM Language_Str MSG_VOLUMETRIC_ENABLED               = _UxGT("E in mm");
  PROGMEM Language_Str MSG_FILAMENT_DIAM                    = _UxGT("Diam. filo");
  PROGMEM Language_Str MSG_FILAMENT_UNLOAD                  = _UxGT("Rimuovi mm");
  PROGMEM Language_Str MSG_FILAMENT_LOAD                    = _UxGT("Carica mm");
  PROGMEM Language_Str MSG_ADVANCE_K                        = _UxGT("K Avanzamento");
  PROGMEM Language_Str MSG_CONTRAST                         = _UxGT("Contrasto LCD");
  PROGMEM Language_Str MSG_STORE_EEPROM                     = _UxGT("Salva impostazioni");
  PROGMEM Language_Str MSG_LOAD_EEPROM                      = _UxGT("Carica impostazioni");
  PROGMEM Language_Str MSG_RESTORE_FAILSAFE                 = _UxGT("Ripristina imp.");
  PROGMEM Language_Str MSG_INIT_EEPROM                      = _UxGT("Inizializza EEPROM");
  PROGMEM Language_Str MSG_SD_UPDATE                        = _UxGT("Aggiorna SD");
  PROGMEM Language_Str MSG_RESET_PRINTER                    = _UxGT("Resetta stampante");
  PROGMEM Language_Str MSG_REFRESH                          = LCD_STR_REFRESH _UxGT("Aggiorna");
  PROGMEM Language_Str MSG_WATCH                            = _UxGT("Schermata info");
  PROGMEM Language_Str MSG_PREPARE                          = _UxGT("Prepara");
  PROGMEM Language_Str MSG_TUNE                             = _UxGT("Regola");
  PROGMEM Language_Str MSG_START_PRINT                      = _UxGT("Avvia stampa");
  PROGMEM Language_Str MSG_BUTTON_NEXT                      = _UxGT("Prossimo");
  PROGMEM Language_Str MSG_BUTTON_INIT                      = _UxGT("Inizializza");
  PROGMEM Language_Str MSG_BUTTON_STOP                      = _UxGT("Stop");
  PROGMEM Language_Str MSG_BUTTON_PRINT                     = _UxGT("Stampa");
  PROGMEM Language_Str MSG_BUTTON_RESET                     = _UxGT("Resetta");
  PROGMEM Language_Str MSG_BUTTON_CANCEL                    = _UxGT("Annulla");
  PROGMEM Language_Str MSG_BUTTON_DONE                      = _UxGT("Fatto");
  PROGMEM Language_Str MSG_PAUSE_PRINT                      = _UxGT("Pausa stampa");
  PROGMEM Language_Str MSG_RESUME_PRINT                     = _UxGT("Riprendi stampa");
  PROGMEM Language_Str MSG_STOP_PRINT                       = _UxGT("Arresta stampa");
  PROGMEM Language_Str MSG_RESTART                          = _UxGT("Restart");
  PROGMEM Language_Str MSG_CARD_MENU                        = _UxGT("Stampa da SD");
  PROGMEM Language_Str MSG_NO_CARD                          = _UxGT("SD non presente");
  PROGMEM Language_Str MSG_DWELL                            = _UxGT("Sospensione...");
  PROGMEM Language_Str MSG_USERWAIT                         = _UxGT("Premi tasto..");
  PROGMEM Language_Str MSG_PRINT_PAUSED                     = _UxGT("Stampa sospesa");
  PROGMEM Language_Str MSG_PRINTING                         = _UxGT("Stampa...");
  PROGMEM Language_Str MSG_RESUMING                         = _UxGT("Riprendi Stampa");
  PROGMEM Language_Str MSG_PRINT_ABORTED                    = _UxGT("Stampa annullata");
  PROGMEM Language_Str MSG_NO_MOVE                          = _UxGT("Nessun Movimento");
  PROGMEM Language_Str MSG_KILLED                           = _UxGT("UCCISO. ");
  PROGMEM Language_Str MSG_STOPPED                          = _UxGT("ARRESTATO. ");
  PROGMEM Language_Str MSG_CONTROL_RETRACT                  = _UxGT("Ritrai mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_SWAP             = _UxGT("Scamb. Ritrai mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACTF                 = _UxGT("Ritrai  V");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_ZHOP             = _UxGT("Salta mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER          = _UxGT("UnRet mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAP     = _UxGT("Scamb. UnRet mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVERF         = _UxGT("UnRet V");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAPF    = _UxGT("S UnRet V");
  PROGMEM Language_Str MSG_AUTORETRACT                      = _UxGT("AutoRitrai");
  PROGMEM Language_Str MSG_FILAMENT_SWAP_LENGTH             = _UxGT("Dist. swap");
  PROGMEM Language_Str MSG_FILAMENT_PURGE_LENGTH            = _UxGT("Dist. pulizia");
  PROGMEM Language_Str MSG_TOOL_CHANGE                      = _UxGT("Cambio utensile");
  PROGMEM Language_Str MSG_TOOL_CHANGE_ZLIFT                = _UxGT("Risalita Z");
  PROGMEM Language_Str MSG_SINGLENOZZLE_PRIME_SPD           = _UxGT("Velocita' pulizia");
  PROGMEM Language_Str MSG_SINGLENOZZLE_RETRACT_SPD         = _UxGT("Velocita' retrazione");
  PROGMEM Language_Str MSG_FILAMENTCHANGE                   = _UxGT("Cambia filamento");
  PROGMEM Language_Str MSG_FILAMENTLOAD                     = _UxGT("Carica filamento");
  PROGMEM Language_Str MSG_FILAMENTUNLOAD                   = _UxGT("Rimuovi filamento");
  PROGMEM Language_Str MSG_FILAMENTUNLOAD_ALL               = _UxGT("Rimuovi tutto");
  PROGMEM Language_Str MSG_INIT_SDCARD                      = _UxGT("Iniz. SD card");
  PROGMEM Language_Str MSG_CHANGE_SDCARD                    = _UxGT("Cambia SD card");
  PROGMEM Language_Str MSG_RELEASE_SDCARD                   = _UxGT("Rilascia SD card");
  PROGMEM Language_Str MSG_ZPROBE_OUT                       = _UxGT("Z probe fuori piatto");
  PROGMEM Language_Str MSG_BLTOUCH                          = _UxGT("BLTouch");
  PROGMEM Language_Str MSG_BLTOUCH_SELFTEST                 = _UxGT("Autotest");
  PROGMEM Language_Str MSG_BLTOUCH_RESET                    = _UxGT("Resetta");
  PROGMEM Language_Str MSG_BLTOUCH_DEPLOY                   = _UxGT("Estendi");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_SW                  = _UxGT("Modo SW");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_5V                  = _UxGT("Modo 5V");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_OD                  = _UxGT("Modo OD");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE               = _UxGT("Mode Store");
  PROGMEM Language_Str MSG_BLTOUCH_STOW                     = _UxGT("Ritrai");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE_5V            = _UxGT("Setta a 5V");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE_OD            = _UxGT("Setta a OD");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_ECHO                = _UxGT("Report Drain");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_CHANGE              = _UxGT("PERICOLO: Impostazioni errate possono causare danni! Procedere comunque?");
  PROGMEM Language_Str MSG_MANUAL_DEPLOY                    = _UxGT("Estendi Probe");
  PROGMEM Language_Str MSG_MANUAL_STOW                      = _UxGT("Ritrai Probe");
  PROGMEM Language_Str MSG_HOME_FIRST                       = _UxGT("Home %s%s%s prima");
  PROGMEM Language_Str MSG_ZPROBE_ZOFFSET                   = _UxGT("Z Offset");
  PROGMEM Language_Str MSG_BABYSTEP_X                       = _UxGT("Babystep X");
  PROGMEM Language_Str MSG_BABYSTEP_Y                       = _UxGT("Babystep Y");
  PROGMEM Language_Str MSG_BABYSTEP_Z                       = _UxGT("Babystep Z");
  PROGMEM Language_Str MSG_ENDSTOP_ABORT                    = _UxGT("Finecorsa abort");
  PROGMEM Language_Str MSG_HEATING_FAILED                   = _UxGT("Riscald. Fallito");
  PROGMEM Language_Str MSG_ERR_REDUNDANT_TEMP               = _UxGT("Err: TEMP RIDONDANTE");
  PROGMEM Language_Str MSG_THERMAL_RUNAWAY                  = _UxGT("TEMP FUORI CONTROLLO");
  PROGMEM Language_Str MSG_AD595                            = _UxGT("AD595 Offset & Gain");
  PROGMEM Language_Str MSG_ERR_MAXTEMP                      = _UxGT("Err: TEMP MASSIMA");
  PROGMEM Language_Str MSG_ERR_MINTEMP                      = _UxGT("Err: TEMP MINIMA");
  PROGMEM Language_Str MSG_ERR_MAXTEMP_BED                  = _UxGT("Err: TEMP MAX PIATTO");
  PROGMEM Language_Str MSG_ERR_MINTEMP_BED                  = _UxGT("Err: TEMP MIN PIATTO");
  PROGMEM Language_Str MSG_ERR_MAXTEMP_CHAMBER              = _UxGT("Err: TEMP MAX CAMERA");
  PROGMEM Language_Str MSG_ERR_MINTEMP_CHAMBER              = _UxGT("Err: TEMP MIN CAMERA");
  PROGMEM Language_Str MSG_ERR_MAXTEMP_COOLER               = _UxGT("Err: TEMP MAX COOLER");
  PROGMEM Language_Str MSG_ERR_MINTEMP_COOLER               = _UxGT("Err: TEMP MIN COOLER");
  PROGMEM Language_Str MSG_ERR_Z_HOMING                     = _UxGT("Home XY prima");
  PROGMEM Language_Str MSG_HALTED                           = _UxGT("STAMPANTE FERMATA");
  PROGMEM Language_Str MSG_PLEASE_RESET                     = _UxGT("Riavviare prego");
  PROGMEM Language_Str MSG_LONG_DAY                         = _UxGT("giorni");
  PROGMEM Language_Str MSG_LONG_HOUR                        = _UxGT("ore");
  PROGMEM Language_Str MSG_LONG_MINUTE                      = _UxGT("minuti");
  PROGMEM Language_Str MSG_HEATING                          = _UxGT("Riscaldamento...");
  PROGMEM Language_Str MSG_HEATING_COMPLETE                 = _UxGT("Risc. completato");
  PROGMEM Language_Str MSG_COOLING                          = _UxGT("Raffreddamento..");
  PROGMEM Language_Str MSG_COOLING_COMPLETE                 = _UxGT("Raff.completato.");
  PROGMEM Language_Str MSG_BED_HEATING                      = _UxGT("Risc. piatto...");
  PROGMEM Language_Str MSG_BED_COOLING                      = _UxGT("Raffr. piatto...");
  PROGMEM Language_Str MSG_BED_DONE                         = _UxGT("Piatto pronto");
  PROGMEM Language_Str MSG_CHAMBER_HEATING                  = _UxGT("Risc. camera.");
  PROGMEM Language_Str MSG_CHAMBER_COOLING                  = _UxGT("Raffr. camera.");
  PROGMEM Language_Str MSG_CHAMBER_DONE                     = _UxGT("Camera pronta.");
  PROGMEM Language_Str MSG_COOLER_COOLING                   = _UxGT("Raffreddamento...");
  PROGMEM Language_Str MSG_COOLER_DONE                      = _UxGT("Raffreddamento finito.");

  // Calibrate Delta
  PROGMEM Language_Str MSG_DELTA_CALIBRATE                  = _UxGT("Calibraz. Delta");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_X                = _UxGT("Calibra X");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_Y                = _UxGT("Calibra Y");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_Z                = _UxGT("Calibra Z");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_CENTER           = _UxGT("Calibra centro");
  PROGMEM Language_Str MSG_DELTA_SETTINGS                   = _UxGT("Impostaz. Delta");
  PROGMEM Language_Str MSG_DELTA_AUTO_CALIBRATE             = _UxGT("Auto calibrazione");
  PROGMEM Language_Str MSG_DELTA_HEIGHT_CALIBRATE           = _UxGT("Imp. altezza Delta");
  PROGMEM Language_Str MSG_DELTA_DIAG_ROD                   = _UxGT("Barra Diagonale");
  PROGMEM Language_Str MSG_DELTA_HEIGHT                     = _UxGT("Altezza");
  PROGMEM Language_Str MSG_DELTA_RADIUS                     = _UxGT("Raggio");
  PROGMEM Language_Str MSG_DELTA_AUTO_CALIBRATE_OK          = _UxGT("Calibrazione OK");

  // Info printers
  PROGMEM Language_Str MSG_INFO_MENU                        = _UxGT("Info. stampante");
  PROGMEM Language_Str MSG_INFO_FIRMWARE_MENU               = _UxGT("Info. Firmware");
  PROGMEM Language_Str MSG_3POINT_LEVELING                  = _UxGT("Livel. a 3 punti");
  PROGMEM Language_Str MSG_LINEAR_LEVELING                  = _UxGT("Livel. Lineare");
  PROGMEM Language_Str MSG_BILINEAR_LEVELING                = _UxGT("Livel. Bilineare");
  PROGMEM Language_Str MSG_UBL_LEVELING                     = _UxGT("Livel.piatto unific.");
  PROGMEM Language_Str MSG_MESH_LEVELING                    = _UxGT("Livel. Mesh");
  PROGMEM Language_Str MSG_INFO_STATS_MENU                  = _UxGT("Statistiche");
  PROGMEM Language_Str MSG_INFO_BOARD_MENU                  = _UxGT("Info. scheda");
  PROGMEM Language_Str MSG_INFO_THERMISTOR_MENU             = _UxGT("Termistori");
  PROGMEM Language_Str MSG_INFO_EXTRUDERS                   = _UxGT("Estrusori");
  PROGMEM Language_Str MSG_INFO_HOTENDS                     = _UxGT("Ugelli");
  PROGMEM Language_Str MSG_INFO_BEDS                        = _UxGT("Letti");
  PROGMEM Language_Str MSG_INFO_CHAMBERS                    = _UxGT("Camere calde");
  PROGMEM Language_Str MSG_INFO_COOLER                      = _UxGT("Raffreddamento");
  PROGMEM Language_Str MSG_INFO_BAUDRATE                    = _UxGT("Baud");
  PROGMEM Language_Str MSG_INFO_PROTOCOL                    = _UxGT("Protocollo");
  PROGMEM Language_Str MSG_CASE_LIGHT                       = _UxGT("Luci Case");
  PROGMEM Language_Str MSG_CASE_LIGHT_BRIGHTNESS            = _UxGT("Luminosita' Luci");
  PROGMEM Language_Str MSG_EXPECTED_PRINTER                 = _UxGT("Stampante errata");

  #if LCD_WIDTH >= 20
    PROGMEM Language_Str MSG_INFO_PRINT_COUNT               = _UxGT("Stampe totali");
    PROGMEM Language_Str MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completate");
    PROGMEM Language_Str MSG_INFO_PRINT_TIME                = _UxGT("Tempo totale");
    PROGMEM Language_Str MSG_INFO_PRINT_LONGEST             = _UxGT("Stampa piu' lunga");
    PROGMEM Language_Str MSG_INFO_POWER_ON                  = _UxGT("Accesa per tempo");
    PROGMEM Language_Str MSG_INFO_PRINT_FILAMENT            = _UxGT("Filamento usato");
  #else
    PROGMEM Language_Str MSG_INFO_PRINT_COUNT               = _UxGT("Stampe");
    PROGMEM Language_Str MSG_INFO_COMPLETED_PRINTS          = _UxGT("Completate");
    PROGMEM Language_Str MSG_INFO_PRINT_TIME                = _UxGT("Tempo tot.");
    PROGMEM Language_Str MSG_INFO_PRINT_LONGEST             = _UxGT("Piu' lunga");
    PROGMEM Language_Str MSG_INFO_POWER_ON                  = _UxGT("Accesa per");
    PROGMEM Language_Str MSG_INFO_PRINT_FILAMENT            = _UxGT("Fil. usato");
  #endif

  PROGMEM Language_Str MSG_INFO_PWRCONSUMED                 = _UxGT("PWR");
  PROGMEM Language_Str MSG_INFO_MIN_TEMP                    = _UxGT("Temp min");
  PROGMEM Language_Str MSG_INFO_MAX_TEMP                    = _UxGT("Temp max");
  PROGMEM Language_Str MSG_INFO_PSU                         = _UxGT("Alimentatore");
  PROGMEM Language_Str MSG_DRIVE_STRENGTH                   = _UxGT("Potenza Drive");
  PROGMEM Language_Str MSG_DAC_PERCENT                      = _UxGT("Driver %");
  PROGMEM Language_Str MSG_DAC_EEPROM_WRITE                 = _UxGT("Scrivi DAC EEPROM");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER           = _UxGT("CAMBIO FILAMENTO");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_PAUSE     = _UxGT("STAMPA IN PAUSA");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_LOAD      = _UxGT("CARICA FILAMENTO");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_UNLOAD    = _UxGT("RIMUOVI FILAMENTO");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_HEADER    = _UxGT("OPZIONI RIPRESA:");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_PURGE     = _UxGT("Spurga ancora");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_RESUME    = _UxGT("Riprendi stampa");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_NOZZLE           = _UxGT("  Ugello: ");
  PROGMEM Language_Str MSG_RUNOUT_SENSOR                    = _UxGT("Sensore filo");
  PROGMEM Language_Str MSG_RUNOUT_DISTANCE_MM               = _UxGT("Dist mm filo term.");
  PROGMEM Language_Str MSG_LCD_HOMING_FAILED                = _UxGT("Home fallito");
  PROGMEM Language_Str MSG_LCD_PROBING_FAILED               = _UxGT("Probe fallito");
  PROGMEM Language_Str MSG_M600_TOO_COLD                    = _UxGT("M600:Troppo freddo");

  PROGMEM Language_Str MSG_MMU2_CHOOSE_FILAMENT_HEADER      = _UxGT("SCELTA FILAMENTO");
  PROGMEM Language_Str MSG_MMU2_MENU                        = _UxGT("MMU");
  PROGMEM Language_Str MSG_MMU2_WRONG_FIRMWARE              = _UxGT("Agg.firmware MMU!");
  PROGMEM Language_Str MSG_MMU2_NOT_RESPONDING              = _UxGT("MMU chiede attenz.");
  PROGMEM Language_Str MSG_MMU2_RESUME                      = _UxGT("Riprendi stampa");
  PROGMEM Language_Str MSG_MMU2_RESUMING                    = _UxGT("Ripresa...");
  PROGMEM Language_Str MSG_MMU2_LOAD_FILAMENT               = _UxGT("Carica filamento");
  PROGMEM Language_Str MSG_MMU2_LOAD_ALL                    = _UxGT("Carica tutto");
  PROGMEM Language_Str MSG_MMU2_LOAD_TO_NOZZLE              = _UxGT("Carica ugello");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT              = _UxGT("Espelli filamento");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT0             = _UxGT("Espelli filam.1");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT1             = _UxGT("Espelli filam.2");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT2             = _UxGT("Espelli filam.3");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT3             = _UxGT("Espelli filam.4");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT4             = _UxGT("Espelli filam.5");
  PROGMEM Language_Str MSG_MMU2_UNLOAD_FILAMENT             = _UxGT("Scarica filamento");
  PROGMEM Language_Str MSG_MMU2_LOADING_FILAMENT            = _UxGT("Caric.fil. %i...");
  PROGMEM Language_Str MSG_MMU2_EJECTING_FILAMENT           = _UxGT("Esplus.filam. ...");
  PROGMEM Language_Str MSG_MMU2_UNLOADING_FILAMENT          = _UxGT("Scaric.filam. ...");
  PROGMEM Language_Str MSG_MMU2_ALL                         = _UxGT("Tutto");
  PROGMEM Language_Str MSG_MMU2_FILAMENT0                   = _UxGT("Filamento 1");
  PROGMEM Language_Str MSG_MMU2_FILAMENT1                   = _UxGT("Filamento 2");
  PROGMEM Language_Str MSG_MMU2_FILAMENT2                   = _UxGT("Filamento 3");
  PROGMEM Language_Str MSG_MMU2_FILAMENT3                   = _UxGT("Filamento 4");
  PROGMEM Language_Str MSG_MMU2_FILAMENT4                   = _UxGT("Filamento 5");
  PROGMEM Language_Str MSG_MMU2_RESET                       = _UxGT("Azzera MMU");
  PROGMEM Language_Str MSG_MMU2_RESETTING                   = _UxGT("Azzeramento MMU...");
  PROGMEM Language_Str MSG_MMU2_EJECT_RECOVER               = _UxGT("Rimuovi, click");

  PROGMEM Language_Str MSG_MIX                              = _UxGT("Miscela");
  PROGMEM Language_Str MSG_MIX_COMPONENT                    = _UxGT("Componente");
  PROGMEM Language_Str MSG_MIXER                            = _UxGT("Miscelatore");
  PROGMEM Language_Str MSG_GRADIENT                         = _UxGT("Gradiente");
  PROGMEM Language_Str MSG_FULL_GRADIENT                    = _UxGT("Gradiente pieno");
  PROGMEM Language_Str MSG_TOGGLE_MIX                       = _UxGT("Alterna miscela");
  PROGMEM Language_Str MSG_CYCLE_MIX                        = _UxGT("Ciclo miscela");
  PROGMEM Language_Str MSG_GRADIENT_MIX                     = _UxGT("Miscela gradiente");
  PROGMEM Language_Str MSG_REVERSE_GRADIENT                 = _UxGT("Inverti gradiente");
  PROGMEM Language_Str MSG_ACTIVE_VTOOL                     = _UxGT("V-tool attivo");
  PROGMEM Language_Str MSG_START_VTOOL                      = _UxGT("V-tool iniziale");
  PROGMEM Language_Str MSG_END_VTOOL                        = _UxGT("V-tool finale");
  PROGMEM Language_Str MSG_GRADIENT_ALIAS                   = _UxGT("V-tool alias");
  PROGMEM Language_Str MSG_RESET_VTOOLS                     = _UxGT("Ripristina V-tools");
  PROGMEM Language_Str MSG_COMMIT_VTOOL                     = _UxGT("Commit mix V-tool");
  PROGMEM Language_Str MSG_VTOOLS_RESET                     = _UxGT("V-tools ripristin.");
  PROGMEM Language_Str MSG_START_Z                          = _UxGT("Z inizio");
  PROGMEM Language_Str MSG_END_Z                            = _UxGT("Z fine");

  PROGMEM Language_Str MSG_GAMES                            = _UxGT("Giochi");
  PROGMEM Language_Str MSG_BRICKOUT                         = _UxGT("Brickout");
  PROGMEM Language_Str MSG_INVADERS                         = _UxGT("Invaders");
  PROGMEM Language_Str MSG_SNAKE                            = _UxGT("Sn4k3");
  PROGMEM Language_Str MSG_MAZE                             = _UxGT("Maze");

  //
  // Le schermate di Cambio Filamento possono visualizzare fino a 3 linee su un display a 4 righe
  //                                                  ...o fino a 2 linee su un display a 3 righe.
  #if LCD_HEIGHT >= 4
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING_1       = _UxGT("Premi per");
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING_2       = _UxGT("riprendere");
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING_3       = _UxGT("la stampa");
    PROGMEM Language_Str MSG_PAUSE_PRINT_INIT_1             = _UxGT("Parcheggiando...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_1         = _UxGT("Attendere avvio");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_2         = _UxGT("del cambio");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_3         = _UxGT("di filamento");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_1       = _UxGT("Inserisci il");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_2       = _UxGT("filamento e premi");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_3       = _UxGT("per continuare");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT_1         = _UxGT("Premi per");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT_2         = _UxGT("riscaldare ugello");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_ZZZ_1          = _UxGT(" z   z   z");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_ZZZ_2          = _UxGT("Z   Z   Z");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING_1      = _UxGT("Riscaldam. ugello");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING_2      = _UxGT("Attendere prego...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD_1       = _UxGT("Attendere");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD_2       = _UxGT("l'espulsione");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD_3       = _UxGT("del filamento");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD_1         = _UxGT("Attendere");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD_2         = _UxGT("il caricamento");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD_3         = _UxGT("del filamento");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE_1        = _UxGT("Attendere");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE_2        = _UxGT("lo spurgo");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE_3        = _UxGT("del filamento");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE_1   = _UxGT("Premi x terminare");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE_2   = _UxGT("lo spurgo");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE_3   = _UxGT("del filamento");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME_1       = _UxGT("Attendere");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME_2       = _UxGT("la ripresa");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME_3       = _UxGT("della stampa...");
  #else // LCD_HEIGHT < 4
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING_1       = _UxGT("Premi x continuare");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT_1         = _UxGT("Attendere...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT_1       = _UxGT("Inserisci e premi");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT_1         = _UxGT("Riscalda ugello");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING_1      = _UxGT("Riscaldamento...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD_1       = _UxGT("Espulsione...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_ZZZ_1          = _UxGT(" Zz   Zz   Zz");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD_1         = _UxGT("Caricamento...");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE_1        = _UxGT("Spurgo filamento");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE_1   = _UxGT("Premi x terminare");
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME_1       = _UxGT("Ripresa...");
  #endif // LCD_HEIGHT < 4

  // TMC Driver
  PROGMEM Language_Str MSG_TMC_DRIVERS                      = _UxGT("Drivers TMC");
  PROGMEM Language_Str MSG_TMC_CURRENT                      = _UxGT("Driver in uso");
  PROGMEM Language_Str MSG_TMC_MICROSTEP                    = _UxGT("Driver microstep");
  PROGMEM Language_Str MSG_TMC_HYBRID_THRS                  = _UxGT("Soglia modo ibrido");
  PROGMEM Language_Str MSG_TMC_HOMING_THRS                  = _UxGT("Azzer. sensorles");
  PROGMEM Language_Str MSG_TMC_STEPPING_MODE                = _UxGT("Modo stepping");
  PROGMEM Language_Str MSG_TMC_STEALTH_ENABLED              = _UxGT("StealthChop abil.");

  PROGMEM Language_Str MSG_SERVICE_RESET                    = _UxGT("Reset Servizi");
  PROGMEM Language_Str MSG_SERVICE_IN                       = _UxGT(" tra:");;

  // Max Inactivity Time
  PROGMEM Language_Str MSG_MAX_INACTIVITY_TIME              = _UxGT("Riscaldamenti spenti da safety timer.");

  // Max extruder
  PROGMEM Language_Str MSG_MAX_EXTRUDERS                    = _UxGT("Estrusori");

  // Extra
  PROGMEM Language_Str MSG_LASER                            = _UxGT("Laser Preset");
  PROGMEM Language_Str MSG_NEED_TUNE_PID                    = _UxGT("Necessita Tune PID");
  PROGMEM Language_Str MSG_ARE_YOU_SURE                     = _UxGT("Sei sicuro");

  // Rfid module
  PROGMEM Language_Str MSG_RFID_SPOOL                       = _UxGT("Bobina su E");
  PROGMEM Language_Str MSG_RFID_BRAND                       = _UxGT("Marca: ");
  PROGMEM Language_Str MSG_RFID_TYPE                        = _UxGT("Tipo: ");
  PROGMEM Language_Str MSG_RFID_COLOR                       = _UxGT("Colore: ");
  PROGMEM Language_Str MSG_RFID_SIZE                        = _UxGT("Size: ");
  PROGMEM Language_Str MSG_RFID_TEMP_HOTEND                 = _UxGT("Temperatura Hotend: ");
  PROGMEM Language_Str MSG_RFID_TEMP_BED                    = _UxGT("Temperatura Bed: ");
  PROGMEM Language_Str MSG_RFID_TEMP_USER_HOTEND            = _UxGT("Temperatura utente Hotend: ");
  PROGMEM Language_Str MSG_RFID_TEMP_USER_BED               = _UxGT("Temperatura utente Bed: ");
  PROGMEM Language_Str MSG_RFID_DENSITY                     = _UxGT("Densita': ");
  PROGMEM Language_Str MSG_RFID_SPOOL_LENGHT                = _UxGT("Lunghezza bobina: ");

  // Sound
  PROGMEM Language_Str MSG_SOUND_MODE_ON                    = _UxGT("Suono          [on]");
  PROGMEM Language_Str MSG_SOUND_MODE_SILENT                = _UxGT("Suono  [silenzioso]");
  PROGMEM Language_Str MSG_SOUND_MODE_MUTE                  = _UxGT("Suono        [muto]");

  // EEPROM Allert
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_1          = _UxGT("ATTENZIONE...");
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_2          = _UxGT("EEPROM Cambiata.");
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_3          = _UxGT("Premere il bottone");
  PROGMEM Language_Str MSG_EEPROM_CHANGED_ALLERT_4          = _UxGT("per continuare...");

  // Nextion Allert
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_1         = _UxGT("ATTENTION...");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_2         = _UxGT("NEXTION FW cambiato.");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_3         = _UxGT("Aggiornare con nuovo FW");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_4         = _UxGT("Premere il bottone");
  PROGMEM Language_Str MSG_NEXTION_CHANGED_ALLERT_5         = _UxGT("per continuare...");

  // Nextion M0 M1
  PROGMEM Language_Str MSG_NEXTION_M0_M1_1                  = _UxGT("Premi enter per");
  PROGMEM Language_Str MSG_NEXTION_M0_M1_2                  = _UxGT("riprendere la stampa");

  // DHT
  PROGMEM Language_Str MSG_DHT                               = _UxGT("DHT");
  PROGMEM Language_Str MSG_DHT_11                            = _UxGT("DHT11");
  PROGMEM Language_Str MSG_DHT_12                            = _UxGT("DHT12");
  PROGMEM Language_Str MSG_DHT_21                            = _UxGT("DHT21");
  PROGMEM Language_Str MSG_DHT_22                            = _UxGT("DHT22");
  PROGMEM Language_Str MSG_DHT_TEMPERATURE                   = _UxGT("Temperatura (C):");
  PROGMEM Language_Str MSG_DHT_HUMIDITY                      = _UxGT("Umidita' (%):");
  PROGMEM Language_Str MSG_DHT_DEWPOINT                      = _UxGT("Dew Point (C):");
}
