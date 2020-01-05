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
 * Catalan
 *
 * LCD Menu Messages
 *
 */

namespace language_ca {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Catalan"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" preparada."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Targeta detectada."));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Targeta extreta."));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstops"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menú principal"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Inici automatic"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Desactiva motors"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Menu de depuracio"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Test barra progres"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Ves a l'origen"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("X a origen"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Y a origen"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Z a origen"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Origen XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Premeu per iniciar"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Següent punt"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Anivellament fet!"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Ajusta decalatge"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Decalatge aplicat"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Estableix origen"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Preescalfa ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Preescalfa ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" End"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" End ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" Tot"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" Llit"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Preheat ") PREHEAT_1_LABEL _UxGT(" Conf."));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Preescalfa ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Preescalfa ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" End"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" End ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" Tot"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" Llit"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Preheat ") PREHEAT_2_LABEL _UxGT(" Conf."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Refreda"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Switch power on"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Switch power off"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrudeix"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retreu"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Mou eixos"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Anivella llit"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Anivella llit"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Movent.."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("XY lliures"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Mou X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Mou Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Mou Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extrusor"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extrusor *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Mou %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Mou 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Mou 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Mou 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Velocitat"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Llit Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Nozzle"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Nozzle ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Llit"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Vel. Ventilador"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Vel. Ventilador ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Flux"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Flux ~"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("VViatge min"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Accel. max ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Accel. max ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Accel. max ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Accel. max ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Accel. max *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("Accel. retracc"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("Accel. Viatge"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Passos/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT("passos/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT("passos/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT("passos/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("Epassos/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("*passos/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Moviment"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E en mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Diam. Fil."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Diam. Fil. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Contrast de LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Desa memoria"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Carrega memoria"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Restaura valors"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Actualitza"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Pantalla Info."));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Prepara"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Ajusta"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausa impressio"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Repren impressio"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Atura impressio."));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Imprimeix de SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("No hi ha targeta"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("En repos..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Esperant usuari.."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Imp. cancelada"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Sense moviment."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("MATAT."));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ATURADA."));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retreu mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Swap Retreure mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retreu V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Aixeca mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("DesRet +mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Swap DesRet +mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("DesRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Auto retraccio"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Canvia filament"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Canvia filament *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Inicialitza SD"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Canvia SD"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonda Z fora"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Reinicia BLTouch"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Home %s%s%s primer"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Decalatge Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Micropas X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Micropas Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Micropas Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Cancel. Endstop"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Error al escalfar"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Err: TEMP REDUNDANT"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("THERMAL RUNAWAY"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err: TEMP MAXIMA"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err: TEMP MINIMA"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Home XY primer"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("IMPRESSORA PARADA"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Reinicieu"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Escalfant..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Escalfant llit..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibratge Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibra X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibra Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibra Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibra el centre"));

  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Quant a la impr."));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Info Impressora"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Estadistiques"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Info placa"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistors"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extrusors"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baud"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocol"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Llum"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Total impressions"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Acabades"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Temps imprimint"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Treball mes llarg"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Total extrudit"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Impressions"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Acabades"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Mes llarg"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extrudit"));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Temp. mínima"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Temp. màxima"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Font alimentacio"));

  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Força motor"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driver %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("DAC EEPROM Write"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Repren impressió"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Impressora incorrecta"));

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Esperant per", "iniciar el canvi", "de filament")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Esperant per", "treure filament")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Poseu filament", "i premeu el boto", "per continuar...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Premeu boto per", "escalfar nozzle.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Escalfant nozzle", "Espereu...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Esperant carrega", "de filament")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Esperant per", "reprendre")));
  #else // LCD_HEIGHT < 4
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Espereu...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Expulsant...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Insereix i prem")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Escalfant...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Carregant...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Reprenent...")));
  #endif // LCD_HEIGHT < 4
}
