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
 * Aragonese
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

namespace language_an {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 1;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Aragonese"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" parada."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Tarcheta mesa"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Tarcheta sacada"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstops")); // Max length 8 characters
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu prencipal"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Inicio automatico"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Amortar motors"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Levar a l'orichen"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Orichen X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Orichen Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Orichen Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Orichen XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Encetar (pretar)"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Vinient punto"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Nivelacion feita!"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Achustar desfases"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Desfase aplicau"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Establir orichen"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Precalentar ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Precalentar ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Boquilla"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Precal. ") PREHEAT_1_LABEL _UxGT(" Boquilla ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Tot"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Base"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Conf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Precalentar ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Precalentar ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Precal. ") PREHEAT_2_LABEL _UxGT(" Boquilla"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Precal. ") PREHEAT_2_LABEL _UxGT(" Boquilla ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Tot"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Base"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Conf"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Enfriar"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Enchegar Fuent"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Amortar Fuent"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extruir"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retraer"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Mover Eixes"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Nivelar base"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Nivelar base"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Mover X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Mover Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Mover Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extrusor"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extrusor *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Mover %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Mover 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Mover 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Mover 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Velocidat"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Base Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Boquilla"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Boquilla ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Base"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Ixoriador"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Ixoriador ,"));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Fluxo"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Fluxo ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Control"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Temperatura Auto."));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Trigar"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Trigar *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Aceleracion"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("Vel. viache min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Accel"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Acel. max") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Acel. max") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Acel. max") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Acel. max") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Acel. max *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("Acel. retrac."));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("Acel. Viaje"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Trangos/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" trangos/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" trangos/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" trangos/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E trangos/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* trangos/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Movimiento"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filamento"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Fil. Dia."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Fil. Dia. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Contraste"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Alzar memoria"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Cargar memoria"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Restaurar memoria"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH _UxGT("Tornar a cargar"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Informacion"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Preparar"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Achustar"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausar impresion"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Contin. impresion"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Detener Impresion"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Menu de SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("No i hai tarcheta"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Reposo..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Aguardand ordines"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Impres. cancelada"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Sin movimiento"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("Aturada d'emerch."));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("Aturada."));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retraer mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Swap Retraer mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retraer  F"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Devantar mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("DesRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Swap DesRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("DesRet F"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Retraccion auto."));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Cambear filamento"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Cambear filamento *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Encetan. tarcheta"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Cambiar tarcheta"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonda Z fuera"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Reset BLTouch"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Desfase Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Micropaso X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Micropaso Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Micropaso Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Cancelado - Endstop"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Error: en calentar"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Error: temperatura"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("Error de temperatura"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Error: Temp Max"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Error: Temp Min"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Home XY first"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("IMPRESORA ATURADA"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Per favor reinic."));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d"));
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h"));
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Calentando..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Calentando base..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibracion Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibrar X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibrar Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibrar Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibrar Centro"));

  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Inf. Impresora"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Inf. Impresora"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Estadisticas Imp."));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Inf. Controlador"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistors"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extrusors"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baudios"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocolo"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Luz"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Conteo de impresion"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completadas"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tiempo total d'imp."));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Impresion mas larga"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Total d'extrusion"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Impresions"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completadas"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Mas larga"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extrusion"));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Temperatura menima"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Temperatura maxima"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Fuente de aliment"));

  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Fuerza d'o driver"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Escri. DAC EEPROM"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Resumir imp."));

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //

  #if LCD_HEIGHT >= 4
    // Up to 3 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Aguardand iniciar", "d'o filamento", "cambear")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Meta o filamento", "y prete lo boton", "pa continar...")));
  #else // LCD_HEIGHT < 4
    // Up to 2 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_2_LINE("Aguardand iniciar", "d'o fil. cambear")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_2_LINE("Meta o filamento", "y prete lo boton")));
  #endif // LCD_HEIGHT < 4

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD           , _UxGT(MSG_2_LINE("Aguardando a", "expulsar filament")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD             , _UxGT(MSG_2_LINE("Aguardando a", "cargar filamento")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME           , _UxGT(MSG_2_LINE("Aguardando impre.", "pa continar")));
}
