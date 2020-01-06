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
 * Galician language (ISO "gl")
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

namespace language_gl {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 1;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Galician"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" lista."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Tarxeta inserida"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Tarxeta retirada"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("FinCarro"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu principal"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Autoarranque"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Apagar motores"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Ir a orixe"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Ir orixe X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Ir orixe Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Ir orixe Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Ir orixes XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Prema pulsador"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Seguinte punto"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Nivelado feito"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Offsets na orixe"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Offsets fixados"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Fixar orixe"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Prequentar ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Prequentar ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Bico"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Bico ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Todo"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" Cama"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Preque. ") PREHEAT_1_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Prequentar ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Prequentar ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Preque. ") PREHEAT_2_LABEL _UxGT(" Bico"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Preque. ") PREHEAT_2_LABEL _UxGT(" Bico ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Preque. ") PREHEAT_2_LABEL _UxGT(" Todo"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Preque. ") PREHEAT_2_LABEL _UxGT(" Cama"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Preque. ") PREHEAT_2_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Arrefriar"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Acender"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Apagar"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrudir"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retraer"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Mover eixe"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Nivelar cama"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Nivelar cama"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Mover X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Mover Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Mover Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extrusor"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extrusor *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Mover %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Mover 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Mover 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Mover 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Velocidade"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Cama Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Bico"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Bico ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Cama"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Velocidade vent."));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Velocidade vent. ~"));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Fluxo"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Fluxo ~"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fact"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Escolla"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Escolla *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Acel"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Pasos/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" pasos/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" pasos/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" pasos/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E pasos/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* pasos/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Movemento"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filamento"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E en mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Diam. fil."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Diam. fil. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Constraste LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Gardar en memo."));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Cargar de memo."));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Cargar de firm."));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Volver a cargar"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Monitorizacion"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Preparar"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Axustar"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausar impres."));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Seguir impres."));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Deter impres."));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Tarxeta SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Sen tarxeta SD"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("En repouso..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("A espera..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Impre. cancelada"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Sen movemento."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("PROGRAMA MORTO"));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("PROGRAMA PARADO"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retraccion mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Cambio retra. mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retraccion V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Alzar Z mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("Recup. retra. mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Cambio recup. mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("Recuperacion V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Retraccion auto."));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Cambiar filamen."));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Cambiar filamen. *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Iniciando SD"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Cambiar SD"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonda-Z sen cama"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Comprobar BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Iniciar BLTouch"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Offset Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Micropaso X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Micropaso Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Micropaso Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Erro fin carro"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Fallo quentando"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Erro temperatura"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("Temp. excesiva"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("SISTEMA MORTO"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Debe reiniciar!"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Quentando..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Quentando cama..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibracion Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibrar X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibrar Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibrar Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibrar Centro"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Acerca de..."));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Informacion"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Estadisticas"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Placa nai"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistores"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extrusores"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baudios"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocolo"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Luz"));
  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Total traballos"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Total completos"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tempo impresion"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Traballo +longo"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Total extruido"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Traballos"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completos"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tempo"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("O +longo"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extruido"));
  #endif
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Min Temp"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Max Temp"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Fonte alime."));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Potencia motor"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Garda DAC EEPROM"));

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Segue traballo"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Impresora incorrecta"));

  #if LCD_HEIGHT >= 4
    // Up to 3 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Agarde para", "iniciar troco", "de filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_3_LINE("Agarde pola", "descarga do", "filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Introduza o", "filamento e", "faga click")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_3_LINE("Agarde pola", "carga do", "filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_3_LINE("Agarde para", "seguir co", "traballo")));
  #else // LCD_HEIGHT < 4
    // Up to 2 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Agarde...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Descargando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Introduza e click")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Cargando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Seguindo...")));
  #endif // LCD_HEIGHT < 4
}
