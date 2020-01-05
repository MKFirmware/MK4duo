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
 * Portuguese
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 *
 */

 #define DISPLAY_CHARSET_ISO10646_1

namespace language_pt {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Portuguese"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" pronta."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Cartão inserido"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Cartão removido"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu principal"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Desactivar motores"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Ir para origem"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Ir para origem X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Ir para origem Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Ir para origem Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Indo para origem"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Click para iniciar"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Próximo ponto"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Pronto !"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Definir desvio"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Offsets aplicados"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Definir origem"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Pre-aquecer ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Pre-aquecer ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Pre-aquecer ") PREHEAT_1_LABEL _UxGT(" Bico"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Pre-aquecer ") PREHEAT_1_LABEL _UxGT(" Bico ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Pre-aq. ") PREHEAT_1_LABEL _UxGT(" Tudo"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Pre-aq. ") PREHEAT_1_LABEL _UxGT(" ") LCD_STR_THERMOMETER _UxGT("Base"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Definições ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Pre-aquecer ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Pre-aquecer ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Pre-aquecer ") PREHEAT_2_LABEL _UxGT(" Bico"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Pre-aquecer ") PREHEAT_2_LABEL _UxGT(" Bico ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Pre-aq. ") PREHEAT_2_LABEL _UxGT(" Tudo"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Pre-aq. ") PREHEAT_2_LABEL _UxGT(" ") LCD_STR_THERMOMETER _UxGT("Base"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Definições ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Arrefecer"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Ligar"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Desligar"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrudir"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retrair"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Mover eixo"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Mover X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Mover Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Mover Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Mover Extrusor"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Mover Extrusor *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Mover %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Mover 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Mover 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Mover 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Velocidade"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Base Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , " " LCD_STR_THERMOMETER _UxGT(" Bico"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , " " LCD_STR_THERMOMETER _UxGT(" Bico ~"));
  FSTRINGVALUE(MSG_BED                              , " " LCD_STR_THERMOMETER _UxGT(" Base"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Vel. ventoinha"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Vel. ventoinha ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Fluxo"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Fluxo ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Controlo"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fact"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-retracção"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-movimento"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Passo/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" passo/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" passo/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" passo/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E passo/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* passo/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Movimento"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filamento"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E em mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Fil. Diam."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Fil. Diam. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Contraste"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Guardar na memoria"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Carregar da memoria"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Rest. de emergen."));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH _UxGT(" Recarregar"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Monitorizar"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Preparar"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Afinar"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausar impressão"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Retomar impressão"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Parar impressão"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Imprimir do SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Sem cartão SD"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Em espera..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Á espera de ordem"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Impressão cancelada"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Sem movimento"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("EMERGÊNCIA. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("PARADO. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT(" Retrair mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Troca Retrair mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT(" Retrair  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT(" Levantar mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT(" DesRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Troca DesRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT(" DesRet  V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT(" AutoRetr."));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Trocar filamento"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Trocar filamento *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Inici. cartão SD"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Trocar cartão SD"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sensor fora/base"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Desvio Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Fim de curso"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Aquecimento falhou"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err: T Máxima"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err: T Mínima"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Err: T Base Máxima"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Err: T Base Mínima"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Aquecendo..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Aquecendo base..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibração Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibrar X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibrar Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibrar Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibrar Centro"));

  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Fim de curso"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Impressora Incorreta"));
}
