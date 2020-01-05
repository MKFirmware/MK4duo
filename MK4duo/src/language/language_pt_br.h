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
 * Portuguese (Brazil)
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 * See also http://marlinfw.org/docs/development/lcd_language.html
 *
 */

namespace language_pt_br {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Portuguese (BR)"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" pronto."));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Voltar"));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Cartão inserido"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Cartão removido"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Fins de curso"));
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Soft Fins curso"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu principal"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Config. Avançada"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Configuração"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Início automático"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Desabilit. motores"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Menu Debug"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Testar Barra Progres"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Ir a origem XYZ"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Ir na origem X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Ir na origem Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Ir na origem Z"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Auto alinhar Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Indo para origem"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Clique para Iniciar"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Próximo Ponto"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Fim nivelação!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Suavizar altura"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Compensar origem"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Alteração aplicada"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Ajustar Origem"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Pre-aquecer ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Pre-aquecer ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Extrusora ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Extrusora ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Pre-aq.Todo ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Pre-aq.Mesa ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Ajustar ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Pre-aquecer ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Pre-aquecer ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Extrusora ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Extrusora ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Pre-aq.Todo ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Pre-aq.Mesa ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Ajustar ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Customizar Pre-aq."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Esfriar"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Ligar"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Desligar"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrusar"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retrair"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Mover eixo"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Nivelação Mesa"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Nivelar Mesa"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Nivelar Cantos"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Próximo Canto"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Editar Malha"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Fim da Edição"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Índice X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Índice Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Valor Z"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Comando customizado"));

  FSTRINGVALUE(MSG_IDEX_MENU                        , _UxGT("Modo IDEX"));
  FSTRINGVALUE(MSG_IDEX_MODE_AUTOPARK               , _UxGT("Auto-Estacionar"));
  FSTRINGVALUE(MSG_IDEX_MODE_DUPLICATE              , _UxGT("Duplicação"));
  FSTRINGVALUE(MSG_IDEX_MODE_MIRRORED_COPY          , _UxGT("Cópia espelhada"));
  FSTRINGVALUE(MSG_IDEX_MODE_FULL_CTRL              , _UxGT("Controle Total"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2o bico X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2o bico Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2o bico Z"));

  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("Executando G29"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("Ferramentas UBL"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Nivel. Mesa Unif."));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Fazer malha manual"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Calçar e calibrar"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Medir"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Remover e calibrar"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Movendo para Próximo"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Ativar UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Desativar UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Temp. Mesa"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Bed Temp"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Temp. Extrusora"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Hotend Temp"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Editar Malha"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Editar Malha Custom"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Ajuste Fino da Malha"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Fim da Edição"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Montar Malha Custom"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Montar "));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Montar ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Montar ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Montar Malha fria"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Ajustar Altura"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Quant. de Altura"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Validar Malha"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Checar ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Checar ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Validar Malha Custom"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Continuar Malha"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Nivelação da Malha"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("Nivelação 3 pontos"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Nivelação Grid"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Nivelar Malha"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Cantos"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Tipo de Mapa"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Salvar Mapa da Malha"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Enviar Para Host"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Salvar Malha CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Salvar Backup"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Informação do UBL"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Qtd de Enchimento"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Enchimento Manual"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Enchimento Smart"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Preencher malha"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Invalidar tudo"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Invalidar próximo"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Ajuste Fino de Todos"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Ajustar Mais Próximo"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Armazenamento Malha"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Slot de Memória"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Ler Malha"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Salvar Malha"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Malha %i carregada"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Malha %i salva"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Sem armazenamento"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Erro ao salvar UBL"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Erro no restauro UBL"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Compensação Z parou"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("UBL passo a passo"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Montar Malha fria"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Enchimento Smart"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Validar Malha"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Ajuste Fino de Todos"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Validar Malha"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Ajuste Fino de Todos"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Salvar Malha"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Controle do LED"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Luz"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Configuração da Luz"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Luz Vermelha"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Luz Laranja"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Luz Amarela"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Luz Verde"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Luz Azul"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Luz Indigo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Luz Violeta"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Luz Branca"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Luz Padrão"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Luz Customizada"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Intensidade Vermelho"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Intensidade Verde"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Intensidade Azul"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Intensidade Branco"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Brilho"));
  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Movendo..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Liberar XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Mover X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Mover Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Mover Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Mover Extrusor"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Mover Extrusor *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Extrus. mto fria"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Mover %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Mover 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Mover 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Mover 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Velocidade"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Base Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Bocal"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Bocal ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Mesa"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Vel. Ventoinha"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Vel. Ventoinha ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("+Vel. Ventoinha"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("+Vel. Ventoinha ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Vazão"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Vazão ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Controle"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Máx"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fator"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Temp. Automática"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Ligado"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Desligado"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Selecionar"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Selecionar *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Acel."));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Arrancada"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("arrancada V") LCD_STR_A);
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("arrancada V") LCD_STR_B);
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("arrancada V") LCD_STR_C);
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("arrancada VE"));
  FSTRINGVALUE(MSG_JUNCTION_DEVIATION               , _UxGT("Desv. Junção"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Velocidade"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("VDeslocamento min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Aceleração"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("Retrair A"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("Movimento A"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Passo/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , _UxGT("Passo ") LCD_STR_A _UxGT("/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , _UxGT("Passo ") LCD_STR_B _UxGT("/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , _UxGT("Passo ") LCD_STR_C _UxGT("/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("*/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Movimento"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filamento"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("Extrusão em mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Diâmetro Fil."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Diâmetro Fil. *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Descarr. mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Carregar mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("Avanço K"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("Avanço K *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Contraste"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Salvar Configuração"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Ler Configuração"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Restauro seguro"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Iniciar EEPROM"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("Atualiz. SD"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Resetar Impressora"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Atualização"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Informações"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Preparar"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Ajustar"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausar impressão"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Resumir impressão"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Parar impressão"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Recuperar Impressão"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Imprimir do SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Sem cartão SD"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Dormindo..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Clique para retomar"));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Impressão Pausada"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Imprimindo..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Impressão Abortada"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Sem movimento"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("PARADA DE EMERGÊNCIA"));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("PAROU. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retrair mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Retrair Troca mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retrair V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Saltar mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("Des-Retrair mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Des-RetTroca mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("Des-Retrair  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("Des-RetTroca V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Retração Automática"));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Distancia Retração"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Mudar Ferramenta"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Levantar Z"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Preparar Veloc."));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Veloc. Retração"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Trocar Filamento"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Trocar Filamento *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Carregar Filamento *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Descarreg. Filamento *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Descarregar Todos"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Iniciar SD"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Trocar SD"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonda fora da mesa"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Fator de Cisalho"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Testar BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Reiniciar BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Estender BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Recolher BLTouch"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Estender Sonda-Z"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Recolher Sonda-Z"));

  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Home %s%s%s Primeiro"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Compensar Sonda em Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Passinho X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Passinho Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Passinho Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Abortar Fim de Curso"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Aquecimento falhou"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_BED           , _UxGT("Aquecer mesa falhou"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Erro:Temp Redundante"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("ESCAPE TÉRMICO"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_BED              , _UxGT("ESCAPE TÉRMICO MESA"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Erro:Temp Máxima"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Erro:Temp Mínima"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Erro:Temp Mesa Máx"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Erro:Temp Mesa Mín"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Home XY Primeiro"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("IMPRESSORA PAROU"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Favor resetar"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d"));
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h"));
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Aquecendo..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Resfriando..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Aquecendo mesa..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Esfriando mesa..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibrar Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibrar X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibrar Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibrar Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibrar Centro"));

  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Configuração Delta"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto-Calibração"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Calibrar Altura"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Desloc. Sonda Z"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Haste Diagonal"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Altura"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Raio"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Sobre"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Impressora"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("Nivelamento 3 pontos"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Nivelamento Linear"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Nivelamento Bilinear"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Nivelamento UBL"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Nivelamento da Malha"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Estatísticas"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Info. da Placa"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistores"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extrusoras"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Taxa de Transmissão"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocolo"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Luz da Impressora"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Intensidade Brilho"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Impressora Incorreta"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Total de Impressões"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Realizadas"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tempo de Impressão"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Trabalho Mais longo"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Total de Extrusão"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Qtd de Impressões"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Realizadas"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tempo de Impressão"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Maior trabalho"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("T. Extrusão"));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Temp Mín"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Temp Máx"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("PSU"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Força do Motor"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Escrever EEPROM DAC"));

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("IMPRESSÃO PAUSADA"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("CARREGAR FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("DESCARREG. FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("Config. de Retomada"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Purgar mais"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Continuar Impressão"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Bocal: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Sensor filamento"));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Falha ao ir à origem"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Falha ao sondar"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: Muito frio"));

  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Aperte o botão para", "continuar impressão")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Esperando o", "inicio da", "troca de filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Coloque filamento", "pressione o botão", "para continuar...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Pressione o botão", "p/ aquecer o bocal")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Aquecendo o bocal", "Aguarde...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Esperando", "remoção de filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Esperando", "filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_2_LINE("Espere pela", "purga de filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_2_LINE("Clique para finaliz.", "purga de filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Esperando impressão", "continuar")));
  #else // LCD_HEIGHT < 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Clique p. continuar")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Aguarde...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Insira e Clique")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Aquecendo...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Ejetando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Carregando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Purgando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Clique p. finalizar")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Continuando...")));
  #endif
}
