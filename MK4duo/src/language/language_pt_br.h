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
 * Portuguese (Brazil)
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 * See also http://marlinfw.org/docs/development/lcd_language.html
 *
 */

namespace Language_pt_br {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t    CHARSIZE                            = 2;
  Language_Str LANGUAGE                            = _UxGT("Portuguese (BR)");

  Language_Str WELCOME_MSG                         = MACHINE_NAME _UxGT(" pronto.");

  Language_Str MSG_BACK                            = _UxGT("Voltar");
  Language_Str MSG_MEDIA_INSERTED                  = _UxGT("Cartão inserido");
  Language_Str MSG_MEDIA_REMOVED                   = _UxGT("Cartão removido");
  Language_Str MSG_LCD_ENDSTOPS                    = _UxGT("Fins de curso");
  Language_Str MSG_LCD_SOFT_ENDSTOPS               = _UxGT("Soft Fins curso");
  Language_Str MSG_MAIN                            = _UxGT("Menu principal");
  Language_Str MSG_ADVANCED_SETTINGS               = _UxGT("Config. Avançada");
  Language_Str MSG_CONFIGURATION                   = _UxGT("Configuração");
  Language_Str MSG_AUTOSTART                       = _UxGT("Início automático");
  Language_Str MSG_DISABLE_STEPPERS                = _UxGT("Desabilit. motores");
  Language_Str MSG_DEBUG_MENU                      = _UxGT("Menu Debug");
  Language_Str MSG_PROGRESS_BAR_TEST               = _UxGT("Testar Barra Progres");
  Language_Str MSG_AUTO_HOME                       = _UxGT("Ir a origem XYZ");
  Language_Str MSG_AUTO_HOME_X                     = _UxGT("Ir na origem X");
  Language_Str MSG_AUTO_HOME_Y                     = _UxGT("Ir na origem Y");
  Language_Str MSG_AUTO_HOME_Z                     = _UxGT("Ir na origem Z");
  Language_Str MSG_AUTO_Z_ALIGN                    = _UxGT("Auto alinhar Z");
  Language_Str MSG_LEVEL_BED_HOMING                = _UxGT("Indo para origem");
  Language_Str MSG_LEVEL_BED_WAITING               = _UxGT("Clique para Iniciar");
  Language_Str MSG_LEVEL_BED_NEXT_POINT            = _UxGT("Próximo Ponto");
  Language_Str MSG_LEVEL_BED_DONE                  = _UxGT("Fim nivelação!");
  Language_Str MSG_Z_FADE_HEIGHT                   = _UxGT("Suavizar altura");
  Language_Str MSG_SET_HOME_OFFSETS                = _UxGT("Compensar origem");
  Language_Str MSG_HOME_OFFSETS_APPLIED            = _UxGT("Alteração aplicada");
  Language_Str MSG_SET_ORIGIN                      = _UxGT("Ajustar Origem");
  Language_Str MSG_PREHEAT_1                       = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL;
  Language_Str MSG_PREHEAT_1_H                     = _UxGT("Pre-aquecer ") PREHEAT_1_LABEL " ~";
  Language_Str MSG_PREHEAT_1_END                   = _UxGT("Extrusora ") PREHEAT_1_LABEL;
  Language_Str MSG_PREHEAT_1_END_E                 = _UxGT("Extrusora ") PREHEAT_1_LABEL " ~";
  Language_Str MSG_PREHEAT_1_ALL                   = _UxGT("Pre-aq.Todo ") PREHEAT_1_LABEL;
  Language_Str MSG_PREHEAT_1_BEDONLY               = _UxGT("Pre-aq.Mesa ") PREHEAT_1_LABEL;
  Language_Str MSG_PREHEAT_1_SETTINGS              = _UxGT("Ajustar ") PREHEAT_1_LABEL;
  Language_Str MSG_PREHEAT_2                       = _UxGT("Pre-aquecer ") PREHEAT_2_LABEL;
  Language_Str MSG_PREHEAT_2_H                     = _UxGT("Pre-aquecer ") PREHEAT_2_LABEL " ~";
  Language_Str MSG_PREHEAT_2_END                   = _UxGT("Extrusora ") PREHEAT_2_LABEL;
  Language_Str MSG_PREHEAT_2_END_E                 = _UxGT("Extrusora ") PREHEAT_2_LABEL " ~";
  Language_Str MSG_PREHEAT_2_ALL                   = _UxGT("Pre-aq.Todo ") PREHEAT_2_LABEL;
  Language_Str MSG_PREHEAT_2_BEDONLY               = _UxGT("Pre-aq.Mesa ") PREHEAT_2_LABEL;
  Language_Str MSG_PREHEAT_2_SETTINGS              = _UxGT("Ajustar ") PREHEAT_2_LABEL;
  Language_Str MSG_PREHEAT_CUSTOM                  = _UxGT("Customizar Pre-aq.");
  Language_Str MSG_COOLDOWN                        = _UxGT("Esfriar");
  Language_Str MSG_SWITCH_PS_ON                    = _UxGT("Ligar");
  Language_Str MSG_SWITCH_PS_OFF                   = _UxGT("Desligar");
  Language_Str MSG_EXTRUDE                         = _UxGT("Extrusar");
  Language_Str MSG_RETRACT                         = _UxGT("Retrair");
  Language_Str MSG_MOVE_AXIS                       = _UxGT("Mover eixo");
  Language_Str MSG_BED_LEVELING                    = _UxGT("Nivelação Mesa");
  Language_Str MSG_LEVEL_BED                       = _UxGT("Nivelar Mesa");
  Language_Str MSG_LEVEL_CORNERS                   = _UxGT("Nivelar Cantos");
  Language_Str MSG_NEXT_CORNER                     = _UxGT("Próximo Canto");
  Language_Str MSG_EDIT_MESH                       = _UxGT("Editar Malha");
  Language_Str MSG_EDITING_STOPPED                 = _UxGT("Fim da Edição");
  Language_Str MSG_MESH_X                          = _UxGT("Índice X");
  Language_Str MSG_MESH_Y                          = _UxGT("Índice Y");
  Language_Str MSG_MESH_EDIT_Z                     = _UxGT("Valor Z");
  Language_Str MSG_USER_MENU                       = _UxGT("Comando customizado");

  Language_Str MSG_IDEX_MENU                       = _UxGT("Modo IDEX");
  Language_Str MSG_IDEX_MODE_AUTOPARK              = _UxGT("Auto-Estacionar");
  Language_Str MSG_IDEX_MODE_DUPLICATE             = _UxGT("Duplicação");
  Language_Str MSG_IDEX_MODE_MIRRORED_COPY         = _UxGT("Cópia espelhada");
  Language_Str MSG_IDEX_MODE_FULL_CTRL             = _UxGT("Controle Total");
  Language_Str MSG_X_OFFSET                        = _UxGT("2o bico X");
  Language_Str MSG_Y_OFFSET                        = _UxGT("2o bico Y");
  Language_Str MSG_Z_OFFSET                        = _UxGT("2o bico Z");

  Language_Str MSG_UBL_DOING_G29                   = _UxGT("Executando G29");
  Language_Str MSG_UBL_TOOLS                       = _UxGT("Ferramentas UBL");
  Language_Str MSG_UBL_LEVEL_BED                   = _UxGT("Nivel. Mesa Unif.");
  Language_Str MSG_UBL_MANUAL_MESH                 = _UxGT("Fazer malha manual");
  Language_Str MSG_UBL_BC_INSERT                   = _UxGT("Calçar e calibrar");
  Language_Str MSG_UBL_BC_INSERT2                  = _UxGT("Medir");
  Language_Str MSG_UBL_BC_REMOVE                   = _UxGT("Remover e calibrar");
  Language_Str MSG_UBL_MOVING_TO_NEXT              = _UxGT("Movendo para Próximo");
  Language_Str MSG_UBL_ACTIVATE_MESH               = _UxGT("Ativar UBL");
  Language_Str MSG_UBL_DEACTIVATE_MESH             = _UxGT("Desativar UBL");
  Language_Str MSG_UBL_SET_TEMP_BED                = _UxGT("Temp. Mesa");
  Language_Str MSG_UBL_BED_TEMP_CUSTOM             = _UxGT("Bed Temp");
  Language_Str MSG_UBL_SET_TEMP_HOTEND             = _UxGT("Temp. Extrusora");
  Language_Str MSG_UBL_HOTEND_TEMP_CUSTOM          = _UxGT("Hotend Temp");
  Language_Str MSG_UBL_MESH_EDIT                   = _UxGT("Editar Malha");
  Language_Str MSG_UBL_EDIT_CUSTOM_MESH            = _UxGT("Editar Malha Custom");
  Language_Str MSG_UBL_FINE_TUNE_MESH              = _UxGT("Ajuste Fino da Malha");
  Language_Str MSG_UBL_DONE_EDITING_MESH           = _UxGT("Fim da Edição");
  Language_Str MSG_UBL_BUILD_CUSTOM_MESH           = _UxGT("Montar Malha Custom");
  Language_Str MSG_UBL_BUILD_MESH_MENU             = _UxGT("Montar ");
  Language_Str MSG_UBL_BUILD_MESH_M1               = _UxGT("Montar ") PREHEAT_1_LABEL;
  Language_Str MSG_UBL_BUILD_MESH_M2               = _UxGT("Montar ") PREHEAT_2_LABEL;
  Language_Str MSG_UBL_BUILD_COLD_MESH             = _UxGT("Montar Malha fria");
  Language_Str MSG_UBL_MESH_HEIGHT_ADJUST          = _UxGT("Ajustar Altura");
  Language_Str MSG_UBL_MESH_HEIGHT_AMOUNT          = _UxGT("Quant. de Altura");
  Language_Str MSG_UBL_VALIDATE_MESH_MENU          = _UxGT("Validar Malha");
  Language_Str MSG_UBL_VALIDATE_MESH_M1            = _UxGT("Checar ") PREHEAT_1_LABEL;
  Language_Str MSG_UBL_VALIDATE_MESH_M2            = _UxGT("Checar ") PREHEAT_2_LABEL;
  Language_Str MSG_UBL_VALIDATE_CUSTOM_MESH        = _UxGT("Validar Malha Custom");
  Language_Str MSG_UBL_CONTINUE_MESH               = _UxGT("Continuar Malha");
  Language_Str MSG_UBL_MESH_LEVELING               = _UxGT("Nivelação da Malha");
  Language_Str MSG_UBL_3POINT_MESH_LEVELING        = _UxGT("Nivelação 3 pontos");
  Language_Str MSG_UBL_GRID_MESH_LEVELING          = _UxGT("Nivelação Grid");
  Language_Str MSG_UBL_MESH_LEVEL                  = _UxGT("Nivelar Malha");
  Language_Str MSG_UBL_SIDE_POINTS                 = _UxGT("Cantos");
  Language_Str MSG_UBL_MAP_TYPE                    = _UxGT("Tipo de Mapa");
  Language_Str MSG_UBL_OUTPUT_MAP                  = _UxGT("Salvar Mapa da Malha");
  Language_Str MSG_UBL_OUTPUT_MAP_HOST             = _UxGT("Enviar Para Host");
  Language_Str MSG_UBL_OUTPUT_MAP_CSV              = _UxGT("Salvar Malha CSV");
  Language_Str MSG_UBL_OUTPUT_MAP_BACKUP           = _UxGT("Salvar Backup");
  Language_Str MSG_UBL_INFO_UBL                    = _UxGT("Informação do UBL");
  Language_Str MSG_UBL_FILLIN_AMOUNT               = _UxGT("Qtd de Enchimento");
  Language_Str MSG_UBL_MANUAL_FILLIN               = _UxGT("Enchimento Manual");
  Language_Str MSG_UBL_SMART_FILLIN                = _UxGT("Enchimento Smart");
  Language_Str MSG_UBL_FILLIN_MESH                 = _UxGT("Preencher malha");
  Language_Str MSG_UBL_INVALIDATE_ALL              = _UxGT("Invalidar tudo");
  Language_Str MSG_UBL_INVALIDATE_CLOSEST          = _UxGT("Invalidar próximo");
  Language_Str MSG_UBL_FINE_TUNE_ALL               = _UxGT("Ajuste Fino de Todos");
  Language_Str MSG_UBL_FINE_TUNE_CLOSEST           = _UxGT("Ajustar Mais Próximo");
  Language_Str MSG_UBL_STORAGE_MESH_MENU           = _UxGT("Armazenamento Malha");
  Language_Str MSG_UBL_STORAGE_SLOT                = _UxGT("Slot de Memória");
  Language_Str MSG_UBL_LOAD_MESH                   = _UxGT("Ler Malha");
  Language_Str MSG_UBL_SAVE_MESH                   = _UxGT("Salvar Malha");
  Language_Str MSG_MESH_LOADED                     = _UxGT("M117 Malha %i carregada");
  Language_Str MSG_MESH_SAVED                      = _UxGT("M117 Malha %i salva");
  Language_Str MSG_UBL_NO_STORAGE                  = _UxGT("Sem armazenamento");
  Language_Str MSG_UBL_SAVE_ERROR                  = _UxGT("Erro ao salvar UBL");
  Language_Str MSG_UBL_RESTORE_ERROR               = _UxGT("Erro no restauro UBL");
  Language_Str MSG_UBL_Z_OFFSET_STOPPED            = _UxGT("Compensação Z parou");
  Language_Str MSG_UBL_STEP_BY_STEP_MENU           = _UxGT("UBL passo a passo");
  Language_Str MSG_UBL_1_BUILD_COLD_MESH           = _UxGT("1.Montar Malha fria");
  Language_Str MSG_UBL_2_SMART_FILLIN              = _UxGT("2.Enchimento Smart");
  Language_Str MSG_UBL_3_VALIDATE_MESH_MENU        = _UxGT("3.Validar Malha");
  Language_Str MSG_UBL_4_FINE_TUNE_ALL             = _UxGT("4.Ajuste Fino de Todos");
  Language_Str MSG_UBL_5_VALIDATE_MESH_MENU        = _UxGT("5.Validar Malha");
  Language_Str MSG_UBL_6_FINE_TUNE_ALL             = _UxGT("6.Ajuste Fino de Todos");
  Language_Str MSG_UBL_7_SAVE_MESH                 = _UxGT("7.Salvar Malha");

  Language_Str MSG_LED_CONTROL                     = _UxGT("Controle do LED");
  Language_Str MSG_LEDS                            = _UxGT("Luz");
  Language_Str MSG_LED_PRESETS                     = _UxGT("Configuração da Luz");
  Language_Str MSG_SET_LEDS_RED                    = _UxGT("Luz Vermelha");
  Language_Str MSG_SET_LEDS_ORANGE                 = _UxGT("Luz Laranja");
  Language_Str MSG_SET_LEDS_YELLOW                 = _UxGT("Luz Amarela");
  Language_Str MSG_SET_LEDS_GREEN                  = _UxGT("Luz Verde");
  Language_Str MSG_SET_LEDS_BLUE                   = _UxGT("Luz Azul");
  Language_Str MSG_SET_LEDS_INDIGO                 = _UxGT("Luz Indigo");
  Language_Str MSG_SET_LEDS_VIOLET                 = _UxGT("Luz Violeta");
  Language_Str MSG_SET_LEDS_WHITE                  = _UxGT("Luz Branca");
  Language_Str MSG_SET_LEDS_DEFAULT                = _UxGT("Luz Padrão");
  Language_Str MSG_CUSTOM_LEDS                     = _UxGT("Luz Customizada");
  Language_Str MSG_INTENSITY_R                     = _UxGT("Intensidade Vermelho");
  Language_Str MSG_INTENSITY_G                     = _UxGT("Intensidade Verde");
  Language_Str MSG_INTENSITY_B                     = _UxGT("Intensidade Azul");
  Language_Str MSG_INTENSITY_W                     = _UxGT("Intensidade Branco");
  Language_Str MSG_LED_BRIGHTNESS                  = _UxGT("Brilho");
  Language_Str MSG_MOVING                          = _UxGT("Movendo...");
  Language_Str MSG_FREE_XY                         = _UxGT("Liberar XY");
  Language_Str MSG_MOVE_X                          = _UxGT("Mover X");
  Language_Str MSG_MOVE_Y                          = _UxGT("Mover Y");
  Language_Str MSG_MOVE_Z                          = _UxGT("Mover Z");
  Language_Str MSG_MOVE_E                          = _UxGT("Mover Extrusor");
  Language_Str MSG_MOVE_EN                         = _UxGT("Mover Extrusor *");
  Language_Str MSG_HOTEND_TOO_COLD                 = _UxGT("Extrus. mto fria");
  Language_Str MSG_MOVE_Z_DIST                     = _UxGT("Mover %smm");
  Language_Str MSG_MOVE_01MM                       = _UxGT("Mover 0.1mm");
  Language_Str MSG_MOVE_1MM                        = _UxGT("Mover 1mm");
  Language_Str MSG_MOVE_10MM                       = _UxGT("Mover 10mm");
  Language_Str MSG_SPEED                           = _UxGT("Velocidade");
  Language_Str MSG_BED_Z                           = _UxGT("Base Z");
  Language_Str MSG_NOZZLE                          = _UxGT("Bocal");
  Language_Str MSG_NOZZLE_N                        = _UxGT("Bocal ~");
  Language_Str MSG_BED                             = _UxGT("Mesa");
  Language_Str MSG_FAN_SPEED                       = _UxGT("Vel. Ventoinha");
  Language_Str MSG_FAN_SPEED_N                     = _UxGT("Vel. Ventoinha =");
  Language_Str MSG_EXTRA_FAN_SPEED                 = _UxGT("+Vel. Ventoinha");
  Language_Str MSG_EXTRA_FAN_SPEED_N               = _UxGT("+Vel. Ventoinha =");
  Language_Str MSG_FLOW                            = _UxGT("Vazão");
  Language_Str MSG_FLOW_N                          = _UxGT("Vazão ~");
  Language_Str MSG_CONTROL                         = _UxGT("Controle");
  Language_Str MSG_MIN                             = " " LCD_STR_THERMOMETER _UxGT(" Min");
  Language_Str MSG_MAX                             = " " LCD_STR_THERMOMETER _UxGT(" Máx");
  Language_Str MSG_FACTOR                          = " " LCD_STR_THERMOMETER _UxGT(" Fator");
  Language_Str MSG_AUTOTEMP                        = _UxGT("Temp. Automática");
  Language_Str MSG_LCD_ON                          = _UxGT("Ligado");
  Language_Str MSG_LCD_OFF                         = _UxGT("Desligado");
  Language_Str MSG_SELECT                          = _UxGT("Selecionar");
  Language_Str MSG_SELECT_E                        = _UxGT("Selecionar *");
  Language_Str MSG_ACC                             = _UxGT("Acel.");
  Language_Str MSG_JERK                            = _UxGT("Arrancada");
  Language_Str MSG_VA_JERK                         = _UxGT("arrancada V") LCD_STR_A;
  Language_Str MSG_VB_JERK                         = _UxGT("arrancada V") LCD_STR_B;
  Language_Str MSG_VC_JERK                         = _UxGT("arrancada V") LCD_STR_C;
  Language_Str MSG_VE_JERK                         = _UxGT("arrancada VE");
  Language_Str MSG_JUNCTION_DEVIATION              = _UxGT("Desv. Junção");
  Language_Str MSG_VELOCITY                        = _UxGT("Velocidade");
  Language_Str MSG_VTRAV_MIN                       = _UxGT("VDeslocamento min");
  Language_Str MSG_ACCELERATION                    = _UxGT("Aceleração");
  Language_Str MSG_A_RETRACT                       = _UxGT("Retrair A");
  Language_Str MSG_A_TRAVEL                        = _UxGT("Movimento A");
  Language_Str MSG_STEPS_PER_MM                    = _UxGT("Passo/mm");
  Language_Str MSG_A_STEPS                         = _UxGT("Passo ") LCD_STR_A _UxGT("/mm");
  Language_Str MSG_B_STEPS                         = _UxGT("Passo ") LCD_STR_B _UxGT("/mm");
  Language_Str MSG_C_STEPS                         = _UxGT("Passo ") LCD_STR_C _UxGT("/mm");
  Language_Str MSG_E_STEPS                         = _UxGT("E/mm");
  Language_Str MSG_EN_STEPS                        = _UxGT("*/mm");
  Language_Str MSG_TEMPERATURE                     = _UxGT("Temperatura");
  Language_Str MSG_MOTION                          = _UxGT("Movimento");
  Language_Str MSG_FILAMENT                        = _UxGT("Filamento");
  Language_Str MSG_VOLUMETRIC_ENABLED              = _UxGT("Extrusão em mm3");
  Language_Str MSG_FILAMENT_DIAM                   = _UxGT("Diâmetro Fil.");
  Language_Str MSG_FILAMENT_DIAM_E                 = _UxGT("Diâmetro Fil. *");
  Language_Str MSG_FILAMENT_UNLOAD                 = _UxGT("Descarr. mm");
  Language_Str MSG_FILAMENT_LOAD                   = _UxGT("Carregar mm");
  Language_Str MSG_ADVANCE_K                       = _UxGT("Avanço K");
  Language_Str MSG_ADVANCE_K_E                     = _UxGT("Avanço K *");
  Language_Str MSG_CONTRAST                        = _UxGT("Contraste");
  Language_Str MSG_STORE_EEPROM                    = _UxGT("Salvar Configuração");
  Language_Str MSG_LOAD_EEPROM                     = _UxGT("Ler Configuração");
  Language_Str MSG_RESTORE_FAILSAFE                = _UxGT("Restauro seguro");
  Language_Str MSG_INIT_EEPROM                     = _UxGT("Iniciar EEPROM");
  Language_Str MSG_MEDIA_UPDATE                    = _UxGT("Atualiz. SD");
  Language_Str MSG_RESET_PRINTER                   = _UxGT("Resetar Impressora");
  Language_Str MSG_REFRESH                         = LCD_STR_REFRESH  _UxGT("Atualização");
  Language_Str MSG_WATCH                           = _UxGT("Informações");
  Language_Str MSG_PREPARE                         = _UxGT("Preparar");
  Language_Str MSG_TUNE                            = _UxGT("Ajustar");
  Language_Str MSG_PAUSE_PRINT                     = _UxGT("Pausar impressão");
  Language_Str MSG_RESUME_PRINT                    = _UxGT("Resumir impressão");
  Language_Str MSG_STOP_PRINT                      = _UxGT("Parar impressão");
  Language_Str MSG_OUTAGE_RECOVERY                 = _UxGT("Recuperar Impressão");
  Language_Str MSG_MEDIA_MENU                      = _UxGT("Imprimir do SD");
  Language_Str MSG_NO_MEDIA                        = _UxGT("Sem cartão SD");
  Language_Str MSG_DWELL                           = _UxGT("Dormindo...");
  Language_Str MSG_USERWAIT                        = _UxGT("Clique para retomar");
  Language_Str MSG_PRINT_PAUSED                    = _UxGT("Impressão Pausada");
  Language_Str MSG_PRINTING                        = _UxGT("Imprimindo...");
  Language_Str MSG_PRINT_ABORTED                   = _UxGT("Impressão Abortada");
  Language_Str MSG_NO_MOVE                         = _UxGT("Sem movimento");
  Language_Str MSG_KILLED                          = _UxGT("PARADA DE EMERGÊNCIA");
  Language_Str MSG_STOPPED                         = _UxGT("PAROU. ");
  Language_Str MSG_CONTROL_RETRACT                 = _UxGT("Retrair mm");
  Language_Str MSG_CONTROL_RETRACT_SWAP            = _UxGT("Retrair Troca mm");
  Language_Str MSG_CONTROL_RETRACTF                = _UxGT("Retrair V");
  Language_Str MSG_CONTROL_RETRACT_ZHOP            = _UxGT("Saltar mm");
  Language_Str MSG_CONTROL_RETRACT_RECOVER         = _UxGT("Des-Retrair mm");
  Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAP    = _UxGT("Des-RetTroca mm");
  Language_Str MSG_CONTROL_RETRACT_RECOVERF        = _UxGT("Des-Retrair  V");
  Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAPF   = _UxGT("Des-RetTroca V");
  Language_Str MSG_AUTORETRACT                     = _UxGT("Retração Automática");
  Language_Str MSG_FILAMENT_SWAP_LENGTH            = _UxGT("Distancia Retração");
  Language_Str MSG_TOOL_CHANGE                     = _UxGT("Mudar Ferramenta");
  Language_Str MSG_TOOL_CHANGE_ZLIFT               = _UxGT("Levantar Z");
  Language_Str MSG_SINGLENOZZLE_PRIME_SPD          = _UxGT("Preparar Veloc.");
  Language_Str MSG_SINGLENOZZLE_RETRACT_SPD        = _UxGT("Veloc. Retração");
  Language_Str MSG_FILAMENTCHANGE                  = _UxGT("Trocar Filamento");
  Language_Str MSG_FILAMENTCHANGE_E                = _UxGT("Trocar Filamento *");
  Language_Str MSG_FILAMENTLOAD_E                  = _UxGT("Carregar Filamento *");
  Language_Str MSG_FILAMENTUNLOAD_E                = _UxGT("Descarreg. Filamento *");
  Language_Str MSG_FILAMENTUNLOAD_ALL              = _UxGT("Descarregar Todos");
  Language_Str MSG_INIT_MEDIA                      = _UxGT("Iniciar SD");
  Language_Str MSG_CHANGE_MEDIA                    = _UxGT("Trocar SD");
  Language_Str MSG_ZPROBE_OUT                      = _UxGT("Sonda fora da mesa");
  Language_Str MSG_SKEW_FACTOR                     = _UxGT("Fator de Cisalho");
  Language_Str MSG_BLTOUCH                         = _UxGT("BLTouch");
  Language_Str MSG_BLTOUCH_SELFTEST                = _UxGT("Testar BLTouch");
  Language_Str MSG_BLTOUCH_RESET                   = _UxGT("Reiniciar BLTouch");
  Language_Str MSG_BLTOUCH_DEPLOY                  = _UxGT("Estender BLTouch");
  Language_Str MSG_BLTOUCH_STOW                    = _UxGT("Recolher BLTouch");
  Language_Str MSG_MANUAL_DEPLOY                   = _UxGT("Estender Sonda-Z");
  Language_Str MSG_MANUAL_STOW                     = _UxGT("Recolher Sonda-Z");

  Language_Str MSG_HOME_FIRST                      = _UxGT("Home %s%s%s Primeiro");
  Language_Str MSG_ZPROBE_ZOFFSET                  = _UxGT("Compensar Sonda em Z");
  Language_Str MSG_BABYSTEP_X                      = _UxGT("Passinho X");
  Language_Str MSG_BABYSTEP_Y                      = _UxGT("Passinho Y");
  Language_Str MSG_BABYSTEP_Z                      = _UxGT("Passinho Z");
  Language_Str MSG_ENDSTOP_ABORT                   = _UxGT("Abortar Fim de Curso");
  Language_Str MSG_HEATING_FAILED_LCD              = _UxGT("Aquecimento falhou");
  Language_Str MSG_HEATING_FAILED_LCD_BED          = _UxGT("Aquecer mesa falhou");
  Language_Str MSG_ERR_REDUNDANT_TEMP              = _UxGT("Erro:Temp Redundante");
  Language_Str MSG_THERMAL_RUNAWAY                 = _UxGT("ESCAPE TÉRMICO");
  Language_Str MSG_THERMAL_RUNAWAY_BED             = _UxGT("ESCAPE TÉRMICO MESA");
  Language_Str MSG_ERR_MAXTEMP                     = _UxGT("Erro:Temp Máxima");
  Language_Str MSG_ERR_MINTEMP                     = _UxGT("Erro:Temp Mínima");
  Language_Str MSG_ERR_MAXTEMP_BED                 = _UxGT("Erro:Temp Mesa Máx");
  Language_Str MSG_ERR_MINTEMP_BED                 = _UxGT("Erro:Temp Mesa Mín");
  Language_Str MSG_ERR_Z_HOMING                    = _UxGT("Home XY Primeiro");
  Language_Str MSG_HALTED                          = _UxGT("IMPRESSORA PAROU");
  Language_Str MSG_PLEASE_RESET                    = _UxGT("Favor resetar");
  Language_Str MSG_SHORT_DAY                       = _UxGT("d");
  Language_Str MSG_SHORT_HOUR                      = _UxGT("h");
  Language_Str MSG_SHORT_MINUTE                    = _UxGT("m");
  Language_Str MSG_HEATING                         = _UxGT("Aquecendo...");
  Language_Str MSG_COOLING                         = _UxGT("Resfriando...");
  Language_Str MSG_BED_HEATING                     = _UxGT("Aquecendo mesa...");
  Language_Str MSG_BED_COOLING                     = _UxGT("Esfriando mesa...");
  Language_Str MSG_DELTA_CALIBRATE                 = _UxGT("Calibrar Delta");
  Language_Str MSG_DELTA_CALIBRATE_X               = _UxGT("Calibrar X");
  Language_Str MSG_DELTA_CALIBRATE_Y               = _UxGT("Calibrar Y");
  Language_Str MSG_DELTA_CALIBRATE_Z               = _UxGT("Calibrar Z");
  Language_Str MSG_DELTA_CALIBRATE_CENTER          = _UxGT("Calibrar Centro");

  Language_Str MSG_DELTA_SETTINGS                  = _UxGT("Configuração Delta");
  Language_Str MSG_DELTA_AUTO_CALIBRATE            = _UxGT("Auto-Calibração");
  Language_Str MSG_DELTA_HEIGHT_CALIBRATE          = _UxGT("Calibrar Altura");
  Language_Str MSG_DELTA_Z_OFFSET_CALIBRATE        = _UxGT("Desloc. Sonda Z");
  Language_Str MSG_DELTA_DIAG_ROD                  = _UxGT("Haste Diagonal");
  Language_Str MSG_DELTA_HEIGHT                    = _UxGT("Altura");
  Language_Str MSG_DELTA_RADIUS                    = _UxGT("Raio");
  Language_Str MSG_INFO_MENU                       = _UxGT("Sobre");
  Language_Str MSG_INFO_PRINTER_MENU               = _UxGT("Impressora");
  Language_Str MSG_3POINT_LEVELING                 = _UxGT("Nivelamento 3 pontos");
  Language_Str MSG_LINEAR_LEVELING                 = _UxGT("Nivelamento Linear");
  Language_Str MSG_BILINEAR_LEVELING               = _UxGT("Nivelamento Bilinear");
  Language_Str MSG_UBL_LEVELING                    = _UxGT("Nivelamento UBL");
  Language_Str MSG_MESH_LEVELING                   = _UxGT("Nivelamento da Malha");
  Language_Str MSG_INFO_STATS_MENU                 = _UxGT("Estatísticas");
  Language_Str MSG_INFO_BOARD_MENU                 = _UxGT("Info. da Placa");
  Language_Str MSG_INFO_THERMISTOR_MENU            = _UxGT("Termistores");
  Language_Str MSG_INFO_EXTRUDERS                  = _UxGT("Extrusoras");
  Language_Str MSG_INFO_BAUDRATE                   = _UxGT("Taxa de Transmissão");
  Language_Str MSG_INFO_PROTOCOL                   = _UxGT("Protocolo");
  Language_Str MSG_CASE_LIGHT                      = _UxGT("Luz da Impressora");
  Language_Str MSG_CASE_LIGHT_BRIGHTNESS           = _UxGT("Intensidade Brilho");

  Language_Str MSG_EXPECTED_PRINTER                = _UxGT("Impressora Incorreta");

  #if LCD_WIDTH >= 20
    Language_Str MSG_INFO_PRINT_COUNT              = _UxGT("Total de Impressões");
    Language_Str MSG_INFO_COMPLETED_PRINTS         = _UxGT("Realizadas");
    Language_Str MSG_INFO_PRINT_TIME               = _UxGT("Tempo de Impressão");
    Language_Str MSG_INFO_PRINT_LONGEST            = _UxGT("Trabalho Mais longo");
    Language_Str MSG_INFO_PRINT_FILAMENT           = _UxGT("Total de Extrusão");
  #else
    Language_Str MSG_INFO_PRINT_COUNT              = _UxGT("Qtd de Impressões");
    Language_Str MSG_INFO_COMPLETED_PRINTS         = _UxGT("Realizadas");
    Language_Str MSG_INFO_PRINT_TIME               = _UxGT("Tempo de Impressão");
    Language_Str MSG_INFO_PRINT_LONGEST            = _UxGT("Maior trabalho");
    Language_Str MSG_INFO_PRINT_FILAMENT           = _UxGT("T. Extrusão");
  #endif

  Language_Str MSG_INFO_MIN_TEMP                   = _UxGT("Temp Mín");
  Language_Str MSG_INFO_MAX_TEMP                   = _UxGT("Temp Máx");
  Language_Str MSG_INFO_PSU                        = _UxGT("PSU");
  Language_Str MSG_DRIVE_STRENGTH                  = _UxGT("Força do Motor");
  Language_Str MSG_DAC_EEPROM_WRITE                = _UxGT("Escrever EEPROM DAC");

  Language_Str MSG_FILAMENT_CHANGE_HEADER_PAUSE    = _UxGT("IMPRESSÃO PAUSADA");
  Language_Str MSG_FILAMENT_CHANGE_HEADER_LOAD     = _UxGT("CARREGAR FILAMENTO");
  Language_Str MSG_FILAMENT_CHANGE_HEADER_UNLOAD   = _UxGT("DESCARREG. FILAMENTO");
  Language_Str MSG_FILAMENT_CHANGE_OPTION_HEADER   = _UxGT("Config. de Retomada");
  Language_Str MSG_FILAMENT_CHANGE_OPTION_PURGE    = _UxGT("Purgar mais");
  Language_Str MSG_FILAMENT_CHANGE_OPTION_RESUME   = _UxGT("Continuar Impressão");
  Language_Str MSG_FILAMENT_CHANGE_NOZZLE          = _UxGT("  Bocal: ");
  Language_Str MSG_RUNOUT_SENSOR                   = _UxGT("Sensor filamento");
  Language_Str MSG_LCD_HOMING_FAILED               = _UxGT("Falha ao ir à origem");
  Language_Str MSG_LCD_PROBING_FAILED              = _UxGT("Falha ao sondar");
  Language_Str MSG_M600_TOO_COLD                   = _UxGT("M600: Muito frio");

  #if LCD_HEIGHT >= 4
    Language_Str MSG_ADVANCED_PAUSE_WAITING        = _UxGT(MSG_2_LINE("Aperte o botão para", "continuar impressão"));
    Language_Str MSG_FILAMENT_CHANGE_INIT          = _UxGT(MSG_3_LINE("Esperando o", "inicio da", "troca de filamento"));
    Language_Str MSG_FILAMENT_CHANGE_INSERT        = _UxGT(MSG_3_LINE("Coloque filamento", "pressione o botão", "para continuar..."));
    Language_Str MSG_FILAMENT_CHANGE_HEAT          = _UxGT(MSG_2_LINE("Pressione o botão", "p/ aquecer o bocal"));
    Language_Str MSG_FILAMENT_CHANGE_HEATING       = _UxGT(MSG_2_LINE("Aquecendo o bocal", "Aguarde..."));
    Language_Str MSG_FILAMENT_CHANGE_UNLOAD        = _UxGT(MSG_2_LINE("Esperando", "remoção de filamento"));
    Language_Str MSG_FILAMENT_CHANGE_LOAD          = _UxGT(MSG_2_LINE("Esperando", "filamento"));
    Language_Str MSG_FILAMENT_CHANGE_PURGE         = _UxGT(MSG_2_LINE("Espere pela", "purga de filamento"));
    Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE    = _UxGT(MSG_2_LINE("Clique para finaliz.", "purga de filamento"));
    Language_Str MSG_FILAMENT_CHANGE_RESUME        = _UxGT(MSG_2_LINE("Esperando impressão", "continuar"));
  #else // LCD_HEIGHT < 4
    Language_Str MSG_ADVANCED_PAUSE_WAITING        = _UxGT(MSG_1_LINE("Clique p. continuar"));
    Language_Str MSG_FILAMENT_CHANGE_INIT          = _UxGT(MSG_1_LINE("Aguarde..."));
    Language_Str MSG_FILAMENT_CHANGE_INSERT        = _UxGT(MSG_1_LINE("Insira e Clique"));
    Language_Str MSG_FILAMENT_CHANGE_HEATING       = _UxGT(MSG_1_LINE("Aquecendo..."));
    Language_Str MSG_FILAMENT_CHANGE_UNLOAD        = _UxGT(MSG_1_LINE("Ejetando..."));
    Language_Str MSG_FILAMENT_CHANGE_LOAD          = _UxGT(MSG_1_LINE("Carregando..."));
    Language_Str MSG_FILAMENT_CHANGE_PURGE         = _UxGT(MSG_1_LINE("Purgando..."));
    Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE    = _UxGT(MSG_1_LINE("Clique p. finalizar"));
    Language_Str MSG_FILAMENT_CHANGE_RESUME        = _UxGT(MSG_1_LINE("Continuando..."));
  #endif
}
