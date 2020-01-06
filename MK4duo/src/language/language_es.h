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
 * Spanish
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace language_es {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Spanish"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" lista"));
  FSTRINGVALUE(MSG_YES                              , _UxGT("SI"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("NO"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Atrás"));
  FSTRINGVALUE(MSG_MEDIA_ABORTING                   , _UxGT("Cancelando..."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("SD/USB insertado"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("SD/USB retirado"));
  FSTRINGVALUE(MSG_MEDIA_RELEASED                   , _UxGT("SD/USB lanzado"));
  FSTRINGVALUE(MSG_MEDIA_WAITING                    , _UxGT("Esperando al SD/USB"));
  FSTRINGVALUE(MSG_MEDIA_READ_ERROR                 , _UxGT("Error lectura SD/USB"));
  FSTRINGVALUE(MSG_MEDIA_USB_REMOVED                , _UxGT("Disp. USB retirado"));
  FSTRINGVALUE(MSG_MEDIA_USB_FAILED                 , _UxGT("Inicio USB fallido"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstops")); // Max length 8 characters
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Soft Endstops"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menú principal"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Ajustes avanzados"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Configuración"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Inicio automático"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Apagar motores"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Menú depuración"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Prob. barra progreso"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Llevar al origen"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Origen X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Origen Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Origen Z"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Auto alineado Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Origen XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Pulsar para comenzar"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Siguiente punto"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("¡Nivelación lista!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Compensación Altura"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Ajustar desfases"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Desfase aplicada"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Establecer origen"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Precalentar ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Precalentar ") PREHEAT_1_LABEL " H");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Fin"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Fin E"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Todo"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Cama"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Precalentar ") PREHEAT_1_LABEL _UxGT(" Ajuste"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Precalentar ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Precalentar ") PREHEAT_2_LABEL " H");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Fin"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Fin E"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Todo"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Cama"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Precalentar ") PREHEAT_2_LABEL _UxGT(" Ajuste"));
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Precalen. Personali."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Enfriar"));
  FSTRINGVALUE(MSG_LASER_MENU                       , _UxGT("Control Láser"));
  FSTRINGVALUE(MSG_LASER_OFF                        , _UxGT("Láser Apagado"));
  FSTRINGVALUE(MSG_LASER_ON                         , _UxGT("Láser Encendido"));
  FSTRINGVALUE(MSG_LASER_POWER                      , _UxGT("Potencia Láser"));
  FSTRINGVALUE(MSG_SPINDLE_MENU                     , _UxGT("Control Mandrino"));
  FSTRINGVALUE(MSG_SPINDLE_OFF                      , _UxGT("Mandrino Apagado"));
  FSTRINGVALUE(MSG_SPINDLE_ON                       , _UxGT("Mandrino Encendido"));
  FSTRINGVALUE(MSG_SPINDLE_POWER                    , _UxGT("Potencia Mandrino"));
  FSTRINGVALUE(MSG_SPINDLE_REVERSE                  , _UxGT("Invertir giro"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Encender Fuente"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Apagar Fuente"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extruir"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retraer"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Mover ejes"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Nivelando Cama"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Nivelar Cama"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Nivelar Esquinas"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Siguente Esquina"));
  FSTRINGVALUE(MSG_MESH_EDITOR                      , _UxGT("Editor Mallado"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Editar Mallado"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Ed. Mallado parada"));
  FSTRINGVALUE(MSG_PROBING_MESH                     , _UxGT("Sondear Punto"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Índice X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Índice Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Valor Z"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Comandos Personaliz."));
  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("M48 Probar Sonda"));
  FSTRINGVALUE(MSG_M48_POINT                        , _UxGT("M48 Punto"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Desviación"));
  FSTRINGVALUE(MSG_IDEX_MENU                        , _UxGT("Modo IDEX"));
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Desfase Boquillas"));
  FSTRINGVALUE(MSG_IDEX_MODE_AUTOPARK               , _UxGT("Auto-Aparcado"));
  FSTRINGVALUE(MSG_IDEX_MODE_DUPLICATE              , _UxGT("Duplicar"));
  FSTRINGVALUE(MSG_IDEX_MODE_MIRRORED_COPY          , _UxGT("Copia Reflejada"));
  FSTRINGVALUE(MSG_IDEX_MODE_FULL_CTRL              , _UxGT("Control Total"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2ª Boquilla X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2ª Boquilla Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2ª Boquilla Z"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("Hacer G29"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("Herramientas UBL"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Nivel.Cama.Uni.(UBL)"));
  FSTRINGVALUE(MSG_LCD_TILTING_MESH                 , _UxGT("Punto de inclinación"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Crear Mallado man."));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Colocar cuña y Medir"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Medir"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Retirar y Medir Cama"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Mover al Siguente"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Activar UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Desactivar UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Temp. Cama"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Bed Temp"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT ("Hotend Temp"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Hotend Temp"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Editar Mallado"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Edit. Mallado perso."));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Ajuste fino Mallado"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Term. edici. Mallado"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Crear Mallado Perso."));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Crear Mallado"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Crear Mallado (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Crear Mallado (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Crear Mallado Frío"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Ajustar alt. Mallado"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Cantidad de altura"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Validar Mallado"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Validar Mallado (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Validar Mallado (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Vali. Mallado perso."));
  FSTRINGVALUE(MSG_G26_HEATING_BED                  , _UxGT("G26 Calentando Cama"));
  FSTRINGVALUE(MSG_G26_HEATING_NOZZLE               , _UxGT("G26 Calent. Boquilla"));
  FSTRINGVALUE(MSG_G26_MANUAL_PRIME                 , _UxGT("Imprimado manual..."));
  FSTRINGVALUE(MSG_G26_FIXED_LENGTH                 , _UxGT("Impri. longit. fija"));
  FSTRINGVALUE(MSG_G26_PRIME_DONE                   , _UxGT("Imprimación Lista"));
  FSTRINGVALUE(MSG_G26_CANCELED                     , _UxGT("G26 Cancelado"));
  FSTRINGVALUE(MSG_G26_LEAVING                      , _UxGT("Dejando G26"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Contin. Mallado cama"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Nivelando Mallado"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("Nivelando 3Puntos"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Nivel. Mallado cuad."));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Nivel de Mallado"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Puntos Laterales"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Tipo de mapa "));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Salida Mapa mallado"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Salida para el host"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Salida para CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Off Printer Backup"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Salida Info. UBL"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Cantidad de relleno"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Relleno manual"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Relleno inteligente"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Mallado de relleno"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Invalidar todo"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Invalidar proximos"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Ajustar Fino Todo"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Ajustar Fino proxi."));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Almacen de Mallado"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Huecos de memoria"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Cargar Mallado cama"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Guardar Mallado cama"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Mallado %i Cargado"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Mallado %i Guardado"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Sin guardar"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Error: Guardar UBL"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Error: Restaurar UBL"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET                     , _UxGT("Desfase de Z: "));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Desfase de Z Parado"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("UBL Paso a Paso"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Crear Mallado Frío"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Relleno inteligente"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Validar Mallado"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Ajustar Fino Todo"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Validar Mallado"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Ajustar Fino Todo"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Guardar Mallado cama"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Control LED"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Luzes"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Luz predefinida"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Rojo"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Naranja"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Amarillo"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Verde"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Azul"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Índigo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Violeta"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Blanco"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Por defecto"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Luces personalizadas"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Intensidad Rojo"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Intensidad Verde"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Intensidad Azul"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Intensidad Blanco"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Brillo"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Moviendo..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Libre XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Mover X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Mover Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Mover Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extrusor"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extrusor *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Hotend muy frio"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Mover %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Mover 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Mover 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Mover 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Velocidad"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Cama Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Boquilla"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Boquilla H"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Cama"));
  FSTRINGVALUE(MSG_CHAMBER                          , _UxGT("Recinto"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Ventilador"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Ventilador ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Vel. Ext. ventilador"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Vel. Ext. ventilador ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Flujo"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Flujo E"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Control"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fact"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Temperatura Auto."));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Encender"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Apagar"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Seleccionar"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Seleccionar *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Aceleración"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Velocidad"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("Vel. viaje min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Accel"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Acel. max") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Acel. max") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Acel. max") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Acel. max") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Acel. max *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("Acel. retrac."));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("Acel. Viaje"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Pasos/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" pasos/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" pasos/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" pasos/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E pasos/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* pasos/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Movimiento"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filamento"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E en mm³"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Fil. Dia."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Fil. Dia. *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Descarga mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Carga mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("Avance K"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("Avance K *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Contraste LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Guardar EEPROM"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Cargar EEPROM"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Rest. fábrica"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Inicializar EEPROM"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("Actualizar SD/USB"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Resetear Impresora"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Recargar"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Pantalla de Inf."));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Preparar"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Ajustar"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Iniciar impresión"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Siguinte"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Iniciar"));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Parar"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Imprimir"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Reiniciar"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Cancelar"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Listo"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausar impresión"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Reanudar impresión"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Detener impresión"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Recuper. por interr."));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Imprim. desde SD/USB"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("SD/USB no presente"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Reposo..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Pulsar para Reanudar"));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Impresión Pausada"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Imprimiendo..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Impresión cancelada"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Sin movimiento"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("MUERTA"));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("DETENIDA"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retraer mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Interc. Retraer mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retraer  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Levantar mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("DesRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Interc. DesRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("DesRet V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Retracción Auto."));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Inter. longitud"));
  FSTRINGVALUE(MSG_FILAMENT_PURGE_LENGTH            , _UxGT("Purgar longitud"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Cambiar Herramienta"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Aumentar Z"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Prime Speed"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Vel. de retracción"));
  FSTRINGVALUE(MSG_NOZZLE_STANDBY                   , _UxGT("Colocar boquilla"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Cambiar filamento"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Cambiar filamento *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Cargar filamento"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Cargar filamento *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Descargar filamento"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Descargar fil. *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Descargar todo"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Iniciar SD/USB"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Cambiar SD/USB"));
  FSTRINGVALUE(MSG_RELEASE_MEDIA                    , _UxGT("Lanzar SD/USB"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonda Z fuera cama"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Factor de desviación"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Cmd: Auto-Prueba"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Cmd: Reiniciar"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Cmd: Bajar pistón"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Cmd: Subir pistón"));
  FSTRINGVALUE(MSG_BLTOUCH_SW_MODE                  , _UxGT("Cmd: Modo Software"));
  FSTRINGVALUE(MSG_BLTOUCH_5V_MODE                  , _UxGT("Cmd: Modo 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_OD_MODE                  , _UxGT("Cmd: Modo OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE               , _UxGT("Cmd: Modo almacenar"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_5V            , _UxGT("Poner BLTouch a 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_OD            , _UxGT("Poner BLTouch a OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_ECHO                , _UxGT("Informe de drenaje"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_CHANGE              , _UxGT("PELIGRO: ¡Una mala configuración puede producir daños! ¿Proceder igualmente?"));
  FSTRINGVALUE(MSG_TOUCHMI_PROBE                    , _UxGT("TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_INIT                     , _UxGT("Iniciar TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_ZTEST                    , _UxGT("Test de desfase Z"));
  FSTRINGVALUE(MSG_TOUCHMI_SAVE                     , _UxGT("Guardar"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY_TOUCHMI            , _UxGT("Subir TouchMI"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Subir Sonda Z"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Bajar Sonda Z"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Origen %s%s%s Primero"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Desfase Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Micropaso X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Micropaso Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Micropaso Z"));
  FSTRINGVALUE(MSG_BABYSTEP_TOTAL                   , _UxGT("Total"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Cancelado - Endstop"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Calent. fallido"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_BED           , _UxGT("Calent. cama fallido"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_CHAMBER       , _UxGT("Calent. Cám. fallido"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Err: TEMP. REDUN."));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("FUGA TÉRMICA"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_BED              , _UxGT("FUGA TÉRMICA CAMA"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_CHAMBER          , _UxGT("FUGA TÉRMICA CAMARA"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err:TEMP. MÁX"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err:TEMP. MIN"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Err:TEMP. MÁX CAMA"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Err:TEMP. MIN CAMA"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_CHAMBER              , _UxGT("Err:TEMP. MÁX CÁMARA"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_CHAMBER              , _UxGT("Err:TEMP. MIN CÁMARA"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Origen XY Primero"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("IMPRESORA DETENIDA"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Por favor, reinicie"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Calentando..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Enfriando..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Calentando Cama..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Enfriando Cama..."));
  FSTRINGVALUE(MSG_CHAMBER_HEATING                  , _UxGT("Calentando Cámara..."));
  FSTRINGVALUE(MSG_CHAMBER_COOLING                  , _UxGT("Enfriando Cámara..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibración Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibrar X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibrar Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibrar Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibrar Centro"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Configuración Delta"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto Calibración"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Est. Altura Delta"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Ajustar Sonda Z"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Barra Diagonal"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Altura"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Radio"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Acerca de Impresora"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Info. Impresora"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("Nivelando 3puntos"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Nivelando Lineal"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Nivelando Bilineal"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Nivelando UBL"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Nivelando en Mallado"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Estadísticas Imp."));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Info. Controlador"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistores"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extrusores"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baudios"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocolo"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Luz cabina"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Brillo cabina"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Impresora incorrecta"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Cont. de impresión"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completadas"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tiempo total de imp."));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Impresión más larga"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Total Extruido"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Impresiones"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Completadas"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Más larga"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extruido"));
  #endif
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Temp. Mínima"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Temp. Máxima"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Fuente alimentación"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Fuerza de empuje"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driver %"));
  FSTRINGVALUE(MSG_ERROR_TMC                        , _UxGT("ERROR CONEX. TMC"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Escribe DAC EEPROM"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER           , _UxGT("CAMBIAR FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("IMPRESIÓN PAUSADA"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("CARGAR FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("DESCARGAR FILAMENTO"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("OPC. REINICIO:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Purgar más"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Continuar imp."));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Boquilla: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Sensor de sección"));
  FSTRINGVALUE(MSG_RUNOUT_DISTANCE_MM               , _UxGT("Dist de secc. mm"));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Ir a origen Fallado"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Sondeo Fallado"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: Muy Frio"));

  FSTRINGVALUE(MSG_MMU2_CHOOSE_FILAMENT_HEADER      , _UxGT("ELIJE FILAMENTO"));
  FSTRINGVALUE(MSG_MMU2_MENU                        , _UxGT("MMU"));
  FSTRINGVALUE(MSG_MMU2_WRONG_FIRMWARE              , _UxGT("¡Actu. MMU Firmware!"));
  FSTRINGVALUE(MSG_MMU2_NOT_RESPONDING              , _UxGT("MMU Necesita Cuidado"));
  FSTRINGVALUE(MSG_MMU2_RESUME                      , _UxGT("Continuar imp."));
  FSTRINGVALUE(MSG_MMU2_RESUMING                    , _UxGT("Resumiendo..."));
  FSTRINGVALUE(MSG_MMU2_LOAD_FILAMENT               , _UxGT("Cargar Filamento"));
  FSTRINGVALUE(MSG_MMU2_LOAD_ALL                    , _UxGT("Cargar Todo"));
  FSTRINGVALUE(MSG_MMU2_LOAD_TO_NOZZLE              , _UxGT("Cargar hasta boqui."));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT              , _UxGT("Expulsar Filamento"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT_N            , _UxGT("Expulsar Filamento E"));
  FSTRINGVALUE(MSG_MMU2_UNLOAD_FILAMENT             , _UxGT("Descargar Filamento"));
  FSTRINGVALUE(MSG_MMU2_LOADING_FILAMENT            , _UxGT("Cargando Fil. %i..."));
  FSTRINGVALUE(MSG_MMU2_EJECTING_FILAMENT           , _UxGT("Expulsando Fil. ..."));
  FSTRINGVALUE(MSG_MMU2_UNLOADING_FILAMENT          , _UxGT("Descargando Fil...."));
  FSTRINGVALUE(MSG_MMU2_ALL                         , _UxGT("Todo"));
  FSTRINGVALUE(MSG_MMU2_FILAMENT_N                  , _UxGT("Filamento E"));
  FSTRINGVALUE(MSG_MMU2_RESET                       , _UxGT("Reiniciar MMU"));
  FSTRINGVALUE(MSG_MMU2_RESETTING                   , _UxGT("Reiniciando MMU..."));
  FSTRINGVALUE(MSG_MMU2_EJECT_RECOVER               , _UxGT("Retirar, y pulsar"));

  FSTRINGVALUE(MSG_MIX                              , _UxGT("Mezcla"));
  FSTRINGVALUE(MSG_MIX_COMPONENT_N                  , _UxGT("Componente ="));
  FSTRINGVALUE(MSG_MIXER                            , _UxGT("Miezclador"));
  FSTRINGVALUE(MSG_GRADIENT                         , _UxGT("Degradado"));
  FSTRINGVALUE(MSG_FULL_GRADIENT                    , _UxGT("Degradado Total"));
  FSTRINGVALUE(MSG_TOGGLE_MIX                       , _UxGT("Mezcla Conmutada"));
  FSTRINGVALUE(MSG_CYCLE_MIX                        , _UxGT("Mezcla Cíclica"));
  FSTRINGVALUE(MSG_GRADIENT_MIX                     , _UxGT("Mezcla de Degradado"));
  FSTRINGVALUE(MSG_REVERSE_GRADIENT                 , _UxGT("Degradado inverso"));
  FSTRINGVALUE(MSG_ACTIVE_VTOOL                     , _UxGT("Activar Herr.V"));
  FSTRINGVALUE(MSG_START_VTOOL                      , _UxGT("Inicio Herr.V"));
  FSTRINGVALUE(MSG_END_VTOOL                        , _UxGT("   Fin Herr.V"));
  FSTRINGVALUE(MSG_GRADIENT_ALIAS                   , _UxGT("Alias Herr.V"));
  FSTRINGVALUE(MSG_RESET_VTOOLS                     , _UxGT("Reiniciar Herr.V"));
  FSTRINGVALUE(MSG_COMMIT_VTOOL                     , _UxGT("Cometer mezc. Herr.V"));
  FSTRINGVALUE(MSG_VTOOLS_RESET                     , _UxGT("Herr.V reiniciados"));
  FSTRINGVALUE(MSG_START_Z                          , _UxGT("Inicio Z:"));
  FSTRINGVALUE(MSG_END_Z                            , _UxGT("   Fin Z:"));

  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Games"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Brickout"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Invaders"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Sn4k3"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Maze"));

  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Pulsar el botón para", "reanudar impresión")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Aparcando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Esperando para", "iniciar el cambio", "de filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Inserte el filamento", "y pulse el botón", "para continuar...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Pulse el botón para", "calentar la boquilla")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Calentando boquilla", "Espere por favor...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Espere para", "liberar el filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_2_LINE("Espere para", "purgar el filamento")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_2_LINE("Pulse para finalizar", "la purga de filamen.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Esperando impresora", "para reanudar...")));
  #else
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Pulse para continuar")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Aparcando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Por Favor espere...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Inserte y Pulse")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Pulse para Calentar")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Calentando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Liberando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Cargando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Purgando...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Pulse para finalizar")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Reanudando...")));
  #endif

  FSTRINGVALUE(MSG_TMC_DRIVERS                      , _UxGT("Controladores TMC"));
  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Amperaje Controlador"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Límite Hibrido"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Origen sin sensores"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Modo de pasos"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("StealthChop Habilit."));

  FSTRINGVALUE(MSG_SERVICE_RESET                    , _UxGT("Reiniciar"));
  FSTRINGVALUE(MSG_SERVICE_IN                       , _UxGT(" dentro:"));

  FSTRINGVALUE(MSG_BACKLASH_CORRECTION              , _UxGT("Correction"));
  FSTRINGVALUE(MSG_BACKLASH_SMOOTHING               , _UxGT("Suavizado"));
}
