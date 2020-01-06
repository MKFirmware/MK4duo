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
 * Basque-Euskera
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

namespace language_eu {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 1;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Basque-Euskera"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" prest."));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Atzera"));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Txartela sartuta"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Txartela kenduta"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu nagusia"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Auto hasiera"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Itzali motoreak"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Arazketa Menua"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Prog. Barra Proba"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Hasierara joan"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("X jatorrira"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Y jatorrira"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Z jatorrira"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("XYZ hasieraratzen"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Klik egin hasteko"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Hurrengo Puntua"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Berdintzea eginda"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Etxe. offset eza."));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Offsetak ezarrita"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Hasiera ipini"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Berotu ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Berotu ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Amaia"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Amaia ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Guztia"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Ohea"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Berotu ") PREHEAT_1_LABEL _UxGT(" Ezarp."));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Berotu ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Berotu ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Berotu ") PREHEAT_2_LABEL _UxGT(" Amaia"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Berotu ") PREHEAT_2_LABEL _UxGT(" Amaia ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Berotu ") PREHEAT_2_LABEL _UxGT(" Guztia"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Berotu ") PREHEAT_2_LABEL _UxGT(" Ohea"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Berotu ") PREHEAT_2_LABEL _UxGT(" Ezarp."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Hoztu"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Energia piztu"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Energia itzali"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Estruitu"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Atzera eragin"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Ardatzak mugitu"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Ohe berdinketa"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Ohea berdindu"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Ertzak berdindu"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Hurrengo ertza"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Sarea editatu"));

  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("G29 exekutatzen"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("UBL Tresnak"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Sarea eskuz sortu"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Neurtu"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("UBL aktibatu"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("UBL desaktibatu"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Ohearen tenperatura"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Bed Temp"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Mutur beroaren tenp."));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Hotend Temp"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Sarea editatu"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Sarea editatzea eginda"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Sarea sortu"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , PREHEAT_1_LABEL _UxGT(" sarea sortu"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , PREHEAT_2_LABEL _UxGT(" sarea sortu"));
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Sare hotza sortu"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Sarearen altuera doitu"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Sarea balioetsi"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , PREHEAT_1_LABEL _UxGT(" sarea balioetsi"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , PREHEAT_2_LABEL _UxGT(" sarea balioetsi"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Ohe sarea balioetsi"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Sare berdinketa"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("3 puntuko berdinketa"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Lauki-sare berdinketa"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Sarea berdindu"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Mapa mota"));
  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("LED ezarpenak"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Argiak"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Argi aurrehautaketak"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Gorria"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Laranja"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Horia"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Berdea"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Urdina"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Indigo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Bioleta"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Zuria"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Lehenetsia"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Argi pertsonalizatuak"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Intentsitate gorria"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Intentsitate berdea"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Intentsitate urdina"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Intentsitate zuria"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Distira"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Mugitzen..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Askatu XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Mugitu X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Mugitu Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Mugitu Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Estrusorea"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Estrusorea *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Mugitu %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Mugitu 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Mugitu 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Mugitu 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Abiadura"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Z Ohea"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Pita"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Pita ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Ohea"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Haizagailu abiadura"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Haizagailu abiadura ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Haiz.gehig. abiadura"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Haiz.gehig. abiadura ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Fluxua"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Fluxua ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Kontrola"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fakt"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Auto tenperatura"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Aukeratu"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Aukeratu *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Azelerazioa"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Astindua"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("V") LCD_STR_A _UxGT("-astindua"));
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("V") LCD_STR_B _UxGT("-astindua"));
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("V") LCD_STR_C _UxGT("-astindua"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Ve-astindua"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("VBidaia min"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-retrakt"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-bidaia"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Pausoak/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" pausoak/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" pausoak/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" pausoak/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E pausoak/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* pausoak/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Tenperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Mugimendua"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Harizpia"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E mm3-tan"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Hariz. Dia."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Hariz. Dia. *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Deskargatu mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Kargatu mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("K Aurrerapena"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("K Aurrerapena *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD kontrastea"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Gorde memoria"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Kargatu memoria"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Larri. berriz."));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("EEPROM-a hasieratu"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Berriz kargatu"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Pantaila info"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Prestatu"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Doitu"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pausatu inprimak."));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Jarraitu inprima."));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Gelditu inprima."));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("SD-tik inprimatu"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Ez dago SD-rik"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Lo egin..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Aginduak zain..."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Inprim. geldi."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Inprim. deusezta."));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Mugimendu gabe."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("AKABATUTA. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("GELDITUTA. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Atzera egin mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Swap Atzera mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Atzera egin V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Igo mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("Atzera egin mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Swap Atzera mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("Atzera egin V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Atzera egin"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Aldatu harizpia"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Aldatu harizpia *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Harizpia kargatu"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Harizpia kargatu *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Harizpia deskargatu"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Harizpia deskargatu *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Erabat deskargatu"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Hasieratu SD-a"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Aldatu txartela"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z zunda kanpora"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Okertze faktorea"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch AutoProba"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("BLTouch berrabia."));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("BLTouch jaitsi/luzatu"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("BLTouch igo/jaso"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Etxera %s%s%s lehenengo"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Z Konpentsatu"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Mikro-urratsa X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Mikro-urratsa Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Mikro-urratsa Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Endstop deusezta."));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Err: Beroketa"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Err: Tenperatura"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("TENP. KONTROL EZA"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err: Tenp Maximoa"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err: Tenp Minimoa"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Err: Ohe Tenp Max"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Err: Ohe Tenp Min"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Etxera XY lehenengo"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("INPRIMA. GELDIRIK"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Berrabia. Mesedez"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Berotzen..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Ohea Berotzen..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta Kalibraketa"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibratu X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibratu Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibratu Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibratu Zentrua"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Delta ezarpenak"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto Kalibraketa"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Delta Alt. Ezar."));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Barra diagonala"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Altuera"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Erradioa"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Inprimagailu Inf."));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Inprimagailu Inf."));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("3 puntuko berdinketa"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Berdinketa lineala"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Berdinketa bilinearra"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Sare berdinketa"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Inprima. estatis."));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Txartelaren Info."));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistoreak"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Estrusoreak"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baudioak"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protokoloa"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Kabina Argia"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS
   , );
  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Inprim. Zenbaketa"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Burututa"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Inprim. denbora"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Imprimatze luzeena"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Estruituta guztira"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Inprimatze"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Burututa"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Guztira"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Luzeena"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Estrusio"));
  #endif
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Tenp. Minimoa"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Tenp. Maximoa"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Elikadura-iturria"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Driver-aren potentzia"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driver %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Idatzi DAC EEPROM"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("HARIZPIA ALDATU"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("HARIZPIA KARGATU"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("HARIZPIA DESKARGATU"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("ALDAKETA AUKERAK:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Inprima. jarraitu"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Pita: "));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Hasi. huts egin du"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Neurketak huts egin du"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: hotzegi"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Inprimagailu okerra"));

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT             , _UxGT(MSG_1_LINE("Mesedez, itxaron...")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD           , _UxGT(MSG_1_LINE("Deskargatzen...")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT           , _UxGT(MSG_1_LINE("Sartu eta click egin")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING          , _UxGT(MSG_1_LINE("Berotzen...")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD             , _UxGT(MSG_1_LINE("Kargatzen...")));
}
