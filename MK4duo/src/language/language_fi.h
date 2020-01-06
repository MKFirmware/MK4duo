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
 * Finnish
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace language_fi {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Finnish"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" valmis."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Kortti asetettu"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Kortti poistettu"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Palaa"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Automaatti"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Vapauta moottorit"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Aja referenssiin"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Aseta origo"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Esilämmitä ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Esilämmitä ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT("Suutin"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT("Suutin ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT(" Kaikki"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Esilä. ") PREHEAT_1_LABEL _UxGT(" Alusta"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Esilämm. ") PREHEAT_1_LABEL _UxGT(" konf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Esilämmitä ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Esilämmitä ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Esilä. ") PREHEAT_2_LABEL _UxGT("Suutin"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Esilä. ") PREHEAT_2_LABEL _UxGT("Suutin ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Esilä. ") PREHEAT_2_LABEL _UxGT(" Kaikki"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Esilä. ") PREHEAT_2_LABEL _UxGT(" Alusta"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Esilämm. ") PREHEAT_2_LABEL _UxGT(" konf"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Jäähdytä"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Virta päälle"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Virta pois"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Pursota"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Vedä takaisin"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Liikuta akseleita"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Liikuta X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Liikuta Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Liikuta Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extruder"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extruder *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Liikuta %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Liikuta 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Liikuta 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Liikuta 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Nopeus"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Suutin"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Suutin ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Alusta"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Tuul. nopeus"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Tuul. nopeus ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Virtaus"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Virtaus ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Kontrolli"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Kerr"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Autotemp"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Kiihtyv"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("VLiike min"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-peruuta"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Lämpötila"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Liike"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm³"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD kontrasti"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Tallenna muistiin"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Lataa muistista"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Palauta oletus"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Päivitä"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Seuraa"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Valmistele"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Säädä"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Keskeytä tulostus"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Jatka tulostusta"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Pysäytä tulostus"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Korttivalikko"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Ei korttia"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Nukkumassa..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Odotet. valintaa"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Ei liiketta."));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Vedä mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Va. Vedä mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Vedä V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Z mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Va. UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet  V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("AutoVeto."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta Kalibrointi"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibroi X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibroi Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibroi Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibroi Center"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Väärä tulostin"));
}
