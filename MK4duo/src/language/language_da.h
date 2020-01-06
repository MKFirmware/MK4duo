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
 * Danish
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace language_da {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Danish"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" er klar"));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Kort isat"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Kort fjernet"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Slå alle steppere fra"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Auto Home")); // G28
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Klik når du er klar"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Næste punkt"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Bed level er færdig!"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Sæt forsk. af home"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Forsk. er nu aktiv"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Sæt origin"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Forvarm ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Forvarm ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Forvarm ") PREHEAT_1_LABEL _UxGT(" end")
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Forvarm ") PREHEAT_1_LABEL _UxGT(" end ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Forvarm ") PREHEAT_1_LABEL _UxGT(" Alle"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Forvarm ") PREHEAT_1_LABEL _UxGT(" Bed"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Forvarm ") PREHEAT_1_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Forvarm ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Forvarm ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Forvarm ") PREHEAT_2_LABEL _UxGT(" end")
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Forvarm ") PREHEAT_2_LABEL _UxGT(" end ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Forvarm ") PREHEAT_2_LABEL _UxGT(" Alle"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Forvarm ") PREHEAT_2_LABEL _UxGT(" Bed"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Forvarm ") PREHEAT_2_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Afkøl"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Slå strøm til"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Slå strøm fra"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extruder"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Flyt akser"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Juster bed"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Juster bed"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Flyt X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Flyt Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Flyt Z"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Flyt %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Flyt 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Flyt 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Flyt 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Hastighed"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Plade Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Dyse"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Dyse ~"));

  FSTRINGVALUE(MSG_BED                              , _UxGT("Plade"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Blæser hastighed"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Blæser hastighed ="));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Kontrol"));
  FSTRINGVALUE(MSG_MIN                              , _UxGT(" \002 Min"));
  FSTRINGVALUE(MSG_MAX                              , _UxGT(" \002 Max"));
  FSTRINGVALUE(MSG_FACTOR                           , _UxGT(" \002 Fact"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Autotemp"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Til"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Fra"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Vælg"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Vælg *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-retract"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-rejse"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatur"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Bevægelse"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E i mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Fil. Dia."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Fil. Dia. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD kontrast"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Gem i EEPROM"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Hent fra EEPROM"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Gendan failsafe"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Genopfrisk"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Info skærm"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Forbered"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pause printet"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Forsæt printet"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Stop printet"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Print fra SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Intet SD kort"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Dvale..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Venter på bruger..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Print annulleret"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Ingen bevægelse."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("DRÆBT. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("STOPPET. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Tilbagetræk mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Skift Re.mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Tilbagetræk V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Hop mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Skift UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet  V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("AutoRetr."));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Skift filament"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Skift filament *"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Skift SD kort"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Probe udenfor plade"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch Selv-Test"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Home %s%s%s først"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Opvarmning fejlet"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Fejl: reserve temp"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("Temp løber løbsk"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Fejl: Maks temp"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Fejl: Min temp"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Fejl: Maks Plade temp"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Fejl: Min Plade temp"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Home XY først"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("PRINTER STOPPET"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Reset Venligst"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d")); // Kun et bogstav
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h")); // Kun et bogstav
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); // Kun et bogstav
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Opvarmer..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Opvarmer plade..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta Kalibrering"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibrer X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibrer Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibrer Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibrerings Center"));

  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Om Printer"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Kort Info"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Thermistors"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Ant. Prints"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Færdige"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total print tid"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Længste print"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Total Extruderet"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Prints"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Færdige"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Længste"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extruderet"));
  #endif

  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Strømfors."));

  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Driv Styrke"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driv %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("DAC EEPROM Skriv"));

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Forsæt print"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Forkert printer"));

  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Vent på start", "af filament", "skift")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Vent på", "filament udskyd.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Indsæt filament", "og tryk på knap", "for at fortsætte...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Vent på", "filament indtag")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Vent på at print", "fortsætter")));
  #else // LCD_HEIGHT < 4
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Vent venligst...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Udskyder...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Indsæt og klik")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Indtager...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Fortsætter...")));
  #endif // LCD_HEIGHT < 4
}
