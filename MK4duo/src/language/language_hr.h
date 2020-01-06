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
 * Croatian (Hrvatski)
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1 // use the better font on full graphic displays.

namespace language_hr {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Croatian"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" spreman."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("SD kartica umetnuta"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("SD kartica uklonjena"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstops")); // Max length 8 characters
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Main"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Auto pokretanje"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Ugasi steppere"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Automatski homing"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Home-aj X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Home-aj Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Home-aj Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Home-aj XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Klikni za početak"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Sljedeća točka"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Niveliranje gotovo!"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Postavi home offsete"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Offsets postavljeni"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Postavi ishodište"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Predgrij ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Predgrij ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Predgrij ") PREHEAT_1_LABEL _UxGT(" Dizna"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Predgrij ") PREHEAT_1_LABEL _UxGT(" Dizna ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Predgrij ") PREHEAT_1_LABEL _UxGT(" Sve"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Predgrij ") PREHEAT_1_LABEL _UxGT(" Bed"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Predgrij ") PREHEAT_1_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Predgrij ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Predgrij ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Predgrij ") PREHEAT_2_LABEL _UxGT(" Dizna"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Predgrij ") PREHEAT_2_LABEL _UxGT(" Dizna ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Predgrij ") PREHEAT_2_LABEL _UxGT(" Sve"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Predgrij ") PREHEAT_2_LABEL _UxGT(" Bed"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Predgrij ") PREHEAT_2_LABEL _UxGT(" conf"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Hlađenje"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Uključi napajanje"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Isključi napajanje"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Miči os"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Niveliraj bed"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Niveliraj bed"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Miči X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Miči Y"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Miči %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Miči 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Miči 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Miči 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Brzina"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Bed Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Dizna"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Dizna ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Bed"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Brzina ventilatora"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Brzina ventilatora ~"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Odaberi"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Odaberi *"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperature"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Gibanje"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Fil. Dia."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Fil. Dia. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Kontrast LCD-a"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Pohrani u memoriju"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Učitaj memoriju"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Učitaj failsafe"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Osvježi"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Info screen"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Pripremi"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pauziraj print"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Nastavi print"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Zaustavi print"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Printaj s SD kartice"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Nema SD kartice"));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Čekaj korisnika..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Print otkazan"));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ZAUSTAVLJEN. "));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Promijeni filament"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Promijeni filament *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Init. SD karticu"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Promijeni SD karticu"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Grijanje neuspješno"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Grijanje..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Grijanje Bed-a..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta Kalibracija"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibriraj X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibriraj Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibriraj Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibriraj Središte"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("O printeru"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Podaci o printeru"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Statistika printera"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Podaci o elektronici"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistori"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extruderi"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baud"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protokol"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Osvjetljenje"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Neispravan pisač"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Broj printova"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Završeni"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Ukupno printanja"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Najduži print"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extrudirano ukupno"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Printovi"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Završeni"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Ukupno"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Najduži"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extrudirano"));
  #endif

  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Napajanje"));

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Nastavi print"));

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD           , _UxGT(MSG_2_LINE("Čekaj", "filament unload")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD             , _UxGT(MSG_2_LINE("Pričekaj", "filament load")));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME           , _UxGT(MSG_1_LINE("Nastavljam...")));

  #if LCD_HEIGHT >= 4
    // Up to 3 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Čekaj početak", "filamenta", "promijeni")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Umetni filament", "i pritisni tipku", "za nastavak...")));
  #else
    // Up to 2 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT("Pričekaj..."));
    //FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT       , _UxGT(MSG_2_LINE("?", "?")));
  #endif
}
