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
 * Polish for DOGM display - includes accented characters
 */

#define DISPLAY_CHARSET_ISO10646_PL

namespace language_pl {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Polish"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" gotowy."));
  FSTRINGVALUE(MSG_YES                              , _UxGT("TAK"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("NIE"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Wstecz"));
  FSTRINGVALUE(MSG_MEDIA_ABORTING                   , _UxGT("Przerywanie..."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Karta wlozona"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Karta usunieta"));
  FSTRINGVALUE(MSG_MEDIA_RELEASED                   , _UxGT("Karta zwolniona"));
  FSTRINGVALUE(MSG_MEDIA_WAITING                    , _UxGT("Oczekiwanie na karte"));
  FSTRINGVALUE(MSG_MEDIA_READ_ERROR                 , _UxGT("Blad odczytu karty"));
  FSTRINGVALUE(MSG_MEDIA_USB_REMOVED                , _UxGT("Urzadzenie USB usuniete"));
  FSTRINGVALUE(MSG_MEDIA_USB_FAILED                 , _UxGT("Blad uruchomienia USB"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Kranców.")); // Max length 8 characters
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Progr. Krancówki"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu glówne"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Ustawienie zaawansowane"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Konfiguracja"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Autostart"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Wylacz silniki"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Menu Debugowania"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Testowy pasek postepu"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Pozycja zerowa"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Zeruj X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Zeruj Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Zeruj Z"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Autowyrównanie Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Pozycja zerowa"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Kliknij by rozp."));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Nastepny punkt"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Wypoziomowano!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Wys. zanikania"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Ust. poz. zer."));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Poz. zerowa ust."));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Ustaw punkt zero"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Rozgrzej ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Rozgrzej ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" Dysza"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" Dysza ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" wsz."));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" stól"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Rozgrzej ") PREHEAT_1_LABEL _UxGT(" ustaw."));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Rozgrzej ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Rozgrzej ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Rozgrzej ") PREHEAT_2_LABEL _UxGT(" Dysza"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Rozgrzej ") PREHEAT_2_LABEL _UxGT(" Dysza ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Rozgrzej ") PREHEAT_2_LABEL _UxGT(" wsz."));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Rozgrzej ") PREHEAT_2_LABEL _UxGT(" stól"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Rozgrzej ") PREHEAT_2_LABEL _UxGT(" ustaw."));
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Rozgrzej wlasne ust."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Chlodzenie"));
  FSTRINGVALUE(MSG_LASER_MENU                       , _UxGT("Sterowanie Lasera"));
  FSTRINGVALUE(MSG_LASER_OFF                        , _UxGT("Wylacz Laser"));
  FSTRINGVALUE(MSG_LASER_ON                         , _UxGT("Wlacz Laser"));
  FSTRINGVALUE(MSG_LASER_POWER                      , _UxGT("Zasilanie Lasera"));
  FSTRINGVALUE(MSG_SPINDLE_MENU                     , _UxGT("Sterowanie wrzeciona"));
  FSTRINGVALUE(MSG_SPINDLE_OFF                      , _UxGT("Wylacz wrzeciono"));
  FSTRINGVALUE(MSG_SPINDLE_ON                       , _UxGT("Wlacz wrzeciono"));
  FSTRINGVALUE(MSG_SPINDLE_POWER                    , _UxGT("Zasilanie wrzeciona"));
  FSTRINGVALUE(MSG_SPINDLE_REVERSE                  , _UxGT("Rewers wrzeciona"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Wlacz zasilacz"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Wylacz zasilacz"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Ekstruzja"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Wycofanie"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Ruch osi"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Poziomowanie stolu"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Wypoziomuj stól"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Narozniki poziomowania"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Nastepny naroznik"));
  FSTRINGVALUE(MSG_MESH_EDITOR                      , _UxGT("Edytor siatki"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Edycja siatki"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Edycja siatki zatrzymana"));
  FSTRINGVALUE(MSG_PROBING_MESH                     , _UxGT("Punkt pomiarowy"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Indeks X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Indeks Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Wartosc Z"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Wlasne Polecenia"));
  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("M48 Test sondy"));
  FSTRINGVALUE(MSG_M48_POINT                        , _UxGT("M48 Punky"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Odchylenie"));
  FSTRINGVALUE(MSG_IDEX_MENU                        , _UxGT("Tryb IDEX"));
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Przesuniecie narzedzia"));
  FSTRINGVALUE(MSG_IDEX_MODE_AUTOPARK               , _UxGT("Auto-Parkowanie"));
  FSTRINGVALUE(MSG_IDEX_MODE_DUPLICATE              , _UxGT("Duplikowanie"));
  FSTRINGVALUE(MSG_IDEX_MODE_MIRRORED_COPY          , _UxGT("Kopia lustrzana"));
  FSTRINGVALUE(MSG_IDEX_MODE_FULL_CTRL              , _UxGT("Pelne sterowanie"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2ga dysza X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2ga dysza Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2ga dysza Z"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("Wykonywanie G29"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("Narzedzia UBL"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_LCD_TILTING_MESH                 , _UxGT("Punkt pochylenia"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Reczne Budowanie Siatki"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Umiesc podkladke i zmierz"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Zmierz"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Usun & Zmierz Stól"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Przesuwanie do nastepnego"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Aktywacja UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Dezaktywacja UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Temperatura stolu"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Temperatura stolu"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Temperatura dyszy"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Temperatura dyszy"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Edycja siatki"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Edycja wlasnej siatki"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Dostrajanie siatki"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Koniec edycji siati"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Buduj wlasna siatke"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Buduj siatke"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Buduj siatke (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Buduj siatke (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Buduj siatke na zimno"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Dostrojenie wysokosci siatki"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Wartosc wysokosci"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Sprawdzenie siatki"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Sprawdzenie siatki (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Sprawdzenie siatki (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Sprawdzenie wlasnej siatki"));
  FSTRINGVALUE(MSG_G26_HEATING_BED                  , _UxGT("G26 Nagrzewanie stolu"));
  FSTRINGVALUE(MSG_G26_HEATING_NOZZLE               , _UxGT("G26 Nagrzewanie dyszy"));
  FSTRINGVALUE(MSG_G26_MANUAL_PRIME                 , _UxGT("Napelnianie reczne..."));
  FSTRINGVALUE(MSG_G26_FIXED_LENGTH                 , _UxGT("Napelnij kreslona dlugoscia"));
  FSTRINGVALUE(MSG_G26_PRIME_DONE                   , _UxGT("Napelianie zakonczone"));
  FSTRINGVALUE(MSG_G26_CANCELED                     , _UxGT("G26 Przewane"));
  FSTRINGVALUE(MSG_G26_LEAVING                      , _UxGT("Opuszczanie G26"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Kontynuuj tworzenie siatki"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Poziomowanie siatka"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("Poziomowaie 3-punktowe"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Poziomowaie wedlug siatki"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Poziomuj siatke"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Punkty boczne"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Rodzaj mapy"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Wyslij mape siatki"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Wyslij do Hosta"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Wyslij do CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Kopia poza drukarka"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Wyslij info UBL"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Stopien wypelnienia"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Reczne wypelnienie"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Inteligentne wypelnienie"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Wypelnienie siatki"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Uniewaznij wszystko"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Uniewaznij najblizszy"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Dostrajaj wszystko"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Dostrajaj najblizszy"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Przechowywanie siatki"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Slot Pamieci"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Zaladuj siatke stolu"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Zapisz siatke stolu"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Siatka %i zaladowana"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Siatka %i Zapisana"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Brak magazynu"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Blad: Zapis UBL"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Blad: Odczyt UBL"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET                     , _UxGT("Przesuniecie Z: "));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Przesuniecie Z zatrzymane"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("UBL Krok po kroku"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1. Tworzeni ezimnej siatki"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2. Inteligentne wypelnienie"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3. Sprawdzenie siatki"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4. Dostrojenie wszystkiego"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5. Sprawdzenie siatki"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6. Dostrojenie wszystkiego"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7. Zapis siatki stolu"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Sterowanie LED"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Swiatla"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Ustawienia swiatel"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Czerwony"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Pomaranczowy"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Zólty"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Zielony"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Niebieski"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Indygo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Fioletowy"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Bialy"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Domyslny"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Wlasne swiatla"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Czerwony"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Zielony"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Niebieski"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Bialy"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Jasnosc"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Ruch..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Swobodne XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Przesun w X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Przesun w Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Przesun w Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Ekstruzja (os E)"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Ekstruzja (os E) *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Dysza za zimna"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Przesun co %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Przesun co .1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Przesun co 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Przesun co 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Predkosc"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Stól Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Dysza"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Dysza ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Stól"));
  FSTRINGVALUE(MSG_CHAMBER                          , _UxGT("Obudowa"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Obroty wiatraka"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Obroty wiatraka ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Obroty dodatkowego wiatraka"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Obroty dodatkowego wiatraka ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Przeplyw"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Przeplyw ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Ustawienia"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Mnoznik"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Auto. temperatura"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Wl."));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Wyl."));
  FSTRINGVALUE(MSG_PID_AUTOTUNE                     , _UxGT("PID Autostrojenie"));
  FSTRINGVALUE(MSG_PID_AUTOTUNE_E                   , _UxGT("PID Autostrojenie *"));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID-P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID-I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID-D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID-C *"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Wybierz"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Wybierz *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Przyspieszenie"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Zryw"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("Zryw V") LCD_STR_A);
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("Zryw V") LCD_STR_B);
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("Zryw V") LCD_STR_C);
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Zryw Ve"));
  FSTRINGVALUE(MSG_JUNCTION_DEVIATION               , _UxGT("Junction Dev"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Predkosc (V)"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("Vmax ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("Vmax ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("Vmax ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vmax ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("Vmax *"));
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("Vmin"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("Vskok min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Przyspieszenie (A)"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Amax ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Amax ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Amax ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Amax ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Amax *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-wycofanie"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-przesun."));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("kroki/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , _UxGT("kroki") LCD_STR_A _UxGT("/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , _UxGT("kroki") LCD_STR_B _UxGT("/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , _UxGT("kroki") LCD_STR_C _UxGT("/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("krokiE/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("kroki */mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatura"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Ruch"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E w mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Sr. fil."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Sr. fil. *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Wyladuj mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Zaladuj mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("Advance K"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("Advance K *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Kontrast LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Zapisz w pamieci"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Wczytaj z pamieci"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Ustaw. fabryczne"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Initializuj EEPROM"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("Uaktualnij karte"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Resetuj drukarke"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Odswiez"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Ekran glówny"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Przygotuj"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Strojenie"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Start wydruku"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Nastepny"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Inic."));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Stop"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Drukuj"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Resetuj"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Przerwij"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Gotowe"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Wstrzymaj druk"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Wznowienie"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Stop"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Odzyskiwanie po awarii"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Karta SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Brak karty"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Uspij..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Oczekiwanie..."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Druk wstrzymany"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Drukowanie..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Druk przerwany"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Brak ruchu"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("Ubity. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("Zatrzymany. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Wycofaj mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Z Wycof. mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Wycofaj  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Skok Z mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("Cof. wycof. mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Z Cof. wyc. mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("Cof. wycof.  V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Auto. wycofanie"));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Dlugosc zmiany"));
  FSTRINGVALUE(MSG_FILAMENT_PURGE_LENGTH            , _UxGT("Dlugosc oczyszczania"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Zmiana narzedzia"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Podniesienie Z"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Predkosc napelniania"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Predkosc wycofania"));
  FSTRINGVALUE(MSG_NOZZLE_STANDBY                   , _UxGT("Dysza w oczekiwaniu"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Zmien filament"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Zmien filament *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Zaladuj Filament"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Zaladuj Filament *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Wyladuj Filament"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Wyladuj Filament *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Wyladuj wszystkie"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Inicjal. karty SD"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Zmiana karty SD"));
  FSTRINGVALUE(MSG_RELEASE_MEDIA                    , _UxGT("Zwolnienie karty"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonda Z za stolem"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Wspólczynik skrzywienia"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch Self-Test"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Reset BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Cmd: Stow"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Cmd: Deploy"));
  FSTRINGVALUE(MSG_BLTOUCH_SW_MODE                  , _UxGT("Cmd: SW-Mode"));
  FSTRINGVALUE(MSG_BLTOUCH_5V_MODE                  , _UxGT("Cmd: 5V-Mode"));
  FSTRINGVALUE(MSG_BLTOUCH_OD_MODE                  , _UxGT("Cmd: OD-Mode"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE               , _UxGT("Cmd: Mode-Store"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_5V            , _UxGT("Set BLTouch to 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_OD            , _UxGT("Set BLTouch to OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_ECHO                , _UxGT("Report Drain"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_CHANGE              , _UxGT("NIEBEZPIECZNE: Zle ustawienia moga uszkodzic drukarke. Kontynuowac?"));
  FSTRINGVALUE(MSG_TOUCHMI_PROBE                    , _UxGT("TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_INIT                     , _UxGT("Init TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_ZTEST                    , _UxGT("Z Offset Test"));
  FSTRINGVALUE(MSG_TOUCHMI_SAVE                     , _UxGT("Save"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY_TOUCHMI            , _UxGT("Deploy TouchMI"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Deploy Z-Probe"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Stow Z-Probe"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Najpierw Home %s%s%s"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Offset Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Babystep X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Babystep Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Babystep Z"));
  FSTRINGVALUE(MSG_BABYSTEP_TOTAL                   , _UxGT("Lacznie"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Blad krancówki"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Rozgrz. nieudane"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_BED           , _UxGT("Rozgrz. stolu nieudane"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_CHAMBER       , _UxGT("Rozgrz. komory nieudane"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Blad temperatury"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("ZANIK TEMPERATURY"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_BED              , _UxGT("ZANIK TEMP. STOLU"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_CHAMBER          , _UxGT("ZANIK TEMP.KOMORY"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Blad: MAXTEMP"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Blad: MINTEMP"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Blad: MAXTEMP STÓL"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Blad: MINTEMP STÓL"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_CHAMBER              , _UxGT("Blad: MAXTEMP KOMORA"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_CHAMBER              , _UxGT("Blad: MINTEMP KOMORA"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Najpierw Home XY"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("Drukarka zatrzym."));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Prosze zresetowac"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("g")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Rozgrzewanie..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Chlodzenie..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Rozgrzewanie stolu..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Chlodzenie stolu..."));
  FSTRINGVALUE(MSG_CHAMBER_HEATING                  , _UxGT("Rozgrzewanie komory..."));
  FSTRINGVALUE(MSG_CHAMBER_COOLING                  , _UxGT("Chlodzenie komory..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Kalibrowanie Delty"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibruj X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibruj Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibruj Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibruj srodek"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Ustawienia delty"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto kalibrowanie"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Ustaw wysokosc delty"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Przesun. Z sondy"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Ukosne ramie"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Wysokosc"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Promien"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("O drukarce"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Info drukarki"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("Poziomowanie 3-punktowe"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Poziomowanie liniowe"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Poziomowanie biliniowe"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Poziomowanie siatka"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Statystyki"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Info plyty"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistory"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Ekstrudery"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Predkosc USB"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protokól"));
  FSTRINGVALUE(MSG_INFO_RUNAWAY_OFF                 , _UxGT("Zegar pracy: OFF"));
  FSTRINGVALUE(MSG_INFO_RUNAWAY_ON                  , _UxGT("Zegar pracy: ON"));

  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Oswietlenie obudowy"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Jasnosc oswietlenia"));
  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Niepoprawna drukarka"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Wydrukowano"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Ukonczono"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Czas druku"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Najdl. druk"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Uzyty fil."));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Wydrukowano"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Ukonczono"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Razem"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Najdl. druk"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Uzyty fil."));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Min Temp"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Max Temp"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Zasilacz"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Sila silnika"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Sila %"));
  FSTRINGVALUE(MSG_ERROR_TMC                        , _UxGT("TMC BLAD POLACZENIA"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Zapisz DAC EEPROM"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER           , _UxGT("ZMIEN FILAMENT"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("WYDRUK WSTRZYMANY"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("ZALADUJ FILAMENT"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("WYLADUJ FILAMENT"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("OPCJE WZNOWIENIA:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Oczysc wiecej"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Kontynuuj"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Dysza: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Czujnik filamentu"));
  FSTRINGVALUE(MSG_RUNOUT_DISTANCE_MM               , _UxGT("Dystans do czujnika mm"));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Zerowanie nieudane"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Sondowanie nieudane"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: za zimne"));

  FSTRINGVALUE(MSG_MMU2_CHOOSE_FILAMENT_HEADER      , _UxGT("WYBIERZ FILAMENT"));
  FSTRINGVALUE(MSG_MMU2_MENU                        , _UxGT("MMU"));
  FSTRINGVALUE(MSG_MMU2_WRONG_FIRMWARE              , _UxGT("Uaktualnij firmware MMU!"));
  FSTRINGVALUE(MSG_MMU2_NOT_RESPONDING              , _UxGT("MMU wymaga uwagi."));
  FSTRINGVALUE(MSG_MMU2_RESUME                      , _UxGT("Wznów wydruk"));
  FSTRINGVALUE(MSG_MMU2_RESUMING                    , _UxGT("Wznawianie..."));
  FSTRINGVALUE(MSG_MMU2_LOAD_FILAMENT               , _UxGT("Zaladuj filament"));
  FSTRINGVALUE(MSG_MMU2_LOAD_ALL                    , _UxGT("Zaladuj wszystko"));
  FSTRINGVALUE(MSG_MMU2_LOAD_TO_NOZZLE              , _UxGT("Zaladuj do dyszy"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT              , _UxGT("Wysun filament"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT_N            , _UxGT("Wysun filament ~"));
  FSTRINGVALUE(MSG_MMU2_UNLOAD_FILAMENT             , _UxGT("Wyladuj filament"));
  FSTRINGVALUE(MSG_MMU2_LOADING_FILAMENT            , _UxGT("Ladowanie fil. %i..."));
  FSTRINGVALUE(MSG_MMU2_EJECTING_FILAMENT           , _UxGT("Wysuwanie fil. ..."));
  FSTRINGVALUE(MSG_MMU2_UNLOADING_FILAMENT          , _UxGT("Wyladowywanie fil...."));
  FSTRINGVALUE(MSG_MMU2_ALL                         , _UxGT("Wszystko"));
  FSTRINGVALUE(MSG_MMU2_FILAMENT_N                  , _UxGT("Filament ~"));
  FSTRINGVALUE(MSG_MMU2_RESET                       , _UxGT("Resetuj MMU"));
  FSTRINGVALUE(MSG_MMU2_RESETTING                   , _UxGT("Resetowanie MMU..."));
  FSTRINGVALUE(MSG_MMU2_EJECT_RECOVER               , _UxGT("Usun, kliknij"));

  FSTRINGVALUE(MSG_MIX                              , _UxGT("Miks"));
  FSTRINGVALUE(MSG_MIX_COMPONENT_N                  , _UxGT("Komponent ~"));
  FSTRINGVALUE(MSG_MIXER                            , _UxGT("Mikser"));
  FSTRINGVALUE(MSG_GRADIENT                         , _UxGT("Gradient"));
  FSTRINGVALUE(MSG_FULL_GRADIENT                    , _UxGT("Pelny gradient"));
  FSTRINGVALUE(MSG_TOGGLE_MIX                       , _UxGT("Przelacz miks"));
  FSTRINGVALUE(MSG_CYCLE_MIX                        , _UxGT("Cycle Mix"));
  FSTRINGVALUE(MSG_GRADIENT_MIX                     , _UxGT("Gradient Mix"));
  FSTRINGVALUE(MSG_REVERSE_GRADIENT                 , _UxGT("Odwrotny gradient"));
  FSTRINGVALUE(MSG_ACTIVE_VTOOL                     , _UxGT("Active V-tool"));
  FSTRINGVALUE(MSG_START_VTOOL                      , _UxGT("Start V-tool"));
  FSTRINGVALUE(MSG_END_VTOOL                        , _UxGT("  End V-tool"));
  FSTRINGVALUE(MSG_GRADIENT_ALIAS                   , _UxGT("Alias V-tool"));
  FSTRINGVALUE(MSG_RESET_VTOOLS                     , _UxGT("Reset V-tools"));
  FSTRINGVALUE(MSG_COMMIT_VTOOL                     , _UxGT("Commit V-tool Mix"));
  FSTRINGVALUE(MSG_VTOOLS_RESET                     , _UxGT("V-tools Were Reset"));
  FSTRINGVALUE(MSG_START_Z                          , _UxGT("Start Z:"));
  FSTRINGVALUE(MSG_END_Z                            , _UxGT("  End Z:"));

  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Games"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Brickout"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Invaders"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Sn4k3"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Maze"));

  #define MSG_1_LINE(A)     A "\0"   "\0"
  #define MSG_2_LINE(A,B)   A "\0" B "\0"
  #define MSG_3_LINE(A,B,C) A "\0" B "\0" C

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Nacisnik przycisk", "by wznowic drukowanie")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parkowanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Czekam na", "zmiane filamentu", "by wystartowac")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Wlóz filament", "i nacisnij przycisk", "by kontynuowac")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Nacisnij przycisk", "by nagrzac dysze")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Nagrzewanie dyszy", "Prosze czekac...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Czekam na", "wyjecie filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Czekam na", "wlozenie filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_2_LINE("Czekam na", "oczyszczenie filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_2_LINE("Kliknij by zakonczyc", "oczyszczanie filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Czekam na", "wznowienie wydruku...")));
  #else
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Kliknij by kontynuowac")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parkowanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Prosze czekac...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Wlóz i kliknij")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Kliknij by nagrzac")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Nagrzewanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Wysuwanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Ladowanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Oczyszczanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Kliknij by zakonczyc")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Wznawianie...")));
  #endif
  FSTRINGVALUE(MSG_TMC_DRIVERS                      , _UxGT("TMC Drivers"));
  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Driver Current"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Hybrid Threshold"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Sensorless Homing"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Stepping Mode"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("StealthChop Enabled"));
  FSTRINGVALUE(MSG_SERVICE_RESET                    , _UxGT("Reset"));
  FSTRINGVALUE(MSG_SERVICE_IN                       , _UxGT(" in:"));
  FSTRINGVALUE(MSG_BACKLASH                         , _UxGT("Backlash"));
  FSTRINGVALUE(MSG_BACKLASH_A                       , LCD_STR_A);
  FSTRINGVALUE(MSG_BACKLASH_B                       , LCD_STR_B);
  FSTRINGVALUE(MSG_BACKLASH_C                       , LCD_STR_C);
  FSTRINGVALUE(MSG_BACKLASH_CORRECTION              , _UxGT("Correction"));
  FSTRINGVALUE(MSG_BACKLASH_SMOOTHING               , _UxGT("Smoothing"));
}
