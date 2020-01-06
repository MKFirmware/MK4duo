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
 * Slovak
 * UTF-8 for Graphical Display
 *
 * LCD Menu Messages
 *
 * Translated by Michal Holeš, Farma MaM
 * http://www.facebook.com/farmamam
 *
 */

#define DISPLAY_CHARSET_ISO10646_SK

namespace language_sk {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Slovak"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" pripravená."));
  FSTRINGVALUE(MSG_YES                              , _UxGT("ÁNO"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("NIE"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Naspäť"));
  FSTRINGVALUE(MSG_MEDIA_ABORTING                   , _UxGT("Ruším..."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Karta vložená"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Karta vybraná"));
  FSTRINGVALUE(MSG_MEDIA_RELEASED                   , _UxGT("Karta odpojená"));
  FSTRINGVALUE(MSG_MEDIA_WAITING                    , _UxGT("Čakám na kartu"));
  FSTRINGVALUE(MSG_MEDIA_READ_ERROR                 , _UxGT("Chyba čítania karty"));
  FSTRINGVALUE(MSG_MEDIA_USB_REMOVED                , _UxGT("USB zaria. odstrán."));
  FSTRINGVALUE(MSG_MEDIA_USB_FAILED                 , _UxGT("Chyba spúšťania USB"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstopy")); // max 8 znakov
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Soft. endstopy"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Hlavná ponuka"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Pokročilé nastav."));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Konfigurácia"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Auto-štart"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Uvolniť motory"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Ponuka ladenia"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Test uk. priebehu"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Domovská pozícia"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Domov os X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Domov os Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Domov os Z"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Auto-zarovn. Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Parkovanie XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Kliknutím spusťte"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Ďalší bod"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Meranie hotové!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Výška rovnania"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Nastaviť ofsety"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Ofsety nastavené"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Nastaviť začiatok"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Zahriať ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Zahriať ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Zahriať ") PREHEAT_1_LABEL _UxGT(" hotend"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Zahriať ") PREHEAT_1_LABEL _UxGT(" hotend ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Zahriať ") PREHEAT_1_LABEL _UxGT(" všetko"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Zahriať ") PREHEAT_1_LABEL _UxGT(" podlož"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Zahriať ") PREHEAT_1_LABEL _UxGT(" nast."));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Zahriať ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Zahriať ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Zahriať ") PREHEAT_2_LABEL _UxGT(" hotend"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Zahriať ") PREHEAT_2_LABEL _UxGT(" hotend ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Zahriať ") PREHEAT_2_LABEL _UxGT(" všetko"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Zahriať ") PREHEAT_2_LABEL _UxGT(" podlož"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Zahriať ") PREHEAT_2_LABEL _UxGT(" nast."));
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Vlastná teplota"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Schladiť"));
  FSTRINGVALUE(MSG_LASER_MENU                       , _UxGT("Nastavenie lasera"));
  FSTRINGVALUE(MSG_LASER_OFF                        , _UxGT("Vypnúť laser"));
  FSTRINGVALUE(MSG_LASER_ON                         , _UxGT("Zapnúť laser"));
  FSTRINGVALUE(MSG_LASER_POWER                      , _UxGT("Výkon lasera"));
  FSTRINGVALUE(MSG_SPINDLE_MENU                     , _UxGT("Nastavenie vretena"));
  FSTRINGVALUE(MSG_SPINDLE_OFF                      , _UxGT("Vypnúť vreteno"));
  FSTRINGVALUE(MSG_SPINDLE_ON                       , _UxGT("Zapnúť vreteno"));
  FSTRINGVALUE(MSG_SPINDLE_POWER                    , _UxGT("Výkon vretena"));
  FSTRINGVALUE(MSG_SPINDLE_REVERSE                  , _UxGT("Spätný chod"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Zapnúť napájanie"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Vypnúť napájanie"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Vytlačiť (extr.)"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Vytiahnuť (retr.)"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Posunúť osy"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Vyrovnanie podložky"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Vyrovnať podložku"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Vyrovnať rohy"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Ďalší roh"));
  FSTRINGVALUE(MSG_MESH_EDITOR                      , _UxGT("Editor sieťe bodov"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Upraviť sieť bodov"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Koniec úprav siete"));
  FSTRINGVALUE(MSG_PROBING_MESH                     , _UxGT("Skúšam bod"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Index X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Index Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Hodnota Z"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Vlastné príkazy"));
  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("M48 Test sondy"));
  FSTRINGVALUE(MSG_M48_POINT                        , _UxGT("M48 Bod"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Odchýlka"));
  FSTRINGVALUE(MSG_IDEX_MENU                        , _UxGT("IDEX režim"));
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Ofset nástrojov"));
  FSTRINGVALUE(MSG_IDEX_MODE_AUTOPARK               , _UxGT("Auto-parkovanie"));
  FSTRINGVALUE(MSG_IDEX_MODE_DUPLICATE              , _UxGT("Duplikácia"));
  FSTRINGVALUE(MSG_IDEX_MODE_MIRRORED_COPY          , _UxGT("Zrkadlená kópia"));
  FSTRINGVALUE(MSG_IDEX_MODE_FULL_CTRL              , _UxGT("Plná kontrola"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2. tryska X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2. tryska Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2. tryska Z"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("Vykonávam G29"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("Nástroje UBL"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("UBL rovnanie"));
  FSTRINGVALUE(MSG_LCD_TILTING_MESH                 , _UxGT("Vyrovnávam bod"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Manuálna sieť bodov"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Položte a zmerajte"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Zmerajte"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Odstráňte a zmerajte"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Presun na ďalší"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Aktivovať UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Deaktivovať UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Teplota podložky"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Teplota podložky"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Teplota hotendu"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Teplota hotendu"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Úprava siete bodov"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Upraviť vlastnú sieť"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Doladiť sieť bodov"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Koniec úprav siete"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Vlastná sieť"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Vytvoriť sieť"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Sieť bodov ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Sieť bodov ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Studená sieť bodov"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Upraviť výšku siete"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Výška"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Skontrolovať sieť"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Kontrola siete ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Kontrola siete ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Kontrola vlast.siete"));
  FSTRINGVALUE(MSG_G26_HEATING_BED                  , _UxGT("G26 ohrev podlž."));
  FSTRINGVALUE(MSG_G26_HEATING_NOZZLE               , _UxGT("G26 ohrev trysky"));
  FSTRINGVALUE(MSG_G26_MANUAL_PRIME                 , _UxGT("Ručné čistenie..."));
  FSTRINGVALUE(MSG_G26_FIXED_LENGTH                 , _UxGT("Čistenie pevn. dĺž."));
  FSTRINGVALUE(MSG_G26_PRIME_DONE                   , _UxGT("Čistenie dokončené"));
  FSTRINGVALUE(MSG_G26_CANCELED                     , _UxGT("G26 zrušený"));
  FSTRINGVALUE(MSG_G26_LEAVING                      , _UxGT("Opúšťam G26"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Pokračovať v sieti"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Sieťové rovnanie"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("3-bodové rovnanie"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Mriežkové rovnanie"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Vyrovnať podložku"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Postranné body"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Typ siete bodov"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Exportovať sieť"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Exportovať do PC"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Exportovať do CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Záloha do PC"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Info. o UBL do PC"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Hustota mriežky"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Ručné vyplnenie"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Chytré vyplnenie"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Vyplniť mriežku"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Zrušiť všetko"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Zrušiť najbližší"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Upraviť všetky"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Upraviť najbližší"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Úložisko sietí"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Pamäťový slot"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Načítať sieť bodov"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Uložiť sieť bodov"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Sieť %i načítaná"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Sieť %i uložená"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Nedostatok miesta"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Chyba: Ukladanie UBL"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Chyba: Obnovenie UBL"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET                     , _UxGT("Z-Ofset: "));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Koniec kompenz. Z"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("Postupné UBL"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Studená sieť bodov"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Chytré vyplnenie"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Skontrolovať sieť"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Точная настр. всего"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Skontrolovať sieť"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Точная настр. всего"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Uložiť sieť bodov"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Nastavenie LED"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Svetlo"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Prednastavené farby"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Červená"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Oranžová"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Žltá"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Zelená"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Modrá"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Indigo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Fialová"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Biela"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Obnoviť nastavenie"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Vlastná farba"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Inten. červenej"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Inten. zelenej"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Inten. modrej"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Inten. bielej"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Jas"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Posúvam..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Uvolniť XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Posunúť X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Posunúť Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Posunúť Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extrudér"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extrudér *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Hotend je studený"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Posunúť o %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Posunúť o 0,1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Posunúť o 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Posunúť o 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Rýchlosť"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Výška podl."));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Tryska"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Tryska ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Podložka"));
  FSTRINGVALUE(MSG_CHAMBER                          , _UxGT("Komora"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Rýchlosť vent."));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Rýchlosť vent. ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Rýchlosť ex. vent."));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Rýchlosť ex. vent. ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Prietok"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Prietok ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Ovládanie"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fakt"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Auto-teplota"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Zap"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Vyp"));
  FSTRINGVALUE(MSG_PID_AUTOTUNE                     , _UxGT("PID kalibrácia"));
  FSTRINGVALUE(MSG_PID_AUTOTUNE_E                   , _UxGT("PID kalibrácia *"));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID-P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID-I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID-D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID-C *"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Vybrať"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Vybrať *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Zrýchlenie"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Skok"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("V") LCD_STR_A _UxGT("-skok"));
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("V") LCD_STR_B _UxGT("-skok"));
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("V") LCD_STR_C _UxGT("-skok"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Ve-skok"));
  FSTRINGVALUE(MSG_JUNCTION_DEVIATION               , _UxGT("Junction Dev"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Rýchlosť"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("Vmax ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("Vmax ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("Vmax ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vmax ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("Vmax *"));
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("Vmin"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("VPrej Min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Akcelerácia"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Amax ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Amax ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Amax ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Amax ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Amax *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-retrakt"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-prejazd"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Kroky/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT("krokov/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT("krokov/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT("krokov/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("Ekrokov/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("*krokov/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Teplota"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Pohyb"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E na mm³"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Priem. fil."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Priem. fil. *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Vysunúť mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Zaviesť mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("K pre posun"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("K pre posun *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Kontrast LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Uložiť nastavenie"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Načítať nastavenie"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Obnoviť nastavenie"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Inicializ. EEPROM"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("Aktualizovať z SD"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Reštart. tlačiar."));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Obnoviť"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Info. obrazovka"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Príprava tlače"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Doladenie tlače"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Spustiť tlač"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Ďalší"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Inicial."));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Zastaviť"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Tlačiť"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Vynulovať"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Zrušiť"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Hotovo"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pozastaviť tlač"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Obnoviť tlač"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Zastaviť tlač"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Obnova po výp. nap."));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Tlačiť z SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Žiadna SD karta"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Spím..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Pokrač. kliknutím..."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Tlač pozastavená"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Tlačím..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Tlač zrušená"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Žiadny pohyb."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("PRERUŠENÉ. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ZASTAVENÉ. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retrakt mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Výmena Re.mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retraktovať  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Zdvih Z mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("S UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("AutoRetr."));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Dĺžka výmeny"));
  FSTRINGVALUE(MSG_FILAMENT_PURGE_LENGTH            , _UxGT("Dĺžka vytlačenia"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Výmena nástroja"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Zdvihnúť Z"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Primárna rýchl."));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Rýchl. retrakcie"));
  FSTRINGVALUE(MSG_NOZZLE_STANDBY                   , _UxGT("Záložná tryska"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Vymeniť filament"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Vymeniť filament *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Zaviesť filament"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Zaviesť filament *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Vysunúť filament"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Vysunúť filament *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Vysunúť všetko"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Načítať SD kartu"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Vymeniť SD kartu"));
  FSTRINGVALUE(MSG_RELEASE_MEDIA                    , _UxGT("Odpojiť SD kartu"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonda Z mimo podl."));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Faktor skosenia"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Cmd: Self-Test"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Cmd: Reset"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Cmd: Zasunúť"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Cmd: Vysunúť"));
  FSTRINGVALUE(MSG_BLTOUCH_SW_MODE                  , _UxGT("Cmd: Režim SW"));
  FSTRINGVALUE(MSG_BLTOUCH_5V_MODE                  , _UxGT("Cmd: Režim 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_OD_MODE                  , _UxGT("Cmd: Režim OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE               , _UxGT("Cmd: Ulož. režim"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_5V            , _UxGT("Prepnúť do 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_OD            , _UxGT("Prepnúť do OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_ECHO                , _UxGT("Zobraziť režim"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_CHANGE              , _UxGT("POZOR: Zlé nastav. môže spôsobiť poškoden. Pokračovať?"));
  FSTRINGVALUE(MSG_TOUCHMI_PROBE                    , _UxGT("TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_INIT                     , _UxGT("Inicializ. TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_ZTEST                    , _UxGT("Test ofsetu Z"));
  FSTRINGVALUE(MSG_TOUCHMI_SAVE                     , _UxGT("Uložiť"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY_TOUCHMI            , _UxGT("Zasunúť TouchMI"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Zasunúť sondu Z"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Vysunúť sondu Z"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Najskôr os %s%s%s domov"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Ofset sondy Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Babystep X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Babystep Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Babystep Z"));
  FSTRINGVALUE(MSG_BABYSTEP_TOTAL                   , _UxGT("Celkom"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Zastavenie Endstop"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Chyba ohrevu"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_BED           , _UxGT("Chyba ohrevu podl."));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_CHAMBER       , _UxGT("Chyba ohrevu komory"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Chyba: REDUND. TEP."));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("TEPLOTNÝ SKOK"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_BED              , _UxGT("TEPLOTNÝ SKOK PODL."));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_CHAMBER          , _UxGT("TEPLOTNÝ SKOK KOMO."));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Chyba: MAXTEMP"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Chyba: MINTEMP"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Chyba: MAXTEMP PODL."));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Chyba: MINTEMP PODL."));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_CHAMBER              , _UxGT("Chyba: MAXTEMP KOMO."));
  FSTRINGVALUE(MSG_ERR_MINTEMP_CHAMBER              , _UxGT("Chyba: MINTEMP KOMO."));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Najskôr os XY domov"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("TLAČIAREŇ ZASTAVENÁ"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Reštartuje ju"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d"));
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h"));
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Ohrev..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Ochladzovanie..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Ohrev podložky..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Ochladz. podložky..."));
  FSTRINGVALUE(MSG_CHAMBER_HEATING                  , _UxGT("Ohrev komory..."));
  FSTRINGVALUE(MSG_CHAMBER_COOLING                  , _UxGT("Ochladz. komory..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta kalibrácia"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibrovať X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibrovať Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibrovať Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibrovať stred"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Delta nastavenia"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto-kalibrácia"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Nast. výšku delty"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Ofset sondy Z"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Diag. rameno"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Výška"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Polomer"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("O tlačiarni"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Info. o tlačiarni"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("3-bodové rovnanie"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Lineárne rovnanie"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Bilineárne rovnanie"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("UBL rovnanie"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Mriežkové rovnanie"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Štatistika"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Info. o doske"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistory"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extrudéry"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Rýchlosť"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protokol"));
  FSTRINGVALUE(MSG_INFO_RUNAWAY_OFF                 , _UxGT("Tepl. ochrana: VYP"));
  FSTRINGVALUE(MSG_INFO_RUNAWAY_ON                  , _UxGT("Tepl. ochrana: ZAP"));

  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Osvetlenie"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Jas svetla"));
  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Nesprávna tlačiareň"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Počet tlačí"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Dokončené"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Celkový čas"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Najdlhšia tlač"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Celkom vytlačené"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Tlače"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Hotovo"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Čas"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Najdlhšia"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Vytlačené"));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Teplota min"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Teplota max"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Nap. zdroj"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Budenie motorov"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Motor %"));
  FSTRINGVALUE(MSG_ERROR_TMC                        , _UxGT("CHYBA KOMUNIKÁ. TMC"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Uložiť do EEPROM"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER           , _UxGT("VÝMENA FILAMENTU"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("PAUZA TLAČE"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("ZAVEDENIE FILAMENTU"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("VYSUNUTIE FILAMENTU"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("MOŽNOSTI POKRAČ.:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Vytlačiť viacej"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Obnoviť tlač"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Tryska: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Senzor filamentu"));
  FSTRINGVALUE(MSG_RUNOUT_DISTANCE_MM               , _UxGT("Vzd. mm fil. senz."));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Parkovanie zlyhalo"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Kalibrácia zlyhala"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: Príliš studený"));

  FSTRINGVALUE(MSG_MMU2_CHOOSE_FILAMENT_HEADER      , _UxGT("VYBERTE FILAMENT"));
  FSTRINGVALUE(MSG_MMU2_MENU                        , _UxGT("MMU2"));
  FSTRINGVALUE(MSG_MMU2_WRONG_FIRMWARE              , _UxGT("Aktualizujte FW MMU!"));
  FSTRINGVALUE(MSG_MMU2_NOT_RESPONDING              , _UxGT("MMU potrebuje zásah."));
  FSTRINGVALUE(MSG_MMU2_RESUME                      , _UxGT("Obnoviť tlač"));
  FSTRINGVALUE(MSG_MMU2_RESUMING                    , _UxGT("Obnovovanie..."));
  FSTRINGVALUE(MSG_MMU2_LOAD_FILAMENT               , _UxGT("Zaviesť filament"));
  FSTRINGVALUE(MSG_MMU2_LOAD_ALL                    , _UxGT("Zaviesť všetky"));
  FSTRINGVALUE(MSG_MMU2_LOAD_TO_NOZZLE              , _UxGT("Zaviesť po trysku"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT              , _UxGT("Vysunúť filament"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT_N            , _UxGT("Vysunúť filament ~"));
  FSTRINGVALUE(MSG_MMU2_UNLOAD_FILAMENT             , _UxGT("Vyňať filament"));
  FSTRINGVALUE(MSG_MMU2_LOADING_FILAMENT            , _UxGT("Zavádzanie fil. %i..."));
  FSTRINGVALUE(MSG_MMU2_EJECTING_FILAMENT           , _UxGT("Vysúvanie fil. ..."));
  FSTRINGVALUE(MSG_MMU2_UNLOADING_FILAMENT          , _UxGT("Vysúvanie fil. ..."));
  FSTRINGVALUE(MSG_MMU2_ALL                         , _UxGT("Všetky"));
  FSTRINGVALUE(MSG_MMU2_FILAMENT_N                  , _UxGT("Filament ~"));
  FSTRINGVALUE(MSG_MMU2_RESET                       , _UxGT("Reštartovať MMU"));
  FSTRINGVALUE(MSG_MMU2_RESETTING                   , _UxGT("Reštart MMU..."));
  FSTRINGVALUE(MSG_MMU2_EJECT_RECOVER               , _UxGT("Odstráňte, kliknite"));

  FSTRINGVALUE(MSG_MIX                              , _UxGT("Mix"));
  FSTRINGVALUE(MSG_MIX_COMPONENT_N                  , _UxGT("Zložka ~"));
  FSTRINGVALUE(MSG_MIXER                            , _UxGT("Mixér"));
  FSTRINGVALUE(MSG_GRADIENT                         , _UxGT("Gradient"));
  FSTRINGVALUE(MSG_FULL_GRADIENT                    , _UxGT("Plný gradient"));
  FSTRINGVALUE(MSG_TOGGLE_MIX                       , _UxGT("Prepnúť mix"));
  FSTRINGVALUE(MSG_CYCLE_MIX                        , _UxGT("Cyklický mix"));
  FSTRINGVALUE(MSG_GRADIENT_MIX                     , _UxGT("Gradientný mix"));
  FSTRINGVALUE(MSG_REVERSE_GRADIENT                 , _UxGT("Otočiť gradient"));
  FSTRINGVALUE(MSG_ACTIVE_VTOOL                     , _UxGT("Aktívny V-tool"));
  FSTRINGVALUE(MSG_START_VTOOL                      , _UxGT("Počiat. V-tool"));
  FSTRINGVALUE(MSG_END_VTOOL                        , _UxGT("Konečn. V-tool"));
  FSTRINGVALUE(MSG_GRADIENT_ALIAS                   , _UxGT("Alias V-tool"));
  FSTRINGVALUE(MSG_RESET_VTOOLS                     , _UxGT("Vynulovať V-tools"));
  FSTRINGVALUE(MSG_COMMIT_VTOOL                     , _UxGT("Uložiť V-tool Mix"));
  FSTRINGVALUE(MSG_VTOOLS_RESET                     , _UxGT("V-tools vynulované"));
  FSTRINGVALUE(MSG_START_Z                          , _UxGT("Počiat.Z:"));
  FSTRINGVALUE(MSG_END_Z                            , _UxGT("Konečn.Z:"));

  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Hry"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Brickout"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Nájazdníci"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Had"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Bludisko"));

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Stlačte tlačidlo", "pre obnovu tlače")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parkovanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Čakajte prosím", "na spustenie", "výmeny filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Vložte filament", "a stlačte tlačidlo", "pre pokračovanie")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Stlačte tlačidlo", "pre ohrev trysky")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Ohrev trysky", "Čakajte prosím...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_3_LINE("Čakajte prosím", "na vysunutie", "filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_3_LINE("Čakajte prosím", "na zavedenie", "filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_3_LINE("Čakajte prosím", "na vytlačenie", "filamentu")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_3_LINE("Stlačte tlačidlo", "pre dokončenie", "vytláčania filam.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Čakajte prosím na", "obnovenie tlače...")));
  #else
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Kliknite pre pokr.")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parkovanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Čakajte prosím...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Vložte a kliknite")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Kliknite pre ohrev")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Ohrev...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Vysúvanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Zavádzanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Vytlačovanie...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Klik. pre dokonč.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Pokračovanie...")));
  #endif
  FSTRINGVALUE(MSG_TMC_DRIVERS                      , _UxGT("Ovládače TMC"));
  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Prúd ovládača"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Hybridný prah"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Bezsenzor. domov"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Režim krokovania"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("StealthChop zapnutý"));
  FSTRINGVALUE(MSG_SERVICE_RESET                    , _UxGT("Vynulovať"));
  FSTRINGVALUE(MSG_SERVICE_IN                       , _UxGT("za:"));
  FSTRINGVALUE(MSG_BACKLASH                         , _UxGT("Kompenz. vôle"));
  FSTRINGVALUE(MSG_BACKLASH_A                       , LCD_STR_A);
  FSTRINGVALUE(MSG_BACKLASH_B                       , LCD_STR_B);
  FSTRINGVALUE(MSG_BACKLASH_C                       , LCD_STR_C);
  FSTRINGVALUE(MSG_BACKLASH_CORRECTION              , _UxGT("Korekcia"));
  FSTRINGVALUE(MSG_BACKLASH_SMOOTHING               , _UxGT("Vyhladzovanie"));
}
