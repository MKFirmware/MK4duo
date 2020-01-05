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
 * German
 *
 * LCD Menu Messages
 *
 */

namespace language_de {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Deutsche"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" bereit"));

  FSTRINGVALUE(MSG_YES                              , _UxGT("JA"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("NEIN"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Zurück"));
  FSTRINGVALUE(MSG_MEDIA_ABORTING                   , _UxGT("Abbruch..."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Medium erkannt"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Medium entfernt"));
  FSTRINGVALUE(MSG_MEDIA_RELEASED                   , _UxGT("Medium freigegeben"));
  FSTRINGVALUE(MSG_MEDIA_WAITING                    , _UxGT("Warten auf Medium"));
  FSTRINGVALUE(MSG_MEDIA_READ_ERROR                 , _UxGT("Medium Lesefehler"));
  FSTRINGVALUE(MSG_MEDIA_USB_REMOVED                , _UxGT("USB Gerät entfernt"));
  FSTRINGVALUE(MSG_MEDIA_USB_FAILED                 , _UxGT("USB Start fehlge."));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstopp")); // Max length 8 characters
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Software-Endstopp"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Hauptmenü"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Erw. Einstellungen"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Konfiguration"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Autostart"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Motoren deaktivieren")); // M84 :: Max length 19 characters
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Debug-Menü"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Statusbalken-Test"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Z-Achsen ausgleichen"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Klick zum Starten"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Nächste Koordinate"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Nivellieren fertig!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Ausblendhöhe"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Setze Homeversatz"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Homeversatz aktiv"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Setze Nullpunkte")); //"G92 X0 Y0 Z0" commented out in ultralcd.cpp
  FSTRINGVALUE(MSG_PREHEAT_1                        , PREHEAT_1_LABEL _UxGT(" Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , PREHEAT_1_LABEL _UxGT(" Vorwärmen") " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , PREHEAT_1_LABEL _UxGT(" Extr. Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , PREHEAT_1_LABEL _UxGT(" Extr. Vorwärm. ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , PREHEAT_1_LABEL _UxGT(" Alles Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , PREHEAT_1_LABEL _UxGT(" Bett Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , PREHEAT_1_LABEL _UxGT(" Einstellungen"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , PREHEAT_2_LABEL _UxGT(" Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , PREHEAT_2_LABEL _UxGT(" Vorwärmen") " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , PREHEAT_2_LABEL _UxGT(" Extr. Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , PREHEAT_2_LABEL _UxGT(" Extr. Vorwärm. ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , PREHEAT_2_LABEL _UxGT(" Alles Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , PREHEAT_2_LABEL _UxGT(" Bett Vorwärmen"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , PREHEAT_2_LABEL _UxGT(" Einstellungen"));
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("benutzerdef. Heizen"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Abkühlen"));
  FSTRINGVALUE(MSG_LASER_MENU                       , _UxGT("Laser"));
  FSTRINGVALUE(MSG_LASER_OFF                        , _UxGT("Laser aus"));
  FSTRINGVALUE(MSG_LASER_ON                         , _UxGT("Laser an"));
  FSTRINGVALUE(MSG_LASER_POWER                      , _UxGT("Laserleistung"));
  FSTRINGVALUE(MSG_SPINDLE_MENU                     , _UxGT("Spindel-Steuerung"));
  FSTRINGVALUE(MSG_SPINDLE_OFF                      , _UxGT("Spindel aus"));
  FSTRINGVALUE(MSG_SPINDLE_ON                       , _UxGT("Spindel an"));
  FSTRINGVALUE(MSG_SPINDLE_POWER                    , _UxGT("Spindelleistung"));
  FSTRINGVALUE(MSG_SPINDLE_REVERSE                  , _UxGT("Spindelrichtung"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Netzteil ein"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Netzteil aus"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrudieren"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Einzug"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Achsen bewegen"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Bett-Nivellierung"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Bett nivellieren"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Ecken nivellieren"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Nächste Ecke"));
  FSTRINGVALUE(MSG_MESH_EDITOR                      , _UxGT("Netz Editor"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Netz bearbeiten"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Netzbearb. angeh."));
  FSTRINGVALUE(MSG_PROBING_MESH                     , _UxGT("Messpunkt"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Index X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Index Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Z-Wert"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Benutzer-Menü"));
  FSTRINGVALUE(MSG_LCD_TILTING_MESH                 , _UxGT("Berührungspunkt"));
  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("M48 Sondentest"));
  FSTRINGVALUE(MSG_M48_POINT                        , _UxGT("M48 Punkt"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Abweichung"));
  FSTRINGVALUE(MSG_IDEX_MENU                        , _UxGT("IDEX-Modus"));
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Werkzeugversätze"));
  FSTRINGVALUE(MSG_IDEX_MODE_AUTOPARK               , _UxGT("Autom. parken"));
  FSTRINGVALUE(MSG_IDEX_MODE_DUPLICATE              , _UxGT("Duplizieren"));
  FSTRINGVALUE(MSG_IDEX_MODE_MIRRORED_COPY          , _UxGT("Spiegelkopie"));
  FSTRINGVALUE(MSG_IDEX_MODE_FULL_CTRL              , _UxGT("vollstä. Kontrolle"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2. Düse X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2. Düse Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2. Düse Z"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("G29 ausführen"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("UBL-Werkzeuge"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Netz manuell erst."));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Unterlegen & messen"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Messen"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Entfernen & messen"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Nächster Punkt..."));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("UBL aktivieren"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("UBL deaktivieren"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Betttemperatur"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Betttemperatur"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Hotend-Temp."));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Hotend-Temp."));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Netz bearbeiten"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Eigenes Netz bearb."));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Feineinstellung..."));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Bearbeitung beendet"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Eigenes Netz erst."));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Netz erstellen"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , PREHEAT_1_LABEL _UxGT(" Netz erstellen"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , PREHEAT_2_LABEL _UxGT(" Netz erstellen"));
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Netz erstellen kalt"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Netzhöhe einst."));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Höhe"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Netz validieren"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , PREHEAT_1_LABEL _UxGT(" Netz validieren"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , PREHEAT_2_LABEL _UxGT(" Netz validieren"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Eig. Netz validieren"));
  FSTRINGVALUE(MSG_G26_HEATING_NOZZLE               , _UxGT("G26 Heating Nozzle"));
  FSTRINGVALUE(MSG_G26_HEATING_BED                  , _UxGT("G26 heizt Bett"));
  FSTRINGVALUE(MSG_G26_FIXED_LENGTH                 , _UxGT("Feste Länge Prime"));
  FSTRINGVALUE(MSG_G26_PRIME_DONE                   , _UxGT("Priming fertig"));
  FSTRINGVALUE(MSG_G26_CANCELED                     , _UxGT("G26 abgebrochen"));
  FSTRINGVALUE(MSG_G26_LEAVING                      , _UxGT("G26 verlassen"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Netzerst. forts."));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Netz-Nivellierung"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("3-Punkt-Nivell."));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Gitternetz-Nivell."));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Netz nivellieren"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Eckpunkte"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Kartentyp"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Karte ausgeben"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Ausgabe für Host"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Ausgabe für CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Externe Sicherung"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("UBL-Info ausgeben"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Menge an Füllung"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Manuelles Füllen"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Cleveres Füllen"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Netz Füllen"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Alles annullieren"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Nächstlieg. ann."));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Feineinst. Alles"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Feineinst. Nächstl."));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Netz-Speicherplatz"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Speicherort"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Bettnetz laden"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Bettnetz speichern"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Netz %i geladen"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Netz %i gespeichert"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Kein Speicher"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Err:UBL speichern"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Err:UBL wiederherst."));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET                     , _UxGT("Z-Versatz: "));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Z-Versatz angehalten"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("Schrittweises UBL"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Netz erstellen kalt"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Cleveres Füllen"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Netz validieren"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Feineinst. Alles"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Netz validieren"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Feineinst. Alles"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Bettnetz speichern"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Licht-Steuerung"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Licht"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Licht-Einstellung"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Rot"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Orange"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Gelb"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Grün"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Blau"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Indigo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Violett"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Weiß"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Standard"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Benutzerdefiniert"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Intensität Rot"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Intensität Grün"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Intensität Blau"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Intensität Weiß"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Helligkeit"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("In Bewegung..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Abstand XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Bewege X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Bewege Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Bewege Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Bewege Extruder"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Bewege Extruder *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Hotend zu kalt"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT(" %s mm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT(" 0,1   mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT(" 1,0   mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("10,0   mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Geschw."));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Bett Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Düse"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Düse ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Bett"));
  FSTRINGVALUE(MSG_CHAMBER                          , _UxGT("Gehäuse"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Lüfter"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Lüfter ~"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Geschw. Extralüfter"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Geschw. Extralüfter ~"));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Flussrate"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Flussrate ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Einstellungen"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Faktor"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Auto Temperatur"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("an"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("aus"));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID P"));
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID I"));
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID D"));
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID C"));
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID C *"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Auswählen"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Auswählen *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Beschleunigung"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Jerk"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("V ") LCD_STR_A _UxGT(" Jerk"));
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("V ") LCD_STR_B _UxGT(" Jerk"));
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("V ") LCD_STR_C _UxGT(" Jerk"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("V E Jerk"));
  FSTRINGVALUE(MSG_JUNCTION_DEVIATION               , _UxGT("Junction Dev"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Geschwindigkeit"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("V max ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("V max ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("V max ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("V max ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("V max *"));
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("V min "));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("V min Leerfahrt"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Beschleunigung"));
  FSTRINGVALUE(MSG_AMAX                             , _UxGT("A max ")); // space intentional
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("A max ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("A max ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("A max ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("A max ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("A max *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A Einzug"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A Leerfahrt"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Steps/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" Steps/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" Steps/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" Steps/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , LCD_STR_E _UxGT(" Steps/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* Steps/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatur"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Bewegung"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm³"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Filamentdurchmesser"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Filamentdurchmesser *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Entladen mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Laden mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("Vorschubfaktor"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("Vorschubfaktor *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD-Kontrast"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Konfig. speichern"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Konfig. laden"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Standardwerte laden"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Werkseinstellungen"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("FW Update vom Medium"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Drucker neustarten"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Aktualisieren"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Info"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Vorbereitung"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Justierung"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Starte Druck"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Weiter"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Bestätigen"));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Stop"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Drucken"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Reseten"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Abbrechen"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Fertig"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("SD-Druck pausieren"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("SD-Druck fortsetzen"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("SD-Druck abbrechen"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Wiederh. n. Stroma."));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Druck vom Medium"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Kein Medium"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Warten..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Klick zum Fortsetzen"));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Druck pausiert..."));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Druckt..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Druck abgebrochen"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Motoren angeschaltet"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("ABGEBROCHEN"));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ANGEHALTEN"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Einzug mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Wechs. Einzug mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("V Einzug"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Z-Sprung mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Wechs. UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Autom. Einzug"));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Einzugslänge"));
  FSTRINGVALUE(MSG_FILAMENT_PURGE_LENGTH            , _UxGT("Entladelänge"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Werkzeugwechsel"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Z anheben"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Prime-Geschwin."));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Einzug-Geschwin."));
  FSTRINGVALUE(MSG_NOZZLE_STANDBY                   , _UxGT("Düsen-Standby"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Filament wechseln"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Filament wechseln *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Filament laden"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Filament laden *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Filament entladen"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Filament entladen *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Alles entladen"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Medium initial."));  // Manually initialize the SD-card via user interface
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Medium getauscht")); // SD-card changed by user. For machines with no autocarddetect. Both send "M21"
  FSTRINGVALUE(MSG_RELEASE_MEDIA                    , _UxGT("Medium freigeben")); // if Marlin gets confused - M22
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z-Sonde außerhalb"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Korrekturfaktor"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch Selbsttest"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("BLTouch zurücks."));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("BLTouch ausfahren"));
  FSTRINGVALUE(MSG_BLTOUCH_SW_MODE                  , _UxGT("BLTouch SW-Modus"));
  FSTRINGVALUE(MSG_BLTOUCH_5V_MODE                  , _UxGT("BLTouch 5V-Modus"));
  FSTRINGVALUE(MSG_BLTOUCH_OD_MODE                  , _UxGT("BLTouch OD-Modus"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE               , _UxGT("BLTouch Mode Store"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_5V            , _UxGT("BLTouch auf 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_OD            , _UxGT("BLTouch auf OD"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("BLTouch einfahren"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_ECHO                , _UxGT("BLTouch Modus: "));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_CHANGE              , _UxGT("ACHTUNG: Falsche Einstellung - kann zu Beschädigung führen! Fortfahren?"));
  FSTRINGVALUE(MSG_TOUCHMI_PROBE                    , _UxGT("TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_INIT                     , _UxGT("TouchMI initial."));
  FSTRINGVALUE(MSG_TOUCHMI_ZTEST                    , _UxGT("Test Z-Versatz"));
  FSTRINGVALUE(MSG_TOUCHMI_SAVE                     , _UxGT("Speichern"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY_TOUCHMI            , _UxGT("TouchMI ausfahren"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Z-Sonde ausfahren"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Z-Sonde einfahren"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Vorher %s%s%s homen"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Sondenversatz Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Babystep X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Babystep Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Babystep Z"));
  FSTRINGVALUE(MSG_BABYSTEP_TOTAL                   , _UxGT("Total"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Abbr. mit Endstopp"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("HEIZEN ERFOLGLOS"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_BED           , _UxGT("Bett heizen fehlge."));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_CHAMBER       , _UxGT("Geh. heizen fehlge."));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("REDUND. TEMP-ABWEI."));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , " " LCD_STR_THERMOMETER _UxGT(" NICHT ERREICHT"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_BED              , _UxGT("BETT") " " LCD_STR_THERMOMETER _UxGT(" NICHT ERREICHT"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_CHAMBER          , _UxGT("GEH.") " " LCD_STR_THERMOMETER _UxGT(" NICHT ERREICHT"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , " " LCD_STR_THERMOMETER _UxGT(" ÜBERSCHRITTEN"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , " " LCD_STR_THERMOMETER _UxGT(" UNTERSCHRITTEN"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("BETT ") LCD_STR_THERMOMETER _UxGT(" ÜBERSCHRITTEN"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("BETT ") LCD_STR_THERMOMETER _UxGT(" UNTERSCHRITTEN"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_CHAMBER              , _UxGT("Err:Gehäuse max Temp"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_CHAMBER              , _UxGT("Err:Gehäuse min Temp"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Vorher XY homen"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("DRUCKER GESTOPPT"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Bitte neustarten"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("t")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("heizt..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("kühlt..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Bett heizt..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Bett kühlt..."));
  FSTRINGVALUE(MSG_CHAMBER_HEATING                  , _UxGT("Gehäuse heizt..."));
  FSTRINGVALUE(MSG_CHAMBER_COOLING                  , _UxGT("Gehäuse kühlt..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta kalibrieren"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibriere X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibriere Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibriere Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibriere Mitte"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Delta Einst. anzeig."));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Autom. Kalibrierung"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Delta Höhe setzen"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Sondenversatz Z"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Diag Rod"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Höhe"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Radius"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Über den Drucker"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Drucker-Info"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("3-Punkt-Nivellierung"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Lineare Nivellierung"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Bilineare Nivell."));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Unified Bed Leveling"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Netz-Nivellierung"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Drucker-Statistik"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Board-Info"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Thermistoren"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extruder"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baudrate"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protokoll"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Beleuchtung"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Helligkeit"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Falscher Drucker"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Gesamte Drucke"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Komplette Drucke"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Gesamte Druckzeit"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Längste Druckzeit"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Gesamt Extrudiert"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Drucke"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Komplette"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Gesamte"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Längste"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extrud."));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Min Temp"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Max Temp"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Netzteil"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Motorleistung"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Treiber %"));
  FSTRINGVALUE(MSG_ERROR_TMC                        , _UxGT("TMC Verbindungsfehler"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Werte speichern"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER           , _UxGT("FILAMENT WECHSEL"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("DRUCK PAUSIERT"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("FILAMENT LADEN"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("FILAMENT ENTLADEN"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("FORTS. OPTIONEN:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Mehr entladen"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Druck weiter"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Düse: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Runout-Sensor"));
  FSTRINGVALUE(MSG_RUNOUT_DISTANCE_MM               , _UxGT("Runout-Weg mm"));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Homing gescheitert"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Probing gescheitert"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: zu kalt"));

  FSTRINGVALUE(MSG_MMU2_CHOOSE_FILAMENT_HEADER      , _UxGT("FILAMENT WÄHLEN"));
  FSTRINGVALUE(MSG_MMU2_MENU                        , _UxGT("MMU"));
  FSTRINGVALUE(MSG_MMU2_WRONG_FIRMWARE              , _UxGT("Update MMU Firmware!"));
  FSTRINGVALUE(MSG_MMU2_NOT_RESPONDING              , _UxGT("MMU handeln erfor."));
  FSTRINGVALUE(MSG_MMU2_RESUME                      , _UxGT("Druck fortsetzen"));
  FSTRINGVALUE(MSG_MMU2_RESUMING                    , _UxGT("Fortfahren..."));
  FSTRINGVALUE(MSG_MMU2_LOAD_FILAMENT               , _UxGT("Filament laden"));
  FSTRINGVALUE(MSG_MMU2_LOAD_ALL                    , _UxGT("Lade alle"));
  FSTRINGVALUE(MSG_MMU2_LOAD_TO_NOZZLE              , _UxGT("Düse laden"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT              , _UxGT("Filament auswerfen"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT_N            , _UxGT("Filament E auswerfen"));
  FSTRINGVALUE(MSG_MMU2_UNLOAD_FILAMENT             , _UxGT("Filament entladen "));
  FSTRINGVALUE(MSG_MMU2_LOADING_FILAMENT            , _UxGT("Lade Fila. %i..."));
  FSTRINGVALUE(MSG_MMU2_EJECTING_FILAMENT           , _UxGT("Fila. auswerfen..."));
  FSTRINGVALUE(MSG_MMU2_UNLOADING_FILAMENT          , _UxGT("Fila. entladen..."));
  FSTRINGVALUE(MSG_MMU2_ALL                         , _UxGT("Alle"));
  FSTRINGVALUE(MSG_MMU2_FILAMENT_N                  , _UxGT("Filament ~"));
  FSTRINGVALUE(MSG_MMU2_RESET                       , _UxGT("setze MMU zurück"));
  FSTRINGVALUE(MSG_MMU2_RESETTING                   , _UxGT("MMU zurücksetzen..."));
  FSTRINGVALUE(MSG_MMU2_EJECT_RECOVER               , _UxGT("Entfernen, klicken"));

  FSTRINGVALUE(MSG_MIX                              , _UxGT("Mix"));
  FSTRINGVALUE(MSG_MIX_COMPONENT_N                  , _UxGT("Komponente ~"));
  FSTRINGVALUE(MSG_MIXER                            , _UxGT("Mixer"));
  FSTRINGVALUE(MSG_GRADIENT                         , _UxGT("Gradient")); // equal Farbverlauf
  FSTRINGVALUE(MSG_FULL_GRADIENT                    , _UxGT("Volle Gradient"));
  FSTRINGVALUE(MSG_TOGGLE_MIX                       , _UxGT("Mix umschalten"));
  FSTRINGVALUE(MSG_CYCLE_MIX                        , _UxGT("Zyklus Mix"));
  FSTRINGVALUE(MSG_GRADIENT_MIX                     , _UxGT("Gradient Mix"));
  FSTRINGVALUE(MSG_REVERSE_GRADIENT                 , _UxGT("Umgekehrte Gradient"));
  FSTRINGVALUE(MSG_ACTIVE_VTOOL                     , _UxGT("Aktives V-Tool"));
  FSTRINGVALUE(MSG_START_VTOOL                      , _UxGT("V-Tool Start"));
  FSTRINGVALUE(MSG_END_VTOOL                        , _UxGT("V-Tool Ende"));
  FSTRINGVALUE(MSG_GRADIENT_ALIAS                   , _UxGT("V-Tool Alias"));
  FSTRINGVALUE(MSG_RESET_VTOOLS                     , _UxGT("V-Tools Reseten"));
  FSTRINGVALUE(MSG_COMMIT_VTOOL                     , _UxGT("V-Tool Mix sichern"));
  FSTRINGVALUE(MSG_VTOOLS_RESET                     , _UxGT("V-Tools ist resetet"));
  FSTRINGVALUE(MSG_START_Z                          , _UxGT("Z Start:"));
  FSTRINGVALUE(MSG_END_Z                            , _UxGT("Z Ende:"));
  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Spiele"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Brickout"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Invaders"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Sn4k3"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Maze"));

  //
  // Die Filament-Change-Bildschirme können bis zu 3 Zeilen auf einem 4-Zeilen-Display anzeigen
  //                                       ...oder 2 Zeilen auf einem 3-Zeilen-Display.
  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Knopf drücken um", "Druck fortzusetzen")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_2_LINE("Druck ist", "pausiert...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Warte auf den", "Start des", "Filamentwechsels...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Filament einlegen", "und Knopf drücken", "um fortzusetzen")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Knopf drücken um", "Düse aufzuheizen")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Düse heizt auf", "bitte warten...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_3_LINE("Warte auf", "Herausnahme", "des Filaments...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_3_LINE("Warte auf", "Laden des", "Filaments...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_3_LINE("Warte auf", "Entladen des", "Filaments...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_3_LINE("Klicke um", "die Fila-Entladung", "zu beenden")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_3_LINE("Warte auf", "Fortsetzen des", "Drucks...")));
  #else // LCD_HEIGHT < 4
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Klick zum Fortsetzen")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Pausiert...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Bitte warten...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Laden und Klick")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Klick zum Heizen")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Heizen...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Entladen...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Laden...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Entladen...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_2_LINE("Klick zum beenden", "der Fila-Entladung")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Fortsetzen...")));
  #endif // LCD_HEIGHT < 4

  FSTRINGVALUE(MSG_TMC_DRIVERS                      , _UxGT("TMC Treiber")); // Max length 18 characters
  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Treiber Strom"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Hybrid threshold"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Sensorloses Homing"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Schrittmodus"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("StealthChop einsch."));
  FSTRINGVALUE(MSG_SERVICE_RESET                    , _UxGT("Reset"));
  FSTRINGVALUE(MSG_SERVICE_IN                       , _UxGT(" im:"));
  FSTRINGVALUE(MSG_BACKLASH                         , _UxGT("Spiel"));
  FSTRINGVALUE(MSG_BACKLASH_A                       , LCD_STR_A);
  FSTRINGVALUE(MSG_BACKLASH_B                       , LCD_STR_B);
  FSTRINGVALUE(MSG_BACKLASH_C                       , LCD_STR_C);
  FSTRINGVALUE(MSG_BACKLASH_CORRECTION              , _UxGT("Korrektur"));
  FSTRINGVALUE(MSG_BACKLASH_SMOOTHING               , _UxGT("Glätten"));
}
