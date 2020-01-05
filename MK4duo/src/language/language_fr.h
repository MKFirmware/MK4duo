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
 * French
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace language_fr {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Français"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" prête."));
  FSTRINGVALUE(MSG_YES                              , _UxGT("Oui"));
  FSTRINGVALUE(MSG_NO                               , _UxGT("Non"));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Retour"));
  FSTRINGVALUE(MSG_MEDIA_ABORTING                   , _UxGT("Annulation..."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Média inséré"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Média retiré"));
  FSTRINGVALUE(MSG_MEDIA_RELEASED                   , _UxGT("Média libéré"));
  FSTRINGVALUE(MSG_MEDIA_WAITING                    , _UxGT("Attente média"));
  FSTRINGVALUE(MSG_MEDIA_READ_ERROR                 , _UxGT("Err lecture média"));
  FSTRINGVALUE(MSG_MEDIA_USB_REMOVED                , _UxGT("USB débranché"));
  FSTRINGVALUE(MSG_MEDIA_USB_FAILED                 , _UxGT("Erreur média USB"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Butées"));
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Butées SW"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Menu principal"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Config. avancée"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Configuration"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Exéc. auto#.gcode"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Arrêter moteurs"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Menu debug"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Test barre progress."));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Origine auto"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Origine X auto"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Origine Y auto"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Origine Z auto"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Align. Z auto"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Origine XYZ..."));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Clic pour commencer"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Point suivant"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Mise à niveau OK!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Hauteur lissée"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Régl. décal origine"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Décalages appliqués"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Régler origine"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Préchauffage ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Préchauffage ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Préch. ") PREHEAT_1_LABEL _UxGT(" buse"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Préch. ") PREHEAT_1_LABEL _UxGT(" buse ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Préch. ") PREHEAT_1_LABEL _UxGT(" Tout"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Préch. ") PREHEAT_1_LABEL _UxGT(" lit"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Régler préch. ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Préchauffage ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Préchauffage ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Préch. ") PREHEAT_2_LABEL _UxGT(" buse"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Préch. ") PREHEAT_2_LABEL _UxGT(" buse ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Préch. ") PREHEAT_2_LABEL _UxGT(" Tout"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Préch. ") PREHEAT_2_LABEL _UxGT(" lit"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Régler préch. ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Préchauffage perso"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Refroidir"));
  FSTRINGVALUE(MSG_LASER_MENU                       , _UxGT("Contrôle Laser"));
  FSTRINGVALUE(MSG_LASER_POWER                      , _UxGT("Puissance"));
  FSTRINGVALUE(MSG_SPINDLE_REVERSE                  , _UxGT("Inverser broches"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Allumer alim."));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Eteindre alim."));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrusion"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retrait"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Déplacer un axe"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Régler Niv. lit"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Niveau du lit"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Niveau des coins"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Coin suivant"));
  FSTRINGVALUE(MSG_MESH_EDITOR                      , _UxGT("Edition Maillage"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Modifier maille"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Arrêt édit. maillage"));
  FSTRINGVALUE(MSG_PROBING_MESH                     , _UxGT("Mesure point"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("Index X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("Index Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Valeur Z"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Commandes perso"));

  FSTRINGVALUE(MSG_LCD_TILTING_MESH                 , _UxGT("Touche point"));
  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("Ecart sonde Z M48"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Ecart"));
  FSTRINGVALUE(MSG_M48_POINT                        , _UxGT("Point M48"));
  FSTRINGVALUE(MSG_IDEX_MENU                        , _UxGT("Mode IDEX"));
  FSTRINGVALUE(MSG_IDEX_MODE_AUTOPARK               , _UxGT("Auto-Park"));
  FSTRINGVALUE(MSG_IDEX_MODE_DUPLICATE              , _UxGT("Duplication"));
  FSTRINGVALUE(MSG_IDEX_MODE_MIRRORED_COPY          , _UxGT("Copie miroir"));
  FSTRINGVALUE(MSG_IDEX_MODE_FULL_CTRL              , _UxGT("Contrôle complet"));
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Offsets Outil"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("Buse 2 X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("Buse 2 Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("Buse 2 Z"));
  FSTRINGVALUE(MSG_G26_HEATING_BED                  , _UxGT("G26 Chauffe lit"));
  FSTRINGVALUE(MSG_G26_HEATING_NOZZLE               , _UxGT("G26 Chauffe buse"));
  FSTRINGVALUE(MSG_G26_MANUAL_PRIME                 , _UxGT("Amorce manuelle..."));
  FSTRINGVALUE(MSG_G26_FIXED_LENGTH                 , _UxGT("Amorce longueur fixe"));
  FSTRINGVALUE(MSG_G26_PRIME_DONE                   , _UxGT("Amorce terminée"));
  FSTRINGVALUE(MSG_G26_CANCELED                     , _UxGT("G26 annulé"));
  FSTRINGVALUE(MSG_G26_LEAVING                      , _UxGT("Sortie G26"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("G29 en cours"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("Outils UBL"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Niveau lit unifié"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Maillage manuel"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Poser câle & mesurer"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Mesure"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("ôter et mesurer lit"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Aller au suivant"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Activer l'UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Désactiver l'UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Température lit"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Température lit"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Température buse"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Température buse"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Editer maille"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Editer maille perso"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Réglage fin maille"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Terminer maille"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Créer maille perso"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Créer maille"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Créer maille ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Créer maille ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Créer maille froide"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Ajuster haut. maille"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Hauteur"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Valider maille"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Valider maille ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Valider maille ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Valider maille perso"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Continuer maille"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Niveau par maille"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("Niveau à 3 points"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Niveau grille"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Maille de niveau"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Point latéral"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Type de carte"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Voir maille"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Voir pour hôte"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Voir pour CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Voir pour sauveg."));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Voir info UBL"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Taux de remplissage"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Remplissage manuel"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Remplissage auto"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Maille remplissage"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Tout annuler"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Annuler le plus près"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Réglage fin (tous)"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Réglage fin (proche)"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Stockage maille"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Slot mémoire"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Charger maille"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Sauver maille"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Maille %i chargée"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Maille %i enreg."));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Pas de mémoire"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Err: Enreg. UBL"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Err: Ouvrir UBL"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET                     , _UxGT("Z-Offset: "));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Décal. Z arrêté"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("UBL Pas à pas"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Créer maille froide"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Remplissage auto"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Valider maille"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Réglage fin (tous)"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Valider maille"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Réglage fin (tous)"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Sauver maille"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Contrôle LED"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Lumière"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Préregl. LED"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Rouge"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Orange"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Jaune"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Vert"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Bleu"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Indigo"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Violet"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Blanc"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Defaut"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("LEDs perso."));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Intensité rouge"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Intensité vert"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Intensité bleu"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Intensité blanc"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Luminosité"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Déplacement..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Débloquer XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Déplacer X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Déplacer Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Déplacer Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extrudeur"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extrudeur *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Buse trop froide"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Déplacer %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Déplacer 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Déplacer 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Déplacer 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Vitesse"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Lit Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Buse"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Buse ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Lit"));
  FSTRINGVALUE(MSG_CHAMBER                          , _UxGT("Caisson"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Vitesse ventil."));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Vitesse ventil. ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Extra V ventil."));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Extra V ventil. ="));

  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Flux"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Flux ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Contrôler"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Facteur"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Temp. Auto."));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Marche"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Arrêt"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Sélectionner"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Sélectionner *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Accélération"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Jerk"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("V") LCD_STR_A _UxGT(" jerk"));
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("V") LCD_STR_B _UxGT(" jerk"));
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("V") LCD_STR_C _UxGT(" jerk"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Ve jerk"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Vélocité"));
  FSTRINGVALUE(MSG_JUNCTION_DEVIATION               , _UxGT("Déviat. jonct."));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("V dépl. min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Accélération"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A retrait"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A dépl."));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Pas/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" pas/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" pas/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" pas/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E pas/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* pas/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Température"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Mouvement"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E en mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Diamètre fil."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Diamètre fil. *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Retrait mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Charger mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("Avance K"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("Avance K *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Contraste LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Enregistrer config."));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Charger config."));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Restaurer défauts"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Initialiser EEPROM"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("MaJ Firmware SD"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("RaZ imprimante"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Actualiser"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Surveiller"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Préparer"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Régler"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Démarrer impression"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Suivant"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Init."));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Stop"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Imprimer"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Reset"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Annuler"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Terminé"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Pause impression"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Reprendre impr."));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Arrêter impr."));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Récupér. coupure"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Impression SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Pas de média"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Repos..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Attente utilis."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Impr. en pause"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Impression"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Impr. annulée"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Moteurs bloqués"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("KILLED"));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("STOPPÉ"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retrait mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Ech. Retr. mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retrait V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Saut Z mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("Rappel mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Ech. Rappel mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("Rappel V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("Ech. Rappel V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Retrait auto"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Changement outil"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Augmenter Z"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Vitesse primaire"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Vitesse retrait"));
  FSTRINGVALUE(MSG_NOZZLE_STANDBY                   , _UxGT("Attente buse"));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("Distance retrait"));
  FSTRINGVALUE(MSG_FILAMENT_PURGE_LENGTH            , _UxGT("Longueur de purge"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Changer filament"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Changer filament *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Charger filament"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Charger filament *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Retrait filament *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Décharger tout"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Charger le média"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Actualiser média"));
  FSTRINGVALUE(MSG_RELEASE_MEDIA                    , _UxGT("Retirer le média"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Sonde Z hors lit"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Facteur écart"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Cmd: Self-Test"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Cmd: Reset"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Cmd: Ranger"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Cmd: Déployer"));
  FSTRINGVALUE(MSG_BLTOUCH_SW_MODE                  , _UxGT("Cmd: Mode SW"));
  FSTRINGVALUE(MSG_BLTOUCH_5V_MODE                  , _UxGT("Cmd: Mode 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_OD_MODE                  , _UxGT("Cmd: Mode OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE               , _UxGT("Appliquer Mode"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_5V            , _UxGT("Mise en 5V"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_STORE_OD            , _UxGT("Mise en OD"));
  FSTRINGVALUE(MSG_BLTOUCH_MODE_ECHO                , _UxGT("Afficher Mode"));
  FSTRINGVALUE(MSG_TOUCHMI_PROBE                    , _UxGT("TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_INIT                     , _UxGT("Init. TouchMI"));
  FSTRINGVALUE(MSG_TOUCHMI_ZTEST                    , _UxGT("Test décalage Z"));
  FSTRINGVALUE(MSG_TOUCHMI_SAVE                     , _UxGT("Sauvegarde"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY_TOUCHMI            , _UxGT("Déployer TouchMI"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Déployer Sonde Z"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Ranger Sonde Z"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Origine %s%s%s Premier"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Décalage Z"));
  FSTRINGVALUE(MSG_BABYSTEP_TOTAL                   , _UxGT("Total"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Butée abandon"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Err de chauffe"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Err TEMP. REDONDANTE"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("Err THERMIQUE"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err TEMP. MAX"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err TEMP. MIN"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Origine XY Premier"));

  FSTRINGVALUE(MSG_HALTED                           , _UxGT("IMPR. STOPPÉE"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Redémarrer SVP"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("j")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); // One character only

  FSTRINGVALUE(MSG_HEATING                          , _UxGT("En chauffe..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Refroidissement"));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Lit en chauffe..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Refroid. du lit..."));
  FSTRINGVALUE(MSG_CHAMBER_HEATING                  , _UxGT("Chauffe caisson..."));
  FSTRINGVALUE(MSG_CHAMBER_COOLING                  , _UxGT("Refroid. caisson..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Calibration Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Calibrer X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Calibrer Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Calibrer Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Calibrer centre"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Réglages Delta"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Calibration Auto"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Hauteur Delta"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Delta Z sonde"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Diagonale"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Hauteur"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Rayon"));

  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Infos imprimante"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Infos imprimante"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("Niveau à 3 points"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Niveau linéaire"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Niveau bilinéaire"));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Niveau lit unifié"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Niveau maillage"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Stats. imprimante"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Infos carte"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Thermistances"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extrudeurs"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Bauds"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocole"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Lumière caisson"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Luminosité"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Imprimante incorrecte"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Nbre impressions"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Terminées"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Tps impr. total"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Impr. la + longue"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Total filament"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Impressions"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Terminées"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Total"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("+ long"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Filament"));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Temp Min"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Temp Max"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Alimentation"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Puiss. moteur "));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driver %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("DAC EEPROM sauv."));
  FSTRINGVALUE(MSG_ERROR_TMC                        , _UxGT("ERREUR CONNEXION TMC"));

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER           , _UxGT("CHANGER FILAMENT"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("IMPR. PAUSE"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("CHARGER FIL"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("DECHARGER FIL"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("OPTIONS REPRISE:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Purger encore"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Reprendre impr."));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Buse: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Capteur fil."));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Echec origine"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Echec sonde"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: Trop froid"));

  FSTRINGVALUE(MSG_MMU2_CHOOSE_FILAMENT_HEADER      , _UxGT("CHOISIR FILAMENT"));
  FSTRINGVALUE(MSG_MMU2_MENU                        , _UxGT("MMU"));
  FSTRINGVALUE(MSG_MMU2_NOT_RESPONDING              , _UxGT("MMU ne répond plus"));
  FSTRINGVALUE(MSG_MMU2_RESUME                      , _UxGT("Continuer impr."));
  FSTRINGVALUE(MSG_MMU2_RESUMING                    , _UxGT("Reprise..."));
  FSTRINGVALUE(MSG_MMU2_LOAD_FILAMENT               , _UxGT("Charger filament"));
  FSTRINGVALUE(MSG_MMU2_LOAD_ALL                    , _UxGT("Charger tous"));
  FSTRINGVALUE(MSG_MMU2_LOAD_TO_NOZZLE              , _UxGT("Charger dans buse"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT              , _UxGT("Ejecter filament"));
  FSTRINGVALUE(MSG_MMU2_EJECT_FILAMENT_N            , _UxGT("Ejecter fil. ~"));
  FSTRINGVALUE(MSG_MMU2_UNLOAD_FILAMENT             , _UxGT("Retrait filament"));
  FSTRINGVALUE(MSG_MMU2_LOADING_FILAMENT            , _UxGT("Chargem. fil. %i..."));
  FSTRINGVALUE(MSG_MMU2_EJECTING_FILAMENT           , _UxGT("Ejection fil..."));
  FSTRINGVALUE(MSG_MMU2_UNLOADING_FILAMENT          , _UxGT("Retrait fil...."));
  FSTRINGVALUE(MSG_MMU2_ALL                         , _UxGT("Tous"));
  FSTRINGVALUE(MSG_MMU2_FILAMENT_N                  , _UxGT("Filament ~"));
  FSTRINGVALUE(MSG_MMU2_RESET                       , _UxGT("Réinit. MMU"));
  FSTRINGVALUE(MSG_MMU2_RESETTING                   , _UxGT("Réinit. MMU..."));
  FSTRINGVALUE(MSG_MMU2_EJECT_RECOVER               , _UxGT("Retrait, click"));

  FSTRINGVALUE(MSG_MIX_COMPONENT_N                  , _UxGT("Composante ~"));
  FSTRINGVALUE(MSG_MIXER                            , _UxGT("Mixeur"));
  FSTRINGVALUE(MSG_GRADIENT                         , _UxGT("Dégradé"));
  FSTRINGVALUE(MSG_FULL_GRADIENT                    , _UxGT("Dégradé complet"));
  FSTRINGVALUE(MSG_TOGGLE_MIX                       , _UxGT("Toggle mix"));
  FSTRINGVALUE(MSG_CYCLE_MIX                        , _UxGT("Cycle mix"));
  FSTRINGVALUE(MSG_GRADIENT_MIX                     , _UxGT("Mix dégradé"));
  FSTRINGVALUE(MSG_REVERSE_GRADIENT                 , _UxGT("Inverser dégradé"));
  FSTRINGVALUE(MSG_ACTIVE_VTOOL                     , _UxGT("Active V-tool"));
  FSTRINGVALUE(MSG_START_VTOOL                      , _UxGT("Début V-tool"));
  FSTRINGVALUE(MSG_END_VTOOL                        , _UxGT("  Fin V-tool"));
  FSTRINGVALUE(MSG_GRADIENT_ALIAS                   , _UxGT("Alias V-tool"));
  FSTRINGVALUE(MSG_RESET_VTOOLS                     , _UxGT("Réinit. V-tools"));
  FSTRINGVALUE(MSG_COMMIT_VTOOL                     , _UxGT("Valider Mix V-tool"));
  FSTRINGVALUE(MSG_VTOOLS_RESET                     , _UxGT("V-tools réinit. ok"));
  FSTRINGVALUE(MSG_START_Z                          , _UxGT("Début Z:"));
  FSTRINGVALUE(MSG_END_Z                            , _UxGT("  Fin Z:"));
  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Jeux"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Casse-briques"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Invaders"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Sn4k3"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Labyrinthe"));

  #if LCD_HEIGHT >= 4
    // Up to 3 lines allowed
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Presser bouton", "pour reprendre")));
    FSTRINGVALUE(MSG_PAUSE_PRINT_INIT               , _UxGT(MSG_1_LINE("Parking...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_2_LINE("Attente filament", "pour démarrer")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Insérer filament", "et app. bouton", "pour continuer...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Presser le bouton", "pour chauffer...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Buse en chauffe", "Patienter SVP...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Attente", "retrait du filament")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Attente", "chargement filament")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_2_LINE("Attente", "Purge filament")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_2_LINE("Presser pour finir", "la purge du filament")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Attente reprise", "impression")));
  #else // LCD_HEIGHT < 4
    // Up to 2 lines allowed
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Clic pour continuer")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Patience...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Insérer fil.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Chauffer ?")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Chauffage...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Retrait fil...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Chargement...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Purge...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Terminer ?")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Reprise...")));
  #endif // LCD_HEIGHT < 4

  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Courant driver"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Seuil hybride"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Home sans capteur"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Mode pas à pas"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("StealthChop activé"));
  FSTRINGVALUE(MSG_SERVICE_RESET                    , _UxGT("Réinit."));
  FSTRINGVALUE(MSG_SERVICE_IN                       , _UxGT("  dans:"));
  FSTRINGVALUE(MSG_BACKLASH_CORRECTION              , _UxGT("Correction"));
  FSTRINGVALUE(MSG_BACKLASH_SMOOTHING               , _UxGT("Lissage"));
}
