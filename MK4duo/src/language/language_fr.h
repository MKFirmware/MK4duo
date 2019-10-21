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
 * French
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1

namespace Language_fr {
  using namespace Language_en; // Inherit undefined strings from English

  constexpr uint8_t    CHARSIZE                            = 2;
  PROGMEM Language_Str LANGUAGE                            = _UxGT("Fran�ais");

  PROGMEM Language_Str WELCOME_MSG                         = MACHINE_NAME _UxGT(" pr�te.");
  PROGMEM Language_Str MSG_YES                             = _UxGT("Oui");
  PROGMEM Language_Str MSG_NO                              = _UxGT("Non");
  PROGMEM Language_Str MSG_BACK                            = _UxGT("Retour");
  PROGMEM Language_Str MSG_MEDIA_ABORTING                  = _UxGT("Annulation...");
  PROGMEM Language_Str MSG_MEDIA_INSERTED                  = _UxGT("M�dia ins�r�");
  PROGMEM Language_Str MSG_MEDIA_REMOVED                   = _UxGT("M�dia retir�");
  PROGMEM Language_Str MSG_MEDIA_RELEASED                  = _UxGT("M�dia lib�r�");
  PROGMEM Language_Str MSG_MEDIA_WAITING                   = _UxGT("Attente m�dia");
  PROGMEM Language_Str MSG_MEDIA_READ_ERROR                = _UxGT("Err lecture m�dia");
  PROGMEM Language_Str MSG_MEDIA_USB_REMOVED               = _UxGT("USB d�branch�");
  PROGMEM Language_Str MSG_MEDIA_USB_FAILED                = _UxGT("Erreur m�dia USB");
  PROGMEM Language_Str MSG_LCD_ENDSTOPS                    = _UxGT("But�es");
  PROGMEM Language_Str MSG_LCD_SOFT_ENDSTOPS               = _UxGT("But�es SW");
  PROGMEM Language_Str MSG_MAIN                            = _UxGT("Menu principal");
  PROGMEM Language_Str MSG_ADVANCED_SETTINGS               = _UxGT("Config. avanc�e");
  PROGMEM Language_Str MSG_CONFIGURATION                   = _UxGT("Configuration");
  PROGMEM Language_Str MSG_AUTOSTART                       = _UxGT("Ex�c. auto#.gcode");
  PROGMEM Language_Str MSG_DISABLE_STEPPERS                = _UxGT("Arr�ter moteurs");
  PROGMEM Language_Str MSG_DEBUG_MENU                      = _UxGT("Menu debug");
  PROGMEM Language_Str MSG_PROGRESS_BAR_TEST               = _UxGT("Test barre progress.");
  PROGMEM Language_Str MSG_AUTO_HOME                       = _UxGT("Origine auto");
  PROGMEM Language_Str MSG_AUTO_HOME_X                     = _UxGT("Origine X auto");
  PROGMEM Language_Str MSG_AUTO_HOME_Y                     = _UxGT("Origine Y auto");
  PROGMEM Language_Str MSG_AUTO_HOME_Z                     = _UxGT("Origine Z auto");
  PROGMEM Language_Str MSG_AUTO_Z_ALIGN                    = _UxGT("Align. Z auto");
  PROGMEM Language_Str MSG_LEVEL_BED_HOMING                = _UxGT("Origine XYZ...");
  PROGMEM Language_Str MSG_LEVEL_BED_WAITING               = _UxGT("Clic pour commencer");
  PROGMEM Language_Str MSG_LEVEL_BED_NEXT_POINT            = _UxGT("Point suivant");
  PROGMEM Language_Str MSG_LEVEL_BED_DONE                  = _UxGT("Mise � niveau OK!");
  PROGMEM Language_Str MSG_Z_FADE_HEIGHT                   = _UxGT("Hauteur liss�e");
  PROGMEM Language_Str MSG_SET_HOME_OFFSETS                = _UxGT("R�gl. d�cal origine");
  PROGMEM Language_Str MSG_HOME_OFFSETS_APPLIED            = _UxGT("D�calages appliqu�s");
  PROGMEM Language_Str MSG_SET_ORIGIN                      = _UxGT("R�gler origine");
  PROGMEM Language_Str MSG_PREHEAT_1                       = _UxGT("Pr�chauffage ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_1_H                     = _UxGT("Pr�chauffage ") PREHEAT_1_LABEL " ~";
  PROGMEM Language_Str MSG_PREHEAT_1_END                   = _UxGT("Pr�ch. ") PREHEAT_1_LABEL _UxGT(" buse");
  PROGMEM Language_Str MSG_PREHEAT_1_END_E                 = _UxGT("Pr�ch. ") PREHEAT_1_LABEL _UxGT(" buse ~");
  PROGMEM Language_Str MSG_PREHEAT_1_ALL                   = _UxGT("Pr�ch. ") PREHEAT_1_LABEL _UxGT(" Tout");
  PROGMEM Language_Str MSG_PREHEAT_1_BEDONLY               = _UxGT("Pr�ch. ") PREHEAT_1_LABEL _UxGT(" lit");
  PROGMEM Language_Str MSG_PREHEAT_1_SETTINGS              = _UxGT("R�gler pr�ch. ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_2                       = _UxGT("Pr�chauffage ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_2_H                     = _UxGT("Pr�chauffage ") PREHEAT_2_LABEL " ~";
  PROGMEM Language_Str MSG_PREHEAT_2_END                   = _UxGT("Pr�ch. ") PREHEAT_2_LABEL _UxGT(" buse");
  PROGMEM Language_Str MSG_PREHEAT_2_END_E                 = _UxGT("Pr�ch. ") PREHEAT_2_LABEL _UxGT(" buse ~");
  PROGMEM Language_Str MSG_PREHEAT_2_ALL                   = _UxGT("Pr�ch. ") PREHEAT_2_LABEL _UxGT(" Tout");
  PROGMEM Language_Str MSG_PREHEAT_2_BEDONLY               = _UxGT("Pr�ch. ") PREHEAT_2_LABEL _UxGT(" lit");
  PROGMEM Language_Str MSG_PREHEAT_2_SETTINGS              = _UxGT("R�gler pr�ch. ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_PREHEAT_CUSTOM                  = _UxGT("Pr�chauffage perso");
  PROGMEM Language_Str MSG_COOLDOWN                        = _UxGT("Refroidir");
  PROGMEM Language_Str MSG_LASER_MENU                      = _UxGT("Contr�le Laser");
  PROGMEM Language_Str MSG_LASER_POWER                     = _UxGT("Puissance");
  PROGMEM Language_Str MSG_SPINDLE_REVERSE                 = _UxGT("Inverser broches");
  PROGMEM Language_Str MSG_SWITCH_PS_ON                    = _UxGT("Allumer alim.");
  PROGMEM Language_Str MSG_SWITCH_PS_OFF                   = _UxGT("Eteindre alim.");
  PROGMEM Language_Str MSG_EXTRUDE                         = _UxGT("Extrusion");
  PROGMEM Language_Str MSG_RETRACT                         = _UxGT("Retrait");
  PROGMEM Language_Str MSG_MOVE_AXIS                       = _UxGT("D�placer un axe");
  PROGMEM Language_Str MSG_BED_LEVELING                    = _UxGT("R�gler Niv. lit");
  PROGMEM Language_Str MSG_LEVEL_BED                       = _UxGT("Niveau du lit");
  PROGMEM Language_Str MSG_LEVEL_CORNERS                   = _UxGT("Niveau des coins");
  PROGMEM Language_Str MSG_NEXT_CORNER                     = _UxGT("Coin suivant");
  PROGMEM Language_Str MSG_MESH_EDITOR                     = _UxGT("Edition Maillage");
  PROGMEM Language_Str MSG_EDIT_MESH                       = _UxGT("Modifier maille");
  PROGMEM Language_Str MSG_EDITING_STOPPED                 = _UxGT("Arr�t �dit. maillage");
  PROGMEM Language_Str MSG_PROBING_MESH                    = _UxGT("Mesure point");
  PROGMEM Language_Str MSG_MESH_X                          = _UxGT("Index X");
  PROGMEM Language_Str MSG_MESH_Y                          = _UxGT("Index Y");
  PROGMEM Language_Str MSG_MESH_EDIT_Z                     = _UxGT("Valeur Z");
  PROGMEM Language_Str MSG_USER_MENU                       = _UxGT("Commandes perso");

  PROGMEM Language_Str MSG_LCD_TILTING_MESH                = _UxGT("Touche point");
  PROGMEM Language_Str MSG_M48_TEST                        = _UxGT("Ecart sonde Z M48");
  PROGMEM Language_Str MSG_M48_DEVIATION                   = _UxGT("Ecart");
  PROGMEM Language_Str MSG_M48_POINT                       = _UxGT("Point M48");
  PROGMEM Language_Str MSG_IDEX_MENU                       = _UxGT("Mode IDEX");
  PROGMEM Language_Str MSG_IDEX_MODE_AUTOPARK              = _UxGT("Auto-Park");
  PROGMEM Language_Str MSG_IDEX_MODE_DUPLICATE             = _UxGT("Duplication");
  PROGMEM Language_Str MSG_IDEX_MODE_MIRRORED_COPY         = _UxGT("Copie miroir");
  PROGMEM Language_Str MSG_IDEX_MODE_FULL_CTRL             = _UxGT("Contr�le complet");
  PROGMEM Language_Str MSG_OFFSETS_MENU                    = _UxGT("Offsets Outil");
  PROGMEM Language_Str MSG_X_OFFSET                        = _UxGT("Buse 2 X");
  PROGMEM Language_Str MSG_Y_OFFSET                        = _UxGT("Buse 2 Y");
  PROGMEM Language_Str MSG_Z_OFFSET                        = _UxGT("Buse 2 Z");
  PROGMEM Language_Str MSG_G26_HEATING_BED                 = _UxGT("G26 Chauffe lit");
  PROGMEM Language_Str MSG_G26_HEATING_NOZZLE              = _UxGT("G26 Chauffe buse");
  PROGMEM Language_Str MSG_G26_MANUAL_PRIME                = _UxGT("Amorce manuelle...");
  PROGMEM Language_Str MSG_G26_FIXED_LENGTH                = _UxGT("Amorce longueur fixe");
  PROGMEM Language_Str MSG_G26_PRIME_DONE                  = _UxGT("Amorce termin�e");
  PROGMEM Language_Str MSG_G26_CANCELED                    = _UxGT("G26 annul�");
  PROGMEM Language_Str MSG_G26_LEAVING                     = _UxGT("Sortie G26");
  PROGMEM Language_Str MSG_UBL_DOING_G29                   = _UxGT("G29 en cours");
  PROGMEM Language_Str MSG_UBL_TOOLS                       = _UxGT("Outils UBL");
  PROGMEM Language_Str MSG_UBL_LEVEL_BED                   = _UxGT("Niveau lit unifi�");
  PROGMEM Language_Str MSG_UBL_MANUAL_MESH                 = _UxGT("Maillage manuel");
  PROGMEM Language_Str MSG_UBL_BC_INSERT                   = _UxGT("Poser c�le & mesurer");
  PROGMEM Language_Str MSG_UBL_BC_INSERT2                  = _UxGT("Mesure");
  PROGMEM Language_Str MSG_UBL_BC_REMOVE                   = _UxGT("�ter et mesurer lit");
  PROGMEM Language_Str MSG_UBL_MOVING_TO_NEXT              = _UxGT("Aller au suivant");
  PROGMEM Language_Str MSG_UBL_ACTIVATE_MESH               = _UxGT("Activer l'UBL");
  PROGMEM Language_Str MSG_UBL_DEACTIVATE_MESH             = _UxGT("D�sactiver l'UBL");
  PROGMEM Language_Str MSG_UBL_SET_TEMP_BED                = _UxGT("Temp�rature lit");
  PROGMEM Language_Str MSG_UBL_BED_TEMP_CUSTOM             = _UxGT("Temp�rature lit");
  PROGMEM Language_Str MSG_UBL_SET_TEMP_HOTEND             = _UxGT("Temp�rature buse");
  PROGMEM Language_Str MSG_UBL_HOTEND_TEMP_CUSTOM          = _UxGT("Temp�rature buse");
  PROGMEM Language_Str MSG_UBL_MESH_EDIT                   = _UxGT("Editer maille");
  PROGMEM Language_Str MSG_UBL_EDIT_CUSTOM_MESH            = _UxGT("Editer maille perso");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_MESH              = _UxGT("R�glage fin maille");
  PROGMEM Language_Str MSG_UBL_DONE_EDITING_MESH           = _UxGT("Terminer maille");
  PROGMEM Language_Str MSG_UBL_BUILD_CUSTOM_MESH           = _UxGT("Cr�er maille perso");
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_MENU             = _UxGT("Cr�er maille");
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_M1               = _UxGT("Cr�er maille ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_UBL_BUILD_MESH_M2               = _UxGT("Cr�er maille ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_UBL_BUILD_COLD_MESH             = _UxGT("Cr�er maille froide");
  PROGMEM Language_Str MSG_UBL_MESH_HEIGHT_ADJUST          = _UxGT("Ajuster haut. maille");
  PROGMEM Language_Str MSG_UBL_MESH_HEIGHT_AMOUNT          = _UxGT("Hauteur");
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_MENU          = _UxGT("Valider maille");
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_M1            = _UxGT("Valider maille ") PREHEAT_1_LABEL;
  PROGMEM Language_Str MSG_UBL_VALIDATE_MESH_M2            = _UxGT("Valider maille ") PREHEAT_2_LABEL;
  PROGMEM Language_Str MSG_UBL_VALIDATE_CUSTOM_MESH        = _UxGT("Valider maille perso");
  PROGMEM Language_Str MSG_UBL_CONTINUE_MESH               = _UxGT("Continuer maille");
  PROGMEM Language_Str MSG_UBL_MESH_LEVELING               = _UxGT("Niveau par maille");
  PROGMEM Language_Str MSG_UBL_3POINT_MESH_LEVELING        = _UxGT("Niveau � 3 points");
  PROGMEM Language_Str MSG_UBL_GRID_MESH_LEVELING          = _UxGT("Niveau grille");
  PROGMEM Language_Str MSG_UBL_MESH_LEVEL                  = _UxGT("Maille de niveau");
  PROGMEM Language_Str MSG_UBL_SIDE_POINTS                 = _UxGT("Point lat�ral");
  PROGMEM Language_Str MSG_UBL_MAP_TYPE                    = _UxGT("Type de carte");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP                  = _UxGT("Voir maille");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_HOST             = _UxGT("Voir pour h�te");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_CSV              = _UxGT("Voir pour CSV");
  PROGMEM Language_Str MSG_UBL_OUTPUT_MAP_BACKUP           = _UxGT("Voir pour sauveg.");
  PROGMEM Language_Str MSG_UBL_INFO_UBL                    = _UxGT("Voir info UBL");
  PROGMEM Language_Str MSG_UBL_FILLIN_AMOUNT               = _UxGT("Taux de remplissage");
  PROGMEM Language_Str MSG_UBL_MANUAL_FILLIN               = _UxGT("Remplissage manuel");
  PROGMEM Language_Str MSG_UBL_SMART_FILLIN                = _UxGT("Remplissage auto");
  PROGMEM Language_Str MSG_UBL_FILLIN_MESH                 = _UxGT("Maille remplissage");
  PROGMEM Language_Str MSG_UBL_INVALIDATE_ALL              = _UxGT("Tout annuler");
  PROGMEM Language_Str MSG_UBL_INVALIDATE_CLOSEST          = _UxGT("Annuler le plus pr�s");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_ALL               = _UxGT("R�glage fin (tous)");
  PROGMEM Language_Str MSG_UBL_FINE_TUNE_CLOSEST           = _UxGT("R�glage fin (proche)");
  PROGMEM Language_Str MSG_UBL_STORAGE_MESH_MENU           = _UxGT("Stockage maille");
  PROGMEM Language_Str MSG_UBL_STORAGE_SLOT                = _UxGT("Slot m�moire");
  PROGMEM Language_Str MSG_UBL_LOAD_MESH                   = _UxGT("Charger maille");
  PROGMEM Language_Str MSG_UBL_SAVE_MESH                   = _UxGT("Sauver maille");
  PROGMEM Language_Str MSG_MESH_LOADED                     = _UxGT("M117 Maille %i charg�e");
  PROGMEM Language_Str MSG_MESH_SAVED                      = _UxGT("M117 Maille %i enreg.");
  PROGMEM Language_Str MSG_UBL_NO_STORAGE                  = _UxGT("Pas de m�moire");
  PROGMEM Language_Str MSG_UBL_SAVE_ERROR                  = _UxGT("Err: Enreg. UBL");
  PROGMEM Language_Str MSG_UBL_RESTORE_ERROR               = _UxGT("Err: Ouvrir UBL");
  PROGMEM Language_Str MSG_UBL_Z_OFFSET                    = _UxGT("Z-Offset: ");
  PROGMEM Language_Str MSG_UBL_Z_OFFSET_STOPPED            = _UxGT("D�cal. Z arr�t�");
  PROGMEM Language_Str MSG_UBL_STEP_BY_STEP_MENU           = _UxGT("UBL Pas � pas");
  PROGMEM Language_Str MSG_UBL_1_BUILD_COLD_MESH           = _UxGT("1.Cr�er maille froide");
  PROGMEM Language_Str MSG_UBL_2_SMART_FILLIN              = _UxGT("2.Remplissage auto");
  PROGMEM Language_Str MSG_UBL_3_VALIDATE_MESH_MENU        = _UxGT("3.Valider maille");
  PROGMEM Language_Str MSG_UBL_4_FINE_TUNE_ALL             = _UxGT("4.R�glage fin (tous)");
  PROGMEM Language_Str MSG_UBL_5_VALIDATE_MESH_MENU        = _UxGT("5.Valider maille");
  PROGMEM Language_Str MSG_UBL_6_FINE_TUNE_ALL             = _UxGT("6.R�glage fin (tous)");
  PROGMEM Language_Str MSG_UBL_7_SAVE_MESH                 = _UxGT("7.Sauver maille");

  PROGMEM Language_Str MSG_LED_CONTROL                     = _UxGT("Contr�le LED");
  PROGMEM Language_Str MSG_LEDS                            = _UxGT("Lumi�re");
  PROGMEM Language_Str MSG_LED_PRESETS                     = _UxGT("Pr�regl. LED");
  PROGMEM Language_Str MSG_SET_LEDS_RED                    = _UxGT("Rouge");
  PROGMEM Language_Str MSG_SET_LEDS_ORANGE                 = _UxGT("Orange");
  PROGMEM Language_Str MSG_SET_LEDS_YELLOW                 = _UxGT("Jaune");
  PROGMEM Language_Str MSG_SET_LEDS_GREEN                  = _UxGT("Vert");
  PROGMEM Language_Str MSG_SET_LEDS_BLUE                   = _UxGT("Bleu");
  PROGMEM Language_Str MSG_SET_LEDS_INDIGO                 = _UxGT("Indigo");
  PROGMEM Language_Str MSG_SET_LEDS_VIOLET                 = _UxGT("Violet");
  PROGMEM Language_Str MSG_SET_LEDS_WHITE                  = _UxGT("Blanc");
  PROGMEM Language_Str MSG_SET_LEDS_DEFAULT                = _UxGT("Defaut");
  PROGMEM Language_Str MSG_CUSTOM_LEDS                     = _UxGT("LEDs perso.");
  PROGMEM Language_Str MSG_INTENSITY_R                     = _UxGT("Intensit� rouge");
  PROGMEM Language_Str MSG_INTENSITY_G                     = _UxGT("Intensit� vert");
  PROGMEM Language_Str MSG_INTENSITY_B                     = _UxGT("Intensit� bleu");
  PROGMEM Language_Str MSG_INTENSITY_W                     = _UxGT("Intensit� blanc");
  PROGMEM Language_Str MSG_LED_BRIGHTNESS                  = _UxGT("Luminosit�");

  PROGMEM Language_Str MSG_MOVING                          = _UxGT("D�placement...");
  PROGMEM Language_Str MSG_FREE_XY                         = _UxGT("D�bloquer XY");
  PROGMEM Language_Str MSG_MOVE_X                          = _UxGT("D�placer X");
  PROGMEM Language_Str MSG_MOVE_Y                          = _UxGT("D�placer Y");
  PROGMEM Language_Str MSG_MOVE_Z                          = _UxGT("D�placer Z");
  PROGMEM Language_Str MSG_MOVE_E                          = _UxGT("Extrudeur");
  PROGMEM Language_Str MSG_MOVE_EN                         = _UxGT("Extrudeur *");
  PROGMEM Language_Str MSG_HOTEND_TOO_COLD                 = _UxGT("Buse trop froide");
  PROGMEM Language_Str MSG_MOVE_Z_DIST                     = _UxGT("D�placer %smm");
  PROGMEM Language_Str MSG_MOVE_01MM                       = _UxGT("D�placer 0.1mm");
  PROGMEM Language_Str MSG_MOVE_1MM                        = _UxGT("D�placer 1mm");
  PROGMEM Language_Str MSG_MOVE_10MM                       = _UxGT("D�placer 10mm");
  PROGMEM Language_Str MSG_SPEED                           = _UxGT("Vitesse");
  PROGMEM Language_Str MSG_BED_Z                           = _UxGT("Lit Z");
  PROGMEM Language_Str MSG_NOZZLE                          = _UxGT("Buse");
  PROGMEM Language_Str MSG_NOZZLE_N                        = _UxGT("Buse ~");
  PROGMEM Language_Str MSG_BED                             = _UxGT("Lit");
  PROGMEM Language_Str MSG_CHAMBER                         = _UxGT("Caisson");
  PROGMEM Language_Str MSG_FAN_SPEED                       = _UxGT("Vitesse ventil.");
  PROGMEM Language_Str MSG_FAN_SPEED_N                     = _UxGT("Vitesse ventil. =");
  PROGMEM Language_Str MSG_EXTRA_FAN_SPEED                 = _UxGT("Extra V ventil.");
  PROGMEM Language_Str MSG_EXTRA_FAN_SPEED_N               = _UxGT("Extra V ventil. =");

  PROGMEM Language_Str MSG_FLOW                            = _UxGT("Flux");
  PROGMEM Language_Str MSG_FLOW_N                          = _UxGT("Flux ~");
  PROGMEM Language_Str MSG_CONTROL                         = _UxGT("Contr�ler");
  PROGMEM Language_Str MSG_MIN                             = " " LCD_STR_THERMOMETER _UxGT(" Min");
  PROGMEM Language_Str MSG_MAX                             = " " LCD_STR_THERMOMETER _UxGT(" Max");
  PROGMEM Language_Str MSG_FACTOR                          = " " LCD_STR_THERMOMETER _UxGT(" Facteur");
  PROGMEM Language_Str MSG_AUTOTEMP                        = _UxGT("Temp. Auto.");
  PROGMEM Language_Str MSG_LCD_ON                          = _UxGT("Marche");
  PROGMEM Language_Str MSG_LCD_OFF                         = _UxGT("Arr�t");
  PROGMEM Language_Str MSG_SELECT                          = _UxGT("S�lectionner");
  PROGMEM Language_Str MSG_SELECT_E                        = _UxGT("S�lectionner *");
  PROGMEM Language_Str MSG_ACC                             = _UxGT("Acc�l�ration");
  PROGMEM Language_Str MSG_JERK                            = _UxGT("Jerk");
  PROGMEM Language_Str MSG_VA_JERK                         = _UxGT("V") LCD_STR_A _UxGT(" jerk");
  PROGMEM Language_Str MSG_VB_JERK                         = _UxGT("V") LCD_STR_B _UxGT(" jerk");
  PROGMEM Language_Str MSG_VC_JERK                         = _UxGT("V") LCD_STR_C _UxGT(" jerk");
  PROGMEM Language_Str MSG_VE_JERK                         = _UxGT("Ve jerk");
  PROGMEM Language_Str MSG_VELOCITY                        = _UxGT("V�locit�");
  PROGMEM Language_Str MSG_JUNCTION_DEVIATION              = _UxGT("D�viat. jonct.");
  PROGMEM Language_Str MSG_VTRAV_MIN                       = _UxGT("V d�pl. min");
  PROGMEM Language_Str MSG_ACCELERATION                    = _UxGT("Acc�l�ration");
  PROGMEM Language_Str MSG_A_RETRACT                       = _UxGT("A retrait");
  PROGMEM Language_Str MSG_A_TRAVEL                        = _UxGT("A d�pl.");
  PROGMEM Language_Str MSG_STEPS_PER_MM                    = _UxGT("Pas/mm");
  PROGMEM Language_Str MSG_A_STEPS                         = LCD_STR_A _UxGT(" pas/mm");
  PROGMEM Language_Str MSG_B_STEPS                         = LCD_STR_B _UxGT(" pas/mm");
  PROGMEM Language_Str MSG_C_STEPS                         = LCD_STR_C _UxGT(" pas/mm");
  PROGMEM Language_Str MSG_E_STEPS                         = _UxGT("E pas/mm");
  PROGMEM Language_Str MSG_EN_STEPS                        = _UxGT("* pas/mm");
  PROGMEM Language_Str MSG_TEMPERATURE                     = _UxGT("Temp�rature");
  PROGMEM Language_Str MSG_MOTION                          = _UxGT("Mouvement");
  PROGMEM Language_Str MSG_FILAMENT                        = _UxGT("Filament");
  PROGMEM Language_Str MSG_VOLUMETRIC_ENABLED              = _UxGT("E en mm3");
  PROGMEM Language_Str MSG_FILAMENT_DIAM                   = _UxGT("Diam�tre fil.");
  PROGMEM Language_Str MSG_FILAMENT_DIAM_E                 = _UxGT("Diam�tre fil. *");
  PROGMEM Language_Str MSG_FILAMENT_UNLOAD                 = _UxGT("Retrait mm");
  PROGMEM Language_Str MSG_FILAMENT_LOAD                   = _UxGT("Charger mm");
  PROGMEM Language_Str MSG_ADVANCE_K                       = _UxGT("Avance K");
  PROGMEM Language_Str MSG_ADVANCE_K_E                     = _UxGT("Avance K *");
  PROGMEM Language_Str MSG_CONTRAST                        = _UxGT("Contraste LCD");
  PROGMEM Language_Str MSG_STORE_EEPROM                    = _UxGT("Enregistrer config.");
  PROGMEM Language_Str MSG_LOAD_EEPROM                     = _UxGT("Charger config.");
  PROGMEM Language_Str MSG_RESTORE_FAILSAFE                = _UxGT("Restaurer d�fauts");
  PROGMEM Language_Str MSG_INIT_EEPROM                     = _UxGT("Initialiser EEPROM");
  PROGMEM Language_Str MSG_MEDIA_UPDATE                    = _UxGT("MaJ Firmware SD");
  PROGMEM Language_Str MSG_RESET_PRINTER                   = _UxGT("RaZ imprimante");
  PROGMEM Language_Str MSG_REFRESH                         = LCD_STR_REFRESH  _UxGT("Actualiser");
  PROGMEM Language_Str MSG_WATCH                           = _UxGT("Surveiller");
  PROGMEM Language_Str MSG_PREPARE                         = _UxGT("Pr�parer");
  PROGMEM Language_Str MSG_TUNE                            = _UxGT("R�gler");
  PROGMEM Language_Str MSG_START_PRINT                     = _UxGT("D�marrer impression");
  PROGMEM Language_Str MSG_BUTTON_NEXT                     = _UxGT("Suivant");
  PROGMEM Language_Str MSG_BUTTON_INIT                     = _UxGT("Init.");
  PROGMEM Language_Str MSG_BUTTON_STOP                     = _UxGT("Stop");
  PROGMEM Language_Str MSG_BUTTON_PRINT                    = _UxGT("Imprimer");
  PROGMEM Language_Str MSG_BUTTON_RESET                    = _UxGT("Reset");
  PROGMEM Language_Str MSG_BUTTON_CANCEL                   = _UxGT("Annuler");
  PROGMEM Language_Str MSG_BUTTON_DONE                     = _UxGT("Termin�");
  PROGMEM Language_Str MSG_PAUSE_PRINT                     = _UxGT("Pause impression");
  PROGMEM Language_Str MSG_RESUME_PRINT                    = _UxGT("Reprendre impr.");
  PROGMEM Language_Str MSG_STOP_PRINT                      = _UxGT("Arr�ter impr.");
  PROGMEM Language_Str MSG_OUTAGE_RECOVERY                 = _UxGT("R�cup�r. coupure");
  PROGMEM Language_Str MSG_MEDIA_MENU                      = _UxGT("Impression SD");
  PROGMEM Language_Str MSG_NO_MEDIA                        = _UxGT("Pas de m�dia");
  PROGMEM Language_Str MSG_DWELL                           = _UxGT("Repos...");
  PROGMEM Language_Str MSG_USERWAIT                        = _UxGT("Attente utilis.");
  PROGMEM Language_Str MSG_PRINT_PAUSED                    = _UxGT("Impr. en pause");
  PROGMEM Language_Str MSG_PRINTING                        = _UxGT("Impression");
  PROGMEM Language_Str MSG_PRINT_ABORTED                   = _UxGT("Impr. annul�e");
  PROGMEM Language_Str MSG_NO_MOVE                         = _UxGT("Moteurs bloqu�s");
  PROGMEM Language_Str MSG_KILLED                          = _UxGT("KILLED");
  PROGMEM Language_Str MSG_STOPPED                         = _UxGT("STOPP�");
  PROGMEM Language_Str MSG_CONTROL_RETRACT                 = _UxGT("Retrait mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_SWAP            = _UxGT("Ech. Retr. mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACTF                = _UxGT("Retrait V");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_ZHOP            = _UxGT("Saut Z mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER         = _UxGT("Rappel mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAP    = _UxGT("Ech. Rappel mm");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVERF        = _UxGT("Rappel V");
  PROGMEM Language_Str MSG_CONTROL_RETRACT_RECOVER_SWAPF   = _UxGT("Ech. Rappel V");
  PROGMEM Language_Str MSG_AUTORETRACT                     = _UxGT("Retrait auto");
  PROGMEM Language_Str MSG_TOOL_CHANGE                     = _UxGT("Changement outil");
  PROGMEM Language_Str MSG_TOOL_CHANGE_ZLIFT               = _UxGT("Augmenter Z");
  PROGMEM Language_Str MSG_SINGLENOZZLE_PRIME_SPD          = _UxGT("Vitesse primaire");
  PROGMEM Language_Str MSG_SINGLENOZZLE_RETRACT_SPD        = _UxGT("Vitesse retrait");
  PROGMEM Language_Str MSG_NOZZLE_STANDBY                  = _UxGT("Attente buse");
  PROGMEM Language_Str MSG_FILAMENT_SWAP_LENGTH            = _UxGT("Distance retrait");
  PROGMEM Language_Str MSG_FILAMENT_PURGE_LENGTH           = _UxGT("Longueur de purge");
  PROGMEM Language_Str MSG_FILAMENTCHANGE                  = _UxGT("Changer filament");
  PROGMEM Language_Str MSG_FILAMENTCHANGE_E                = _UxGT("Changer filament *");
  PROGMEM Language_Str MSG_FILAMENTLOAD                    = _UxGT("Charger filament");
  PROGMEM Language_Str MSG_FILAMENTLOAD_E                  = _UxGT("Charger filament *");
  PROGMEM Language_Str MSG_FILAMENTUNLOAD_E                = _UxGT("Retrait filament *");
  PROGMEM Language_Str MSG_FILAMENTUNLOAD_ALL              = _UxGT("D�charger tout");
  PROGMEM Language_Str MSG_INIT_MEDIA                      = _UxGT("Charger le m�dia");
  PROGMEM Language_Str MSG_CHANGE_MEDIA                    = _UxGT("Actualiser m�dia");
  PROGMEM Language_Str MSG_RELEASE_MEDIA                   = _UxGT("Retirer le m�dia");
  PROGMEM Language_Str MSG_ZPROBE_OUT                      = _UxGT("Sonde Z hors lit");
  PROGMEM Language_Str MSG_SKEW_FACTOR                     = _UxGT("Facteur �cart");
  PROGMEM Language_Str MSG_BLTOUCH                         = _UxGT("BLTouch");
  PROGMEM Language_Str MSG_BLTOUCH_SELFTEST                = _UxGT("Cmd: Self-Test");
  PROGMEM Language_Str MSG_BLTOUCH_RESET                   = _UxGT("Cmd: Reset");
  PROGMEM Language_Str MSG_BLTOUCH_STOW                    = _UxGT("Cmd: Ranger");
  PROGMEM Language_Str MSG_BLTOUCH_DEPLOY                  = _UxGT("Cmd: D�ployer");
  PROGMEM Language_Str MSG_BLTOUCH_SW_MODE                 = _UxGT("Cmd: Mode SW");
  PROGMEM Language_Str MSG_BLTOUCH_5V_MODE                 = _UxGT("Cmd: Mode 5V");
  PROGMEM Language_Str MSG_BLTOUCH_OD_MODE                 = _UxGT("Cmd: Mode OD");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE              = _UxGT("Appliquer Mode");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE_5V           = _UxGT("Mise en 5V");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_STORE_OD           = _UxGT("Mise en OD");
  PROGMEM Language_Str MSG_BLTOUCH_MODE_ECHO               = _UxGT("Afficher Mode");
  PROGMEM Language_Str MSG_TOUCHMI_PROBE                   = _UxGT("TouchMI");
  PROGMEM Language_Str MSG_TOUCHMI_INIT                    = _UxGT("Init. TouchMI");
  PROGMEM Language_Str MSG_TOUCHMI_ZTEST                   = _UxGT("Test d�calage Z");
  PROGMEM Language_Str MSG_TOUCHMI_SAVE                    = _UxGT("Sauvegarde");
  PROGMEM Language_Str MSG_MANUAL_DEPLOY_TOUCHMI           = _UxGT("D�ployer TouchMI");
  PROGMEM Language_Str MSG_MANUAL_DEPLOY                   = _UxGT("D�ployer Sonde Z");
  PROGMEM Language_Str MSG_MANUAL_STOW                     = _UxGT("Ranger Sonde Z");
  PROGMEM Language_Str MSG_HOME_FIRST                      = _UxGT("Origine %s%s%s Premier");
  PROGMEM Language_Str MSG_ZPROBE_ZOFFSET                  = _UxGT("D�calage Z");
  PROGMEM Language_Str MSG_BABYSTEP_TOTAL                  = _UxGT("Total");
  PROGMEM Language_Str MSG_ENDSTOP_ABORT                   = _UxGT("But�e abandon");
  PROGMEM Language_Str MSG_HEATING_FAILED_LCD              = _UxGT("Err de chauffe");
  PROGMEM Language_Str MSG_ERR_REDUNDANT_TEMP              = _UxGT("Err TEMP. REDONDANTE");
  PROGMEM Language_Str MSG_THERMAL_RUNAWAY                 = _UxGT("Err THERMIQUE");
  PROGMEM Language_Str MSG_ERR_MAXTEMP                     = _UxGT("Err TEMP. MAX");
  PROGMEM Language_Str MSG_ERR_MINTEMP                     = _UxGT("Err TEMP. MIN");
  PROGMEM Language_Str MSG_ERR_Z_HOMING                    = _UxGT("Origine XY Premier");

  PROGMEM Language_Str MSG_HALTED                          = _UxGT("IMPR. STOPP�E");
  PROGMEM Language_Str MSG_PLEASE_RESET                    = _UxGT("Red�marrer SVP");
  PROGMEM Language_Str MSG_SHORT_DAY                       = _UxGT("j"); // One character only
  PROGMEM Language_Str MSG_SHORT_HOUR                      = _UxGT("h"); // One character only
  PROGMEM Language_Str MSG_SHORT_MINUTE                    = _UxGT("m"); // One character only

  PROGMEM Language_Str MSG_HEATING                         = _UxGT("En chauffe...");
  PROGMEM Language_Str MSG_COOLING                         = _UxGT("Refroidissement");
  PROGMEM Language_Str MSG_BED_HEATING                     = _UxGT("Lit en chauffe...");
  PROGMEM Language_Str MSG_BED_COOLING                     = _UxGT("Refroid. du lit...");
  PROGMEM Language_Str MSG_CHAMBER_HEATING                 = _UxGT("Chauffe caisson...");
  PROGMEM Language_Str MSG_CHAMBER_COOLING                 = _UxGT("Refroid. caisson...");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE                 = _UxGT("Calibration Delta");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_X               = _UxGT("Calibrer X");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_Y               = _UxGT("Calibrer Y");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_Z               = _UxGT("Calibrer Z");
  PROGMEM Language_Str MSG_DELTA_CALIBRATE_CENTER          = _UxGT("Calibrer centre");
  PROGMEM Language_Str MSG_DELTA_SETTINGS                  = _UxGT("R�glages Delta");
  PROGMEM Language_Str MSG_DELTA_AUTO_CALIBRATE            = _UxGT("Calibration Auto");
  PROGMEM Language_Str MSG_DELTA_HEIGHT_CALIBRATE          = _UxGT("Hauteur Delta");
  PROGMEM Language_Str MSG_DELTA_Z_OFFSET_CALIBRATE        = _UxGT("Delta Z sonde");
  PROGMEM Language_Str MSG_DELTA_DIAG_ROD                  = _UxGT("Diagonale");
  PROGMEM Language_Str MSG_DELTA_HEIGHT                    = _UxGT("Hauteur");
  PROGMEM Language_Str MSG_DELTA_RADIUS                    = _UxGT("Rayon");

  PROGMEM Language_Str MSG_INFO_MENU                       = _UxGT("Infos imprimante");
  PROGMEM Language_Str MSG_INFO_PRINTER_MENU               = _UxGT("Infos imprimante");
  PROGMEM Language_Str MSG_3POINT_LEVELING                 = _UxGT("Niveau � 3 points");
  PROGMEM Language_Str MSG_LINEAR_LEVELING                 = _UxGT("Niveau lin�aire");
  PROGMEM Language_Str MSG_BILINEAR_LEVELING               = _UxGT("Niveau bilin�aire");
  PROGMEM Language_Str MSG_UBL_LEVELING                    = _UxGT("Niveau lit unifi�");
  PROGMEM Language_Str MSG_MESH_LEVELING                   = _UxGT("Niveau maillage");
  PROGMEM Language_Str MSG_INFO_STATS_MENU                 = _UxGT("Stats. imprimante");
  PROGMEM Language_Str MSG_INFO_BOARD_MENU                 = _UxGT("Infos carte");
  PROGMEM Language_Str MSG_INFO_THERMISTOR_MENU            = _UxGT("Thermistances");
  PROGMEM Language_Str MSG_INFO_EXTRUDERS                  = _UxGT("Extrudeurs");
  PROGMEM Language_Str MSG_INFO_BAUDRATE                   = _UxGT("Bauds");
  PROGMEM Language_Str MSG_INFO_PROTOCOL                   = _UxGT("Protocole");
  PROGMEM Language_Str MSG_CASE_LIGHT                      = _UxGT("Lumi�re caisson");
  PROGMEM Language_Str MSG_CASE_LIGHT_BRIGHTNESS           = _UxGT("Luminosit�");

  PROGMEM Language_Str MSG_EXPECTED_PRINTER                = _UxGT("Imprimante incorrecte");

  #if LCD_WIDTH >= 20
    PROGMEM Language_Str MSG_INFO_PRINT_COUNT              = _UxGT("Nbre impressions");
    PROGMEM Language_Str MSG_INFO_COMPLETED_PRINTS         = _UxGT("Termin�es");
    PROGMEM Language_Str MSG_INFO_PRINT_TIME               = _UxGT("Tps impr. total");
    PROGMEM Language_Str MSG_INFO_PRINT_LONGEST            = _UxGT("Impr. la + longue");
    PROGMEM Language_Str MSG_INFO_PRINT_FILAMENT           = _UxGT("Total filament");
  #else
    PROGMEM Language_Str MSG_INFO_PRINT_COUNT              = _UxGT("Impressions");
    PROGMEM Language_Str MSG_INFO_COMPLETED_PRINTS         = _UxGT("Termin�es");
    PROGMEM Language_Str MSG_INFO_PRINT_TIME               = _UxGT("Total");
    PROGMEM Language_Str MSG_INFO_PRINT_LONGEST            = _UxGT("+ long");
    PROGMEM Language_Str MSG_INFO_PRINT_FILAMENT           = _UxGT("Filament");
  #endif

  PROGMEM Language_Str MSG_INFO_MIN_TEMP                   = _UxGT("Temp Min");
  PROGMEM Language_Str MSG_INFO_MAX_TEMP                   = _UxGT("Temp Max");
  PROGMEM Language_Str MSG_INFO_PSU                        = _UxGT("Alimentation");
  PROGMEM Language_Str MSG_DRIVE_STRENGTH                  = _UxGT("Puiss. moteur ");
  PROGMEM Language_Str MSG_DAC_PERCENT                     = _UxGT("Driver %");
  PROGMEM Language_Str MSG_DAC_EEPROM_WRITE                = _UxGT("DAC EEPROM sauv.");
  PROGMEM Language_Str MSG_ERROR_TMC                       = _UxGT("ERREUR CONNEXION TMC");

  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER          = _UxGT("CHANGER FILAMENT");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_PAUSE    = _UxGT("IMPR. PAUSE");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_LOAD     = _UxGT("CHARGER FIL");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEADER_UNLOAD   = _UxGT("DECHARGER FIL");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_HEADER   = _UxGT("OPTIONS REPRISE:");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_PURGE    = _UxGT("Purger encore");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_OPTION_RESUME   = _UxGT("Reprendre impr.");
  PROGMEM Language_Str MSG_FILAMENT_CHANGE_NOZZLE          = _UxGT("  Buse: ");
  PROGMEM Language_Str MSG_RUNOUT_SENSOR                   = _UxGT("Capteur fil.");
  PROGMEM Language_Str MSG_LCD_HOMING_FAILED               = _UxGT("Echec origine");
  PROGMEM Language_Str MSG_LCD_PROBING_FAILED              = _UxGT("Echec sonde");
  PROGMEM Language_Str MSG_M600_TOO_COLD                   = _UxGT("M600: Trop froid");

  PROGMEM Language_Str MSG_MMU2_CHOOSE_FILAMENT_HEADER     = _UxGT("CHOISIR FILAMENT");
  PROGMEM Language_Str MSG_MMU2_MENU                       = _UxGT("MMU");
  PROGMEM Language_Str MSG_MMU2_NOT_RESPONDING             = _UxGT("MMU ne r�pond plus");
  PROGMEM Language_Str MSG_MMU2_RESUME                     = _UxGT("Continuer impr.");
  PROGMEM Language_Str MSG_MMU2_RESUMING                   = _UxGT("Reprise...");
  PROGMEM Language_Str MSG_MMU2_LOAD_FILAMENT              = _UxGT("Charger filament");
  PROGMEM Language_Str MSG_MMU2_LOAD_ALL                   = _UxGT("Charger tous");
  PROGMEM Language_Str MSG_MMU2_LOAD_TO_NOZZLE             = _UxGT("Charger dans buse");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT             = _UxGT("Ejecter filament");
  PROGMEM Language_Str MSG_MMU2_EJECT_FILAMENT_N           = _UxGT("Ejecter fil. ~");
  PROGMEM Language_Str MSG_MMU2_UNLOAD_FILAMENT            = _UxGT("Retrait filament");
  PROGMEM Language_Str MSG_MMU2_LOADING_FILAMENT           = _UxGT("Chargem. fil. %i...");
  PROGMEM Language_Str MSG_MMU2_EJECTING_FILAMENT          = _UxGT("Ejection fil...");
  PROGMEM Language_Str MSG_MMU2_UNLOADING_FILAMENT         = _UxGT("Retrait fil....");
  PROGMEM Language_Str MSG_MMU2_ALL                        = _UxGT("Tous");
  PROGMEM Language_Str MSG_MMU2_FILAMENT_N                 = _UxGT("Filament ~");
  PROGMEM Language_Str MSG_MMU2_RESET                      = _UxGT("R�init. MMU");
  PROGMEM Language_Str MSG_MMU2_RESETTING                  = _UxGT("R�init. MMU...");
  PROGMEM Language_Str MSG_MMU2_EJECT_RECOVER              = _UxGT("Retrait, click");

  PROGMEM Language_Str MSG_MIX_COMPONENT_N                 = _UxGT("Composante ~");
  PROGMEM Language_Str MSG_MIXER                           = _UxGT("Mixeur");
  PROGMEM Language_Str MSG_GRADIENT                        = _UxGT("D�grad�");
  PROGMEM Language_Str MSG_FULL_GRADIENT                   = _UxGT("D�grad� complet");
  PROGMEM Language_Str MSG_TOGGLE_MIX                      = _UxGT("Toggle mix");
  PROGMEM Language_Str MSG_CYCLE_MIX                       = _UxGT("Cycle mix");
  PROGMEM Language_Str MSG_GRADIENT_MIX                    = _UxGT("Mix d�grad�");
  PROGMEM Language_Str MSG_REVERSE_GRADIENT                = _UxGT("Inverser d�grad�");
  PROGMEM Language_Str MSG_ACTIVE_VTOOL                    = _UxGT("Active V-tool");
  PROGMEM Language_Str MSG_START_VTOOL                     = _UxGT("D�but V-tool");
  PROGMEM Language_Str MSG_END_VTOOL                       = _UxGT("  Fin V-tool");
  PROGMEM Language_Str MSG_GRADIENT_ALIAS                  = _UxGT("Alias V-tool");
  PROGMEM Language_Str MSG_RESET_VTOOLS                    = _UxGT("R�init. V-tools");
  PROGMEM Language_Str MSG_COMMIT_VTOOL                    = _UxGT("Valider Mix V-tool");
  PROGMEM Language_Str MSG_VTOOLS_RESET                    = _UxGT("V-tools r�init. ok");
  PROGMEM Language_Str MSG_START_Z                         = _UxGT("D�but Z:");
  PROGMEM Language_Str MSG_END_Z                           = _UxGT("  Fin Z:");
  PROGMEM Language_Str MSG_GAMES                           = _UxGT("Jeux");
  PROGMEM Language_Str MSG_BRICKOUT                        = _UxGT("Casse-briques");
  PROGMEM Language_Str MSG_INVADERS                        = _UxGT("Invaders");
  PROGMEM Language_Str MSG_SNAKE                           = _UxGT("Sn4k3");
  PROGMEM Language_Str MSG_MAZE                            = _UxGT("Labyrinthe");

  #if LCD_HEIGHT >= 4
    // Up to 3 lines allowed
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING        = _UxGT(MSG_2_LINE("Presser bouton", "pour reprendre"));
    PROGMEM Language_Str MSG_PAUSE_PRINT_INIT              = _UxGT(MSG_1_LINE("Parking..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT          = _UxGT(MSG_2_LINE("Attente filament", "pour d�marrer"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT        = _UxGT(MSG_3_LINE("Ins�rer filament", "et app. bouton", "pour continuer..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT          = _UxGT(MSG_2_LINE("Presser le bouton", "pour chauffer..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING       = _UxGT(MSG_2_LINE("Buse en chauffe", "Patienter SVP..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD        = _UxGT(MSG_2_LINE("Attente", "retrait du filament"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD          = _UxGT(MSG_2_LINE("Attente", "chargement filament"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE         = _UxGT(MSG_2_LINE("Attente", "Purge filament"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE    = _UxGT(MSG_2_LINE("Presser pour finir", "la purge du filament"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME        = _UxGT(MSG_2_LINE("Attente reprise", "impression"));
  #else // LCD_HEIGHT < 4
    // Up to 2 lines allowed
    PROGMEM Language_Str MSG_ADVANCED_PAUSE_WAITING        = _UxGT(MSG_1_LINE("Clic pour continuer"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INIT          = _UxGT(MSG_1_LINE("Patience..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_INSERT        = _UxGT(MSG_1_LINE("Ins�rer fil."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEAT          = _UxGT(MSG_1_LINE("Chauffer ?"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_HEATING       = _UxGT(MSG_1_LINE("Chauffage..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_UNLOAD        = _UxGT(MSG_1_LINE("Retrait fil..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_LOAD          = _UxGT(MSG_1_LINE("Chargement..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_PURGE         = _UxGT(MSG_1_LINE("Purge..."));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_CONT_PURGE    = _UxGT(MSG_1_LINE("Terminer ?"));
    PROGMEM Language_Str MSG_FILAMENT_CHANGE_RESUME        = _UxGT(MSG_1_LINE("Reprise..."));
  #endif // LCD_HEIGHT < 4

  PROGMEM Language_Str MSG_TMC_CURRENT                     = _UxGT("Courant driver");
  PROGMEM Language_Str MSG_TMC_HYBRID_THRS                 = _UxGT("Seuil hybride");
  PROGMEM Language_Str MSG_TMC_HOMING_THRS                 = _UxGT("Home sans capteur");
  PROGMEM Language_Str MSG_TMC_STEPPING_MODE               = _UxGT("Mode pas � pas");
  PROGMEM Language_Str MSG_TMC_STEALTH_ENABLED             = _UxGT("StealthChop activ�");
  PROGMEM Language_Str MSG_SERVICE_RESET                   = _UxGT("R�init.");
  PROGMEM Language_Str MSG_SERVICE_IN                      = _UxGT("  dans:");
  PROGMEM Language_Str MSG_BACKLASH_CORRECTION             = _UxGT("Correction");
  PROGMEM Language_Str MSG_BACKLASH_SMOOTHING              = _UxGT("Lissage");
}
