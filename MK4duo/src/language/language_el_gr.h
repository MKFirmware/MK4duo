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
 * Greek (Greece)
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_GREEK

namespace language_el_gr {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Greek (Greece)"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" έτοιμο."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Εισαγωγή κάρτας"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Αφαίρεση κάρτας"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstops")); // Max length 8 characters
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Βασική Οθόνη"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Αυτόματη εκκίνηση"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Απενεργοποίηση βηματιστή"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Αυτομ. επαναφορά στο αρχικό σημείο"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Αρχικό σημείο X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Αρχικό σημείο Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Αρχικό σημείο Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Επαναφορά στο αρχικό σημείο ΧΥΖ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Κάντε κλικ για να ξεκινήσετε"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Επόμενο σημείο"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Ολοκλήρωση επιπεδοποίησης!"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Ορισμός βασικών μετατοπίσεων"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Εφαρμόστηκαν οι μετατοπίσεις"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Ορισμός προέλευσης"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Προθέρμανση ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Προθέρμανση ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" End"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" End ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" όλα"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" κλίνη"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Προθέρμανση ") PREHEAT_1_LABEL _UxGT(" επιβεβαίωση"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Προθέρμανση ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Προθέρμανση ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Προθέρμανση ") PREHEAT_2_LABEL _UxGT(" End"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Προθέρμανση ") PREHEAT_2_LABEL _UxGT(" End ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Προθέρμανση ") PREHEAT_2_LABEL _UxGT(" όλα"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Προθέρμανση ") PREHEAT_2_LABEL _UxGT(" κλίνη"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Προθέρμανση ") PREHEAT_2_LABEL _UxGT(" επιβεβαίωση"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Μειωση θερμοκρασιας"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Ενεργοποίηση"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Απενεργοποίηση"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Εξώθηση"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Ανάσυρση"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Μετακίνηση άξονα"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Επιπεδοποίηση κλίνης"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Επιπεδοποίηση κλίνης"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Μετακίνηση X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Μετακίνηση Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Μετακίνηση Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Εξωθητήρας"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Εξωθητήρας *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Μετακίνηση %s μμ"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Μετακίνηση 0,1 μμ"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Μετακίνηση 1 μμ"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Μετακίνηση 10 μμ"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Ταχύτητα"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Κλίνη Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Ακροφύσιο"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Ακροφύσιο ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Κλίνη"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Ταχύτητα ανεμιστήρα"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Ταχύτητα ανεμιστήρα ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Ροή"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Ροή ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Έλεγχος"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fact"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Αυτομ. ρύθμιση θερμοκρασίας"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Ενεργοποιημένο"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Απενεργοποιημένο"));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID-P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID-I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID-D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID-C *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Επιτάχυνση"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Vαντίδραση"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("Vαντίδραση ") LCD_STR_A);
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("Vαντίδραση ") LCD_STR_B);
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("Vαντίδραση ") LCD_STR_C);
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Vαντίδραση E"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("Vμεγ ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("Vμεγ ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("Vμεγ ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vμεγ ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vμεγ *"));
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("Vελαχ"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("Vελάχ. μετατόπιση"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Accel"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Aμεγ ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Aμεγ ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Aμεγ ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Aμεγ ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Aμεγ *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("Α-ανάσυρση"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("Α-μετατόπιση"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Bήματα ανά μμ"));
  FSTRINGVALUE(MSG_A_STEPS                          , _UxGT("Bήματα ") LCD_STR_A _UxGT(" ανά μμ"));
  FSTRINGVALUE(MSG_B_STEPS                          , _UxGT("Bήματα ") LCD_STR_B _UxGT(" ανά μμ"));
  FSTRINGVALUE(MSG_C_STEPS                          , _UxGT("Bήματα ") LCD_STR_C _UxGT(" ανά μμ"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("Bήματα Ε ανά μμ"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("Bήματα * ανά μμ"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Θερμοκρασία"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Κίνηση"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Νήμα"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("Ε σε μμ3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Διάμετρος νήματος"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Διάμετρος νήματος *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Κοντράστ LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Αποθήκευση"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Φόρτωση"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Επαναφορά ασφαλούς αντιγράφου"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Ανανέωση"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Οθόνη πληροφόρησης"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Προετοιμασία"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Συντονισμός"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Παύση εκτύπωσης"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Συνέχιση εκτύπωσης"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Διακοπή εκτύπωσης"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Εκτύπωση από SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Δεν βρέθηκε SD"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Αναστολή λειτουργίας…"));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Αναμονή για χρήστη…"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Διακόπτεται η εκτύπωση"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Καμία κίνηση."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("ΤΕΡΜΑΤΙΣΜΟΣ. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ΔΙΑΚΟΠΗ. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Ανάσυρση μμ"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Εναλλαγή ανάσυρσης μμ"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Ανάσυρση V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Μεταπήδηση μμ"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("S UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet  V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Αυτόματη ανάσυρση"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Αλλαγή νήματος"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Αλλαγή νήματος *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Προετοιμασία κάρτας SD"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Αλλαγή κάρτας SD"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Διερεύνηση Z εκτός κλίνης"));
  FSTRINGVALUE(MSG_YX_UNHOMED                       , _UxGT("Επαναφορά Χ/Υ πριν από Ζ"));
  FSTRINGVALUE(MSG_XYZ_UNHOMED                      , _UxGT("Επαναφορά ΧΥΖ πρώτα"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Μετατόπιση Ζ"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Μικρό βήμα Χ"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Μικρό βήμα Υ"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Μικρό βήμα Ζ"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Ματαίωση endstop "));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Ανεπιτυχής θέρμανση"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Λάθος: ΠΛΕΟΝΑΖΟΥΣΑ ΘΕΡΜΟΤΗΤΑ"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("ΔΙΑΦΥΓΗ ΘΕΡΜΟΤΗΤΑΣ"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Λάθος: ΜΕΓΙΣΤΗ ΘΕΡΜΟΤΗΤΑ"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Λάθος: ΕΛΑΧΙΣΤΗ ΘΕΡΜΟΤΗΤΑ"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Λάθος: ΜΕΓΙΣΤΗ ΘΕΡΜΟΤΗΤΑ ΚΛΙΝΗΣ"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Λάθος: ΕΛΑΧΙΣΤΗ ΘΕΡΜΟΤΗΤΑ ΚΛΙΝΗΣ"));
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Θερμαίνεται…"));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Θέρμανση κλίνης…"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Βαθμονόμηση Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Βαθμονόμηση X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Βαθμονόμηση Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Βαθμονόμηση Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Βαθμονόμηση κέντρου"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Εσφαλμένος εκτυπωτής"));
}
