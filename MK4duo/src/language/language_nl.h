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
 * Dutch
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_1
#define NOT_EXTENDED_ISO10646_1_5X7

namespace language_nl {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 1;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Dutch"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" gereed."));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Terug"));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Kaart ingestoken"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Kaart verwijderd"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Endstops")); // Max length 8 characters
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Hoofdmenu"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Autostart"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Motoren uit"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Debug Menu")); //accepted English terms
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Vooruitgang Test"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Auto home"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Home X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Home Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Home Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Homing XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Klik voor begin"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Volgende Plaats"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Bed level kompl."));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Zet home offsets"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("H offset toegep."));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Nulpunt instellen"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , PREHEAT_1_LABEL _UxGT(" voorverwarmen"));
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , PREHEAT_1_LABEL _UxGT(" voorverw. ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , PREHEAT_1_LABEL _UxGT(" voorverw. Einde"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , PREHEAT_1_LABEL _UxGT(" voorverw. Einde ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , PREHEAT_1_LABEL _UxGT(" voorverw. aan"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , PREHEAT_1_LABEL _UxGT(" voorverw. Bed"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , PREHEAT_1_LABEL _UxGT(" verw. conf"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , PREHEAT_2_LABEL _UxGT(" voorverwarmen"));
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , PREHEAT_2_LABEL _UxGT(" voorverw. ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , PREHEAT_2_LABEL _UxGT(" voorverw. Einde"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , PREHEAT_2_LABEL _UxGT(" voorverw. Einde ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , PREHEAT_2_LABEL _UxGT(" voorverw. aan"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , PREHEAT_2_LABEL _UxGT(" voorverw. Bed"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , PREHEAT_2_LABEL _UxGT(" verw. conf"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Afkoelen"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Stroom aan"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Stroom uit"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Extrude"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Retract"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("As verplaatsen"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Bed Leveling"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Level bed"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Verplaatsen..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Vrij XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Verplaats X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Verplaats Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Verplaats Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Extruder"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Extruder *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Verplaats %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Verplaats 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Verplaats 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Verplaats 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Snelheid"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Bed Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Nozzle"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Nozzle ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Bed"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Fan snelheid"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Fan snelheid ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Flow"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Flow ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Control"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Fact"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Autotemp"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Aan"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Uit"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Selecteer"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Selecteer *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Versn"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Temperatuur"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Beweging"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filament"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("Advance K"));        //accepted english dutch
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("Advance K *")); //accepted english dutch
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Fil. Dia."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Fil. Dia. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD contrast"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Geheugen opslaan"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Geheugen laden"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Noodstop reset"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Ververs"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Info scherm"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Voorbereiden"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Afstellen"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Print pauzeren"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Print hervatten"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Print stoppen"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Print van SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Geen SD kaart"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Slapen..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Wachten..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Print afgebroken"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Geen beweging."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("Afgebroken. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("Gestopt. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Retract mm"));  //accepted English term in Dutch
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Ruil Retract mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Retract  F"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Hop mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Ruil UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet  F"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("AutoRetr."));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Verv. Filament"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Verv. Filament *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Init. SD kaart"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Verv. SD Kaart"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z probe uit. bed"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch Zelf-Test"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Reset BLTouch"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Home %s%s%s Eerst"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Z Offset"));  //accepted English term in Dutch
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Babystap X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Babystap Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Babystap Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Endstop afbr."));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Voorverw. fout"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Redun. temp fout"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("Therm. wegloop"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Err: Max. temp"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Err: Min. temp"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Err: Max.tmp bed"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Err: Min.tmp bed"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Home XY Eerst"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("PRINTER GESTOPT"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Reset A.U.B."));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("d")); //  One character only. Keep English standard
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("h")); //  One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("m")); //  One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Voorwarmen..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Bed voorverw..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta Calibratie"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Kalibreer X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Kalibreer Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Kalibreer Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Kalibreer Midden"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Auto Calibratie"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Zet Delta Hoogte"));

  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Printer Stats"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Board Info")); //accepted English term in Dutch
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Thermistors"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Extruders"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Baud"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Over Printer"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Printer Info"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protocol"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Case licht"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Onjuiste printer"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Printed Aantal"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Totaal Voltooid"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Totale Printtijd"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Langste Printtijd"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Totaal Extrudeert"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Aantal"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Voltooid"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Printtijd "));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Langste"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Extrud."));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Min Temp"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Max Temp"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("PSU"));  //accepted English term in Dutch

  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Motorstroom"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Driver %"));  //accepted English term in Dutch
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("DAC Opslaan"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Hervat print"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT(" Nozzle: ")); //accepeted English term
  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    // Up to 3 lines
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Wacht voor start", "filament te", "verwisselen")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_3_LINE("Wacht voor", "filament uit", "te laden")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Klik knop om...", "verw. nozzle."))); //nozzle accepted English term
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Nozzle verw.", "Wacht a.u.b.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Laad filament", "en druk knop", "om verder...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_3_LINE("Wacht voor", "filament te", "laden")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_3_LINE("Wacht voor print", "om verder", "te gaan")));
  #else
    // Up to 2 lines
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_2_LINE("Wacht voor", "start...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Wacht voor", "uitladen...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Klik knop om...", "verw. nozzle."))); //nozzle accepted English term
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Verwarmen...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_2_LINE("Laad filament", "en druk knop")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Wacht voor", "inladen...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Wacht voor", "printing...")));
  #endif
}
