/*
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
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

/**
 * Hungarian (Magyar)
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_HU_H
#define LANGUAGE_HU_H

#define MAPPER_C2C3
#define DISPLAY_CHARSET_ISO10646_1 // use the better font on full graphic displays.

#define WELCOME_MSG                         MACHINE_NAME _UxGT(" kész.")
#define MSG_SD                              _UxGT("SD")
#define MSG_SD_INSERTED                     MSG_SD _UxGT(" behelyezve")
#define MSG_SD_REMOVED                      MSG_SD _UxGT(" kivéve")
#define MSG_MAIN                            _UxGT("Alap oldal")
#define MSG_AUTOSTART                       _UxGT("Automatainditás")
#define MSG_DISABLE_STEPPERS                _UxGT("Steppert tilt")
#define MSG_DEBUG_MENU                      _UxGT("Debug menü")
#define MSG_PROGRESS_BAR_TEST               _UxGT("Progress Bar Test")
#define MSG_AUTO_HOME                       _UxGT("Auto otthon")
#define MSG_AUTO_HOME_X                     _UxGT("X haza")
#define MSG_AUTO_HOME_Y                     _UxGT("Y haza")
#define MSG_AUTO_HOME_Z                     _UxGT("Z haza")
#define MSG_LEVEL_BED_HOMING                _UxGT("XYZ haza")
#define MSG_LEVEL_BED_WAITING               _UxGT("Katt a kezdéshez")
#define MSG_LEVEL_BED_NEXT_POINT            _UxGT("Következô pont")
#define MSG_LEVEL_BED_DONE                  _UxGT("Szintezés kész!")
#define MSG_LEVEL_BED_CANCEL                _UxGT("Mégsem")
#define MSG_SET_HOME_OFFSETS                _UxGT("Home offsetet beállít")
#define MSG_HOME_OFFSETS_APPLIED            _UxGT("Offsetek beállítva")
#define MSG_SET_ORIGIN                      _UxGT("Indulás helyet beállít")
#define MSG_PREHEAT                         _UxGT("Melegít")
#define MSG_PREHEAT_1                       MSG_PREHEAT _UxGT(" PLA")
#define MSG_PREHEAT_1_N                     MSG_PREHEAT_1 _UxGT(" ")
#define MSG_PREHEAT_1_ALL                   MSG_PREHEAT_1 _UxGT(" Mind")
#define MSG_PREHEAT_1_BEDONLY               MSG_PREHEAT_1 _UxGT(" Asztal")
#define MSG_PREHEAT_1_SETTINGS              MSG_PREHEAT_1 _UxGT(" conf")
#define MSG_PREHEAT_2                       MSG_PREHEAT _UxGT(" ABS")
#define MSG_PREHEAT_2_N                     MSG_PREHEAT_2 _UxGT(" ")
#define MSG_PREHEAT_2_ALL                   MSG_PREHEAT_2 _UxGT(" Mind")
#define MSG_PREHEAT_2_BEDONLY               MSG_PREHEAT_2 _UxGT(" Asztal")
#define MSG_PREHEAT_2_SETTINGS              MSG_PREHEAT_2 _UxGT(" conf")
#define MSG_PREHEAT_3                       MSG_PREHEAT _UxGT(" GUM")
#define MSG_PREHEAT_3_N                     MSG_PREHEAT_3 _UxGT(" ")
#define MSG_PREHEAT_3_ALL                   MSG_PREHEAT_3 _UxGT(" Mind")
#define MSG_PREHEAT_3_BEDONLY               MSG_PREHEAT_3 _UxGT(" Asztal")
#define MSG_PREHEAT_3_SETTINGS              MSG_PREHEAT_3 _UxGT(" conf")
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     _UxGT("Fej hideg a filament cseréhez")
#define MSG_COOLDOWN                        _UxGT("Lehül")
#define MSG_SWITCH_PS_ON                    _UxGT("Bekapcsol")
#define MSG_SWITCH_PS_OFF                   _UxGT("Kikapcsol")
#define MSG_EXTRUDE                         _UxGT("Extrudál")
#define MSG_RETRACT                         _UxGT("Visszahúz")
#define MSG_PURGE                           _UxGT("Tisztít")
#define MSG_LEVEL_BED                       _UxGT("Asztalt szintez")
#define MSG_SPEED                           _UxGT("Sebesség")
#define MSG_NOZZLE                          _UxGT("Fúvóka")
#define MSG_BED                             _UxGT("Asztal")
#define MSG_CHAMBER                         _UxGT("Kamra")
#define MSG_COOLER                          _UxGT("Hûtô")
#define MSG_BED_Z                           _UxGT("Asztal Z")
#define MSG_FAN_SPEED                       _UxGT("Ventilátor seb.")
#define MSG_FLOW                            _UxGT("Flow")
#define MSG_CONTROL                         _UxGT("Vezérel")
#define MSG_FIX_LOSE_STEPS                  _UxGT("Fix tengely lépések")
#define MSG_MIN                             LCD_STR_THERMOMETER _UxGT(" Min")
#define MSG_MAX                             LCD_STR_THERMOMETER _UxGT(" Max")
#define MSG_FACTOR                          LCD_STR_THERMOMETER _UxGT(" Tényezô")
#define MSG_IDLEOOZING                      _UxGT("Szivárgás ellen")
#define MSG_AUTOTEMP                        _UxGT("Auto hôfok")
#define MSG_ON                              _UxGT("Be ")
#define MSG_OFF                             _UxGT("Ki")
#define MSG_PID_P                           _UxGT("PID-P")
#define MSG_PID_I                           _UxGT("PID-I")
#define MSG_PID_D                           _UxGT("PID-D")
#define MSG_PID_C                           _UxGT("PID-C")
#define MSG_SELECT                          _UxGT("Választ")
#define MSG_ACC                             _UxGT("Gyorsul")
#define MSG_VX_JERK                         _UxGT("Vx-jerk")
#define MSG_VY_JERK                         _UxGT("Vy-jerk")
#define MSG_VZ_JERK                         _UxGT("Vz-jerk")
#define MSG_VE_JERK                         _UxGT("Ve-jerk")
#define MSG_VMAX                            _UxGT("Vmax ")
#define MSG_MOVING                          _UxGT("Mozog...")
#define MSG_FREE_XY                         _UxGT("Szabad XY")
#define MSG_MOVE                            _UxGT("Mozgat")
#define MSG_MOVE_AXIS                       MSG_MOVE _UxGT(" tengely")
#define MSG_MOVE_X                          MSG_MOVE _UxGT(" ") MSG_X
#define MSG_MOVE_Y                          MSG_MOVE _UxGT(" ") MSG_Y
#define MSG_MOVE_Z                          MSG_MOVE _UxGT(" ") MSG_Z
#define MSG_MOVE_01MM                       MSG_MOVE _UxGT(" 0.1mm")
#define MSG_MOVE_1MM                        MSG_MOVE _UxGT(" 1mm")
#define MSG_MOVE_10MM                       MSG_MOVE _UxGT(" 10mm")
#define MSG_MOVE_E                          _UxGT("Extruder")
#define MSG_VMIN                            _UxGT("Vmin")
#define MSG_VTRAV_MIN                       _UxGT("VTrav min")
#define MSG_AMAX                            _UxGT("Amax ")
#define MSG_A_RETRACT                       _UxGT("A-retract")
#define MSG_A_TRAVEL                        _UxGT("A-travel")
#define MSG_XSTEPS                          MSG_X _UxGT(" steps/mm")
#define MSG_YSTEPS                          MSG_Y _UxGT(" steps/mm")
#define MSG_ZSTEPS                          MSG_Z _UxGT(" steps/mm")
#define MSG_E0STEPS                         MSG_E _UxGT("0 steps/mm")
#define MSG_E1STEPS                         MSG_E _UxGT("1 steps/mm")
#define MSG_E2STEPS                         MSG_E _UxGT("2 steps/mm")
#define MSG_E3STEPS                         MSG_E _UxGT("3 steps/mm")
#define MSG_E4STEPS                         MSG_E _UxGT("4 steps/mm")
#define MSG_E5STEPS                         MSG_E _UxGT("5 steps/mm")
#define MSG_TEMPERATURE                     _UxGT("Hômérséklet")
#define MSG_MOTION                          _UxGT("Mozgás")
#define MSG_FILAMENT                        _UxGT("Nyomtatószál")
#define MSG_VOLUMETRIC_ENABLED              MSG_E _UxGT(" in mm3")
#define MSG_FILAMENT_SIZE_EXTRUDER          _UxGT("Fil. Átm.")
#define MSG_CONTRAST                        _UxGT("LCD kontraszt")
#define MSG_STORE_EEPROM                     _UxGT("Tárol memória")
#define MSG_LOAD_EEPROM                      _UxGT("Betölt memória")
#define MSG_RESTORE_FAILSAFE                _UxGT("Üzembiztosra vissza")
#define MSG_REFRESH                         _UxGT("Frissít")
#define MSG_WATCH                           _UxGT("Információs oldal")
#define MSG_PREPARE                         _UxGT("Felkészít")
#define MSG_TUNE                            _UxGT("Hangol")
#define MSG_PAUSE_PRINT                     _UxGT("Nyomtatás szünet")
#define MSG_RESUME_PRINT                    _UxGT("Nyomtatás folytat")
#define MSG_STOP_PRINT                      _UxGT("Nyomtatás megáll")
#define MSG_STOP_SAVE_PRINT                 _UxGT("Ment és megáll")
#define MSG_CARD_MENU                       MSG_SD _UxGT(" nyomtatás") 
#define MSG_NO_CARD                         _UxGT("Nincs ") MSG_SD
#define MSG_DWELL                           _UxGT("Alszik...")
#define MSG_USERWAIT                        _UxGT("Felhasználóra vár...")
#define MSG_RESUMING                        _UxGT("Folytatom a nyomtatást")
#define MSG_PRINT_ABORTED                   _UxGT("Nyomtatás megszakítva")
#define MSG_NO_MOVE                         _UxGT("Nincs mozgás.")
#define MSG_KILLED                          _UxGT("MEGÖLVE. ")
#define MSG_STOPPED                         _UxGT("MEGÁLLÍTVA. ")
#define MSG_CONTROL_RETRACT                 _UxGT("Retract mm")
#define MSG_CONTROL_RETRACT_SWAP            _UxGT("Swap Re. mm")
#define MSG_CONTROL_RETRACTF                _UxGT("Retract  F")
#define MSG_CONTROL_RETRACT_ZLIFT           _UxGT("Hop mm")
#define MSG_CONTROL_RETRACT_RECOVER         _UxGT("UnRet mm")
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    _UxGT("S UnRet mm")
#define MSG_CONTROL_RETRACT_RECOVERF        _UxGT("UnRet  F")
#define MSG_AUTORETRACT                     _UxGT("AutoRetr.")
#define MSG_FILAMENTCHANGE                  _UxGT("Szálat cserél")
#define MSG_INIT_SDCARD                     _UxGT("Beállít. ") MSG_SD
#define MSG_CNG_SDCARD                      _UxGT("Cserél _") MSG_SD
#define MSG_ZPROBE_OUT                      _UxGT("Z próba kint asztalról")
#define MSG_BLTOUCH_SELFTEST                _UxGT("BLTouch ön-teszt")
#define MSG_BLTOUCH_RESET                   _UxGT("Reset BLTouch")
#define MSG_HOME                            _UxGT("Haza")
#define MSG_FIRST                           _UxGT("elsô")
#define MSG_PROBE_OFFSET                  _UxGT("ZProbe ZOffset")
#define MSG_BABYSTEP                        _UxGT("Babystep")
#define MSG_BABYSTEP_X                      MSG_BABYSTEP _UxGT(" ") MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP _UxGT(" ") MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP _UxGT(" ") MSG_Z
#define MSG_ENDSTOP_ABORT                   _UxGT("Endstop leáll")
#define MSG_HEATING_FAILED_LCD              _UxGT("Fûtés sikertelen")
#define MSG_ERR_REDUNDANT_TEMP              _UxGT("REDUNDANT TEMP ERROR")
#define MSG_THERMAL_RUNAWAY                 _UxGT("THERMAL RUNAWAY")
#define MSG_AD595                           _UxGT("AD595 Offset & Gain")
#define MSG_ERR_MAXTEMP                     _UxGT("MAXTEMP ERROR")
#define MSG_ERR_MINTEMP                     _UxGT("MINTEMP ERROR")
#define MSG_ERR_MAXTEMP_BED                 _UxGT("MAXTEMP BED ERROR")
#define MSG_ERR_MINTEMP_BED                 _UxGT("MINTEMP BED ERROR")
#define MSG_ERR_Z_HOMING                    _UxGT("G28 Z Error")
#define MSG_ERR_MAXTEMP_CHAMBER             _UxGT("MAXTEMP CHAMBER ERROR")
#define MSG_ERR_MINTEMP_CHAMBER             _UxGT("MINTEMP CHAMBER ERROR")
#define MSG_ERR_MAXTEMP_COOLER              _UxGT("MAXTEMP COOLER ERROR")
#define MSG_ERR_MINTEMP_COOLER              _UxGT("MINTEMP COOLER ERROR")
#define MSG_HALTED                          _UxGT("PRINTER HALTED")
#define MSG_PLEASE_RESET                    _UxGT("Kérem visszaállítani")
#define MSG_LONG_DAY                         _UxGT("nap")
#define MSG_LONG_HOUR                        _UxGT("óra")
#define MSG_LONG_MINUTE                      _UxGT("perc")
#define MSG_PRINT_TIME                      _UxGT("Nyomtatás ideje ")

// Calibrate Delta
#define MSG_DELTA_CALIBRATE                 _UxGT("Delta Kalibrálás")
#define MSG_DELTA_CALIBRATE_X               _UxGT("Kalibrál ") MSG_X
#define MSG_DELTA_CALIBRATE_Y               _UxGT("Kalibrál ") MSG_Y
#define MSG_DELTA_CALIBRATE_Z               _UxGT("Kalibrál ") MSG_Z
#define MSG_DELTA_CALIBRATE_CENTER          _UxGT("Kalibrál közép")

// Info printers
#define MSG_INFO_MENU                       _UxGT("Nyomtató infók")
#define MSG_INFO_FIRMWARE_MENU              _UxGT("Firmware infó")
#define MSG_INFO_STATS_MENU                 _UxGT("Nyomtató statisztika")
#define MSG_INFO_BOARD_MENU                 _UxGT("Board Infó")
#define MSG_INFO_THERMISTOR_MENU            _UxGT("Thermistors")
#define MSG_INFO_EXTRUDERS                  _UxGT("Extruderek")
#define MSG_INFO_HOTENDS                    _UxGT("Fejek")
#define MSG_INFO_BED                        _UxGT("Asztal")
#define MSG_INFO_CHAMBER                    _UxGT("Kamra")
#define MSG_INFO_COOLER                     _UxGT("Hûtés")
#define MSG_INFO_BAUDRATE                   _UxGT("Baud")
#define MSG_INFO_PROTOCOL                   _UxGT("Protokol")
#define MSG_INFO_TOTAL_PRINTS               _UxGT("Összes nyomtatás")
#define MSG_INFO_FINISHED_PRINTS            _UxGT("Fin. Prints")
#define MSG_INFO_ON_TIME                    _UxGT("Be x")
#define MSG_INFO_PRINT_TIME                 _UxGT("Pr x")
#define MSG_INFO_FILAMENT_USAGE             _UxGT("Fil")
#define MSG_INFO_PWRCONSUMED                _UxGT("PWR")
#define MSG_INFO_MIN_TEMP                   _UxGT("Min Hô")
#define MSG_INFO_MAX_TEMP                   _UxGT("Max Hô")
#define MSG_INFO_PSU                        _UxGT("Tápegység")

// CASE LIGHT
#define MSG_LIGHTS_ON                       _UxGT("Doboz fény be")
#define MSG_LIGHTS_OFF                      _UxGT("Doboz fény ki")

// ADVANCED_PAUSE_FEATURE
#define MSG_FILAMENT_CHANGE_HEADER          _UxGT("CSERÉLJ SZÁLAT")
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   _UxGT("CSERE OPCIÓK:")
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE  _UxGT("Extrudálj többet")
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   _UxGT("Nyomtatást folytat")
#if LCD_HEIGHT >= 4
  #define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("Várok")
  #define MSG_FILAMENT_CHANGE_INIT_2          _UxGT("a szál")
  #define MSG_FILAMENT_CHANGE_INIT_3          _UxGT("cserére")
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("Várok")
  #define MSG_FILAMENT_CHANGE_UNLOAD_2        _UxGT("a szál")
  #define MSG_FILAMENT_CHANGE_UNLOAD_3        _UxGT("kiszedésre")
  #define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("Helyezz be szálat")
  #define MSG_FILAMENT_CHANGE_INSERT_2        _UxGT("és nyomd meg a gombot ")
  #define MSG_FILAMENT_CHANGE_INSERT_3        _UxGT("a folytatáshoz...")
  #define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("Várok a")
  #define MSG_FILAMENT_CHANGE_LOAD_2          _UxGT("szál betöltésre")
  #define MSG_FILAMENT_CHANGE_LOAD_3          _UxGT("")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       _UxGT("Várok a")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_2       _UxGT("szál extrudálásra")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_3       _UxGT("")
  #define MSG_FILAMENT_CHANGE_RESUME_1        _UxGT("Várok a")
  #define MSG_FILAMENT_CHANGE_RESUME_2        _UxGT("nyomtatás")
  #define MSG_FILAMENT_CHANGE_RESUME_3        _UxGT("folytatására")
#else // LCD_HEIGHT < 4
  #define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("Kérlek várj...")
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("Kiadom...")
  #define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("Behelyez és kattint")
  #define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("Betölt...")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       _UxGT("Extrudál...")
  #define MSG_FILAMENT_CHANGE_RESUME_1        _UxGT("Folytat...")
#endif // LCD_HEIGHT < 4

// Heater
#define MSG_HEATING                         _UxGT("Fûtés...")
#define MSG_HEATING_COMPLETE                _UxGT("Fûtés kész.")
#define MSG_BED_HEATING                     _UxGT("Asztal melegít.")
#define MSG_BED_DONE                        _UxGT("Asztal kész.")
#define MSG_CHAMBER_HEATING                 _UxGT("Kamra melegít.")
#define MSG_CHAMBER_DONE                    _UxGT("Chamber done.")
#define MSG_COOLER_COOLING                  _UxGT("Hûtés...")
#define MSG_COOLER_DONE                     _UxGT("Hûtés kész.")

// Extra
#define MSG_LASER                           _UxGT("Lézer elô.beáll.")
#define MSG_CONFIG                          _UxGT("Beálltás")
#define MSG_E_BOWDEN_LENGTH                 MSG_EXTRUDE _UxGT(" ") STRINGIFY(BOWDEN_LENGTH) _UxGT("mm")
#define MSG_R_BOWDEN_LENGTH                 MSG_RETRACT _UxGT(" ") STRINGIFY(BOWDEN_LENGTH) _UxGT("mm")
#define MSG_PURGE_XMM                       MSG_PURGE   _UxGT(" ") STRINGIFY(LCD_PURGE_LENGTH) _UxGT("mm")
#define MSG_RETRACT_XMM                     MSG_RETRACT _UxGT(" ") STRINGIFY(LCD_RETRACT_LENGTH) _UxGT("mm")
#define MSG_SAVED_POS                       _UxGT("Pozíció mentve")
#define MSG_RESTORING_POS                   _UxGT("Pozíció visszaáll")
#define MSG_INVALID_POS_SLOT                _UxGT("Rossz nyílás, össz nyílás: ")

// Rfid module
#define MSG_RFID_SPOOL                      _UxGT("Tekercs E")
#define MSG_RFID_BRAND                      _UxGT("Márka: ")
#define MSG_RFID_TYPE                       _UxGT("Típus: ")
#define MSG_RFID_COLOR                      _UxGT("Szín: ")
#define MSG_RFID_SIZE                       _UxGT("Méret: ")
#define MSG_RFID_TEMP_HOTEND                _UxGT("Fej hômérséklet: ")
#define MSG_RFID_TEMP_BED                   _UxGT("Asztal hômérséklet: ")
#define MSG_RFID_TEMP_USER_HOTEND           _UxGT("Felhasználó fej hőfok: ")
#define MSG_RFID_TEMP_USER_BED              _UxGT("Felhasználó asztal hőfok: ")
#define MSG_RFID_DENSITY                    _UxGT("Tömörség: ")
#define MSG_RFID_SPOOL_LENGHT               _UxGT("Tekercs hossz: ")


#endif // LANGUAGE_HU_H
