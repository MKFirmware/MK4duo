/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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
 * English
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_EN_H
#define LANGUAGE_EN_H

#define WELCOME_MSG                         MACHINE_NAME _UxGT(" ready.")
#define MSG_SD                              _UxGT("SD")
#define MSG_SD_INSERTED                     MSG_SD _UxGT(" inserted")
#define MSG_SD_REMOVED                      MSG_SD _UxGT(" removed")
#define MSG_MAIN                            _UxGT("Main")
#define MSG_AUTOSTART                       _UxGT("Autostart")
#define MSG_DISABLE_STEPPERS                _UxGT("Disable steppers")
#define MSG_DEBUG_MENU                      _UxGT("Debug Menu")
#define MSG_PROGRESS_BAR_TEST               _UxGT("Progress Bar Test")
#define MSG_AUTO_HOME                       _UxGT("Auto home")
#define MSG_AUTO_HOME_X                     _UxGT("Home X")
#define MSG_AUTO_HOME_Y                     _UxGT("Home Y")
#define MSG_AUTO_HOME_Z                     _UxGT("Home Z")
#define MSG_LEVEL_BED_HOMING                _UxGT("Homing XYZ")
#define MSG_LEVEL_BED_WAITING               _UxGT("Click to Begin")
#define MSG_LEVEL_BED_NEXT_POINT            _UxGT("Next Point")
#define MSG_LEVEL_BED_DONE                  _UxGT("Leveling Done!")
#define MSG_LEVEL_BED_CANCEL                _UxGT("Cancel")
#define MSG_SET_HOME_OFFSETS                _UxGT("Set home offsets")
#define MSG_HOME_OFFSETS_APPLIED            _UxGT("Offsets applied")
#define MSG_SET_ORIGIN                      _UxGT("Set origin")
#define MSG_PREHEAT                         _UxGT("Preheat")
#define MSG_PREHEAT_1                       MSG_PREHEAT _UxGT(" PLA")
#define MSG_PREHEAT_1_N                     MSG_PREHEAT_1 _UxGT(" ")
#define MSG_PREHEAT_1_ALL                   MSG_PREHEAT_1 _UxGT(" All")
#define MSG_PREHEAT_1_BEDONLY               MSG_PREHEAT_1 _UxGT(" Bed")
#define MSG_PREHEAT_1_SETTINGS              MSG_PREHEAT_1 _UxGT(" conf")
#define MSG_PREHEAT_2                       MSG_PREHEAT _UxGT(" ABS")
#define MSG_PREHEAT_2_N                     MSG_PREHEAT_2 _UxGT(" ")
#define MSG_PREHEAT_2_ALL                   MSG_PREHEAT_2 _UxGT(" All")
#define MSG_PREHEAT_2_BEDONLY               MSG_PREHEAT_2 _UxGT(" Bed")
#define MSG_PREHEAT_2_SETTINGS              MSG_PREHEAT_2 _UxGT(" conf")
#define MSG_PREHEAT_3                       MSG_PREHEAT _UxGT(" GUM")
#define MSG_PREHEAT_3_N                     MSG_PREHEAT_3 _UxGT(" ")
#define MSG_PREHEAT_3_ALL                   MSG_PREHEAT_3 _UxGT(" All")
#define MSG_PREHEAT_3_BEDONLY               MSG_PREHEAT_3 _UxGT(" Bed")
#define MSG_PREHEAT_3_SETTINGS              MSG_PREHEAT_3 _UxGT(" conf")
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     _UxGT("Hotend too cold to change filament")
#define MSG_COOLDOWN                        _UxGT("Cooldown")
#define MSG_SWITCH_PS_ON                    _UxGT("Switch power on")
#define MSG_SWITCH_PS_OFF                   _UxGT("Switch power off")
#define MSG_EXTRUDE                         _UxGT("Extrude")
#define MSG_RETRACT                         _UxGT("Retract")
#define MSG_PURGE                           _UxGT("Purge")
#define MSG_LEVEL_BED                       _UxGT("Level bed")
#define MSG_SPEED                           _UxGT("Speed")
#define MSG_NOZZLE                          _UxGT("Nozzle")
#define MSG_BED                             _UxGT("Bed")
#define MSG_CHAMBER                         _UxGT("Chamber")
#define MSG_COOLER                          _UxGT("Cooler")
#define MSG_BED_Z                           _UxGT("Bed Z")
#define MSG_FAN_SPEED                       _UxGT("Fan speed")
#define MSG_FLOW                            _UxGT("Flow")
#define MSG_CONTROL                         _UxGT("Control")
#define MSG_FIX_LOSE_STEPS                  _UxGT("Fix axis steps")
#define MSG_MIN                             LCD_STR_THERMOMETER _UxGT(" Min")
#define MSG_MAX                             LCD_STR_THERMOMETER _UxGT(" Max")
#define MSG_FACTOR                          LCD_STR_THERMOMETER _UxGT(" Fact")
#define MSG_IDLEOOZING                      _UxGT("Anti oozing")
#define MSG_AUTOTEMP                        _UxGT("Autotemp")
#define MSG_ON                              _UxGT("On ")
#define MSG_OFF                             _UxGT("Off")
#define MSG_PID_P                           _UxGT("PID-P")
#define MSG_PID_I                           _UxGT("PID-I")
#define MSG_PID_D                           _UxGT("PID-D")
#define MSG_PID_C                           _UxGT("PID-C")
#define MSG_SELECT                          _UxGT("Select")
#define MSG_ACC                             _UxGT("Accel")
#define MSG_VX_JERK                         _UxGT("Vx-jerk")
#define MSG_VY_JERK                         _UxGT("Vy-jerk")
#define MSG_VZ_JERK                         _UxGT("Vz-jerk")
#define MSG_VE_JERK                         _UxGT("Ve-jerk")
#define MSG_VMAX                            _UxGT("Vmax ")
#define MSG_MOVING                          _UxGT("Moving...")
#define MSG_FREE_XY                         _UxGT("Free XY")
#define MSG_MOVE                            _UxGT("Move")
#define MSG_MOVE_AXIS                       MSG_MOVE _UxGT(" axis")
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
#define MSG_TEMPERATURE                     _UxGT("Temperature")
#define MSG_MOTION                          _UxGT("Motion")
#define MSG_FILAMENT                        _UxGT("Filament")
#define MSG_VOLUMETRIC_ENABLED              MSG_E _UxGT(" in mm3")
#define MSG_FILAMENT_SIZE_EXTRUDER          _UxGT("Fil. Dia.")
#define MSG_CONTRAST                        _UxGT("LCD contrast")
#define MSG_STORE_EPROM                     _UxGT("Store memory")
#define MSG_LOAD_EPROM                      _UxGT("Load memory")
#define MSG_RESTORE_FAILSAFE                _UxGT("Restore failsafe")
#define MSG_REFRESH                         _UxGT("Refresh")
#define MSG_WATCH                           _UxGT("Info screen")
#define MSG_PREPARE                         _UxGT("Prepare")
#define MSG_TUNE                            _UxGT("Tune")
#define MSG_PAUSE_PRINT                     _UxGT("Pause print")
#define MSG_RESUME_PRINT                    _UxGT("Resume print")
#define MSG_STOP_PRINT                      _UxGT("Stop print")
#define MSG_STOP_SAVE_PRINT                 _UxGT("Stop and Save")
#define MSG_CARD_MENU                       _UxGT("Print from ") MSG_SD
#define MSG_NO_CARD                         _UxGT("No ") MSG_SD
#define MSG_DWELL                           _UxGT("Sleep...")
#define MSG_USERWAIT                        _UxGT("Wait for user...")
#define MSG_RESUMING                        _UxGT("Resuming print")
#define MSG_PRINT_ABORTED                   _UxGT("Print aborted")
#define MSG_NO_MOVE                         _UxGT("No move.")
#define MSG_KILLED                          _UxGT("KILLED. ")
#define MSG_STOPPED                         _UxGT("STOPPED. ")
#define MSG_CONTROL_RETRACT                 _UxGT("Retract mm")
#define MSG_CONTROL_RETRACT_SWAP            _UxGT("Swap Re. mm")
#define MSG_CONTROL_RETRACTF                _UxGT("Retract  F")
#define MSG_CONTROL_RETRACT_ZLIFT           _UxGT("Hop mm")
#define MSG_CONTROL_RETRACT_RECOVER         _UxGT("UnRet mm")
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    _UxGT("S UnRet mm")
#define MSG_CONTROL_RETRACT_RECOVERF        _UxGT("UnRet  F")
#define MSG_AUTORETRACT                     _UxGT("AutoRetr.")
#define MSG_FILAMENTCHANGE                  _UxGT("Change filament")
#define MSG_INIT_SDCARD                     _UxGT("Init. ") MSG_SD
#define MSG_CNG_SDCARD                      _UxGT("Change _") MSG_SD
#define MSG_ZPROBE_OUT                      _UxGT("Z probe out. bed")
#define MSG_BLTOUCH_SELFTEST                _UxGT("BLTouch Self-Test")
#define MSG_BLTOUCH_RESET                   _UxGT("Reset BLTouch")
#define MSG_HOME                            _UxGT("Home")
#define MSG_FIRST                           _UxGT("first")
#define MSG_ZPROBE_ZOFFSET                  _UxGT("ZProbe ZOffset")
#define MSG_BABYSTEP                        _UxGT("Babystep")
#define MSG_BABYSTEP_X                      MSG_BABYSTEP _UxGT(" ") MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP _UxGT(" ") MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP _UxGT(" ") MSG_Z
#define MSG_ENDSTOP_ABORT                   _UxGT("Endstop abort")
#define MSG_HEATING_FAILED_LCD              _UxGT("Heating failed")
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
#define MSG_PLEASE_RESET                    _UxGT("Please reset")
#define MSG_END_DAY                         _UxGT("days")
#define MSG_END_HOUR                        _UxGT("hours")
#define MSG_END_MINUTE                      _UxGT("minutes")
#define MSG_PRINT_TIME                      _UxGT("Print time ")

// Calibrate Delta
#define MSG_DELTA_CALIBRATE                 _UxGT("Delta Calibration")
#define MSG_DELTA_CALIBRATE_X               _UxGT("Calibrate ") MSG_X
#define MSG_DELTA_CALIBRATE_Y               _UxGT("Calibrate ") MSG_Y
#define MSG_DELTA_CALIBRATE_Z               _UxGT("Calibrate ") MSG_Z
#define MSG_DELTA_CALIBRATE_CENTER          _UxGT("Calibrate Center")

// Info printers
#define MSG_INFO_MENU                       _UxGT("About Printer")
#define MSG_INFO_FIRMWARE_MENU              _UxGT("Firmware Info")
#define MSG_INFO_STATS_MENU                 _UxGT("Printer Stats")
#define MSG_INFO_BOARD_MENU                 _UxGT("Board Info")
#define MSG_INFO_THERMISTOR_MENU            _UxGT("Thermistors")
#define MSG_INFO_EXTRUDERS                  _UxGT("Extruders")
#define MSG_INFO_HOTENDS                    _UxGT("Hotends")
#define MSG_INFO_BED                        _UxGT("Bed")
#define MSG_INFO_CHAMBER                    _UxGT("Hot Chamber")
#define MSG_INFO_COOLER                     _UxGT("Cooler")
#define MSG_INFO_BAUDRATE                   _UxGT("Baud")
#define MSG_INFO_PROTOCOL                   _UxGT("Protocol")
#define MSG_INFO_TOTAL_PRINTS               _UxGT("Total Prints")
#define MSG_INFO_FINISHED_PRINTS            _UxGT("Fin. Prints")
#define MSG_INFO_ON_TIME                    _UxGT("On x")
#define MSG_INFO_PRINT_TIME                 _UxGT("Pr x")
#define MSG_INFO_FILAMENT_USAGE             _UxGT("Fil")
#define MSG_INFO_PWRCONSUMED                _UxGT("PWR")
#define MSG_INFO_MIN_TEMP                   _UxGT("Min Temp")
#define MSG_INFO_MAX_TEMP                   _UxGT("Max Temp")
#define MSG_INFO_PSU                        _UxGT("Power Supply")

// CASE LIGHT
#define MSG_LIGHTS_ON                       _UxGT("Case light on")
#define MSG_LIGHTS_OFF                      _UxGT("Case light off")

// FILAMENT_CHANGE_FEATURE
#define MSG_FILAMENT_CHANGE_HEADER          _UxGT("CHANGE FILAMENT")
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   _UxGT("CHANGE OPTIONS:")
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE  _UxGT("Extrude more")
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   _UxGT("Resume print")
#if LCD_HEIGHT >= 4
  #define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("Wait for start")
  #define MSG_FILAMENT_CHANGE_INIT_2          _UxGT("of the filament")
  #define MSG_FILAMENT_CHANGE_INIT_3          _UxGT("change")
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("Wait for")
  #define MSG_FILAMENT_CHANGE_UNLOAD_2        _UxGT("filament unload")
  #define MSG_FILAMENT_CHANGE_UNLOAD_3        _UxGT("")
  #define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("Insert filament")
  #define MSG_FILAMENT_CHANGE_INSERT_2        _UxGT("and press button")
  #define MSG_FILAMENT_CHANGE_INSERT_3        _UxGT("to continue...")
  #define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("Wait for")
  #define MSG_FILAMENT_CHANGE_LOAD_2          _UxGT("filament load")
  #define MSG_FILAMENT_CHANGE_LOAD_3          _UxGT("")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       _UxGT("Wait for")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_2       _UxGT("filament extrude")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_3       _UxGT("")
  #define MSG_FILAMENT_CHANGE_RESUME_1        _UxGT("Wait for print")
  #define MSG_FILAMENT_CHANGE_RESUME_2        _UxGT("resume")
  #define MSG_FILAMENT_CHANGE_RESUME_3        _UxGT("")
#else // LCD_HEIGHT < 4
  #define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("Please wait...")
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("Ejecting...")
  #define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("Insert and Click")
  #define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("Loading...")
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       _UxGT("Extruding...")
  #define MSG_FILAMENT_CHANGE_RESUME_1        _UxGT("Resuming...")
#endif // LCD_HEIGHT < 4

// Heater
#define MSG_HEATING                         _UxGT("Heating...")
#define MSG_HEATING_COMPLETE                _UxGT("Heating done.")
#define MSG_BED_HEATING                     _UxGT("Bed Heating.")
#define MSG_BED_DONE                        _UxGT("Bed done.")
#define MSG_CHAMBER_HEATING                 _UxGT("Chamber Heating.")
#define MSG_CHAMBER_DONE                    _UxGT("Chamber done.")
#define MSG_COOLER_COOLING                  _UxGT("Cooling...")
#define MSG_COOLER_DONE                     _UxGT("Cooling done.")

// Extra
#define MSG_LASER                           _UxGT("Laser Preset")
#define MSG_CONFIG                          _UxGT("Configuration")
#define MSG_E_BOWDEN_LENGTH                 MSG_EXTRUDE _UxGT(" ") STRINGIFY(BOWDEN_LENGTH) _UxGT("mm")
#define MSG_R_BOWDEN_LENGTH                 MSG_RETRACT _UxGT(" ") STRINGIFY(BOWDEN_LENGTH) _UxGT("mm")
#define MSG_PURGE_XMM                       MSG_PURGE   _UxGT(" ") STRINGIFY(LCD_PURGE_LENGTH) _UxGT("mm")
#define MSG_RETRACT_XMM                     MSG_RETRACT _UxGT(" ") STRINGIFY(LCD_RETRACT_LENGTH) _UxGT("mm")
#define MSG_SAVED_POS                       _UxGT("Saved position")
#define MSG_RESTORING_POS                   _UxGT("Restoring position")
#define MSG_INVALID_POS_SLOT                _UxGT("Invalid slot, total slots: ")

// Rfid module
#define MSG_RFID_SPOOL                      _UxGT("Spool on E")
#define MSG_RFID_BRAND                      _UxGT("Brand: ")
#define MSG_RFID_TYPE                       _UxGT("Type: ")
#define MSG_RFID_COLOR                      _UxGT("Color: ")
#define MSG_RFID_SIZE                       _UxGT("Size: ")
#define MSG_RFID_TEMP_HOTEND                _UxGT("Temperature Hotend: ")
#define MSG_RFID_TEMP_BED                   _UxGT("Temperature Bed: ")
#define MSG_RFID_TEMP_USER_HOTEND           _UxGT("User temperature Hotend: ")
#define MSG_RFID_TEMP_USER_BED              _UxGT("User temperatura Bed: ")
#define MSG_RFID_DENSITY                    _UxGT("Density: ")
#define MSG_RFID_SPOOL_LENGHT               _UxGT("Spool Lenght: ")


#endif // LANGUAGE_EN_H
