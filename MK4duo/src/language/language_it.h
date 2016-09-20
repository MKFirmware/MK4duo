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
 * Italian
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_IT_H
#define LANGUAGE_IT_H

#define MAPPER_NON                  // For direct asci codes
#define DISPLAY_CHARSET_ISO10646_1  // use the better font on full graphic displays.


#define WELCOME_MSG                         MACHINE_NAME " pronta."
#define MSG_SD                              "SD"
#define MSG_SD_INSERTED                     MSG_SD " inserita"
#define MSG_SD_REMOVED                      MSG_SD " rimossa"
#define MSG_MAIN                            "Menu principale"
#define MSG_AUTOSTART                       "Autostart"
#define MSG_DISABLE_STEPPERS                "Disabilita Motori"
#define MSG_AUTO_HOME                       "Auto Home"
#define MSG_AUTO_HOME_X                     "Home asse X"
#define MSG_AUTO_HOME_Y                     "Home asse Y"
#define MSG_AUTO_HOME_Z                     "Home asse Z"
#define MSG_LEVEL_BED_HOMING                "Home assi XYZ"
#define MSG_LEVEL_BED_WAITING               "Premi per iniziare"
#define MSG_LEVEL_BED_NEXT_POINT            "Punto successivo"
#define MSG_LEVEL_BED_DONE                  "Livel. terminato!"
#define MSG_LEVEL_BED_CANCEL                "Annulla"
#define MSG_SET_HOME_OFFSETS                "Setta offset home"
#define MSG_SET_ORIGIN                      "Imposta Origine"
#define MSG_PREHEAT                         "Preriscalda"
#define MSG_PREHEAT_1                       MSG_PREHEAT " PLA"
#define MSG_PREHEAT_1_N                     MSG_PREHEAT_1 " "
#define MSG_PREHEAT_1_ALL                   MSG_PREHEAT_1 " All"
#define MSG_PREHEAT_1_BEDONLY               MSG_PREHEAT_1 " Bed"
#define MSG_PREHEAT_1_SETTINGS              MSG_PREHEAT_1 " conf"
#define MSG_PREHEAT_2                       MSG_PREHEAT " ABS"
#define MSG_PREHEAT_2_N                     MSG_PREHEAT_2 " "
#define MSG_PREHEAT_2_ALL                   MSG_PREHEAT_2 " All"
#define MSG_PREHEAT_2_BEDONLY               MSG_PREHEAT_2 " Bed"
#define MSG_PREHEAT_2_SETTINGS              MSG_PREHEAT_2 " conf"
#define MSG_PREHEAT_3                       MSG_PREHEAT " GOMMA"
#define MSG_PREHEAT_3_N                     MSG_PREHEAT_3 " "
#define MSG_PREHEAT_3_ALL                   MSG_PREHEAT_3 " All"
#define MSG_PREHEAT_3_BEDONLY               MSG_PREHEAT_3 " Bed"
#define MSG_PREHEAT_3_SETTINGS              MSG_PREHEAT_3 " conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend troppo freddo per il cambio filo"
#define MSG_COOLDOWN                        "Raffredda"
#define MSG_SWITCH_PS_ON                    "Accendi aliment."
#define MSG_SWITCH_PS_OFF                   "Spegni aliment."
#define MSG_EXTRUDE                         "Estrudi"
#define MSG_RETRACT                         "Ritrai"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Livella piano"
#define MSG_SPEED                           "Velocita"
#define MSG_NOZZLE                          "Ugello"
#define MSG_BED                             "Piatto"
#define MSG_CHAMBER                         "Camera"
#define MSG_COOLER                          "Raffreddamento"
#define MSG_BED_Z                           "piatto Z"
#define MSG_FAN_SPEED                       "Ventola"
#define MSG_FLOW                            "Flusso"
#define MSG_CONTROL                         "Controllo"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Autotemp"
#define MSG_ON                              "ON "
#define MSG_OFF                             "OFF"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_PID_C                           "PID-C"
#define MSG_SELECT                          "Select"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "Accel"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"
#define MSG_MOVE                            "Muovi"
#define MSG_MOVE_AXIS                       MSG_MOVE " asse"
#define MSG_MOVE_X                          MSG_MOVE " " MSG_X
#define MSG_MOVE_Y                          MSG_MOVE " " MSG_Y
#define MSG_MOVE_Z                          MSG_MOVE " " MSG_Z
#define MSG_MOVE_01MM                       MSG_MOVE " 0.1mm"
#define MSG_MOVE_1MM                        MSG_MOVE " 1mm"
#define MSG_MOVE_10MM                       MSG_MOVE " 10mm"
#define MSG_MOVE_E                          "Estrusore"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          MSG_X " steps/mm"
#define MSG_YSTEPS                          MSG_Y " steps/mm"
#define MSG_ZSTEPS                          MSG_Z " steps/mm"
#define MSG_E0STEPS                         MSG_E "0 steps/mm"
#define MSG_E1STEPS                         MSG_E "1 steps/mm"
#define MSG_E2STEPS                         MSG_E "2 steps/mm"
#define MSG_E3STEPS                         MSG_E "3 steps/mm"
#define MSG_TEMPERATURE                     "Temperatura"
#define MSG_MOTION                          "Movimento"
#define MSG_FILAMENT                        "Filamento"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Diam. filo"
#define MSG_CONTRAST                        "Contrasto LCD"
#define MSG_STORE_EPROM                     "Salva in EEPROM"
#define MSG_LOAD_EPROM                      "Carica da EEPROM"
#define MSG_RESTORE_FAILSAFE                "Impostaz. default"
#define MSG_REFRESH                         "Aggiorna"
#define MSG_WATCH                           "Guarda"
#define MSG_PREPARE                         "Prepara"
#define MSG_TUNE                            "Adatta"
#define MSG_PAUSE_PRINT                     "Pausa"
#define MSG_RESUME_PRINT                    "Riprendi stampa"
#define MSG_STOP_PRINT                      "Arresta stampa"
#define MSG_STOP_SAVE_PRINT                 "Arresta e Salva"
#define MSG_CARD_MENU                       "SD Card Menu"
#define MSG_NO_CARD                         "No SD Card"
#define MSG_DWELL                           "Sospensione..."
#define MSG_USERWAIT                        "Attendi Utente..."
#define MSG_RESUMING                        "Riprendi Stampa"
#define MSG_PRINT_ABORTED                   "Stampa abortita"
#define MSG_NO_MOVE                         "Nessun Movimento"
#define MSG_KILLED                          "UCCISO "
#define MSG_STOPPED                         "ARRESTATO "
#define MSG_CONTROL_RETRACT                 "Ritrai mm"
#define MSG_CONTROL_RETRACT_SWAP            "Scamb. Ritrai mm"
#define MSG_CONTROL_RETRACTF                "Ritrai  V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Salta mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Scamb. UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  V"
#define MSG_AUTORETRACT                     "AutoArretramento"
#define MSG_FILAMENT_CHANGE                 "Cambia filamento"
#define MSG_INIT_SDCARD                     "Iniz. SD-Card"
#define MSG_CNG_SDCARD                      "Cambia SD-Card"
#define MSG_ZPROBE_OUT                      "Z probe out. bed"
#define MSG_RESET_BLTOUCH                   "Reset BLTouch"
#define MSG_HOME                            "Home"
#define MSG_FIRST                           "prima"
#define MSG_ZPROBE_ZOFFSET                  "Zprobe ZOffset"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Finecorsa abort."
#define MSG_HEATING_FAILED_LCD              "Riscaldamento fallito"
#define MSG_ERR_REDUNDANT_TEMP              "Err: TEMP RIDONDANTI"
#define MSG_THERMAL_RUNAWAY                 "TEMP FUORI CONTROLLO"
#define MSG_AD595                           "AD595 Offset & Gain"
#define MSG_ERR_MAXTEMP                     "Err: TEMP MASSIMA"
#define MSG_ERR_MINTEMP                     "Err: TEMP MINIMA"
#define MSG_ERR_MAXTEMP_BED                 "Err: TEMP MASSIMA PIATTO"
#define MSG_ERR_MINTEMP_BED                 "Err: TEMP MINIMA PIATTO"
#define MSG_ERR_Z_HOMING                    "G28 Z Error"
#define MSG_ERR_MAXTEMP_CHAMBER             "MAXTEMP CHAMBER ERROR"
#define MSG_ERR_MINTEMP_CHAMBER             "MINTEMP CHAMBER ERROR"
#define MSG_ERR_MAXTEMP_COOLER              "MAXTEMP COOLER ERROR"
#define MSG_ERR_MINTEMP_COOLER              "MINTEMP COOLER ERROR"
#define MSG_HALTED                          "Stampante bloccata"
#define MSG_PLEASE_RESET                    "Per favore resettare"
#define MSG_END_DAY                         "giorni"
#define MSG_END_HOUR                        "ore"
#define MSG_END_MINUTE                      "minuti"
#define MSG_PRINT_TIME                      "Print time "

#define MSG_ENDSTOPS_HIT                    "Endstop hit: "
#define MSG_BABYSTEPPING                    "Babystepping"
#define MSG_BABYSTEPPING_X                  MSG_BABYSTEPPING " " MSG_X
#define MSG_BABYSTEPPING_Y                  MSG_BABYSTEPPING " " MSG_Y
#define MSG_BABYSTEPPING_Z                  MSG_BABYSTEPPING " " MSG_Z

#define MSG_ENDSTOP_XS                      MSG_X
#define MSG_ENDSTOP_YS                      MSG_Y
#define MSG_ENDSTOP_ZS                      MSG_Z
#define MSG_ENDSTOP_ZPS                     MSG_Z "P"
#define MSG_ENDSTOP_ES                      MSG_E

// Calibrate Delta
#define MSG_DELTA_CALIBRATE                 "Calibraz. Delta"
#define MSG_DELTA_CALIBRATE_X               "Calibra X"
#define MSG_DELTA_CALIBRATE_Y               "Calibra Y"
#define MSG_DELTA_CALIBRATE_Z               "Calibra Z"
#define MSG_DELTA_CALIBRATE_CENTER          "Calibra Centro"

// Info printers
#define MSG_INFO_MENU                       "About Stampante"
#define MSG_INFO_FIRMWARE_MENU              "Info Firmware"
#define MSG_INFO_STATS_MENU                 "Statistiche"
#define MSG_INFO_BOARD_MENU                 "Info Scheda"
#define MSG_INFO_THERMISTOR_MENU            "Termistori"
#define MSG_INFO_EXTRUDERS                  "Estrusori"
#define MSG_INFO_HOTENDS                    "Ugelli"
#define MSG_INFO_BED                        "Letto"
#define MSG_INFO_CHAMBER                    "Camera Calda"
#define MSG_INFO_COOLER                     "Raffreddamento"
#define MSG_INFO_BAUDRATE                   "Baud"
#define MSG_INFO_PROTOCOL                   "Protocollo"
#define MSG_INFO_TOTAL_PRINTS               "Stampe totali"
#define MSG_INFO_FINISHED_PRINTS            "Stampe complete"
#define MSG_INFO_ON_TIME                    "On x"
#define MSG_INFO_PRINT_TIME                 "Pr x"
#define MSG_INFO_FILAMENT_USAGE             "Filo"
#define MSG_INFO_PWRCONSUMED                "PWR"
#define MSG_INFO_MIN_TEMP                   "Min Temp"
#define MSG_INFO_MAX_TEMP                   "Max Temp"
#define MSG_INFO_PSU                        "Alimentatore"

// FILAMENT_CHANGE_FEATURE
#define MSG_FILAMENT_CHANGE_HEADER          "CAMBIO FILO"
#define MSG_FILAMENT_CHANGE_INIT_1          "Attendere"
#define MSG_FILAMENT_CHANGE_INIT_2          "per il cambio"
#define MSG_FILAMENT_CHANGE_INIT_3          "filamento"
#define MSG_FILAMENT_CHANGE_UNLOAD_1        "Attendere lo"
#define MSG_FILAMENT_CHANGE_UNLOAD_2        "scarico filamento"
#define MSG_FILAMENT_CHANGE_UNLOAD_3        ""
#define MSG_FILAMENT_CHANGE_INSERT_1        "Inserire filamento"
#define MSG_FILAMENT_CHANGE_INSERT_2        "premere il bottone"
#define MSG_FILAMENT_CHANGE_INSERT_3        "per continuare..."
#define MSG_FILAMENT_CHANGE_LOAD_1          "Attendere il"
#define MSG_FILAMENT_CHANGE_LOAD_2          "caricamento filo"
#define MSG_FILAMENT_CHANGE_LOAD_3          ""
#define MSG_FILAMENT_CHANGE_EXTRUDE_1       "Attendere"
#define MSG_FILAMENT_CHANGE_EXTRUDE_2       "estrusione filamento"
#define MSG_FILAMENT_CHANGE_EXTRUDE_3       ""
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   "Cosa faccio?"
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE  "Estrudere ancora"
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   "Riprendere la stampa"
#define MSG_FILAMENT_CHANGE_RESUME_1        "Attendere che la"
#define MSG_FILAMENT_CHANGE_RESUME_2        "stampa riprenda"
#define MSG_FILAMENT_CHANGE_RESUME_3        ""

// Heater
#define MSG_HEATING                         "Riscaldamento..."
#define MSG_HEATING_COMPLETE                "Riscaldamento finito."
#define MSG_BED_HEATING                     "Riscaldamento piatto."
#define MSG_BED_DONE                        "Piatto riscaldato."
#define MSG_CHAMBER_HEATING                 "Riscaldamento camera."
#define MSG_CHAMBER_DONE                    "Camera riscaldata."
#define MSG_COOLER_COOLING                  "Raffreddamento..."
#define MSG_COOLER_DONE                     "Raffreddamento finito."

// Extra
#define MSG_LASER                           "Laser Preset"
#define MSG_CONFIG                          "Configurazione"
#define MSG_E_BOWDEN_LENGTH                 "Estrudi " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_R_BOWDEN_LENGTH                 "Retrai " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_PURGE_XMM                       "Purga " STRINGIFY(LCD_PURGE_LENGTH) "mm"
#define MSG_RETRACT_XMM                     "Retrai " STRINGIFY(LCD_RETRACT_LENGTH) "mm"
#define MSG_SAVED_POS                       "Posizione Salvata"
#define MSG_RESTORING_POS                   "Ripristino posizione"
#define MSG_INVALID_POS_SLOT                "Slot invalido, slot totali: "

// Rfid module
#define MSG_RFID_SPOOL                      "Bobina su E"
#define MSG_RFID_BRAND                      "Marca: "
#define MSG_RFID_TYPE                       "Tipo: "
#define MSG_RFID_COLOR                      "Colore: "
#define MSG_RFID_SIZE                       "Size: "
#define MSG_RFID_TEMP_HOTEND                "Temperatura Hotend: "
#define MSG_RFID_TEMP_BED                   "Temperatura Bed: "
#define MSG_RFID_TEMP_USER_HOTEND           "Temperatura utente Hotend: "
#define MSG_RFID_TEMP_USER_BED              "Temperatura utente Bed: "
#define MSG_RFID_DENSITY                    "Densita': "
#define MSG_RFID_SPOOL_LENGHT               "Lunghezza bobina: "

// Firmware Test
#define MSG_FWTEST_YES                      "Dai il comando Y per andare avanti"
#define MSG_FWTEST_NO                       "Dai il comando N per andare avanti"
#define MSG_FWTEST_YES_NO                   "Dai il comando Y o N per andare avanti"
#define MSG_FWTEST_ENDSTOP_ERR              "ENDSTOP ERROR! Controllare cavi e connessioni"
#define MSG_FWTEST_PRESS                    "Premere e tenere premuto l'endstop "
#define MSG_FWTEST_INVERT                   "Invertire valore di "
#define MSG_FWTEST_XAXIS                    "Il nozzle si e' spostato a destra?"
#define MSG_FWTEST_YAXIS                    "Il nozzle si e' spostato in avanti?"
#define MSG_FWTEST_ZAXIS                    "Il nozzle si e' spostato in alto?"
#define MSG_FWTEST_01                       "Muovi manualmente gli assi X, Y e Z lontano dagli endstop"
#define MSG_FWTEST_02                       "Vuoi controllare gli ENDSTOP?"
#define MSG_FWTEST_03                       "Inizio controllo ENDSTOP"
#define MSG_FWTEST_04                       "Inizio controllo MOTORI"
#define MSG_FWTEST_ATTENTION                "ATTENZIONE! Controlla che i tre assi siano a piu' di 5 mm dagli endstop!"
#define MSG_FWTEST_END                      "Test finito. Disabilitare FIRMWARE_TEST e ricompilare."
#define MSG_FWTEST_INTO                     "in "
#define MSG_FWTEST_ERROR                    "ERRORE"
#define MSG_FWTEST_OK                       "OK"
#define MSG_FWTEST_NDEF                     "non definito"

#endif // LANGUAGE_IT_H
