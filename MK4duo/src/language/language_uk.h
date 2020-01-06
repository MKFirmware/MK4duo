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
 * Ukrainian
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_5

namespace language_uk {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Ukranian"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" готовий."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Картка вставлена"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Картка видалена"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Кінцевик")); // Max length 8 characters
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Меню"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Автостарт"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Вимк. двигуни"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Авто паркування"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Паркування X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Паркування Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Паркування Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Паркування XYZ"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Почати"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Слідуюча Точка"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Завершено!"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Зберегти паркув."));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Зміщення застос."));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Встанов. початок"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Нагрів ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Нагрів ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" Сопло"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" Сопло ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" Все"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" Стіл"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Нагрів ") PREHEAT_1_LABEL _UxGT(" нал."));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Нагрів ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Нагрів ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Нагрів ") PREHEAT_2_LABEL _UxGT(" Сопло"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Нагрів ") PREHEAT_2_LABEL _UxGT(" Сопло ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Нагрів ") PREHEAT_2_LABEL _UxGT(" Все"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Нагрів ") PREHEAT_2_LABEL _UxGT(" Стіл"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Нагрів ") PREHEAT_2_LABEL _UxGT(" нал."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Охолодження"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Увімкнути живлення"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Вимкнути живлення"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Екструзія"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Втягування"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Рух по осям"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Нівелювання столу"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Нівелювання столу"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Рух по X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Рух по Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Рух по Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Екструдер"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Екструдер *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Рух по %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Рух по 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Рух по 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Рух по 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Швидкість"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Z Столу"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Сопло"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Сопло ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Стіл"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Охолодж."));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Охолодж. ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Потік"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Потік ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Налаштування"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Мін"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Макс"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Факт"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Автотемпер."));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Увімк."));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Вимк."));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID-P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID-I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID-D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID-C *"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Вибрати"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Вибрати *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Приск."));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Ривок"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("V") LCD_STR_A _UxGT("-ривок"));
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("V") LCD_STR_B _UxGT("-ривок"));
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("V") LCD_STR_C _UxGT("-ривок"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Ve-ривок"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("Vмакс") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("Vмакс") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("Vмакс") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vмакс") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("Vмакс *"));
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("Vмін"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("Vруху мін"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Aмакс ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Aмакс ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Aмакс ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Aмакс ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Aмакс *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-втягув."));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-руху"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Кроків/мм"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT("кроків/мм"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT("кроків/мм"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT("кроків/мм"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("Eкроків/мм"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("*кроків/мм"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Температура"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Рух"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Волокно"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E в мм3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Діам. волок."));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Діам. волок. *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("контраст LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Зберегти в ПЗП"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Зчитати з ПЗП"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Відновити базові"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Поновити"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Інформація"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Підготувати"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Підлаштування"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Призупинити друк"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Відновити друк"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Скасувати друк"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Друкувати з SD"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Відсутня SD карт."));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Сплячка..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Очікування дій..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Друк скасовано"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Немає руху."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("ПЕРЕРВАНО. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ЗУПИНЕНО. "));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Зміна волокна"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Зміна волокна *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Старт SD картки"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Заміна SD карти"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z дет. не в межах"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch Само-Тест"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Скинути BLTouch"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Дім %s%s%s перший"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Зміщення Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Мікрокрок X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Мікрокрок Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Мікрокрок Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("невдача кінцевика"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Невдалий нагрів"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("ЗБІЙ ТЕМПЕРАТУРИ"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Дім XY перший"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("ПРИНТЕР ЗУПИНЕНО"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Перезавантажте"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("д")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("г")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("х")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Нагрівання..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Нагрівання столу..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Калібр. Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Калібрування X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Калібрування Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Калібрування Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Калібр. Центру"));

  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Про принтер"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Інформація"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Статистика"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Про плату"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Термістори"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Екструдери"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("біт/с"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Протокол"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Підсвітка"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Неправильний принтер"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("К-сть друків"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Завершено"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Весь час друку"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Найдовший час"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Екструдовано"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Друків"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Завершено"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Загалом"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Найдовший"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Ексдруд."));
  #endif

  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Мін Темп."));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Макс Темп."));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Джерело жив."));

  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Сила мотору"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("% мотору"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Запис ЦАП на ПЗП"));

  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Відновити друк"));

  #if LCD_HEIGHT >= 4
    // Up to 3 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Зачекайте на", "початок заміни", "волокна")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Зачекайте на", "вивід волокна")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Вставте волокно", "та натисніть для", "продовження...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Зачекайте на", "ввід волокна")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_3_LINE("Зачекайте на", "відновлення", "друку")));
  #else
    // Up to 2 lines allowed
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Зачекайте...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Вивід...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Вставте і нат.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Ввід...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Відновлення...")));
  #endif
}
