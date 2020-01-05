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
 * Bulgarian
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_5

namespace language_bg {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Bulgarian"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" Готов."));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Картата е поставена"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Картата е извадена"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Меню"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Автостарт"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Изкл. двигатели"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Паркиране"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Задай Начало"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Изходна точка"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Подгряване ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Подгряване ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Подгряване ") PREHEAT_1_LABEL _UxGT(" Дюза"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Подгряване ") PREHEAT_1_LABEL _UxGT(" Дюза ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Подгр. ") PREHEAT_1_LABEL _UxGT(" Всички"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Подгр. ") PREHEAT_1_LABEL _UxGT(" Легло"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Настройки ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Подгряване ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Подгряване ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Подгряване ") PREHEAT_2_LABEL _UxGT(" Дюза"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Подгряване ") PREHEAT_2_LABEL _UxGT(" Дюза ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Подгр. ") PREHEAT_2_LABEL _UxGT(" Всички"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Подгр. ") PREHEAT_2_LABEL _UxGT(" Легло"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Настройки ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Охлаждане"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Вкл. захранване"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Изкл. захранване"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Екструзия"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Откат"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Движение по ос"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Нивелиране"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Нивелиране"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Движение по X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Движение по Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Движение по Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Екструдер"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Екструдер *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Премести с %smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Премести с 0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Премести с 1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Премести с 10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Скорост"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Bed Z"));
  FSTRINGVALUE(MSG_NOZZLE                           , " " LCD_STR_THERMOMETER _UxGT(" Дюза"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , " " LCD_STR_THERMOMETER _UxGT(" Дюза ~"));
  FSTRINGVALUE(MSG_BED                              , " " LCD_STR_THERMOMETER _UxGT(" Легло"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Вентилатор"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Вентилатор ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Поток"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Поток ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Управление"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Минимум"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Максимум"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Фактор"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Авто-темп."));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Вкл."));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Изкл."));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-откат"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-travel"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Стъпки/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT("стъпки/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT("стъпки/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT("стъпки/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E стъпки/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* стъпки/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Температура"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Движение"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Нишка"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E in mm3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Диам. нишка"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Диам. нишка *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD контраст"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Запази в EPROM"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Зареди от EPROM"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Фабрични настройки"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH _UxGT("Обнови"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Преглед"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Действия"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Настройка"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Пауза"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Възобнови печата"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Спри печата"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Меню карта"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Няма карта"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Почивка..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Изчакване"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Печатът е прекъснат"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Няма движение"));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("УБИТО."));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("СПРЯНО."));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Откат mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Смяна Откат mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Откат  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Скок mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("Възврат mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Смяна Възврат mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("Възврат  V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Автоoткат"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Смяна нишка"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Смяна нишка *"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Иниц. SD-Карта"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Смяна SD-Карта"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z-сондата е извадена"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Z Отстояние"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Министъпка X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Министъпка Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Министъпка Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Стоп Кр.Изключватели"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Делта Калибровка"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Калибровка X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Калибровка Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Калибровка Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Калибровка Център"));
  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Неправилен принтер"));
}
