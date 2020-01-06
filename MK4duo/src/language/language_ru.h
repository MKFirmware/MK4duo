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
 * Russian
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_5

namespace language_ru {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Russian"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" готов."));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Назад"));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("Карта вставлена"));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("Карта извлечена"));
  FSTRINGVALUE(MSG_MEDIA_RELEASED                   , _UxGT("SD карта не активна"));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Эндстопы")); // Max length 8 characters
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Прогр. эндстопы"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Меню"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Другие настройки"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Настройки"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Автостарт"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Выкл. двигатели"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Меню отладки"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Тест индикатора"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Авто парковка"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("Парковка X"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Парковка Y"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Парковка Z"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("Нулевое положение"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Нажмите чтобы начать"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Следующая точка"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Выравнивание готово!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Высота спада"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Запомнить парковку"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Коррекции применены"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Запомнить ноль"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Преднагрев ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Преднагрев ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Преднагрев ") PREHEAT_1_LABEL _UxGT(" сопло"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Преднагрев ") PREHEAT_1_LABEL _UxGT(" сопло ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Преднагрев ") PREHEAT_1_LABEL _UxGT(" всё"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Преднагрев ") PREHEAT_1_LABEL _UxGT(" стол"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Преднагрев ") PREHEAT_1_LABEL _UxGT(" настр."));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Преднагрев ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Преднагрев ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Преднагрев ") PREHEAT_2_LABEL _UxGT(" сопло"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Преднагрев ") PREHEAT_2_LABEL _UxGT(" сопло ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Преднагрев ") PREHEAT_2_LABEL _UxGT(" всё"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Преднагрев ") PREHEAT_2_LABEL _UxGT(" стол"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Преднагрев ") PREHEAT_2_LABEL _UxGT(" настр."));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Охлаждение"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Включить питание"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Выключить питание"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Экструзия"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Втягивание"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Движение по осям"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Калибровка стола"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Калибровать стол"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Выровнять углы"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Следующий угол"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Редактировать сетку"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Ред. сетки завершено"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Свои команды"));

  FSTRINGVALUE(MSG_M48_TEST                         , _UxGT("Проверка датчика Z"));
  FSTRINGVALUE(MSG_M48_DEVIATION                    , _UxGT("Отклонение"));
  FSTRINGVALUE(MSG_M48_POINT                        , _UxGT("Измерение"));

  // TODO: IDEX Menu
  FSTRINGVALUE(MSG_OFFSETS_MENU                     , _UxGT("Размещение сопел"));

  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2-е сопло X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2-е сопло Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2-е сопло Z"));

  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("Выполняем G29"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("Утилиты UBL"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("Калибровка UBL"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Постр. сетку от руки"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Пост. шимм и измер."));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Измерение"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Удал. и измер. стол"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Двигаемся дальше"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("Активировать UBL"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("Деактивировать UBL"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Температура стола"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Bed Temp"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Температура сопла"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Hotend Temp"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Редактор сеток"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Редакт. свою сетку"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("Точная настр. сетки"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Ред. сетки завершено"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Построить свою сетку"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Построить сетку"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Построить сетку ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Построить сетку ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Построить хол. сетку"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Устан. высоту сетки"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Высота"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Проверить сетку"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Проверить сетку ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Проверить сетку ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Проверить свою сетку"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Продолжить сетку"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Калибровка сетки"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("Калибровка 3-х точек"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Калибровка растера"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Выровнить сетку"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Крайние точки"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Тип карты"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Вывести карту сетки"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Вывести на хост"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("Вывести в CSV"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Забекапить сетку"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("Вывод информации UBL"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Кол-во заполнителя"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Ручное заполнение"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Умное заполнение"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Заполнить сетку"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Аннулировать всё"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Аннулир. ближ. точку"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Точная настр. всего"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Настр. ближ. точки"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Хранилище сетей"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Слот памяти"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Загрузить сетку стола"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Сохранить сетку стола"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Сетка %i загружена"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Сетка %i сохранена"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Нет хранилища"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Ошибка: Сохран. UBL"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Ошибка: Восстан. UBL"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Смещение Z останов."));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("Пошаговое UBL"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Построить хол. сетку"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Умное заполнение"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Проверить сетку"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Точная настр. всего"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Проверить сетку"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Точная настр. всего"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Сохранить сетку стола"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("Настройки LED"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Подсветку"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Предустановки света"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Красный свет"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Оранжевый свет"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Жёлтый свет"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Зелёный свет"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Синий свет"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Индиго свет"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Фиолетовый свет"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Белый свет"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Свет по умолчанию"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Свои настр. света"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Интенсивн. красного"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Интенсивн. зелёного"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Интенсивн. синего"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Интенсивн. белого"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Яркость"));

  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Движемся..."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Освобождаем XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("Движение по X"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Движение по Y"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Движение по Z"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Экструдер"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Экструдер *"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("Движение %sмм"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("Движение 0.1мм"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("Движение 1мм"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("Движение 10мм"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Скорость"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Z стола"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Сопло"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Сопло ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Стол"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Кулер"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Кулер ~"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Кулер доп."));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Кулер доп. ~"));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Поток"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Поток ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Настройки"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Мин"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Макс"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Фактор"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Автотемпература"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Вкл."));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Выкл."));
  FSTRINGVALUE(MSG_PID_P                            , _UxGT("PID-P"));
  FSTRINGVALUE(MSG_PID_P_E                          , _UxGT("PID-P *"));
  FSTRINGVALUE(MSG_PID_I                            , _UxGT("PID-I"));
  FSTRINGVALUE(MSG_PID_I_E                          , _UxGT("PID-I *"));
  FSTRINGVALUE(MSG_PID_D                            , _UxGT("PID-D"));
  FSTRINGVALUE(MSG_PID_D_E                          , _UxGT("PID-D *"));
  FSTRINGVALUE(MSG_PID_C                            , _UxGT("PID-C"));
  FSTRINGVALUE(MSG_PID_C_E                          , _UxGT("PID-C *"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Выбор"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Выбор *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("Ускорение"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Рывок"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("V") LCD_STR_A _UxGT("-рывок"));
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("V") LCD_STR_B _UxGT("-рывок"));
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("V") LCD_STR_C _UxGT("-рывок"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Ve-рывок"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Быстрота"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("Vмакс ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("Vмакс ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("Vмакс ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("Vмакс ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("Vмакс *"));
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("Vмин"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("Vпутеш. мин"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Ускорение"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Aмакс ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Aмакс ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Aмакс ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Aмакс ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Aмакс *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("A-втягивание"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("A-путеш."));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Шаг/мм"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT("шаг/мм"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT("шаг/мм"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT("шаг/мм"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("Eшаг/мм"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("*шаг/мм"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Температура"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Движение"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Филамент"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("E в мм3"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Диаметр филамента"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Диаметр филамента *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Загрузка мм"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Выгрузка мм"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("K продвижения"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("K продвижения *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("Контраст LCD"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Сохранить настройки"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Загрузить настройки"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Вернуть настройки"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("Инициализация EEPROM"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("Обновление прошивки"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Сброс принтера"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Обновить"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Информационный экран"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Подготовить"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Настроить"));
  FSTRINGVALUE(MSG_START_PRINT                      , _UxGT("Начало печати"));
  FSTRINGVALUE(MSG_BUTTON_NEXT                      , _UxGT("Дальше"));
  FSTRINGVALUE(MSG_BUTTON_INIT                      , _UxGT("Инициализация"));
  FSTRINGVALUE(MSG_BUTTON_STOP                      , _UxGT("Остановить"));
  FSTRINGVALUE(MSG_BUTTON_PRINT                     , _UxGT("Печать"));
  FSTRINGVALUE(MSG_BUTTON_RESET                     , _UxGT("Сброс"));
  FSTRINGVALUE(MSG_BUTTON_CANCEL                    , _UxGT("Отмена"));
  FSTRINGVALUE(MSG_BUTTON_DONE                      , _UxGT("Готово"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Пауза печати"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Продолжить печать"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Остановить печать"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Восстановение сбоя"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("Печать с SD карты"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("Нет SD карты"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Сон..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Продолжить..."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Печать на паузе"));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Печать отменена"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("Нет движения."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("УБИТО. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("ОСТАНОВЛЕНО. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Втягивание мм"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Втягивание смены мм"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Втягивание V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Втяг. прыжка мм"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("Возврат мм"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("Возврат смены мм"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("Возврат V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("Возврат смены V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Авто Втягивание"));

  // TODO: Filament Change Swap / Purge Length

  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Смена сопел"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Поднятие по Z"));

  // TODO: Singlenozzle, nozzle standby

  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Смена филамента"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Смена филамента *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Загрузка филамента"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Загрузка филамента *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Выгрузка филамента *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Выгрузить всё"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("Активировать SD"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("Сменить SD карту"));
  FSTRINGVALUE(MSG_RELEASE_MEDIA                    , _UxGT("Деактивировать SD"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z датчик вне стола"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Фактор наклона"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("Тестирование BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Сброс BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("Установка BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("Набивка BLTouch"));

  // TODO: TouchMI Probe, Manual deploy/stow

  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Паркуй %s%s%s сначала"));
  FSTRINGVALUE(MSG_ZPROBE_ZOFFSET                   , _UxGT("Смещение Z"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Микрошаг X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Микрошаг Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Микрошаг Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Сработал концевик"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Разогрев не удался"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_BED           , _UxGT("Неудача нагрева стола"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Ошибка: Избыточная Т"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("УБЕГАНИЕ ТЕПЛА"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_BED              , _UxGT("УБЕГАНИЕ ТЕПЛА СТОЛА"));
  // TODO: Heated chamber
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Ошибка: Т макс."));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Ошибка: Т мин."));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Ошибка: Т стола макс"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Ошибка: Т стола мин."));
  // TODO: Heated chamber
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Паркуй XY сначала"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("ПРИНТЕР ОСТАНОВЛЕН"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Сделайте сброс"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("д")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("ч")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("м")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Нагрев..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Охлаждение..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Нагрев стола..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Охлаждение стола..."));
  // TODO: Heated chamber
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Калибровка Delta"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Калибровать X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Калибровать Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Калибровать Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Калибровать центр"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Настройки Delta"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Авто калибровка"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Задать высоту Delta"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Задать Z-смещение"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Диаг. стержень"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Высота"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Радиус"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("О принтере"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Инф. о принтере"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("Калибровка 3-х точек"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Линейная калибровка"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("Билинейная калибр."));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Калибровка UBL"));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Калибровка сетки"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("Статистика принтера"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Информация о плате"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Термисторы"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Экструдеры"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("Бод"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Протокол"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Подсветка корпуса"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Яркость подсветки"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Неверный принтер"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Счётчик печати"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Закончено"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Общее время печати"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Наидольшее задание"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Длина филамента"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Отпечатков"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Закончено"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Всего"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("Наидольшее"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Выдавлено"));
  #endif
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Мин. Т"));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Макс. Т"));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("БП"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Сила привода"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Привод %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("Запись DAC EEPROM"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("ПЕЧАТЬ НА ПАУЗЕ"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("ЗАГРУЗКА ФИЛАМЕНТА"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("ВЫГРУЗКА ФИЛАМЕНТА"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("ОПЦИИ ВОЗОБНОВЛЕНИЯ:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Выдавить ещё"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Возобновить печать"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Сопла: "));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Возврат не удался"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Не удалось прощупать"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: Низкая Т"));

  // TODO: MMU2

  // TODO: Mixing

  FSTRINGVALUE(MSG_GAMES                            , _UxGT("Игры"));
  FSTRINGVALUE(MSG_BRICKOUT                         , _UxGT("Кирпичи"));
  FSTRINGVALUE(MSG_INVADERS                         , _UxGT("Вторжение"));
  FSTRINGVALUE(MSG_SNAKE                            , _UxGT("Змейка"));
  FSTRINGVALUE(MSG_MAZE                             , _UxGT("Лабиринт"));

  //
  // Filament Change screens show up to 3 lines on a 4-line display
  //                        ...or up to 2 lines on a 3-line display
  //
  #if LCD_HEIGHT >= 4
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Ожидайте", "начала смены", "филамента")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_3_LINE("Ожидайте", "выгрузки", "филамента")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Вставьте пруток", "и нажмите кнопку", "для продолжения")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Нажмите кнопку для", "нагрева сопла...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Нагрев сопла", "Ждите...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Ожидайте", "загрузки прутка")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_3_LINE("Ожидайте", "экструзии", "филамента")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_3_LINE("Ожидайте", "возобновления", "печати")));
  #else
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Ожидайте...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Выгрузка...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Вставь и нажми")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Нагрев...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Загрузка...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Выдавливание...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Возобновление...")));
  #endif

  FSTRINGVALUE(MSG_TMC_DRIVERS                      , _UxGT("Драйвера TMC"));
  FSTRINGVALUE(MSG_TMC_CURRENT                      , _UxGT("Текущие настройки"));
  FSTRINGVALUE(MSG_TMC_HYBRID_THRS                  , _UxGT("Гибридный режим"));
  FSTRINGVALUE(MSG_TMC_HOMING_THRS                  , _UxGT("Режим без эндстопов"));
  FSTRINGVALUE(MSG_TMC_STEPPING_MODE                , _UxGT("Режим шага"));
  FSTRINGVALUE(MSG_TMC_STEALTH_ENABLED              , _UxGT("Тихий режим вкл"));

}
