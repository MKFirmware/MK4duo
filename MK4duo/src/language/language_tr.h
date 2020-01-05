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
 * Turkish
 *
 * LCD Menu Messages
 *
 */

#define DISPLAY_CHARSET_ISO10646_TR

namespace language_tr {
  using namespace language_en; // Inherit undefined strings from English

  constexpr uint8_t CHARSIZE                        = 2;
  FSTRINGVALUE(LANGUAGE                             , _UxGT("Turkish"));

  FSTRINGVALUE(WELCOME_MSG                          , MACHINE_NAME _UxGT(" hazır."));
  FSTRINGVALUE(MSG_BACK                             , _UxGT("Geri"));
  FSTRINGVALUE(MSG_MEDIA_INSERTED                   , _UxGT("SD K. Yerleştirildi."));
  FSTRINGVALUE(MSG_MEDIA_REMOVED                    , _UxGT("SD Kart Çıkarıldı."));
  FSTRINGVALUE(MSG_LCD_ENDSTOPS                     , _UxGT("Enstops")); // Max length 8 characters
  FSTRINGVALUE(MSG_LCD_SOFT_ENDSTOPS                , _UxGT("Yazılımsal Endstops"));
  FSTRINGVALUE(MSG_MAIN                             , _UxGT("Ana"));
  FSTRINGVALUE(MSG_ADVANCED_SETTINGS                , _UxGT("Gelişmiş Ayarlar"));
  FSTRINGVALUE(MSG_CONFIGURATION                    , _UxGT("Yapılandırma"));
  FSTRINGVALUE(MSG_AUTOSTART                        , _UxGT("Oto. Başlat"));
  FSTRINGVALUE(MSG_DISABLE_STEPPERS                 , _UxGT("Motorları Durdur"));
  FSTRINGVALUE(MSG_DEBUG_MENU                       , _UxGT("Hata Ayıklama"));
  FSTRINGVALUE(MSG_PROGRESS_BAR_TEST                , _UxGT("Durum Çubuğu Testi"));
  FSTRINGVALUE(MSG_AUTO_HOME                        , _UxGT("Eksenleri Sıfırla"));
  FSTRINGVALUE(MSG_AUTO_HOME_X                      , _UxGT("X Sıfırla"));
  FSTRINGVALUE(MSG_AUTO_HOME_Y                      , _UxGT("Y Sıfırla"));
  FSTRINGVALUE(MSG_AUTO_HOME_Z                      , _UxGT("Z Sıfırla"));
  FSTRINGVALUE(MSG_AUTO_Z_ALIGN                     , _UxGT("Oto. Z-Hizalama"));
  FSTRINGVALUE(MSG_LEVEL_BED_HOMING                 , _UxGT("XYZ Sıfırlanıyor"));
  FSTRINGVALUE(MSG_LEVEL_BED_WAITING                , _UxGT("Başlatmak için tıkla"));
  FSTRINGVALUE(MSG_LEVEL_BED_NEXT_POINT             , _UxGT("Sonraki Nokta"));
  FSTRINGVALUE(MSG_LEVEL_BED_DONE                   , _UxGT("Hizalama Tamam!"));
  FSTRINGVALUE(MSG_Z_FADE_HEIGHT                    , _UxGT("Kaçınma Yüksekliği"));
  FSTRINGVALUE(MSG_SET_HOME_OFFSETS                 , _UxGT("Offset Ayarla"));
  FSTRINGVALUE(MSG_HOME_OFFSETS_APPLIED             , _UxGT("Offset Tamam"));
  FSTRINGVALUE(MSG_SET_ORIGIN                       , _UxGT("Sıfır Belirle"));
  FSTRINGVALUE(MSG_PREHEAT_1                        , _UxGT("Ön Isınma ") PREHEAT_1_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_1_H                      , _UxGT("Ön Isınma ") PREHEAT_1_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_1_END                    , _UxGT("Ön Isınma ") PREHEAT_1_LABEL _UxGT(" Nozul"));
  FSTRINGVALUE(MSG_PREHEAT_1_END_E                  , _UxGT("Ön Isınma ") PREHEAT_1_LABEL _UxGT(" Nozul ~"));
  FSTRINGVALUE(MSG_PREHEAT_1_ALL                    , _UxGT("Ön Isınma ") PREHEAT_1_LABEL _UxGT(" Tüm"));
  FSTRINGVALUE(MSG_PREHEAT_1_BEDONLY                , _UxGT("Ön Isınma ") PREHEAT_1_LABEL _UxGT(" Tabla"));
  FSTRINGVALUE(MSG_PREHEAT_1_SETTINGS               , _UxGT("Ön Isınma ") PREHEAT_1_LABEL _UxGT(" Ayarlar"));
  FSTRINGVALUE(MSG_PREHEAT_2                        , _UxGT("Ön Isınma ") PREHEAT_2_LABEL);
  FSTRINGVALUE(MSG_PREHEAT_2_H                      , _UxGT("Ön Isınma ") PREHEAT_2_LABEL " ~");
  FSTRINGVALUE(MSG_PREHEAT_2_END                    , _UxGT("Ön Isınma ") PREHEAT_2_LABEL _UxGT(" Nozul"));
  FSTRINGVALUE(MSG_PREHEAT_2_END_E                  , _UxGT("Ön Isınma ") PREHEAT_2_LABEL _UxGT(" Nozul ~"));
  FSTRINGVALUE(MSG_PREHEAT_2_ALL                    , _UxGT("Ön Isınma ") PREHEAT_2_LABEL _UxGT(" Tüm"));
  FSTRINGVALUE(MSG_PREHEAT_2_BEDONLY                , _UxGT("Ön Isınma ") PREHEAT_2_LABEL _UxGT(" Tabla"));
  FSTRINGVALUE(MSG_PREHEAT_2_SETTINGS               , _UxGT("Ön Isınma ") PREHEAT_2_LABEL _UxGT(" Ayarlar"));
  FSTRINGVALUE(MSG_PREHEAT_CUSTOM                   , _UxGT("Özel Ön Isınma"));
  FSTRINGVALUE(MSG_COOLDOWN                         , _UxGT("Soğut"));
  FSTRINGVALUE(MSG_SWITCH_PS_ON                     , _UxGT("Gücü Aç"));
  FSTRINGVALUE(MSG_SWITCH_PS_OFF                    , _UxGT("Gücü Kapat"));
  FSTRINGVALUE(MSG_EXTRUDE                          , _UxGT("Ekstrüzyon"));
  FSTRINGVALUE(MSG_RETRACT                          , _UxGT("Geri Çek"));
  FSTRINGVALUE(MSG_MOVE_AXIS                        , _UxGT("Eksen Hareketleri"));
  FSTRINGVALUE(MSG_BED_LEVELING                     , _UxGT("Tabla Hizalama"));
  FSTRINGVALUE(MSG_LEVEL_BED                        , _UxGT("Tabla Hizası"));
  FSTRINGVALUE(MSG_LEVEL_CORNERS                    , _UxGT("Hizalama Köşeleri"));
  FSTRINGVALUE(MSG_NEXT_CORNER                      , _UxGT("Sonraki Köşe"));
  FSTRINGVALUE(MSG_EDIT_MESH                        , _UxGT("Mesh Düzenle"));
  FSTRINGVALUE(MSG_EDITING_STOPPED                  , _UxGT("Mesh Düzenleme Durdu"));
  FSTRINGVALUE(MSG_MESH_X                           , _UxGT("İndeks X"));
  FSTRINGVALUE(MSG_MESH_Y                           , _UxGT("İndeks Y"));
  FSTRINGVALUE(MSG_MESH_EDIT_Z                      , _UxGT("Z Değeri"));
  FSTRINGVALUE(MSG_USER_MENU                        , _UxGT("Özel Komutlar"));
  FSTRINGVALUE(MSG_IDEX_MENU                        , _UxGT("IDEX Modu"));
  FSTRINGVALUE(MSG_IDEX_MODE_AUTOPARK               , _UxGT("Oto-Park"));
  FSTRINGVALUE(MSG_IDEX_MODE_DUPLICATE              , _UxGT("Kopyala"));
  FSTRINGVALUE(MSG_IDEX_MODE_MIRRORED_COPY          , _UxGT("Yansıtılmış kopya"));
  FSTRINGVALUE(MSG_IDEX_MODE_FULL_CTRL              , _UxGT("Tam Kontrol"));
  FSTRINGVALUE(MSG_X_OFFSET                         , _UxGT("2. nozul X"));
  FSTRINGVALUE(MSG_Y_OFFSET                         , _UxGT("2. nozul Y"));
  FSTRINGVALUE(MSG_Z_OFFSET                         , _UxGT("2. nozul Z"));
  FSTRINGVALUE(MSG_UBL_DOING_G29                    , _UxGT("G29 Çalışıyor"));
  FSTRINGVALUE(MSG_UBL_TOOLS                        , _UxGT("UBL Araçları"));
  FSTRINGVALUE(MSG_UBL_LEVEL_BED                    , _UxGT("UBL Yatak Hizalama"));
  FSTRINGVALUE(MSG_UBL_MANUAL_MESH                  , _UxGT("Elle Mesh Oluştur"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT                    , _UxGT("Altlık & Ölçü Ver"));
  FSTRINGVALUE(MSG_UBL_BC_INSERT2                   , _UxGT("Ölçü"));
  FSTRINGVALUE(MSG_UBL_BC_REMOVE                    , _UxGT("Yataktan Ölçü Kaldır"));
  FSTRINGVALUE(MSG_UBL_MOVING_TO_NEXT               , _UxGT("Sonrakine Git"));
  FSTRINGVALUE(MSG_UBL_ACTIVATE_MESH                , _UxGT("UBL'yi Etkinleştir"));
  FSTRINGVALUE(MSG_UBL_DEACTIVATE_MESH              , _UxGT("UBL'yi Etkisizleştir"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_BED                 , _UxGT("Yatak Sıcaklığı"));
  FSTRINGVALUE(MSG_UBL_BED_TEMP_CUSTOM              , _UxGT("Yatak Sıcaklığı"));
  FSTRINGVALUE(MSG_UBL_SET_TEMP_HOTEND              , _UxGT("Nozul Sıcaklığı"));
  FSTRINGVALUE(MSG_UBL_HOTEND_TEMP_CUSTOM           , _UxGT("Nozul Sıcaklığı"));
  FSTRINGVALUE(MSG_UBL_MESH_EDIT                    , _UxGT("Mesh Düzenleme"));
  FSTRINGVALUE(MSG_UBL_EDIT_CUSTOM_MESH             , _UxGT("Özel Mesh Düzenleme"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_MESH               , _UxGT("İnce Ayar Mesh"));
  FSTRINGVALUE(MSG_UBL_DONE_EDITING_MESH            , _UxGT("Mesh Düzenleme Tamam"));
  FSTRINGVALUE(MSG_UBL_BUILD_CUSTOM_MESH            , _UxGT("Özel Mesh Oluştur"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_MENU              , _UxGT("Mesh Oluştur"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M1                , _UxGT("Mesh Oluştur (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_MESH_M2                , _UxGT("Mesh Oluştur (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_BUILD_COLD_MESH              , _UxGT("Soğuk Mesh Oluştur"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_ADJUST           , _UxGT("Mesh Yükseklik Ayarı"));
  FSTRINGVALUE(MSG_UBL_MESH_HEIGHT_AMOUNT           , _UxGT("Yükseklik miktarı"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_MENU           , _UxGT("Doğrulama Mesh"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M1             , _UxGT("Doğrulama Mesh (") PREHEAT_1_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_MESH_M2             , _UxGT("Doğrulama Mesh (") PREHEAT_2_LABEL _UxGT(")"));
  FSTRINGVALUE(MSG_UBL_VALIDATE_CUSTOM_MESH         , _UxGT("Özel Mesh Doğrulama"));
  FSTRINGVALUE(MSG_UBL_CONTINUE_MESH                , _UxGT("Tabla Mesh Devam et"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVELING                , _UxGT("Mesh Hizalama"));
  FSTRINGVALUE(MSG_UBL_3POINT_MESH_LEVELING         , _UxGT("3-Nokta Hizalama"));
  FSTRINGVALUE(MSG_UBL_GRID_MESH_LEVELING           , _UxGT("Kafes Mesh Hizalama"));
  FSTRINGVALUE(MSG_UBL_MESH_LEVEL                   , _UxGT("Mesh Seviyesi"));
  FSTRINGVALUE(MSG_UBL_SIDE_POINTS                  , _UxGT("Yan Noktalar"));
  FSTRINGVALUE(MSG_UBL_MAP_TYPE                     , _UxGT("Haritalama Türü"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP                   , _UxGT("Mesh Çıkış Haritası"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_HOST              , _UxGT("Host için Çıktı"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_CSV               , _UxGT("CSV için Çıktı"));
  FSTRINGVALUE(MSG_UBL_OUTPUT_MAP_BACKUP            , _UxGT("Yazıcıda Yedek Kpalı"));
  FSTRINGVALUE(MSG_UBL_INFO_UBL                     , _UxGT("UBL Çıkış Bilgisi"));
  FSTRINGVALUE(MSG_UBL_FILLIN_AMOUNT                , _UxGT("Dolgu Miktarı"));
  FSTRINGVALUE(MSG_UBL_MANUAL_FILLIN                , _UxGT("Manuel Dolgu"));
  FSTRINGVALUE(MSG_UBL_SMART_FILLIN                 , _UxGT("Akıllı Dogu"));
  FSTRINGVALUE(MSG_UBL_FILLIN_MESH                  , _UxGT("Mesh Dolgu"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_ALL               , _UxGT("Tümünü Geçersiz Kıl"));
  FSTRINGVALUE(MSG_UBL_INVALIDATE_CLOSEST           , _UxGT("Yakını Geçersiz Kıl"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_ALL                , _UxGT("Tümünü İnce Ayarla"));
  FSTRINGVALUE(MSG_UBL_FINE_TUNE_CLOSEST            , _UxGT("Yakını İnce Ayarla"));
  FSTRINGVALUE(MSG_UBL_STORAGE_MESH_MENU            , _UxGT("Mesh Depolama"));
  FSTRINGVALUE(MSG_UBL_STORAGE_SLOT                 , _UxGT("Bellek Yuvası"));
  FSTRINGVALUE(MSG_UBL_LOAD_MESH                    , _UxGT("Yatak Mesh Yükle"));
  FSTRINGVALUE(MSG_UBL_SAVE_MESH                    , _UxGT("Yatak Mesh Kayıt Et"));
  FSTRINGVALUE(MSG_MESH_LOADED                      , _UxGT("M117 Mesh %i yüklendi"));
  FSTRINGVALUE(MSG_MESH_SAVED                       , _UxGT("M117 Mesh %i kayıtlandı"));
  FSTRINGVALUE(MSG_UBL_NO_STORAGE                   , _UxGT("Depolama Yok"));
  FSTRINGVALUE(MSG_UBL_SAVE_ERROR                   , _UxGT("Hata: UBL Kayıt"));
  FSTRINGVALUE(MSG_UBL_RESTORE_ERROR                , _UxGT("Hata: UBL Yenileme"));
  FSTRINGVALUE(MSG_UBL_Z_OFFSET_STOPPED             , _UxGT("Z-Ofset Durduruldu"));
  FSTRINGVALUE(MSG_UBL_STEP_BY_STEP_MENU            , _UxGT("Adım Adım UBL"));
  FSTRINGVALUE(MSG_UBL_1_BUILD_COLD_MESH            , _UxGT("1.Soğuk Mesh Oluştur"));
  FSTRINGVALUE(MSG_UBL_2_SMART_FILLIN               , _UxGT("2.Akıllı Dogu"));
  FSTRINGVALUE(MSG_UBL_3_VALIDATE_MESH_MENU         , _UxGT("3.Doğrulama Mesh"));
  FSTRINGVALUE(MSG_UBL_4_FINE_TUNE_ALL              , _UxGT("4.Tümünü İnce Ayarla"));
  FSTRINGVALUE(MSG_UBL_5_VALIDATE_MESH_MENU         , _UxGT("5.Doğrulama Mesh"));
  FSTRINGVALUE(MSG_UBL_6_FINE_TUNE_ALL              , _UxGT("6.Tümünü İnce Ayarla"));
  FSTRINGVALUE(MSG_UBL_7_SAVE_MESH                  , _UxGT("7.Yatak Mesh Kayıt Et"));

  FSTRINGVALUE(MSG_LED_CONTROL                      , _UxGT("LED Kontrolü"));
  FSTRINGVALUE(MSG_LEDS                             , _UxGT("Işıklar"));
  FSTRINGVALUE(MSG_LED_PRESETS                      , _UxGT("Işık Hazır Ayarları"));
  FSTRINGVALUE(MSG_SET_LEDS_RED                     , _UxGT("Kırmızı"));
  FSTRINGVALUE(MSG_SET_LEDS_ORANGE                  , _UxGT("Turuncu"));
  FSTRINGVALUE(MSG_SET_LEDS_YELLOW                  , _UxGT("Sarı"));
  FSTRINGVALUE(MSG_SET_LEDS_GREEN                   , _UxGT("Yeşil"));
  FSTRINGVALUE(MSG_SET_LEDS_BLUE                    , _UxGT("Mavi"));
  FSTRINGVALUE(MSG_SET_LEDS_INDIGO                  , _UxGT("Lacivert"));
  FSTRINGVALUE(MSG_SET_LEDS_VIOLET                  , _UxGT("Menekşe"));
  FSTRINGVALUE(MSG_SET_LEDS_WHITE                   , _UxGT("Beyaz"));
  FSTRINGVALUE(MSG_SET_LEDS_DEFAULT                 , _UxGT("Varsayılan"));
  FSTRINGVALUE(MSG_CUSTOM_LEDS                      , _UxGT("Özel Işıklar"));
  FSTRINGVALUE(MSG_INTENSITY_R                      , _UxGT("Kırmızı Şiddeti"));
  FSTRINGVALUE(MSG_INTENSITY_G                      , _UxGT("Yeşil Şiddeti"));
  FSTRINGVALUE(MSG_INTENSITY_B                      , _UxGT("Mavi Şiddeti"));
  FSTRINGVALUE(MSG_INTENSITY_W                      , _UxGT("Beyaz Şiddeti"));
  FSTRINGVALUE(MSG_LED_BRIGHTNESS                   , _UxGT("Parlaklık"));
  FSTRINGVALUE(MSG_MOVING                           , _UxGT("Hareket Ediyor.."));
  FSTRINGVALUE(MSG_FREE_XY                          , _UxGT("Durdur XY"));
  FSTRINGVALUE(MSG_MOVE_X                           , _UxGT("X Hareketi"));
  FSTRINGVALUE(MSG_MOVE_Y                           , _UxGT("Y Hareketi"));
  FSTRINGVALUE(MSG_MOVE_Z                           , _UxGT("Z Hareketi"));
  FSTRINGVALUE(MSG_MOVE_E                           , _UxGT("Ekstruder"));
  FSTRINGVALUE(MSG_MOVE_EN                          , _UxGT("Ekstruder *"));
  FSTRINGVALUE(MSG_HOTEND_TOO_COLD                  , _UxGT("Nozul Çok Soğuk"));
  FSTRINGVALUE(MSG_MOVE_Z_DIST                      , _UxGT("%smm"));
  FSTRINGVALUE(MSG_MOVE_01MM                        , _UxGT("0.1mm"));
  FSTRINGVALUE(MSG_MOVE_1MM                         , _UxGT("1mm"));
  FSTRINGVALUE(MSG_MOVE_10MM                        , _UxGT("10mm"));
  FSTRINGVALUE(MSG_SPEED                            , _UxGT("Hız"));
  FSTRINGVALUE(MSG_BED_Z                            , _UxGT("Z Mesafesi"));
  FSTRINGVALUE(MSG_NOZZLE                           , _UxGT("Nozul"));
  FSTRINGVALUE(MSG_NOZZLE_N                         , _UxGT("Nozul ~"));
  FSTRINGVALUE(MSG_BED                              , _UxGT("Tabla"));
  FSTRINGVALUE(MSG_FAN_SPEED                        , _UxGT("Fan Hızı"));
  FSTRINGVALUE(MSG_FAN_SPEED_N                      , _UxGT("Fan Hızı ="));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED                  , _UxGT("Ekstra Fan Hızı"));
  FSTRINGVALUE(MSG_EXTRA_FAN_SPEED_N                , _UxGT("Ekstra Fan Hızı ="));
  FSTRINGVALUE(MSG_FLOW                             , _UxGT("Akış"));
  FSTRINGVALUE(MSG_FLOW_N                           , _UxGT("Akış ~"));
  FSTRINGVALUE(MSG_CONTROL                          , _UxGT("Kontrol"));
  FSTRINGVALUE(MSG_MIN                              , " " LCD_STR_THERMOMETER _UxGT(" Min"));
  FSTRINGVALUE(MSG_MAX                              , " " LCD_STR_THERMOMETER _UxGT(" Max"));
  FSTRINGVALUE(MSG_FACTOR                           , " " LCD_STR_THERMOMETER _UxGT(" Çarpan"));
  FSTRINGVALUE(MSG_AUTOTEMP                         , _UxGT("Oto. Sıcaklık"));
  FSTRINGVALUE(MSG_LCD_ON                           , _UxGT("Açık"));
  FSTRINGVALUE(MSG_LCD_OFF                          , _UxGT("Kapalı"));
  FSTRINGVALUE(MSG_SELECT                           , _UxGT("Seç"));
  FSTRINGVALUE(MSG_SELECT_E                         , _UxGT("Seç *"));
  FSTRINGVALUE(MSG_ACC                              , _UxGT("İvme"));
  FSTRINGVALUE(MSG_JERK                             , _UxGT("Sarsım"));
  FSTRINGVALUE(MSG_VA_JERK                          , _UxGT("V") LCD_STR_A _UxGT("-Sarsım"));
  FSTRINGVALUE(MSG_VB_JERK                          , _UxGT("V") LCD_STR_B _UxGT("-Sarsım"));
  FSTRINGVALUE(MSG_VC_JERK                          , _UxGT("V") LCD_STR_C _UxGT("-Sarsım"));
  FSTRINGVALUE(MSG_VE_JERK                          , _UxGT("Ve-Sarsım"));
  FSTRINGVALUE(MSG_JUNCTION_DEVIATION               , _UxGT("Jonksiyon Sapması"));
  FSTRINGVALUE(MSG_VELOCITY                         , _UxGT("Hız Vektörü"));
  FSTRINGVALUE(MSG_VMAX_A                           , _UxGT("HızVektör.max ") LCD_STR_A);
  FSTRINGVALUE(MSG_VMAX_B                           , _UxGT("HızVektör.max ") LCD_STR_B);
  FSTRINGVALUE(MSG_VMAX_C                           , _UxGT("HızVektör.max ") LCD_STR_C);
  FSTRINGVALUE(MSG_VMAX_E                           , _UxGT("HızVektör.max ") LCD_STR_E);
  FSTRINGVALUE(MSG_VMAX_EN                          , _UxGT("HızVektör.max *"));
  FSTRINGVALUE(MSG_VMIN                             , _UxGT("HızVektör.min"));
  FSTRINGVALUE(MSG_VTRAV_MIN                        , _UxGT("HV.gezinme min"));
  FSTRINGVALUE(MSG_ACCELERATION                     , _UxGT("Ivme"));
  FSTRINGVALUE(MSG_AMAX_A                           , _UxGT("Max. ivme ") LCD_STR_A);
  FSTRINGVALUE(MSG_AMAX_B                           , _UxGT("Max. ivme ") LCD_STR_B);
  FSTRINGVALUE(MSG_AMAX_C                           , _UxGT("Max. ivme ") LCD_STR_C);
  FSTRINGVALUE(MSG_AMAX_E                           , _UxGT("Max. ivme ") LCD_STR_E);
  FSTRINGVALUE(MSG_AMAX_EN                          , _UxGT("Max. ivme *"));
  FSTRINGVALUE(MSG_A_RETRACT                        , _UxGT("Ivme-geri çekme"));
  FSTRINGVALUE(MSG_A_TRAVEL                         , _UxGT("Ivme-gezinme"));
  FSTRINGVALUE(MSG_STEPS_PER_MM                     , _UxGT("Adım/mm"));
  FSTRINGVALUE(MSG_A_STEPS                          , LCD_STR_A _UxGT(" adım/mm"));
  FSTRINGVALUE(MSG_B_STEPS                          , LCD_STR_B _UxGT(" adım/mm"));
  FSTRINGVALUE(MSG_C_STEPS                          , LCD_STR_C _UxGT(" adım/mm"));
  FSTRINGVALUE(MSG_E_STEPS                          , _UxGT("E adım/mm"));
  FSTRINGVALUE(MSG_EN_STEPS                         , _UxGT("* adım/mm"));
  FSTRINGVALUE(MSG_TEMPERATURE                      , _UxGT("Sıcaklık"));
  FSTRINGVALUE(MSG_MOTION                           , _UxGT("Hareket"));
  FSTRINGVALUE(MSG_FILAMENT                         , _UxGT("Filaman"));
  FSTRINGVALUE(MSG_VOLUMETRIC_ENABLED               , _UxGT("Ekstrüzyon/mm³"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM                    , _UxGT("Filaman Çapı"));
  FSTRINGVALUE(MSG_FILAMENT_DIAM_E                  , _UxGT("Filaman Çapı *"));
  FSTRINGVALUE(MSG_FILAMENT_UNLOAD                  , _UxGT("Çıkart mm"));
  FSTRINGVALUE(MSG_FILAMENT_LOAD                    , _UxGT("Yükle mm"));
  FSTRINGVALUE(MSG_ADVANCE_K                        , _UxGT("K İlerlet"));
  FSTRINGVALUE(MSG_ADVANCE_K_E                      , _UxGT("K İlerlet *"));
  FSTRINGVALUE(MSG_CONTRAST                         , _UxGT("LCD Kontrast"));
  FSTRINGVALUE(MSG_STORE_EEPROM                     , _UxGT("Hafızaya Al"));
  FSTRINGVALUE(MSG_LOAD_EEPROM                      , _UxGT("Hafızadan Yükle"));
  FSTRINGVALUE(MSG_RESTORE_FAILSAFE                 , _UxGT("Fabrika Ayarları"));
  FSTRINGVALUE(MSG_INIT_EEPROM                      , _UxGT("EEPROM'u başlat"));
  FSTRINGVALUE(MSG_MEDIA_UPDATE                     , _UxGT("SD Güncellemesi"));
  FSTRINGVALUE(MSG_RESET_PRINTER                    , _UxGT("Yazıcıyı Resetle"));
  FSTRINGVALUE(MSG_REFRESH                          , LCD_STR_REFRESH  _UxGT("Yenile"));
  FSTRINGVALUE(MSG_INFO_SCREEN                      , _UxGT("Bilgi Ekranı"));
  FSTRINGVALUE(MSG_PREPARE                          , _UxGT("Hazırlık"));
  FSTRINGVALUE(MSG_TUNE                             , _UxGT("Ayar"));
  FSTRINGVALUE(MSG_PAUSE_PRINT                      , _UxGT("Duraklat"));
  FSTRINGVALUE(MSG_RESUME_PRINT                     , _UxGT("Sürdür"));
  FSTRINGVALUE(MSG_STOP_PRINT                       , _UxGT("Durdur"));
  FSTRINGVALUE(MSG_OUTAGE_RECOVERY                  , _UxGT("Kesinti Kurtarma"));
  FSTRINGVALUE(MSG_MEDIA_MENU                       , _UxGT("SD Karttan Yazdır"));
  FSTRINGVALUE(MSG_NO_MEDIA                         , _UxGT("SD Kart Yok!"));
  FSTRINGVALUE(MSG_DWELL                            , _UxGT("Uyku..."));
  FSTRINGVALUE(MSG_USERWAIT                         , _UxGT("Operatör bekleniyor."));
  FSTRINGVALUE(MSG_PRINT_PAUSED                     , _UxGT("Baskı Duraklatıldı"));
  FSTRINGVALUE(MSG_PRINTING                         , _UxGT("Baskı Yapılıyor..."));
  FSTRINGVALUE(MSG_PRINT_ABORTED                    , _UxGT("Baskı Durduruldu!"));
  FSTRINGVALUE(MSG_NO_MOVE                          , _UxGT("İşlem yok."));
  FSTRINGVALUE(MSG_KILLED                           , _UxGT("Kilitlendi. "));
  FSTRINGVALUE(MSG_STOPPED                          , _UxGT("Durdu. "));
  FSTRINGVALUE(MSG_CONTROL_RETRACT                  , _UxGT("Geri Çek mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_SWAP             , _UxGT("Swap Re.mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACTF                 , _UxGT("Geri Çekme V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_ZHOP             , _UxGT("Atlama mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER          , _UxGT("UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAP     , _UxGT("S UnRet mm"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVERF         , _UxGT("UnRet  V"));
  FSTRINGVALUE(MSG_CONTROL_RETRACT_RECOVER_SWAPF    , _UxGT("S UnRet V"));
  FSTRINGVALUE(MSG_AUTORETRACT                      , _UxGT("Oto. Geri Çekme"));
  FSTRINGVALUE(MSG_FILAMENT_SWAP_LENGTH             , _UxGT("G.Çekme Boyu"));
  FSTRINGVALUE(MSG_TOOL_CHANGE                      , _UxGT("Takım Değişimi"));
  FSTRINGVALUE(MSG_TOOL_CHANGE_ZLIFT                , _UxGT("Z Yükselt"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_PRIME_SPD           , _UxGT("Birincil Hız"));
  FSTRINGVALUE(MSG_SINGLENOZZLE_RETRACT_SPD         , _UxGT("Geri Çekme Hızı"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE                   , _UxGT("Filaman Değiştir"));
  FSTRINGVALUE(MSG_FILAMENTCHANGE_E                 , _UxGT("Filaman Değiştir *"));
  FSTRINGVALUE(MSG_FILAMENTLOAD                     , _UxGT("Filaman Yükle"));
  FSTRINGVALUE(MSG_FILAMENTLOAD_E                   , _UxGT("Filaman Yükle *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD                   , _UxGT("Filaman Çıkart"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_E                 , _UxGT("Filaman Çıkart *"));
  FSTRINGVALUE(MSG_FILAMENTUNLOAD_ALL               , _UxGT("Tümünü Çıkart"));
  FSTRINGVALUE(MSG_INIT_MEDIA                       , _UxGT("SD Kart Başlatılıyor"));
  FSTRINGVALUE(MSG_CHANGE_MEDIA                     , _UxGT("SD Kart Değiştir"));
  FSTRINGVALUE(MSG_ZPROBE_OUT                       , _UxGT("Z Prob Açık. Tabla"));
  FSTRINGVALUE(MSG_SKEW_FACTOR                      , _UxGT("Çarpıklık Faktörü"));
  FSTRINGVALUE(MSG_BLTOUCH                          , _UxGT("BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_SELFTEST                 , _UxGT("BLTouch Self-Test"));
  FSTRINGVALUE(MSG_BLTOUCH_RESET                    , _UxGT("Sıfırla BLTouch"));
  FSTRINGVALUE(MSG_BLTOUCH_DEPLOY                   , _UxGT("BLTouch Aç"));
  FSTRINGVALUE(MSG_BLTOUCH_STOW                     , _UxGT("BLTouch Kapat"));
  FSTRINGVALUE(MSG_MANUAL_DEPLOY                    , _UxGT("Z-Prob Aç"));
  FSTRINGVALUE(MSG_MANUAL_STOW                      , _UxGT("Z-Sensör Kapat"));
  FSTRINGVALUE(MSG_HOME_FIRST                       , _UxGT("Sıfırla %s%s%s Önce"));
  FSTRINGVALUE(MSG_BABYSTEP_X                       , _UxGT("Miniadım X"));
  FSTRINGVALUE(MSG_BABYSTEP_Y                       , _UxGT("Miniadım Y"));
  FSTRINGVALUE(MSG_BABYSTEP_Z                       , _UxGT("Miniadım Z"));
  FSTRINGVALUE(MSG_ENDSTOP_ABORT                    , _UxGT("Endstop iptal"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD               , _UxGT("Isınma başarısız"));
  FSTRINGVALUE(MSG_HEATING_FAILED_LCD_BED           , _UxGT("Yatak Isınma Başrsız"));
  FSTRINGVALUE(MSG_ERR_REDUNDANT_TEMP               , _UxGT("Hata: Sıcaklık Aşımı"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY                  , _UxGT("TERMAL PROBLEM"));
  FSTRINGVALUE(MSG_THERMAL_RUNAWAY_BED              , _UxGT("TABLA TERMAL PROBLEM"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP                      , _UxGT("Hata: MAX.SICAKLIK"));
  FSTRINGVALUE(MSG_ERR_MINTEMP                      , _UxGT("Hata: MIN.SICAKLIK"));
  FSTRINGVALUE(MSG_ERR_MAXTEMP_BED                  , _UxGT("Hata: MAX.SIC. TABLA"));
  FSTRINGVALUE(MSG_ERR_MINTEMP_BED                  , _UxGT("Hata: MIN.SIC. TABLA"));
  FSTRINGVALUE(MSG_ERR_Z_HOMING                     , _UxGT("Önce XY Sıfırla"));
  FSTRINGVALUE(MSG_HALTED                           , _UxGT("YAZICI DURDURULDU"));
  FSTRINGVALUE(MSG_PLEASE_RESET                     , _UxGT("Lütfen Resetleyin"));
  FSTRINGVALUE(MSG_SHORT_DAY                        , _UxGT("G")); // One character only
  FSTRINGVALUE(MSG_SHORT_HOUR                       , _UxGT("S")); // One character only
  FSTRINGVALUE(MSG_SHORT_MINUTE                     , _UxGT("D")); // One character only
  FSTRINGVALUE(MSG_HEATING                          , _UxGT("Isınıyor..."));
  FSTRINGVALUE(MSG_COOLING                          , _UxGT("Soğuyor..."));
  FSTRINGVALUE(MSG_BED_HEATING                      , _UxGT("Tabla Isınıyor..."));
  FSTRINGVALUE(MSG_BED_COOLING                      , _UxGT("Tabla Soğuyor..."));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE                  , _UxGT("Delta Kalibrasyonu"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_X                , _UxGT("Ayarla X"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Y                , _UxGT("Ayarla Y"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_Z                , _UxGT("Ayarla Z"));
  FSTRINGVALUE(MSG_DELTA_CALIBRATE_CENTER           , _UxGT("Ayarla Merkez"));
  FSTRINGVALUE(MSG_DELTA_SETTINGS                   , _UxGT("Delta Ayarları"));
  FSTRINGVALUE(MSG_DELTA_AUTO_CALIBRATE             , _UxGT("Oto Kalibrasyon"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT_CALIBRATE           , _UxGT("Delta Yük. Ayarla"));
  FSTRINGVALUE(MSG_DELTA_Z_OFFSET_CALIBRATE         , _UxGT("Z Prob Ofseti"));
  FSTRINGVALUE(MSG_DELTA_DIAG_ROD                   , _UxGT("Çapral Mil"));
  FSTRINGVALUE(MSG_DELTA_HEIGHT                     , _UxGT("Yükseklik"));
  FSTRINGVALUE(MSG_DELTA_RADIUS                     , _UxGT("Yarıçap"));
  FSTRINGVALUE(MSG_INFO_MENU                        , _UxGT("Yazıcı Hakkında"));
  FSTRINGVALUE(MSG_INFO_PRINTER_MENU                , _UxGT("Yazıcı Bilgisi"));
  FSTRINGVALUE(MSG_3POINT_LEVELING                  , _UxGT("3-Nokta Hizalama"));
  FSTRINGVALUE(MSG_LINEAR_LEVELING                  , _UxGT("Doğrusal Hizalama"));
  FSTRINGVALUE(MSG_BILINEAR_LEVELING                , _UxGT("İki Yönlü Doğ. Hiza."));
  FSTRINGVALUE(MSG_UBL_LEVELING                     , _UxGT("Birleşik Tabla Hiza."));
  FSTRINGVALUE(MSG_MESH_LEVELING                    , _UxGT("Mesh Hizalama"));
  FSTRINGVALUE(MSG_INFO_STATS_MENU                  , _UxGT("İstatistikler"));
  FSTRINGVALUE(MSG_INFO_BOARD_MENU                  , _UxGT("Kontrolcü Bilgisi"));
  FSTRINGVALUE(MSG_INFO_THERMISTOR_MENU             , _UxGT("Termistörler"));
  FSTRINGVALUE(MSG_INFO_EXTRUDERS                   , _UxGT("Ekstruderler"));
  FSTRINGVALUE(MSG_INFO_BAUDRATE                    , _UxGT("İletişim Hızı"));
  FSTRINGVALUE(MSG_INFO_PROTOCOL                    , _UxGT("Protokol"));
  FSTRINGVALUE(MSG_CASE_LIGHT                       , _UxGT("Aydınlatmayı Aç"));
  FSTRINGVALUE(MSG_CASE_LIGHT_BRIGHTNESS            , _UxGT("Aydınlatma Parlaklğı"));

  FSTRINGVALUE(MSG_EXPECTED_PRINTER                 , _UxGT("Yanlış Yazıcı"));

  #if LCD_WIDTH >= 20
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Baskı Sayısı"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Tamamlanan"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Toplam Baskı Süresi"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("En Uzun Baskı Süresi"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Toplam Filaman"));
  #else
    FSTRINGVALUE(MSG_INFO_PRINT_COUNT               , _UxGT("Baskı"));
    FSTRINGVALUE(MSG_INFO_COMPLETED_PRINTS          , _UxGT("Tamamlanan"));
    FSTRINGVALUE(MSG_INFO_PRINT_TIME                , _UxGT("Süre"));
    FSTRINGVALUE(MSG_INFO_PRINT_LONGEST             , _UxGT("En Uzun"));
    FSTRINGVALUE(MSG_INFO_PRINT_FILAMENT            , _UxGT("Filaman"));
  #endif
  FSTRINGVALUE(MSG_INFO_MIN_TEMP                    , _UxGT("Min Sıc."));
  FSTRINGVALUE(MSG_INFO_MAX_TEMP                    , _UxGT("Max Sıc."));
  FSTRINGVALUE(MSG_INFO_PSU                         , _UxGT("Güç Kaynağı"));
  FSTRINGVALUE(MSG_DRIVE_STRENGTH                   , _UxGT("Sürücü Gücü"));
  FSTRINGVALUE(MSG_DAC_PERCENT                      , _UxGT("Sürücü %"));
  FSTRINGVALUE(MSG_DAC_EEPROM_WRITE                 , _UxGT("DAC EEPROM Yaz"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_PAUSE     , _UxGT("BASKI DURAKLATILDI"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_LOAD      , _UxGT("FILAMAN YüKLE"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEADER_UNLOAD    , _UxGT("FILAMAN ÇIKART"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_HEADER    , _UxGT("Seçenekler:"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_PURGE     , _UxGT("Daha Fazla Tasviye"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_OPTION_RESUME    , _UxGT("Baskıyı sürdür"));
  FSTRINGVALUE(MSG_FILAMENT_CHANGE_NOZZLE           , _UxGT("  Nozul: "));
  FSTRINGVALUE(MSG_RUNOUT_SENSOR                    , _UxGT("Runout Sensörü"));
  FSTRINGVALUE(MSG_LCD_HOMING_FAILED                , _UxGT("Sıfırlama Başarısız"));
  FSTRINGVALUE(MSG_LCD_PROBING_FAILED               , _UxGT("Probing Başarısız"));
  FSTRINGVALUE(MSG_M600_TOO_COLD                    , _UxGT("M600: Çok Soğuk"));
  //
  // Filament Değiştirme ekranları, 4 satırlı bir ekranda 3 satıra kadar gösterilir
  //                        ...veya 3 satırlı ekranda 2 satıra kadar
  #if LCD_HEIGHT >= 4
    // Up to 3 lines allowed
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_2_LINE("Baskıya devam etmek", "için Butona bas")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_3_LINE("Filaman değişimi", "için başlama", "bekleniyor")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_3_LINE("Filamanı yükle", "ve devam için", "tuşa bas...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_2_LINE("Nozulü Isıtmak için", "Butona Bas.")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_2_LINE("Nozul Isınıyor", "Lütfen Bekleyin...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_2_LINE("Filamanın çıkması", "bekleniyor")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_2_LINE("Filamanın yüklenmesi", "bekleniyor..")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_2_LINE("Filaman Temizlemesi", "için bekle")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_2_LINE("Filaman Temizlemesi", "bitirmek için tıkla")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_2_LINE("Baskının devam ", "etmesi için bekle")));
  #else // LCD_HEIGHT < 4
    // Up to 2 lines allowed
    FSTRINGVALUE(MSG_ADVANCED_PAUSE_WAITING         , _UxGT(MSG_1_LINE("Sürdürmek İçin Tıkla")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INIT           , _UxGT(MSG_1_LINE("Lütfen bekleyiniz...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_INSERT         , _UxGT(MSG_1_LINE("Yükle ve bas")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEAT           , _UxGT(MSG_1_LINE("Isıtmak için Tıkla")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_HEATING        , _UxGT(MSG_1_LINE("Isınıyor...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_UNLOAD         , _UxGT(MSG_1_LINE("Çıkartılıyor...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_LOAD           , _UxGT(MSG_1_LINE("Yüklüyor...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_PURGE          , _UxGT(MSG_1_LINE("Temizleniyor...")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_CONT_PURGE     , _UxGT(MSG_1_LINE("Bitirmek için Tıkla")));
    FSTRINGVALUE(MSG_FILAMENT_CHANGE_RESUME         , _UxGT(MSG_1_LINE("Sürdürülüyor...")));
  #endif // LCD_HEIGHT < 4
}
