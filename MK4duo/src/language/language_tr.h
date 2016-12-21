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
 * Turkish
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_TR_H
#define LANGUAGE_TR_H

#define MAPPER_C2C3_TR
#define DISPLAY_CHARSET_ISO10646_TR

#if DISABLED(DOGLCD)
  #error "Turkish needs a graphical display."
#endif

#define WELCOME_MSG                         MACHINE_NAME _UxGT(" hazir.")                               // hazir.
#define MSG_SD_INSERTED                     _UxGT("SD Yerlesti.")                                       // SD Yerlesti.
#define MSG_SD_REMOVED                      _UxGT("SD �ikarildi.")                                      // SD �ikarildi.
#define MSG_LCD_ENDSTOPS                    _UxGT("Endstops") // Max length 8 characters                // Endstops
#define MSG_MAIN                            _UxGT("Ana")                                                // Ana
#define MSG_AUTOSTART                       _UxGT("Otobaslat")                                          // Otobaslat
#define MSG_DISABLE_STEPPERS                _UxGT("Motorlari Durdur")                                   // Motorlari Durdur
#define MSG_AUTO_HOME                       _UxGT("Eksenleri Sifirla")                                  // Eksenleri Sifirla
#define MSG_AUTO_HOME_X                     _UxGT("X Sifirla")                                          // X Sifirla
#define MSG_AUTO_HOME_Y                     _UxGT("Y Sifirla")                                          // Y Sifirla
#define MSG_AUTO_HOME_Z                     _UxGT("Z Sifirla")                                          // Z Sifirla
#define MSG_LEVEL_BED_HOMING                _UxGT("XYZ Sifirlaniyor")                                   // XYZ Sifirlaniyor
#define MSG_LEVEL_BED_WAITING               _UxGT("Baslatmak i�in tikla")                               // Baslatmak i�in tikla
#define MSG_LEVEL_BED_NEXT_POINT            _UxGT("Siradaki Nokta")                                     // Siradaki Nokta
#define MSG_LEVEL_BED_DONE                  _UxGT("Seviyeleme Tamam!")                                  // Seviyeleme Tamam!
#define MSG_LEVEL_BED_CANCEL                _UxGT("Iptal")                                              // Iptal
#define MSG_SET_HOME_OFFSETS                _UxGT("Offset Ayarla")                                      // Offset Ayarla
#define MSG_HOME_OFFSETS_APPLIED            _UxGT("Offset Tamam")                                       // Offset Tamam
#define MSG_SET_ORIGIN                      _UxGT("Sifir Belirle")                                      // Sifir Belirle
#define MSG_PREHEAT_1                       _UxGT("�n Isinma PLA")                                      // �n Isinma PLA
#define MSG_PREHEAT_1_N                     MSG_PREHEAT_1 _UxGT(" ")                                    //  
#define MSG_PREHEAT_1_ALL                   MSG_PREHEAT_1 _UxGT(" T�m")                                 //  T�m
#define MSG_PREHEAT_1_BEDONLY               MSG_PREHEAT_1 _UxGT(" Tabla")                               //  Tabla
#define MSG_PREHEAT_1_SETTINGS              MSG_PREHEAT_1 _UxGT(" Ayar")                                //  Ayar
#define MSG_PREHEAT_2                       _UxGT("�n Isinma ABS")                                      // �n Isinma ABS
#define MSG_PREHEAT_2_N                     MSG_PREHEAT_2 _UxGT(" ")                                    //  
#define MSG_PREHEAT_2_ALL                   MSG_PREHEAT_2 _UxGT(" T�m")                                 //  T�m
#define MSG_PREHEAT_2_BEDONLY               MSG_PREHEAT_2 _UxGT(" Tabla")                               //  Tabla
#define MSG_PREHEAT_2_SETTINGS              MSG_PREHEAT_2 _UxGT(" Ayar")                                //  Ayar
#define MSG_COOLDOWN                        _UxGT("Sogut")                                              // Sogut
#define MSG_SWITCH_PS_ON                    _UxGT("G�c� A�")                                            // G�c� A�
#define MSG_SWITCH_PS_OFF                   _UxGT("G�c� Kapat")                                         // G�c� Kapat
#define MSG_EXTRUDE                         _UxGT("Extrude")                                            // Extrude
#define MSG_RETRACT                         _UxGT("Geri �ek")                                           // Geri �ek
#define MSG_MOVE_AXIS                       _UxGT("Eksen Y�net")                                        // Eksenleri Y�net
#define MSG_LEVEL_BED                       _UxGT("Tabla Seviyele")                                     // Tabla Seviyele
#define MSG_MOVE_X                          _UxGT("X")                                                  // X
#define MSG_MOVE_Y                          _UxGT("Y")                                                  // Y
#define MSG_MOVE_Z                          _UxGT("Z")                                                  // Z
#define MSG_MOVE_E                          _UxGT("Ekstruder")                                          // Ekstruder
#define MSG_MOVE_01MM                       _UxGT("0.1mm")                                              // 0.1mm
#define MSG_MOVE_1MM                        _UxGT("1mm")                                                // 1mm
#define MSG_MOVE_10MM                       _UxGT("10mm")                                               // 10mm
#define MSG_SPEED                           _UxGT("Hiz")                                                // Hiz
#define MSG_BED_Z                           _UxGT("Tabla Z")                                            // Tabla Z
#define MSG_NOZZLE                          _UxGT("Noz�l")                                              // Noz�l
#define MSG_BED                             _UxGT("Tabla")                                              // Tabla
#define MSG_FAN_SPEED                       _UxGT("Fan Hizi")                                           // Fan Hizi
#define MSG_FLOW                            _UxGT("Akis")                                               // Akis
#define MSG_CONTROL                         _UxGT("Kontrol")                                            // Kontrol
#define MSG_MIN                             _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" Min")                //  Min
#define MSG_MAX                             _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" Max")                //  Max
#define MSG_FACTOR                          _UxGT(" ") LCD_STR_THERMOMETER _UxGT(" �arpan")             //  �arpan
#define MSG_AUTOTEMP                        _UxGT("Autotemp")                                           //  Autotemp
#define MSG_ON                              _UxGT("On ")                                                // On 
#define MSG_OFF                             _UxGT("Off")                                                // Off
#define MSG_PID_P                           _UxGT("PID-P")                                              // PID-P
#define MSG_PID_I                           _UxGT("PID-I")                                              // PID-I
#define MSG_PID_D                           _UxGT("PID-D")                                              // PID-D
#define MSG_PID_C                           _UxGT("PID-C")                                              // PID-C
#define MSG_SELECT                          _UxGT("Se�")                                                // Se�
#define MSG_ACC                             _UxGT("Ivme")                                               // Ivme
#define MSG_VX_JERK                         _UxGT("Vx-Jerk")                                            // Vx-Jerk
#define MSG_VY_JERK                         _UxGT("Vy-Jerk")                                            // Vy-Jerk
#define MSG_VZ_JERK                         _UxGT("Vz-jerk")                                            // Vz-Jerk
#define MSG_VE_JERK                         _UxGT("Ve-jerk")                                            // Ve-Jerk
#define MSG_VMAX                            _UxGT("Vmax ")                                              // Vmax
#define MSG_VMIN                            _UxGT("Vmin")                                               // Vmin
#define MSG_VTRAV_MIN                       _UxGT("VTrav min")                                          // Vtrav min
#define MSG_AMAX                            _UxGT("Amax ")                                              // Amax
#define MSG_A_RETRACT                       _UxGT("A-retract")                                          // A-retract
#define MSG_A_TRAVEL                        _UxGT("A-travel")                                           // A-travel
#define MSG_XSTEPS                          _UxGT("Xsteps/mm")                                          // Xsteps/mm
#define MSG_YSTEPS                          _UxGT("Ysteps/mm")                                          // Ysteps/mm
#define MSG_ZSTEPS                          _UxGT("Zsteps/mm")                                          // Zsteps/mm
#define MSG_ESTEPS                          _UxGT("Esteps/mm")                                          // Esteps/mm
#define MSG_E1STEPS                         _UxGT("E1steps/mm")                                         // E1steps/mm
#define MSG_E2STEPS                         _UxGT("E2steps/mm")                                         // E2steps/mm
#define MSG_E3STEPS                         _UxGT("E3steps/mm")                                         // E3steps/mm
#define MSG_E4STEPS                         _UxGT("E4steps/mm")                                         // E4steps/mm
#define MSG_TEMPERATURE                     _UxGT("Sicaklik")                                           // Sicaklik
#define MSG_MOTION                          _UxGT("Hareket")                                            // Hareket
#define MSG_VOLUMETRIC                      _UxGT("Filaman")                                            // Filaman
#define MSG_VOLUMETRIC_ENABLED              _UxGT("E in mm3")                                           // E in mm3
#define MSG_FILAMENT_DIAM                   _UxGT("Fil. �ap")                                           // Fil. �ap
#define MSG_CONTRAST                        _UxGT("LCD Kontrast")                                       // LCD Kontrast
#define MSG_STORE_EPROM                     _UxGT("Hafizaya Al")                                        // Hafizaya Al
#define MSG_LOAD_EPROM                      _UxGT("Hafizadan Y�kle")                                    // Hafizadan Y�kle
#define MSG_RESTORE_FAILSAFE                _UxGT("Fabrika Ayarlari")                                   // Fabrika Ayarlari
#define MSG_REFRESH                         _UxGT("Yenile")                                             // Yenile
#define MSG_WATCH                           _UxGT("Bilgi Ekrani")                                       // Bilgi Ekrani
#define MSG_PREPARE                         _UxGT("Hazirlik")                                           // Hazirlik
#define MSG_TUNE                            _UxGT("Ayar")                                               // Ayar
#define MSG_PAUSE_PRINT                     _UxGT("Duraklat")                                           // Duraklat
#define MSG_RESUME_PRINT                    _UxGT("S�rd�r")                                             // S�rd�r
#define MSG_STOP_PRINT                      _UxGT("Durdur")                                             // Durdur
#define MSG_CARD_MENU                       _UxGT("SD den Yazdir")                                      // SD den Yazdir
#define MSG_NO_CARD                         _UxGT("SD Kart Yok")                                        // SD Kart Yok
#define MSG_DWELL                           _UxGT("Uyku...")                                            // Uyku...
#define MSG_USERWAIT                        _UxGT("Operat�r bekleniyor...")                             // Operat�r bekleniyor...
#define MSG_RESUMING                        _UxGT("Baski S�rd�r�l�yor")                                 // Baski S�rd�r�l�yor
#define MSG_PRINT_ABORTED                   _UxGT("Baski Durduruldu")                                   // Baski Durduruldu
#define MSG_NO_MOVE                         _UxGT("Islem yok.")                                         // Islem yok.
#define MSG_KILLED                          _UxGT("Kilitlendi. ")                                       // Kilitlendi.
#define MSG_STOPPED                         _UxGT("Durdu. ")                                            // Durdu.
#define MSG_CONTROL_RETRACT                 _UxGT("Geri �ek mm")                                        // Geri �ek mm
#define MSG_CONTROL_RETRACT_SWAP            _UxGT("Swap Re.mm")                                         // Swap Re.mm
#define MSG_CONTROL_RETRACTF                _UxGT("Geri �ekme  V")                                      // Geri �ekme V
#define MSG_CONTROL_RETRACT_ZLIFT           _UxGT("Hop mm")                                             // Hop mm
#define MSG_CONTROL_RETRACT_RECOVER         _UxGT("UnRet +mm")                                          // UnRet +mm
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    _UxGT("S UnRet+mm")                                         // S UnRet+mm
#define MSG_CONTROL_RETRACT_RECOVERF        _UxGT("UnRet  V")                                           // UnRet V
#define MSG_AUTORETRACT                     _UxGT("AutoRetr.")                                          // AutoRetr.
#define MSG_FILAMENTCHANGE                  _UxGT("Filaman Degistir")                                   // Filaman Degistir
#define MSG_INIT_SDCARD                     _UxGT("Init. SD")                                           // Init. SD
#define MSG_CNG_SDCARD                      _UxGT("SD Degistir")                                        // SD Degistir
#define MSG_ZPROBE_OUT                      _UxGT("Z Prob A�ik. Tabla")                                 // Z Prob A�ik. Tabla
#define MSG_BLTOUCH_SELFTEST                _UxGT("BLTouch Self-Test")                                  // BLTouch Self-Test
#define MSG_BLTOUCH_RESET                   _UxGT("Sifirla BLTouch")                                    // Sifirla BLTouch
#define MSG_HOME                            _UxGT("Sifirla")                                            // Sifirla
#define MSG_FIRST                           _UxGT("�nce")                                               // �nce
#define MSG_ZPROBE_ZOFFSET                  _UxGT("Z Offset")                                           // Z Offset
#define MSG_BABYSTEP_X                      _UxGT("Miniadim X")                                         // Miniadim X
#define MSG_BABYSTEP_Y                      _UxGT("Miniadim Y")                                         // Miniadim Y
#define MSG_BABYSTEP_Z                      _UxGT("Miniadim Z")                                         // Miniadim Z
#define MSG_ENDSTOP_ABORT                   _UxGT("Endstop iptal")                                      // Endstop iptal
#define MSG_HEATING_FAILED_LCD              _UxGT("Isinma basarisiz")                                   // Isinma basarisiz
#define MSG_ERR_REDUNDANT_TEMP              _UxGT("Hata: Ge�ersiz Sicaklik")                            // Hata: Ge�ersiz Sicaklik
#define MSG_THERMAL_RUNAWAY                 _UxGT("TERMAL PROBLEM")                                     // TERMAL PROBLEM
#define MSG_ERR_MAXTEMP                     _UxGT("Hata: MAXSICAKLIK")                                  // Hata: MAXSICAKLIK
#define MSG_ERR_MINTEMP                     _UxGT("Hata: MINSICAKLIK")                                  // Hata: MINSICAKLIK
#define MSG_ERR_MAXTEMP_BED                 _UxGT("Hata: MAXSIC. TABLA")                                // Hata: MAXSIC. TABLA
#define MSG_ERR_MINTEMP_BED                 _UxGT("Hata: MINSIC. TABLA")                                // Hata: MINSIC. TABLA
#define MSG_ERR_Z_HOMING                    _UxGT("G28 Z Yapilamaz")                                    // G28 Z Yapilamaz
#define MSG_HALTED                          _UxGT("YAZICI DURDURULDU")                                  // YAZICI DURDURULDU
#define MSG_PLEASE_RESET                    _UxGT("L�tfen resetleyin")                                  // L�tfen resetleyin
#define MSG_SHORT_DAY                       _UxGT("G") // One character only                            // G
#define MSG_SHORT_HOUR                      _UxGT("S") // One character only                            // S
#define MSG_SHORT_MINUTE                    _UxGT("D") // One character only                            // D
#define MSG_HEATING                         _UxGT("Isiniyor...")                                        // Isiniyor...
#define MSG_HEATING_COMPLETE                _UxGT("Isinma tamam.")                                      // Isinma tamam.
#define MSG_BED_HEATING                     _UxGT("Tabla Isiniyor.")                                    // Tabla Isiniyor.
#define MSG_BED_DONE                        _UxGT("Tabla hazir.")                                       // Tabla hazir.
#define MSG_DELTA_CALIBRATE                 _UxGT("Delta Kalibrasyonu")                                 // Delta Kalibrasyonu
#define MSG_DELTA_CALIBRATE_X               _UxGT("Ayarla X")                                           // Ayarla X
#define MSG_DELTA_CALIBRATE_Y               _UxGT("Ayarla Y")                                           // Ayarla Y
#define MSG_DELTA_CALIBRATE_Z               _UxGT("Ayarla Z")                                           // Ayarla Z
#define MSG_DELTA_CALIBRATE_CENTER          _UxGT("Ayarla Merkez")                                      // Ayarla Merkez

#define MSG_INFO_MENU                       _UxGT("Yazici Hakkinda")                                    // Yazici Hakkinda
#define MSG_INFO_PRINTER_MENU               _UxGT("Yazici Bilgisi")                                     // Yazici Bilgisi
#define MSG_INFO_STATS_MENU                 _UxGT("Istatistikler")                                      // Istatistikler
#define MSG_INFO_BOARD_MENU                 _UxGT("Kontrol�r Bilgisi")                                  // Kontrol Bilgisi
#define MSG_INFO_THERMISTOR_MENU            _UxGT("Termist�rler")                                       // Termist�rler
#define MSG_INFO_EXTRUDERS                  _UxGT("Ekstruderler")                                       // Ekstruderler
#define MSG_INFO_BAUDRATE                   _UxGT("Iletisim Hizi")                                      // Iletisim Hizi
#define MSG_INFO_PROTOCOL                   _UxGT("Protokol")                                           // Protokol
#define MSG_LIGHTS_ON                       _UxGT("Aydinlatmayi A�")                                    // Aydinlatmayi A�
#define MSG_LIGHTS_OFF                      _UxGT("Aydinlatmayi Kapa")                                  // Aydinlaymayi Kapa

#if LCD_WIDTH > 19
  #define MSG_INFO_PRINT_COUNT              _UxGT("Baski Sayisi")                                       // Baski Sayisi
  #define MSG_INFO_COMPLETED_PRINTS         _UxGT("Tamamlanan")                                         // Tamamlanan
  #define MSG_INFO_PRINT_TIME               _UxGT("Toplam Baski S�resi")                                // Toplam Baski S�resi
  #define MSG_INFO_PRINT_LONGEST            _UxGT("En Uzun Baski S�resi")                               // En Uzun Baski S�resi
  #define MSG_INFO_PRINT_FILAMENT           _UxGT("Toplam Filaman")                                     // Toplam Filaman
#else
  #define MSG_INFO_PRINT_COUNT              _UxGT("Baski")                                              // Baski
  #define MSG_INFO_COMPLETED_PRINTS         _UxGT("Tamamlanan")                                         // Tamamlanan
  #define MSG_INFO_PRINT_TIME               _UxGT("S�re")                                               // S�re
  #define MSG_INFO_PRINT_LONGEST            _UxGT("En Uzun")                                            // En Uzun
  #define MSG_INFO_PRINT_FILAMENT           _UxGT("Filaman")                                            // Filaman
#endif

#define MSG_INFO_MIN_TEMP                   _UxGT("Min Sic.")                                           // Min Sicak.
#define MSG_INFO_MAX_TEMP                   _UxGT("Max Sic.")                                           // Max Sicak.
#define MSG_INFO_PSU                        _UxGT("G�� Kaynagi")                                        // G�� Kaynagi

#define MSG_DRIVE_STRENGTH                  _UxGT("S�r�c� G�c�")                                        // S�r�c� G�c�
#define MSG_DAC_PERCENT                     _UxGT("S�r�c� %")                                           // S�r�c� %
#define MSG_DAC_EEPROM_WRITE                _UxGT("DAC'i EEPROM'a Yaz")                                 // DAC'i EEPROM'a Yaz
#define MSG_FILAMENT_CHANGE_HEADER          _UxGT("Filaman Degistir")                                   // Filaman Degistir
#define MSG_FILAMENT_CHANGE_OPTION_HEADER   _UxGT("Se�enekler:")                                        // Se�enekler:
#define MSG_FILAMENT_CHANGE_OPTION_EXTRUDE  _UxGT("Daha Akit")                                          // Daha Akit
#define MSG_FILAMENT_CHANGE_OPTION_RESUME   _UxGT("Baskiyi s�rd�r")                                     // Baskiyi s�rd�r

#if LCD_HEIGHT >= 4
  // Up to 3 lines allowed
  #define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("Baslama bekleniyor")                               // Baslama bekleniyor
  #define MSG_FILAMENT_CHANGE_INIT_2          _UxGT("filamanin")                                        // filamanin
  #define MSG_FILAMENT_CHANGE_INIT_3          _UxGT("degisimi")                                         // degisimi
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("Bekleniyor")                                       // Bekleniyor
  #define MSG_FILAMENT_CHANGE_UNLOAD_2        _UxGT("filamanin �ikmasi")                                // filamanin �ikmasi
  #define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("Filamani y�kle")                                   // Filamani y�kle
  #define MSG_FILAMENT_CHANGE_INSERT_2        _UxGT("ve devam i�in")                                    // ve devam i�in
  #define MSG_FILAMENT_CHANGE_INSERT_3        _UxGT("tusa bas...")                                      // tusa bas...
  #define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("Bekleniyor")                                       // Bekleniyor
  #define MSG_FILAMENT_CHANGE_LOAD_2          _UxGT("filamanin y�klenmesi")                             // filamanin y�klenmesi
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       _UxGT("Bekleniyor")                                       // Bekleniyor
  #define MSG_FILAMENT_CHANGE_EXTRUDE_2       _UxGT("filaman akmasi")                                   // filaman akmasi
  #define MSG_FILAMENT_CHANGE_RESUME_1        _UxGT("Baskinin s�rd�r�lmesini")                          // Baskinin s�rd�r�lmesini
  #define MSG_FILAMENT_CHANGE_RESUME_2        _UxGT("bekle")                                            // bekle
#else // LCD_HEIGHT < 4
  // Up to 2 lines allowed
  #define MSG_FILAMENT_CHANGE_INIT_1          _UxGT("L�tfen bekleyiniz...")                             // L�tfen bekleyiniz...
  #define MSG_FILAMENT_CHANGE_UNLOAD_1        _UxGT("�ikartiliyor...")                                  // �ikartiliyor...
  #define MSG_FILAMENT_CHANGE_INSERT_1        _UxGT("Y�kle ve bas")                                     // Y�kle ve bas
  #define MSG_FILAMENT_CHANGE_LOAD_1          _UxGT("Y�kl�yor...")                                      // Y�kl�yor...
  #define MSG_FILAMENT_CHANGE_EXTRUDE_1       _UxGT("Akitiliyor...")                                    // Akitiliyor...
  #define MSG_FILAMENT_CHANGE_RESUME_1        _UxGT("S�rd�r�l�yor...")                                  // S�rd�r�l�yor...
#endif // LCD_HEIGHT < 4

#endif // LANGUAGE_TR_H
