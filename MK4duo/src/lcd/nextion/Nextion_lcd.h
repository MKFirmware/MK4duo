/**
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
 * Nextion_lcd.h
 *
 * Copyright (c) 2014-2016 Alberto Cotronei @MagoKimbra
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _NEXTION_LCD_H_
#define _NEXTION_LCD_H_

#define LCD_DECIMAL_SMALL_XY

#if ENABLED(NEXTION)

  void hotPopCallback(void *ptr);
  void sethotPopCallback(void *ptr);
  void settempPopCallback(void *ptr);

  #if FAN_COUNT > 0
    void setfanPopCallback(void *ptr);
  #endif

  void setmovePopCallback(void *ptr);
  void setgcodePopCallback(void *ptr);
  void sendPopCallback(void *ptr);
  void filamentPopCallback(void *ptr);
  void lcd_key_touch_update();
  void nextion_draw_update();
  void lcd_init();
  void lcd_setstatus(const char* message, const bool persist=false);
  void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...);
  void lcd_setstatusPGM(const char* message, const int8_t level=0);
  void lcd_setalertstatusPGM(const char * const message);
  void lcd_reset_alert_level();
  void lcd_scrollinfo(const char* titolo, const char* message);

  #if ENABLED(NEXTION_GFX)
    void gfx_origin(const float x, const float y, const float z);
    void gfx_scale(const float scale);
    void gfx_clear(const float x, const float y, const float z, bool force_clear=false);
    void gfx_cursor_to(const float x, const float y, const float z, bool force_cursor=false);
    void gfx_line_to(const float x, const float y, const float z);
    void gfx_plane_to(const float x, const float y, const float z);
  #endif

  #if HAS_SDSUPPORT
    void sdmountdismountPopCallback(void *ptr);
    void sdlistPopCallback(void *ptr);
    void sdfilePopCallback(void *ptr);
    void sdfolderPopCallback(void *ptr);
    void sdfolderUpPopCallback(void *ptr);
    void PlayPausePopCallback(void *ptr);
    void StopPopCallback(void *ptr);
    void DFirmwareCallback(void *ptr);
    void setpageSD();
    void UploadNewFirmware();
  #endif

  #if ENABLED(LCD_BED_LEVELING)
    void bedlevelPopCallBack(void *ptr);
    void LcdBedLevelOn();
    void LcdBedLevelOff();
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void lcd_advanced_pause_show_message(AdvancedPauseMessage message);
  #endif

  #if ENABLED(RFID_MODULE)
    void rfidPopCallback(void *ptr);
    void rfid_setText(const char* message, uint32_t color = 65535);
  #endif

  FORCE_INLINE bool lcd_hasstatus() { return false; }
  FORCE_INLINE void lcd_draw_update() {}
  FORCE_INLINE void lcd_refresh() {}

#endif // ENABLED(NEXTION)

#endif /* _NEXTION_LCD_H_ */
