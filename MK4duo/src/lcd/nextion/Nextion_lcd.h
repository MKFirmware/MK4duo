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

// For debug Connect
//#define NEXTION_CONNECT_DEBUG

#if ENABLED(NEXTION)

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    #include "../../feature/advanced_pause/advanced_pause.h"
  #endif

  void sethotPopCallback(void *ptr);

  #if FAN_COUNT > 0
    void setfanPopCallback(void *ptr);
  #endif

  #if HAS_CASE_LIGHT
    void setlightPopCallback(void *ptr);
  #endif

  void setmovePopCallback(void *ptr);
  void setgcodePopCallback(void *ptr);
  void sendPopCallback(void *ptr);
  void filamentPopCallback(void *ptr);
  void nextion_draw_update();
  void lcd_scrollinfo(const char* titolo, const char* message);
  void lcd_yesno(const uint8_t val, const char* msg1="", const char* msg2="", const char* msg3="");

  #if ENABLED(NEXTION_GFX)
    void gfx_origin(const float x, const float y, const float z);
    void gfx_scale(const float scale);
    void gfx_clear(const float x, const float y, const float z, bool force_clear=false);
    void gfx_cursor_to(const float x, const float y, const float z, bool force_cursor=false);
    void gfx_line_to(const float x, const float y, const float z);
    void gfx_plane_to(const float x, const float y, const float z);
  #endif

  #if HAS_SD_SUPPORT
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

  #if ENABLED(PROBE_MANUALLY)
    void ProbelPopCallBack(void *ptr);
    float lcd_probe_pt(const float &lx, const float &ly);
    #if HAS_LEVELING
      void Nextion_ProbeOn();
      void Nextion_ProbeOff();
    #endif
  #endif

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void lcd_advanced_pause_show_message(const AdvancedPauseMessage message,
                                         const AdvancedPauseMode mode=ADVANCED_PAUSE_MODE_PAUSE_PRINT,
                                         const uint8_t extruder=tools.active_extruder);
  #endif

  #if ENABLED(RFID_MODULE)
    void rfidPopCallback(void *ptr);
    void rfid_setText(const char* message, uint32_t color = 65535);
  #endif

#endif // ENABLED(NEXTION)

#endif /* _NEXTION_LCD_H_ */
