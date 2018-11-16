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

//
// status_screen_dogm.cpp
// Standard Status Screen for Graphical Display
//

#include "../../../../MK4duo.h"

#if HAS_GRAPHICAL_LCD

#include "status_screen_dogm.h"
#include "../lcdprint.h"

#if ENABLED(LASER)
  #include "../../../feature/laser/laserbitmaps.h"
#endif

FORCE_INLINE void _draw_centered_temp(const int16_t temp, const uint8_t x, const uint8_t y) {
  PGM_P const str = itostr3(temp);
  lcd_moveto(x - (str[0] != ' ' ? 0 : str[1] != ' ' ? 1 : 2) * MENU_FONT_WIDTH / 2, y);
  lcd_put_u8str(str);
  lcd_put_u8str_P(PSTR(LCD_STR_DEGREE " "));
}

#if DISABLED(HEAT_INDICATOR_X)
  #define HEAT_INDICATOR_X 8
#endif

FORCE_INLINE void _draw_heater_status(const uint8_t x, const uint8_t heater, const bool blink) {

  #if HAS_TEMP_BED
    const bool isBed = (heater == BED_INDEX);
  #else
    const bool isBed = false;
  #endif

  if (PAGE_UNDER(7)) {
    const int16_t targetTemperature = heaters[heater].isIdle() ? heaters[heater].idle_temperature : heaters[heater].target_temperature;
    if (blink || !heaters[heater].isIdle())
      _draw_centered_temp((float)targetTemperature, x, 7);
  }

  if (PAGE_CONTAINS(21, 28))
    _draw_centered_temp(heaters[heater].current_temperature + 0.5f, x, 28);

  if (PAGE_CONTAINS(17, 20)) {
    const uint8_t h = isBed ? 7 : HEAT_INDICATOR_X,
                  y = isBed ? 18 : 17;
    if (heaters[heater].isHeating()) {
      u8g.setColorIndex(0); // white on black
      u8g.drawBox(x + h, y, 2, 2);
      u8g.setColorIndex(1); // black on white
    }
    else {
      u8g.drawBox(x + h, y, 2, 2);
    }
  }
}

//
// Before homing, blink '123' <-> '???'.
// Homed and known, display constantly.
//
FORCE_INLINE void _draw_axis_value(const AxisEnum axis, PGM_P value, const bool blink) {
  if (blink)
    lcd_put_u8str(value);
  else {
    if (!printer.isAxisHomed(axis))
      while (const char c = *value++) lcd_put_wchar(c <= '.' ? c : '?');
    else
      lcd_put_u8str(value);
  }
}

void LcdUI::draw_status_message(const bool blink) {

  // Get the UTF8 character count of the string
  uint8_t slen = utf8_strlen(status_message);

  #if ENABLED(STATUS_MESSAGE_SCROLLING)

    static bool last_blink = false;

    if (slen <= LCD_WIDTH) {
      // The string fits within the line. Print with no scrolling
      lcd_put_u8str(status_message);
      for (; slen < LCD_WIDTH; ++slen) lcd_put_wchar(' ');
    }
    else {
      // String is longer than the available space

      // Get a pointer to the next valid UTF8 character
      PGM_P stat = status_message + status_scroll_offset;

      // Get the string remaining length
      const uint8_t rlen = utf8_strlen(stat);

      if (rlen >= LCD_WIDTH) {
        // The remaining string fills the screen - Print it
        lcd_put_u8str_max(stat, LCD_PIXEL_WIDTH);
      }
      else {
        // The remaining string does not completely fill the screen
        lcd_put_u8str_max(stat, LCD_PIXEL_WIDTH);         // The string leaves space
        uint8_t chars = LCD_WIDTH - rlen;                 // Amount of space left in characters

        lcd_put_wchar('.');                               // Always at 1+ spaces left, draw a dot
        if (--chars) {                                    // Draw a second dot if there's space
          lcd_put_wchar('.');
          if (--chars) {
            // Print a second copy of the message
            lcd_put_u8str_max(status_message, LCD_PIXEL_WIDTH - (rlen + 2) * (MENU_FONT_WIDTH));
          }
        }
      }
      if (last_blink != blink) {
        last_blink = blink;

        // Adjust by complete UTF8 characters
        if (status_scroll_offset < slen) {
          status_scroll_offset++;
          while (!START_OF_UTF8_CHAR(status_message[status_scroll_offset]))
            status_scroll_offset++;
        }
        else
          status_scroll_offset = 0;
      }
    }

  #else // !STATUS_MESSAGE_SCROLLING

    UNUSED(blink);

    // Just print the string to the LCD
    lcd_put_u8str_max(status_message, LCD_PIXEL_WIDTH);

    // Fill the rest with spaces
    for (; slen < LCD_WIDTH; ++slen) lcd_put_wchar(' ');

  #endif // !STATUS_MESSAGE_SCROLLING
}

void LcdUI::draw_status_screen() {

  const bool blink = get_blink();

  // Status Menu Font
  set_font(FONT_STATUSMENU);

  //
  // Fan Animation
  //
  // Draw the entire heading image bitmap rather than each element
  // separately. This is an optimization because it's slower to draw
  // multiple elements than a single bitmap.
  //
  // The bitmap:
  //  - May be offset in X
  //  - Includes all nozzle(s), bed(s), and the fan.
  //
  // TODO:
  //
  //  - Only draw the whole header on the first
  //    entry to the status screen. Nozzle, bed, and
  //    fan outline bits don't change.
  //
  #if FAN_ANIM_FRAMES > 2
    static bool old_blink;
    static uint8_t fan_frame;
    if (old_blink != blink) {
      old_blink = blink;
      if (!fans[0].Speed || ++fan_frame >= FAN_ANIM_FRAMES) fan_frame = 0;
    }
  #endif

  #if ENABLED(LASER)

    if (printer.mode == PRINTER_MODE_LASER) {
      #if ENABLED(LASER_PERIPHERALS)
        if (laser.peripherals_ok()) {
          u8g.drawBitmapP(29,4, LASERENABLE_BYTEWIDTH, LASERENABLE_HEIGHT, laserenable_bmp);
        }
      #endif

      lcd_moveto(3,6);
      if (stepper.laser_status()) {
        u8g.drawBitmapP(5,14, ICON_BYTEWIDTH, ICON_HEIGHT, laseron_bmp);
        lcd_put_u8str(itostr3(stepper.laser_intensity()));
        lcd_put_u8str_P(PSTR("%"));
      } else {
        u8g.drawBitmapP(5,14, ICON_BYTEWIDTH, ICON_HEIGHT, laseroff_bmp);
        lcd_put_u8str_P(PSTR("---%"));
      }
    }
    else

  #endif

  {
    if (PAGE_UNDER(STATUS_SCREENHEIGHT + 1)) {
      u8g.drawBitmapP(
        STATUS_SCREEN_X, STATUS_SCREEN_Y,
        (STATUS_SCREENWIDTH + 7) / 8, STATUS_SCREENHEIGHT,
        #if HAS_FAN0
          #if FAN_ANIM_FRAMES > 2
            fan_frame == 1 ? status_screen1_bmp :
            fan_frame == 2 ? status_screen2_bmp :
            #if FAN_ANIM_FRAMES > 3
              fan_frame == 3 ? status_screen3_bmp :
            #endif
          #else
            blink && fans[0].Speed ? status_screen1_bmp :
          #endif
        #endif
        status_screen0_bmp
      );
    }
  }

  if (printer.mode == PRINTER_MODE_FFF) {

    //
    // Temperature Graphics and Info
    //

    if (PAGE_UNDER(28)) {
      // Hotends
      LOOP_HOTEND() _draw_heater_status(STATUS_SCREEN_HOTEND_TEXT_X(h), h, blink);

      // Heated bed
      #if HOTENDS < 4 && HAS_TEMP_BED
        _draw_heater_status(STATUS_SCREEN_BED_TEXT_X, BED_INDEX, blink);
      #endif

      #if HAS_FAN0
        if (PAGE_CONTAINS(STATUS_SCREEN_FAN_TEXT_Y - 7, STATUS_SCREEN_FAN_TEXT_Y)) {
          // Fan
          const int16_t per = ((fans[0].Speed + 1) * 100) / 256;
          if (per) {
            lcd_moveto(STATUS_SCREEN_FAN_TEXT_X, STATUS_SCREEN_FAN_TEXT_Y);
            lcd_put_u8str(itostr3(per));
            lcd_put_wchar('%');
          }
        }
      #endif
    }
  }

  #if HAS_SD_SUPPORT
    //
    // SD Card Symbol
    //
    if (card.isFileOpen() && PAGE_CONTAINS(42, 51)) {
      // Upper box
      u8g.drawBox(42, 42, 8, 7);     // 42-48 (or 41-47)
      // Right edge
      u8g.drawBox(50, 44, 2, 5);     // 44-48 (or 43-47)
      // Bottom hollow box
      u8g.drawFrame(42, 49, 10, 4);  // 49-52 (or 48-51)
      // Corner pixel
      u8g.drawPixel(50, 43);         // 43 (or 42)
    }
  #endif // SDSUPPORT

  //
  // Progress bar frame
  //

  #define PROGRESS_BAR_X 54
  #define PROGRESS_BAR_WIDTH (LCD_PIXEL_WIDTH - PROGRESS_BAR_X)

  if (PAGE_CONTAINS(49, 52))       // 49-52
    u8g.drawFrame(
      PROGRESS_BAR_X, 49,
      PROGRESS_BAR_WIDTH, 4
    );

  //
  // Progress bar solid part
  //

  if (printer.progress && (PAGE_CONTAINS(50, 51)))  // 50-51
    u8g.drawBox(
      PROGRESS_BAR_X + 1, 50,
      (uint16_t)((PROGRESS_BAR_WIDTH - 2) * printer.progress * 0.01), 2
    );

  //
  // Elapsed Time
  //

  if (PAGE_CONTAINS(41, 48)) {

    char buffer1[10], buffer2[10];
    duration_t elapsed  = print_job_counter.duration();
    duration_t finished = (print_job_counter.duration() * (100 - printer.progress)) / (printer.progress + 0.1);
    (void)elapsed.toDigital(buffer1, false);
    (void)finished.toDigital(buffer2, false);

    #if HAS_LCD_POWER_SENSOR
      if (millis() < print_millis + 1000) {
        lcd_moveto(54, 48);
        lcd_put_wchar('S');
        lcd_put_u8str(buffer1);

        lcd_moveto(92, 48);
        lcd_put_wchar('E');
        lcd_put_u8str(buffer2);
      }
      else {
        lcd_put_u8str(itostr4(powerManager.consumption_hour - powerManager.startpower));
        lcd_put_u8str((char*)"Wh");
      }
    #else
      lcd_moveto(54, 48);
      lcd_put_wchar('S');
      lcd_put_u8str(buffer1);

      lcd_moveto(92, 48);
      lcd_put_wchar('E');
      lcd_put_u8str(buffer2);
    #endif
  }

  //
  // XYZ Coordinates
  //

  #define XYZ_BASELINE (30 + INFO_FONT_ASCENT)

  #define X_LABEL_POS  3
  #define X_VALUE_POS 11
  #define XYZ_SPACING 40

  #if ENABLED(XYZ_HOLLOW_FRAME)
    #define XYZ_FRAME_TOP 29
    #define XYZ_FRAME_HEIGHT INFO_FONT_ASCENT + 3
  #else
    #define XYZ_FRAME_TOP 30
    #define XYZ_FRAME_HEIGHT INFO_FONT_ASCENT + 1
  #endif

  static char xstring[5], ystring[5], zstring[8];
  #if HAS_LCD_FILAMENT_SENSOR
    static char wstring[5], mstring[4];
  #endif

  // At the first page, regenerate the XYZ strings
  if (first_page) {
    strcpy(xstring, ftostr4sign(LOGICAL_X_POSITION(mechanics.current_position[X_AXIS])));
    strcpy(ystring, ftostr4sign(LOGICAL_Y_POSITION(mechanics.current_position[Y_AXIS])));
    strcpy(zstring, ftostr52sp (LOGICAL_Z_POSITION(mechanics.current_position[Z_AXIS])));
    #if HAS_LCD_FILAMENT_SENSOR
      strcpy(wstring, ftostr12ns(filament_width_meas));
      strcpy(mstring, itostr3(100.0 * (
          printer.isVolumetric()
            ? tools.volumetric_area_nominal / tools.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
            : tools.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
        )
      ));
    #endif
  }

  if (PAGE_CONTAINS(XYZ_FRAME_TOP, XYZ_FRAME_TOP + XYZ_FRAME_HEIGHT - 1)) {

    #if ENABLED(XYZ_HOLLOW_FRAME)
      u8g.drawFrame(0, XYZ_FRAME_TOP, LCD_PIXEL_WIDTH, XYZ_FRAME_HEIGHT); // 8: 29-40  7: 29-39
    #else
      u8g.drawBox(0, XYZ_FRAME_TOP, LCD_PIXEL_WIDTH, XYZ_FRAME_HEIGHT);   // 8: 30-39  7: 30-37
    #endif

    if (PAGE_CONTAINS(XYZ_BASELINE - (INFO_FONT_ASCENT - 1), XYZ_BASELINE)) {

      #if DISABLED(XYZ_HOLLOW_FRAME)
        u8g.setColorIndex(0); // white on black
      #endif

      lcd_moveto(0 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      lcd_put_wchar('X');
      lcd_moveto(0 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      _draw_axis_value(X_AXIS, xstring, blink);

      lcd_moveto(1 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      lcd_put_wchar('Y');
      lcd_moveto(1 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      _draw_axis_value(Y_AXIS, ystring, blink);

      lcd_moveto(2 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      lcd_put_wchar('Z');
      lcd_moveto(2 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      _draw_axis_value(Z_AXIS, zstring, blink);

      #if DISABLED(XYZ_HOLLOW_FRAME)
        u8g.setColorIndex(1); // black on white
      #endif
    }
  }

  //
  // Feedrate
  //
  #define EXTRAS_BASELINE 50

  if (PAGE_CONTAINS(EXTRAS_BASELINE - (INFO_FONT_HEIGHT - 1), EXTRAS_BASELINE)) {
    set_font(FONT_MENU);
    lcd_moveto(3, EXTRAS_BASELINE);
    lcd_put_wchar(LCD_STR_FEEDRATE[0]);

    set_font(FONT_STATUSMENU);
    lcd_moveto(12, EXTRAS_BASELINE);
    lcd_put_u8str(itostr3(mechanics.feedrate_percentage));
    lcd_put_wchar('%');

    //
    // Filament sensor display if SD is disabled
    //
    #if HAS_LCD_FILAMENT_SENSOR && DISABLED(SDSUPPORT)
      lcd_moveto(56, EXTRAS_BASELINE);
      lcd_put_u8str(wstring);
      lcd_moveto(102, EXTRAS_BASELINE);
      lcd_put_u8str(mstring);
      lcd_put_wchar('%');
      set_font(FONT_MENU);
      lcd_moveto(47, EXTRAS_BASELINE);
      lcd_put_wchar(LCD_STR_FILAM_DIA[0]); // lcd_put_u8str_P(PSTR(LCD_STR_FILAM_DIA));
      lcd_moveto(93, EXTRAS_BASELINE);
      lcd_put_wchar(LCD_STR_FILAM_MUL[0]);
    #endif
  }

  //
  // Status line
  //

  #define STATUS_BASELINE (LCD_PIXEL_HEIGHT - INFO_FONT_DESCENT)

  if (PAGE_CONTAINS(STATUS_BASELINE - (INFO_FONT_ASCENT - 1), STATUS_BASELINE)) {
    lcd_moveto(0, STATUS_BASELINE);

    #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
      if (PENDING(millis(), previous_status_ms + 5000UL)) { // Display both Status message line and Filament display on the last line
        lcd_implementation_status_message(blink);
      }

      #if HAS_LCD_POWER_SENSOR
        #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT))
          else if (PENDING(millis(), previous_status_ms + 10000UL))
        #else
          else
        #endif
          {
            lcd_put_u8str_P(PSTR("P:"));
            lcd_put_u8str(ftostr31(powerManager.consumption_meas));
            lcd_put_u8str_P(PSTR("W C:"));
            lcd_put_u8str(ltostr7(powerManager.consumption_hour));
            lcd_put_u8str_P(PSTR("Wh"));
          }
      #endif

      #if HAS_LCD_FILAMENT_SENSOR && HAS_SD_SUPPORT
        else {
          lcd_put_u8str_P(PSTR(LCD_STR_FILAM_DIA));
          lcd_put_wchar(':');
          lcd_put_u8str(wstring);
          lcd_put_u8str_P(PSTR("  " LCD_STR_FILAM_MUL));
          lcd_put_wchar(':');
          lcd_put_u8str(mstring);
          lcd_put_wchar('%');
        }
      #endif
    #else
      draw_status_message(blink);
    #endif
  }
}

#endif // HAS_GRAPHICAL_LCD
