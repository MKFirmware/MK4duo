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
 * status_screen_DOGM.h
 *
 * Standard Status Screen for Graphical Display
 */

#ifndef _STATUS_SCREEN_DOGM_H_
#define _STATUS_SCREEN_DOGM_H_

FORCE_INLINE void _draw_centered_temp(const int16_t temp, const uint8_t x, const uint8_t y) {
  const char * const str = itostr3(temp);
  u8g.setPrintPos(x - (str[0] != ' ' ? 0 : str[1] != ' ' ? 1 : 2) * DOG_CHAR_WIDTH / 2, y);
  lcd_print(str);
  lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
}

#if DISABLED(HEAT_INDICATOR_X)
  #define HEAT_INDICATOR_X 8
#endif

FORCE_INLINE void _draw_heater_status(const uint8_t x, const uint8_t heater, const bool blink) {
  #if !HEATER_IDLE_HANDLER
    UNUSED(blink);
  #endif

  #if HAS_TEMP_BED
    const bool isBed = (heater == BED_INDEX);
  #else
    const bool isBed = false;
  #endif

  if (PAGE_UNDER(7)) {
    #if HEATER_IDLE_HANDLER
      const bool isIdle = heaters[heater].isIdle();

      if (blink || !isIdle)
    #endif
    _draw_centered_temp((isBed ? heaters[heater].target_temperature : heaters[heater].target_temperature) + 0.5, x, 7); }

  if (PAGE_CONTAINS(21, 28))
    _draw_centered_temp((isBed ? heaters[heater].current_temperature : heaters[heater].current_temperature) + 0.5, x, 28);

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

FORCE_INLINE void _draw_axis_label(const AxisEnum axis, const char* const pstr, const bool blink) {
  if (blink)
    lcd_printPGM(pstr);
  else {
    if (!printer.isAxisHomed(axis))
      u8g.print('?');
    else
      lcd_printPGM(pstr);
  }
}

inline void lcd_implementation_status_message(const bool blink) {
  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    static bool last_blink = false;
    const uint8_t slen = lcd_strlen(lcd_status_message);
    const char *stat = lcd_status_message + status_scroll_pos;
    if (slen <= LCD_WIDTH)
      lcd_print_utf(stat);                                      // The string isn't scrolling
    else {
      if (status_scroll_pos <= slen - LCD_WIDTH)
        lcd_print_utf(stat);                                    // The string fills the screen
      else {
        uint8_t chars = LCD_WIDTH;
        if (status_scroll_pos < slen) {                         // First string still visible
          lcd_print_utf(stat);                                  // The string leaves space
          chars -= slen - status_scroll_pos;                    // Amount of space left
        }
        u8g.print('.');                                         // Always at 1+ spaces left, draw a dot
        if (--chars) {
          if (status_scroll_pos < slen + 1)                     // Draw a second dot if there's space
            --chars, u8g.print('.');
          if (chars) lcd_print_utf(lcd_status_message, chars);  // Print a second copy of the message
        }
      }
      if (last_blink != blink) {
        last_blink = blink;
        // Skip any non-printing bytes
        if (status_scroll_pos < slen) while (!PRINTABLE(lcd_status_message[status_scroll_pos])) status_scroll_pos++;
        if (++status_scroll_pos >= slen + 2) status_scroll_pos = 0;
      }
    }
  #else
    UNUSED(blink);
    lcd_print_utf(lcd_status_message);
  #endif
}

static void lcd_implementation_status_screen() {

  const bool blink = lcd_blink();

  #if FAN_ANIM_FRAMES > 2
    static bool old_blink;
    static uint8_t fan_frame;
    if (old_blink != blink) {
      old_blink = blink;
      if (!fans[0].Speed || ++fan_frame >= FAN_ANIM_FRAMES) fan_frame = 0;
    }
  #endif

  // Status Menu Font
  lcd_setFont(FONT_STATUSMENU);

  #if ENABLED(LASER)

    if (printer.mode == PRINTER_MODE_LASER) {
      #if ENABLED(LASER_PERIPHERALS)
        if (laser.peripherals_ok()) {
          u8g.drawBitmapP(29,4, LASERENABLE_BYTEWIDTH, LASERENABLE_HEIGHT, laserenable_bmp);
        }
      #endif

      u8g.setPrintPos(3,6);
      if (stepper.current_block->laser_status == LASER_ON) {
        u8g.drawBitmapP(5,14, ICON_BYTEWIDTH, ICON_HEIGHT, laseron_bmp);
        u8g.print(itostr3(stepper.current_block->laser_intensity));
        lcd_printPGM(PSTR("%"));
      } else {
        u8g.drawBitmapP(5,14, ICON_BYTEWIDTH, ICON_HEIGHT, laseroff_bmp);
        lcd_printPGM(PSTR("---%"));
      }
    }
    else

  #endif

  {
    //
    // Fan Animation
    //
    // Draws the whole heading image as a B/W bitmap rather than
    // drawing the elements separately.
    // This was done as an optimization, as it was slower to draw
    // multiple parts compared to a single bitmap.
    //
    // The bitmap:
    // - May be offset in X
    // - Includes all nozzle(s), bed(s), and the fan.
    //
    // TODO:
    //
    // - Only draw the whole header on the first
    //   entry to the status screen. Nozzle, bed, and
    //   fan outline bits don't change.
    //
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
        if (PAGE_CONTAINS(20, 27)) {
          // Fan
          const int16_t per = ((fans[0].Speed + 1) * 100) / 256;
          if (per) {
            u8g.setPrintPos(STATUS_SCREEN_FAN_TEXT_X, STATUS_SCREEN_FAN_TEXT_Y);
            lcd_print(itostr3(per));
            u8g.print('%');
          }
        }
      #endif
    }
  }

  #if HAS_SDSUPPORT

    //
    // SD Card Symbol
    //

    if (card.isFileOpen() && PAGE_CONTAINS(42 - (TALL_FONT_CORRECTION), 51 - (TALL_FONT_CORRECTION))) {
      // Upper box
      u8g.drawBox(42, 42 - (TALL_FONT_CORRECTION), 8, 7);     // 42-48 (or 41-47)
      // Right edge
      u8g.drawBox(50, 44 - (TALL_FONT_CORRECTION), 2, 5);     // 44-48 (or 43-47)
      // Bottom hollow box
      u8g.drawFrame(42, 49 - (TALL_FONT_CORRECTION), 10, 4);  // 49-52 (or 48-51)
      // Corner pixel
      u8g.drawPixel(50, 43 - (TALL_FONT_CORRECTION));         // 43 (or 42)
    }

  #endif

  //
  // Progress bar frame
  //

  #define PROGRESS_BAR_X 54
  #define PROGRESS_BAR_WIDTH (LCD_PIXEL_WIDTH - PROGRESS_BAR_X)

  if (PAGE_CONTAINS(49, 52 - (TALL_FONT_CORRECTION)))       // 49-52 (or 49-51)
    u8g.drawFrame(
      PROGRESS_BAR_X, 49,
      PROGRESS_BAR_WIDTH, 4 - (TALL_FONT_CORRECTION)
    );

  //
  // Progress bar solid part
  //

  if (printer.progress && (PAGE_CONTAINS(50, 51 - (TALL_FONT_CORRECTION))))     // 50-51 (or just 50)
    u8g.drawBox(
      PROGRESS_BAR_X + 1, 50,
      (uint16_t)((PROGRESS_BAR_WIDTH - 2) * printer.progress * 0.01), 2 - (TALL_FONT_CORRECTION)
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
        u8g.setPrintPos(54, 48);
        lcd_print('S');
        lcd_print(buffer1);

        u8g.setPrintPos(92, 48);
        u8g.print('E');
        lcd_print(buffer2);
      }
      else {
        lcd_print(itostr4(powerManager.consumption_hour - powerManager.startpower));
        lcd_print((char*)"Wh");
      }
    #else
      u8g.setPrintPos(54, 48);
      lcd_print('S');
      lcd_print(buffer1);

      u8g.setPrintPos(92, 48);
      u8g.print('E');
      lcd_print(buffer2);
    #endif
  }

  //
  // XYZ Coordinates
  //

  #if ENABLED(USE_SMALL_INFOFONT)
    #define INFO_FONT_HEIGHT 7
  #else
    #define INFO_FONT_HEIGHT 8
  #endif

  #define XYZ_BASELINE (30 + INFO_FONT_HEIGHT)

  #define X_LABEL_POS  3
  #define X_VALUE_POS 11
  #define XYZ_SPACING 40

  #if ENABLED(XYZ_HOLLOW_FRAME)
    #define XYZ_FRAME_TOP 29
    #define XYZ_FRAME_HEIGHT INFO_FONT_HEIGHT + 3
  #else
    #define XYZ_FRAME_TOP 30
    #define XYZ_FRAME_HEIGHT INFO_FONT_HEIGHT + 1
  #endif

  // Before homing the axis letters are blinking 'X' <-> '?'.
  // When axis is homed but axis known position is false the axis letters are blinking 'X' <-> ' '.
  // When everything is ok you see a constant 'X'.

  static char xstring[5], ystring[5], zstring[7];
  #if HAS_LCD_FILAMENT_SENSOR
    static char wstring[5], mstring[4];
  #endif

  // At the first page, regenerate the XYZ strings
  if (page.page == 0) {
    strcpy(xstring, ftostr4sign(LOGICAL_X_POSITION(mechanics.current_position[X_AXIS])));
    strcpy(ystring, ftostr4sign(LOGICAL_Y_POSITION(mechanics.current_position[Y_AXIS])));
    strcpy(zstring, ftostr52sp(FIXFLOAT(LOGICAL_Z_POSITION(mechanics.current_position[Z_AXIS]))));
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

    if (PAGE_CONTAINS(XYZ_BASELINE - (INFO_FONT_HEIGHT - 1), XYZ_BASELINE)) {

      #if DISABLED(XYZ_HOLLOW_FRAME)
        u8g.setColorIndex(0); // white on black
      #endif

      u8g.setPrintPos(0 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      _draw_axis_label(X_AXIS, PSTR(MSG_X), blink);
      u8g.setPrintPos(0 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      lcd_print(xstring);

      u8g.setPrintPos(1 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      _draw_axis_label(Y_AXIS, PSTR(MSG_Y), blink);
      u8g.setPrintPos(1 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      lcd_print(ystring);

      u8g.setPrintPos(2 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      _draw_axis_label(Z_AXIS, PSTR(MSG_Z), blink);
      u8g.setPrintPos(2 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      lcd_print(zstring);

      #if DISABLED(XYZ_HOLLOW_FRAME)
        u8g.setColorIndex(1); // black on white
      #endif
    }
  }

  //
  // Feedrate
  //

  if (PAGE_CONTAINS(51 - INFO_FONT_HEIGHT, 49)) {
    lcd_setFont(FONT_MENU);
    u8g.setPrintPos(3, 50);
    lcd_print(LCD_STR_FEEDRATE[0]);

    lcd_setFont(FONT_STATUSMENU);
    u8g.setPrintPos(12, 50);
    lcd_print(itostr3(mechanics.feedrate_percentage));
    u8g.print('%');

    //
    // Filament sensor display if SD is disabled
    //
    #if HAS_LCD_FILAMENT_SENSOR && DISABLED(SDSUPPORT)
      u8g.setPrintPos(56, 50);
      lcd_print(wstring);
      u8g.setPrintPos(102, 50);
      lcd_print(mstring);
      u8g.print('%');
      lcd_setFont(FONT_MENU);
      u8g.setPrintPos(47, 50);
      lcd_print(LCD_STR_FILAM_DIA);
      u8g.setPrintPos(93, 50);
      lcd_print(LCD_STR_FILAM_MUL);
    #endif
  }

  //
  // Status line
  //

  #define STATUS_BASELINE (55 + INFO_FONT_HEIGHT)

  if (PAGE_CONTAINS(STATUS_BASELINE - (INFO_FONT_HEIGHT - 1), STATUS_BASELINE)) {
    u8g.setPrintPos(0, STATUS_BASELINE);

    #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
      if (PENDING(millis(), previous_lcd_status_ms + 5000UL)) { // Display both Status message line and Filament display on the last line
        lcd_implementation_status_message(blink);
      }

      #if HAS_LCD_POWER_SENSOR
        #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT))
          else if (PENDING(millis(), previous_lcd_status_ms + 10000UL))
        #else
          else
        #endif
          {
            lcd_printPGM(PSTR("P:"));
            lcd_print(ftostr31(powerManager.consumption_meas));
            lcd_printPGM(PSTR("W C:"));
            lcd_print(ltostr7(powerManager.consumption_hour));
            lcd_printPGM(PSTR("Wh"));
          }
      #endif

      #if HAS_LCD_FILAMENT_SENSOR && HAS_SDSUPPORT
        else {
          lcd_printPGM(PSTR(LCD_STR_FILAM_DIA));
          u8g.print(':');
          lcd_print(ftostr12ns(filament_width_meas));
          lcd_printPGM(PSTR("  " LCD_STR_FILAM_MUL));
          u8g.print(':');
          lcd_print(itostr3(100.0 * tools.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]));
          u8g.print('%');
        }
      #endif
    #else
      lcd_implementation_status_message(blink);
    #endif
  }
}

#endif /* _STATUS_SCREEN_DOGM_H_ */
