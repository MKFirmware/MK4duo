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

//
// status_screen_dogm.cpp
// Standard Status Screen for Graphical Display
//

#include "../../../../MK4duo.h"

#if HAS_GRAPHICAL_LCD

#include "status_screen_dogm.h"
#include "ultralcd_dogm.h"
#include "../lcdprint.h"

#if ENABLED(LASER)
  #include "../../../feature/laser/laserbitmaps.h"
#endif

#define X_LABEL_POS      3
#define X_VALUE_POS     11
#define XYZ_SPACING     37
#define XYZ_BASELINE    (30 + INFO_FONT_ASCENT)
#define EXTRAS_BASELINE (40 + INFO_FONT_ASCENT)
#define STATUS_BASELINE (LCD_PIXEL_HEIGHT - INFO_FONT_DESCENT)

#define DO_DRAW_LOGO    (STATUS_LOGO_WIDTH && ENABLED(CUSTOM_STATUS_SCREEN_IMAGE))
#define DO_DRAW_BED     (HAS_BEDS && STATUS_BED_WIDTH)
#define DO_DRAW_CHAMBER (HAS_CHAMBERS && STATUS_CHAMBER_WIDTH)
#define DO_DRAW_FAN     (HAS_FAN && STATUS_FAN_WIDTH && ENABLED(STATUS_FAN_FRAMES))

#define ANIM_HOTEND     (HOTENDS && ENABLED(STATUS_HOTEND_ANIM))
#define ANIM_BED        (DO_DRAW_BED && ENABLED(STATUS_BED_ANIM))
#define ANIM_CHAMBER    (DO_DRAW_CHAMBER && ENABLED(STATUS_CHAMBER_ANIM))

#define ANIM_HBC (ANIM_HOTEND || ANIM_BED || ANIM_CHAMBER)

#if ANIM_HBC
  uint8_t heat_bits;
#endif
#if ANIM_HOTEND
  #define HOTEND_ALT(N) TEST(heat_bits, N)
#else
  #define HOTEND_ALT(N) false
#endif
#if ANIM_BED
  #define BED_ALT()     TEST(heat_bits, 7)
#else
  #define BED_ALT()     false
#endif
#if ANIM_CHAMBER
  #define CHAMBER_ALT() TEST(heat_bits, 6)
#else
  #define CHAMBER_ALT() false
#endif

#define STATUS_HEATERS_BOT  (STATUS_HEATERS_Y + STATUS_HEATERS_HEIGHT - 1)

#define PROGRESS_BAR_X 54
#define PROGRESS_BAR_WIDTH (LCD_PIXEL_WIDTH - PROGRESS_BAR_X)

uint8_t STATUS_BED_X, STATUS_CHAMBER_X;
#define STATUS_BED_TEXT_X     (STATUS_BED_X + 9)
#define STATUS_CHAMBER_TEXT_X (STATUS_CHAMBER_X + 11)

FORCE_INLINE void _draw_centered_temp(const int16_t temp, const uint8_t tx, const uint8_t ty) {
  const char *str = i16tostr3(temp);
  const uint8_t len = str[0] != ' ' ? 3 : str[1] != ' ' ? 2 : 1;
  lcd_put_u8str(tx - len * (INFO_FONT_WIDTH) / 2 + 1, ty, &str[3-len]);
  lcd_put_wchar(LCD_STR_DEGREE[0]);
}

FORCE_INLINE void _draw_heater_status(Heater *act, const bool blink) {

  #if DO_DRAW_BED
    const bool isBed = (act->type == IS_BED);
    #define IFBED(A,B) (isBed ? (A) : (B))
  #else
    #define IFBED(A,B) (B)
  #endif

  const bool    isHeat  = IFBED(BED_ALT(), HOTEND_ALT(act->data.ID));
  const uint8_t tx      = IFBED(STATUS_BED_TEXT_X, STATUS_HOTEND_TEXT_X(act->data.ID));
  const float   temp    = act->deg_current(),
                target  = float(act->isIdle() ? act->deg_idle() : act->deg_target());

  #if DISABLED(STATUS_HOTEND_ANIM)
    #define STATIC_HOTEND true
    #define HOTEND_DOT    isHeat
  #else
    #define STATIC_HOTEND false
    #define HOTEND_DOT    false
  #endif

  #if DISABLED(STATUS_BED_ANIM)
    #define STATIC_BED    true
    #define BED_DOT       isHeat
  #else
    #define STATIC_BED    false
    #define BED_DOT       false
  #endif

  #if ANIM_HOTEND && ENABLED(STATUS_HOTEND_INVERTED) && ENABLED(STATUS_HOTEND_NUMBERLESS)
    #define OFF_BMP(N) status_hotend_b_bmp
    #define ON_BMP(N)  status_hotend_a_bmp
  #elif ANIM_HOTEND && DISABLED(STATUS_HOTEND_INVERTED) && ENABLED(STATUS_HOTEND_NUMBERLESS)
    #define OFF_BMP(N) status_hotend_a_bmp
    #define ON_BMP(N)  status_hotend_b_bmp
  #elif ANIM_HOTEND && ENABLED(STATUS_HOTEND_INVERTED)
    #define OFF_BMP(N) status_hotend##N##_b_bmp
    #define ON_BMP(N)  status_hotend##N##_a_bmp
  #else
    #define OFF_BMP(N) status_hotend##N##_a_bmp
    #define ON_BMP(N)  status_hotend##N##_b_bmp
  #endif

  #if STATUS_HOTEND_BITMAPS > 1
    static const unsigned char* const status_hotend_gfx[STATUS_HOTEND_BITMAPS] PROGMEM = ARRAY_N(STATUS_HOTEND_BITMAPS, OFF_BMP(1), OFF_BMP(2), OFF_BMP(3), OFF_BMP(4), OFF_BMP(5), OFF_BMP(6));
    #if ANIM_HOTEND
      static const unsigned char* const status_hotend_on_gfx[STATUS_HOTEND_BITMAPS] PROGMEM = ARRAY_N(STATUS_HOTEND_BITMAPS, ON_BMP(1), ON_BMP(2), ON_BMP(3), ON_BMP(4), ON_BMP(5), ON_BMP(6));
      #define HOTEND_BITMAP(N,S)  (unsigned char*)pgm_read_ptr((S) ? &status_hotend_on_gfx[(N) % (STATUS_HOTEND_BITMAPS)] : &status_hotend_gfx[(N) % (STATUS_HOTEND_BITMAPS)])
    #else
      #define HOTEND_BITMAP(N,S)  (unsigned char*)pgm_read_ptr(&status_hotend_gfx[(N) % (STATUS_HOTEND_BITMAPS)])
    #endif
  #elif ANIM_HOTEND
    #define HOTEND_BITMAP(N,S)    ((S) ? ON_BMP() : OFF_BMP())
  #else
    #define HOTEND_BITMAP(N,S)    status_hotend_a_bmp
  #endif

  if (PAGE_CONTAINS(STATUS_HEATERS_Y, STATUS_HEATERS_BOT)) {

    #define BAR_TALL (STATUS_HEATERS_HEIGHT - 2)

    const float prop = target - 20,
                perc = prop > 0 && temp >= 20 ? (temp - 20) / prop : 0;
    uint8_t     tall = uint8_t(perc * BAR_TALL + 0.5f);
    NOMORE(tall, BAR_TALL);

    #if ANIM_HOTEND
      // Draw hotend bitmap, either whole or split by the heating percent
      if (IFBED(0, 1)) {
        const uint8_t hx = STATUS_HOTEND_X(act->data.ID),
                      bw = STATUS_HOTEND_BYTEWIDTH(act->data.ID);
        #if ENABLED(STATUS_HEAT_PERCENT)
          if (isHeat && tall <= BAR_TALL) {
            const uint8_t ph = STATUS_HEATERS_HEIGHT - 1 - tall;
            u8g.drawBitmapP(hx, STATUS_HEATERS_Y, bw, ph, HOTEND_BITMAP(act->data.ID, false));
            u8g.drawBitmapP(hx, STATUS_HEATERS_Y + ph, bw, tall + 1, HOTEND_BITMAP(act->data.ID, true) + ph * bw);
          }
          else
        #endif
            u8g.drawBitmapP(hx, STATUS_HEATERS_Y, bw, STATUS_HEATERS_HEIGHT, HOTEND_BITMAP(act->data.ID, isHeat));
      }
    #endif

    // Draw a heating progress bar, if specified
    #if DO_DRAW_BED && ENABLED(STATUS_HEAT_PERCENT)

      if (IFBED(true, STATIC_HOTEND) && isHeat) {
        const uint8_t bx = IFBED(STATUS_BED_X + STATUS_BED_WIDTH, STATUS_HOTEND_X(act->data.ID) + STATUS_HOTEND_WIDTH(act->data.ID)) + 1;
        u8g.drawFrame(bx, STATUS_HEATERS_Y, 3, STATUS_HEATERS_HEIGHT);
        if (tall) {
          const uint8_t ph = STATUS_HEATERS_HEIGHT - 1 - tall;
          if (PAGE_OVER(STATUS_HEATERS_Y + ph))
            u8g.drawVLine(bx + 1, STATUS_HEATERS_Y + ph, tall);
        }
      }

    #endif

  } // PAGE_CONTAINS

  if (PAGE_UNDER(7)) {
    const bool  is_idle = act->isIdle();
    if (blink || !is_idle) _draw_centered_temp(target + 0.5f, tx, 7);
  }

  if (PAGE_CONTAINS(28 - INFO_FONT_ASCENT, 28 - 1))
    _draw_centered_temp(temp + 0.5f, tx, 28);

  if (IFBED(STATIC_BED && BED_DOT, STATIC_HOTEND && HOTEND_DOT) && PAGE_CONTAINS(17, 19)) {
    u8g.setColorIndex(0); // set to white on black
    u8g.drawBox(tx, IFBED(20-2, 20-3), 2, 2);
    u8g.setColorIndex(1); // restore black on white
  }

}

#if DO_DRAW_CHAMBER
  FORCE_INLINE void _draw_chamber_status(const bool blink) {
    const float temp    = chambers[0]->deg_current(),
                target  = chambers[0]->deg_target();

    if (PAGE_UNDER(7)) {
      const bool  is_idle = chambers[0]->isIdle();
      if (blink || !is_idle) _draw_centered_temp(target + 0.5, STATUS_CHAMBER_TEXT_X, 7);
    }
    if (PAGE_CONTAINS(28 - INFO_FONT_ASCENT, 28 - 1))
      _draw_centered_temp(temp + 0.5f, STATUS_CHAMBER_TEXT_X, 28);
  }
#endif

//
// Before homing, blink '123' <-> '???'.
// Homed and known, display constantly.
//
FORCE_INLINE void _draw_axis_value(const AxisEnum axis, PGM_P value, const bool blink) {
  const uint8_t offs = (XYZ_SPACING) * axis;
  lcd_put_wchar(X_LABEL_POS + offs, XYZ_BASELINE, 'X' + axis);
  lcd_moveto(X_VALUE_POS + offs, XYZ_BASELINE);
  if (blink)
    lcd_put_u8str(value);
  else {
    if (!mechanics.isAxisHomed(axis))
      while (const char c = *value++) lcd_put_wchar(c <= '.' ? c : '?');
    else {
      lcd_put_u8str(value);
    }
  }
}

void LcdUI::draw_status_screen() {

  static char xstring[5], ystring[5], zstring[8];
  #if HAS_LCD_FILAMENT_SENSOR
    static char wstring[5], mstring[4];
  #endif

  static uint8_t  progress_bar_solid_width = 0,
                  lastProgress    = 0,
                  lastElapsed     = 0,
                  elapsed_x_pos   = 0,
                  finished_x_pos  = 0;
  static char     elapsed_string[16],
                  finished_string[10];

  // At the first page, regenerate the XYZ strings
  if (first_page) {

    #if ANIM_HBC
      uint8_t new_bits = 0;
      #if ANIM_HOTEND
        LOOP_HOTEND() if (hotends[h]->isHeating()) SBI(new_bits, h);
      #endif
      #if ANIM_BED
        if (beds[0]->isHeating()) SBI(new_bits, 7);
      #endif
      #if DO_DRAW_CHAMBER
        if (chambers[0]->isHeating()) SBI(new_bits, 6);
      #endif
      heat_bits = new_bits;
    #endif
    strcpy(xstring, ftostr4sign(LOGICAL_X_POSITION(mechanics.position.x)));
    strcpy(ystring, ftostr4sign(LOGICAL_Y_POSITION(mechanics.position.y)));
    strcpy(zstring, ftostr52sp (LOGICAL_Z_POSITION(mechanics.position.z)));
    #if HAS_LCD_FILAMENT_SENSOR
      strcpy(wstring, ftostr12ns(filament_width_meas));
      strcpy(mstring, i16tostr3(100.0 * (
          toolManager.isVolumetric()
            ? toolManager.volumetric_area_nominal / toolManager.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
            : toolManager.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
        )
      ));
    #endif

    duration_t elapsed  = print_job_counter.duration();
    const uint8_t p     = printer.progress & 0xFF,
                  ev    = elapsed.value & 0xFF;

    if (printer.progress > 1 && p != lastProgress) {
      lastProgress = p;
      progress_bar_solid_width = uint8_t((PROGRESS_BAR_WIDTH - 2) * printer.progress * 0.01f);
    }
    if (ev != lastElapsed) {
      lastElapsed = ev;
      const bool has_days = (elapsed.value >= 60*60*24L);
      const uint8_t len = elapsed.toDigital(elapsed_string, has_days);
      elapsed_x_pos = (PROGRESS_BAR_X + (PROGRESS_BAR_WIDTH) / 2 - (len+ 1) * (MENU_FONT_WIDTH) / 2);
      if (!(ev & 0x3)) {
        duration_t finished = (elapsed.value * (100 - printer.progress)) / (printer.progress + 0.1);
        const bool has_days = (finished.value >= 60*60*24L);
        const uint8_t len = finished.toDigital(finished_string, has_days);
        finished_x_pos = (PROGRESS_BAR_X + (PROGRESS_BAR_WIDTH) / 2 - (len + 1) * (MENU_FONT_WIDTH) / 2);
      }
    }
  }

  const bool  blink = get_blink(),
              draw_fan      = fanManager.data.fans > 0           && !(tempManager.heater.hotends == 5 || (tempManager.heater.hotends >= 3 && (tempManager.heater.beds > 0 || tempManager.heater.chambers > 0))),
              draw_bed      = tempManager.heater.beds > 0      &&   tempManager.heater.hotends <= 4,
              draw_chamber  = tempManager.heater.chambers > 0  && ((tempManager.heater.hotends <= 4 && tempManager.heater.beds == 0) || (tempManager.heater.hotends <= 3 && tempManager.heater.beds > 0));

  STATUS_BED_X      = LCD_PIXEL_WIDTH - ((STATUS_BED_BYTEWIDTH      + (draw_fan ? STATUS_FAN_BYTEWIDTH : 0)                                        ) * 8),
  STATUS_CHAMBER_X  = LCD_PIXEL_WIDTH - ((STATUS_CHAMBER_BYTEWIDTH  + (draw_fan ? STATUS_FAN_BYTEWIDTH : 0) + (draw_bed ? STATUS_BED_BYTEWIDTH : 0)) * 8);

  // Status Menu Font
  set_font(FONT_STATUSMENU);

  #if DO_DRAW_LOGO
    if (PAGE_CONTAINS(STATUS_LOGO_Y, STATUS_LOGO_Y + STATUS_LOGO_HEIGHT - 1))
      u8g.drawBitmapP(STATUS_LOGO_X, STATUS_LOGO_Y, STATUS_LOGO_BYTEWIDTH, STATUS_LOGO_HEIGHT, status_logo_bmp);
  #endif

  if (printer.mode == PRINTER_MODE_FFF) {

    #if STATUS_HEATERS_WIDTH
      // Draw all heaters (and maybe the bed) in one go
      if (tempManager.heater.hotends > 0 && PAGE_CONTAINS(STATUS_HEATERS_Y, STATUS_HEATERS_Y + STATUS_HEATERS_HEIGHT - 1))
        u8g.drawBitmapP(STATUS_HEATERS_X, STATUS_HEATERS_Y, STATUS_HEATERS_BYTEWIDTH, STATUS_HEATERS_HEIGHT, status_heaters_bmp);
    #endif

    #if DO_DRAW_BED
      #if ANIM_BED
        #define BED_BITMAP(S) ((S) ? status_bed_on_bmp : status_bed_bmp)
      #else
        #define BED_BITMAP(S) status_bed_bmp
      #endif
      const uint8_t bedy = STATUS_BED_Y(BED_ALT()),
                    bedh = STATUS_BED_HEIGHT(BED_ALT());
      if (draw_bed && PAGE_CONTAINS(bedy, bedy + bedh - 1))
        u8g.drawBitmapP(STATUS_BED_X, bedy, STATUS_BED_BYTEWIDTH, bedh, BED_BITMAP(BED_ALT()));
    #endif

    #if DO_DRAW_CHAMBER
      #if ANIM_CHAMBER
        #define CHAMBER_BITMAP(S) ((S) ? status_chamber_on_bmp : status_chamber_bmp)
      #else
        #define CHAMBER_BITMAP(S) status_chamber_bmp
      #endif
      const uint8_t chambery = STATUS_CHAMBER_Y(CHAMBER_ALT()),
                    chamberh = STATUS_CHAMBER_HEIGHT(CHAMBER_ALT());
      if (draw_chamber && PAGE_CONTAINS(chambery, chambery + chamberh - 1))
        u8g.drawBitmapP(STATUS_CHAMBER_X, chambery, STATUS_CHAMBER_BYTEWIDTH, chamberh, CHAMBER_BITMAP(CHAMBER_ALT()));
    #endif

    #if DO_DRAW_FAN
      #if STATUS_FAN_FRAMES > 2
        static bool old_blink;
        static uint8_t fan_frame;
        if (old_blink != blink) {
          old_blink = blink;
          if (!fans[0]->speed || ++fan_frame >= STATUS_FAN_FRAMES) fan_frame = 0;
        }
      #endif
      if (draw_fan && PAGE_CONTAINS(STATUS_FAN_Y, STATUS_FAN_Y + STATUS_FAN_HEIGHT - 1))
        u8g.drawBitmapP(
          STATUS_FAN_X, STATUS_FAN_Y,
          STATUS_FAN_BYTEWIDTH, STATUS_FAN_HEIGHT,
          #if STATUS_FAN_FRAMES > 2
            fan_frame == 1 ? status_fan1_bmp :
            fan_frame == 2 ? status_fan2_bmp :
            #if STATUS_FAN_FRAMES > 3
              fan_frame == 3 ? status_fan3_bmp :
            #endif
          #elif STATUS_FAN_FRAMES > 1
            blink && fans[0]->speed ? status_fan1_bmp :
          #endif
          status_fan0_bmp
        );
    #endif
    //
    // Temperature Graphics and Info
    //

    if (PAGE_UNDER(6 + 1 + 12 + 1 + 6 + 1)) {
      // Hotends
      LOOP_HOTEND()
        _draw_heater_status(hotends[h], blink);

      // Heated bed
      #if DO_DRAW_BED
        if (draw_bed)
          _draw_heater_status(beds[0], blink);
      #endif

      #if DO_DRAW_CHAMBER
        if (draw_chamber)
          _draw_chamber_status(blink);
      #endif

      // Fan, if a bitmap was provided
      #if DO_DRAW_FAN
        if (draw_fan && PAGE_CONTAINS(STATUS_FAN_TEXT_Y - INFO_FONT_ASCENT, STATUS_FAN_TEXT_Y - 1)) {
          const uint8_t spd = fans[0]->actual_speed();
          if (spd) {
            lcd_put_u8str(STATUS_FAN_TEXT_X, STATUS_FAN_TEXT_Y, ui8tostr4pct(spd));
          }
        }
      #endif
    }

  } // printer.mode == PRINTER_MODE_FFF

  #if ENABLED(LASER)

    if (printer.mode == PRINTER_MODE_LASER) {
      #if ENABLED(LASER_PERIPHERALS)
        if (laser.peripherals_ok()) {
          u8g.drawBitmapP(29, 4, LASERENABLE_BYTEWIDTH, LASERENABLE_HEIGHT, laserenable_bmp);
        }
      #endif

      lcd_moveto(3, 6);
      if (stepper.laser_status()) {
        u8g.drawBitmapP(10, 10, ICON_BYTEWIDTH, ICON_HEIGHT, laseron_bmp);
        lcd_moveto(5, 7);
        lcd_put_u8str(i8tostr3(stepper.laser_intensity()));
        lcd_put_u8str_P(PSTR("%"));
      }
      else {
        u8g.drawBitmapP(10, 10, ICON_BYTEWIDTH, ICON_HEIGHT, laseroff_bmp);
        lcd_moveto(5, 7);
        lcd_put_u8str_P(PSTR("---%"));
      }
    }

  #endif

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
  if (PAGE_CONTAINS(49, 52))
    u8g.drawFrame(PROGRESS_BAR_X, 49, PROGRESS_BAR_WIDTH, 4);

  //
  // Progress bar solid part
  //

  if (printer.progress && (PAGE_CONTAINS(50, 51)))
    u8g.drawBox(PROGRESS_BAR_X + 1, 50, progress_bar_solid_width, 2);

  //
  // Elapsed Time
  //

  if (PAGE_CONTAINS(EXTRAS_BASELINE - INFO_FONT_ASCENT, EXTRAS_BASELINE - 1)) {

    if (blink && (finished_string[0] != '\0')) {
      lcd_put_wchar(finished_x_pos, EXTRAS_BASELINE, 'E');
      lcd_put_u8str(finished_string);
    }
    else {
      lcd_put_wchar(elapsed_x_pos, EXTRAS_BASELINE, 'S');
      lcd_put_u8str(elapsed_string);
    }

    #if HAS_LCD_POWER_SENSOR
      if (millis() < print_millis + 1000) {
        lcd_put_wchar(54, 48, 'S');
        lcd_put_u8str(buffer1);
        lcd_put_wchar(92, 48, 'E');
        lcd_put_u8str(buffer2);
      }
      else {
        lcd_put_u8str(54, 48, ui32tostr4(print_job_counter.getConsumptionHour() - powerManager.startpower));
        lcd_put_u8str((char*)"Wh");
      }
    #endif
  }

  //
  // XYZ Coordinates
  //

  #if ENABLED(XYZ_HOLLOW_FRAME)
    #define XYZ_FRAME_TOP 29
    #define XYZ_FRAME_HEIGHT INFO_FONT_ASCENT + 3
  #else
    #define XYZ_FRAME_TOP 30
    #define XYZ_FRAME_HEIGHT INFO_FONT_ASCENT + 1
  #endif

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

      #if HAS_GRADIENT_MIX

        // Two-component mix / gradient instead of XY

        char mixer_messages[12];
        const char *mix_label;
        if (mixer.gradient.enabled) {
          mixer.update_mix_from_gradient();
          mix_label = "Gr";
        }
        else {
          mixer.update_mix_from_vtool();
          mix_label = "Mx";
        }
        sprintf_P(mixer_messages, PSTR("%s %d;%d%% "), mix_label, int(mixer.mix[0]), int(mixer.mix[1]));
        lcd_put_u8str(X_LABEL_POS, XYZ_BASELINE, mixer_messages);

      #else

        _draw_axis_value(X_AXIS, xstring, blink);
        _draw_axis_value(Y_AXIS, ystring, blink);

      #endif

      _draw_axis_value(Z_AXIS, zstring, blink);

      #if DISABLED(XYZ_HOLLOW_FRAME)
        u8g.setColorIndex(1); // black on white
      #endif
    }
  }

  //
  // Feedrate
  //
  #define EXTRAS_2_BASELINE (EXTRAS_BASELINE + 3)

  if (PAGE_CONTAINS(EXTRAS_2_BASELINE - INFO_FONT_ASCENT, EXTRAS_2_BASELINE - 1)) {
    set_font(FONT_MENU);
    lcd_put_wchar(3, EXTRAS_2_BASELINE, LCD_STR_FEEDRATE[0]);

    set_font(FONT_STATUSMENU);
    lcd_put_u8str(12, EXTRAS_2_BASELINE, i16tostr3(mechanics.feedrate_percentage));
    lcd_put_wchar('%');

    //
    // Filament sensor display if SD is disabled
    //
    #if HAS_LCD_FILAMENT_SENSOR && DISABLED(SDSUPPORT)
      lcd_put_u8str(56, EXTRAS_2_BASELINE, wstring);
      lcd_put_u8str(102, EXTRAS_2_BASELINE, mstring);
      lcd_put_wchar('%');
      set_font(FONT_MENU);
      lcd_put_wchar(47, EXTRAS_2_BASELINE, LCD_STR_FILAM_DIA[0]); // lcd_put_u8str_P(PSTR(LCD_STR_FILAM_DIA));
      lcd_put_wchar(93, EXTRAS_2_BASELINE, LCD_STR_FILAM_MUL[0]);
    #endif
  }

  //
  // Status line
  //
  if (PAGE_CONTAINS(STATUS_BASELINE - INFO_FONT_ASCENT, STATUS_BASELINE + INFO_FONT_DESCENT)) {
    lcd_moveto(0, STATUS_BASELINE);

    #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
      if (pending(&previous_status_ms, 5000U)) { // Display both Status message line and Filament display on the last line
        lcd_implementation_status_message(blink);
      }

      #if HAS_LCD_POWER_SENSOR
        #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT))
          else if (pending(&previous_status_ms, 10000U))
        #else
          else
        #endif
          {
            lcd_put_u8str_P(PSTR("P:"));
            lcd_put_u8str(ftostr31(powerManager.consumption_meas));
            lcd_put_u8str_P(PSTR("W C:"));
            lcd_put_u8str(ltostr7(print_job_counter.getConsumptionHour()));
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

void LcdUI::draw_status_message(const bool blink) {

  // Get the UTF8 character count of the string
  uint8_t slen = utf8_strlen(status_message);

  #if ENABLED(STATUS_MESSAGE_SCROLLING)

    static bool last_blink = false;

    if (slen <= LCD_WIDTH) {
      // The string fits within the line. Print with no scrolling
      lcd_put_u8str(status_message);
      while (slen < LCD_WIDTH) { lcd_put_wchar(' '); ++slen; }
    }
    else {
      // String is longer than the available space

      // Get a pointer to the next valid UTF8 character
      // and the string remaining length
      uint8_t rlen;
      const char *stat = status_and_len(rlen);
      lcd_put_u8str_max(stat, LCD_PIXEL_WIDTH);

      // If the remaining string doesn't completely fill the screen
      if (rlen < LCD_WIDTH) {
        lcd_put_wchar('.');                     // Always at 1+ spaces left, draw a dot
        uint8_t chars = LCD_WIDTH - rlen;       // Amount of space left in characters
        if (--chars) {                          // Draw a second dot if there's space
          lcd_put_wchar('.');
          if (--chars) {                        // Print a second copy of the message
            lcd_put_u8str_max(status_message, LCD_PIXEL_WIDTH - (rlen + 2) * (MENU_FONT_WIDTH));
            lcd_put_wchar(' ');
          }
        }
      }
      if (last_blink != blink) {
        last_blink = blink;
        advance_status_scroll();
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

#endif // HAS_GRAPHICAL_LCD
