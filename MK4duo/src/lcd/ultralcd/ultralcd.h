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
#pragma once

#define HAS_ENCODER_ACTION (HAS_LCD_MENU || ENABLED(ULTIPANEL_FEEDMULTIPLY))

#if HAS_GRAPHICAL_LCD
  #include "dogm/ultralcd_DOGM.h"
#endif

#if HAS_SPI_LCD

  #if ENABLED(ADC_KEYPAD)
    uint8_t get_ADC_keyValue();
  #endif

  #if HAS_GRAPHICAL_LCD
    #define SETCURSOR(col, row)     lcd_moveto(col * (MENU_FONT_WIDTH), (row + 1) * (MENU_FONT_HEIGHT))
    #define SETCURSOR_RJ(len, row)  lcd_moveto(LCD_PIXEL_WIDTH - len * (MENU_FONT_WIDTH), (row + 1) * (MENU_FONT_HEIGHT))
  #else
    #define SETCURSOR(col, row)     lcd_moveto(col, row)
    #define SETCURSOR_RJ(len, row)  lcd_moveto(LCD_WIDTH - len, row)
  #endif

  #define LCD_UPDATE_INTERVAL       100

  #define BUTTON_EXISTS(BN)         (ENABLED(BTN_## BN) && BTN_## BN >= 0)
  #define BUTTON_PRESSED(BN)        !READ(BTN_## BN)

  #if HAS_LCD_MENU

    // Manual Movement
    constexpr float manual_feedrate_mm_m[XYZE] = MANUAL_FEEDRATE;
    extern float move_menu_scale;

    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      void lcd_advanced_pause_show_message(const AdvancedPauseMessageEnum message,
                                           const AdvancedPauseModeEnum mode=ADVANCED_PAUSE_MODE_SAME,
                                           const uint8_t hotend=TARGET_HOTEND);
    #endif

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      void lcd_mesh_edit_setup(const float &initial);
      float lcd_mesh_edit();
    #endif

    #if ENABLED(PROBE_MANUALLY) || MECH(DELTA)
      void _man_probe_pt(const float &rx, const float &ry);
    #endif

    #if ENABLED(PROBE_MANUALLY)
      float lcd_probe_pt(const float &rx, const float &ry);
    #endif

  #endif // HAS_LCD_MENU

#endif // HAS_SPI_LCD

#if HAS_DIGITAL_ENCODER

  // Wheel spin pins where BA is 00, 10, 11, 01 (1 bit always changes)
  #define BLEN_A 0
  #define BLEN_B 1

  #define EN_A _BV(BLEN_A)
  #define EN_B _BV(BLEN_B)

  #if BUTTON_EXISTS(ENC)
    #define BLEN_C 2
    #define EN_C _BV(BLEN_C)
  #endif

  #if BUTTON_EXISTS(BACK)
    #define BLEN_D 3
    #define EN_D _BV(BLEN_D)
    #if ENABLED(INVERT_BACK_BUTTON)
      #define LCD_BACK_CLICKED()  !(buttons & EN_D)
    #else
      #define LCD_BACK_CLICKED()  (buttons & EN_D)
    #endif
  #endif

  #if ENABLED(REPRAPWORLD_KEYPAD)
    #define REPRAPWORLD_BTN_OFFSET          0 // Bit offset into buttons for shift register values

    #define BLEN_REPRAPWORLD_KEYPAD_F3      0
    #define BLEN_REPRAPWORLD_KEYPAD_F2      1
    #define BLEN_REPRAPWORLD_KEYPAD_F1      2
    #define BLEN_REPRAPWORLD_KEYPAD_DOWN    3
    #define BLEN_REPRAPWORLD_KEYPAD_RIGHT   4
    #define BLEN_REPRAPWORLD_KEYPAD_MIDDLE  5
    #define BLEN_REPRAPWORLD_KEYPAD_UP      6
    #define BLEN_REPRAPWORLD_KEYPAD_LEFT    7

    #define EN_REPRAPWORLD_KEYPAD_F1        (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F1))
    #define EN_REPRAPWORLD_KEYPAD_F2        (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F2))
    #define EN_REPRAPWORLD_KEYPAD_F3        (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_F3))
    #define EN_REPRAPWORLD_KEYPAD_DOWN      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_DOWN))
    #define EN_REPRAPWORLD_KEYPAD_RIGHT     (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_RIGHT))
    #define EN_REPRAPWORLD_KEYPAD_MIDDLE    (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_MIDDLE))
    #define EN_REPRAPWORLD_KEYPAD_UP        (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_UP))
    #define EN_REPRAPWORLD_KEYPAD_LEFT      (_BV(REPRAPWORLD_BTN_OFFSET + BLEN_REPRAPWORLD_KEYPAD_LEFT))

    #define RRK(B) (buttons_reprapworld_keypad & (B))

    #ifdef EN_C
      #define BUTTON_CLICK() ((buttons & EN_C) || RRK(EN_REPRAPWORLD_KEYPAD_MIDDLE))
    #else
      #define BUTTON_CLICK() RRK(EN_REPRAPWORLD_KEYPAD_MIDDLE)
    #endif

  #elif ENABLED(LCD_I2C_VIKI)

    #define B_I2C_BTN_OFFSET 3 // (the first three bit positions reserved for EN_A, EN_B, EN_C)

    // button and encoder bit positions within 'buttons'
    #define B_LE (BUTTON_LEFT   << B_I2C_BTN_OFFSET)    // The remaining normalized buttons are all read via I2C
    #define B_UP (BUTTON_UP     << B_I2C_BTN_OFFSET)
    #define B_MI (BUTTON_SELECT << B_I2C_BTN_OFFSET)
    #define B_DW (BUTTON_DOWN   << B_I2C_BTN_OFFSET)
    #define B_RI (BUTTON_RIGHT  << B_I2C_BTN_OFFSET)

    #if BUTTON_EXISTS(ENC)                                // The pause/stop/restart button is connected to BTN_ENC when used
      #define B_ST (EN_C)                                 // Map the pause/stop/resume button into its normalized functional name
      #if ENABLED(INVERT_CLICK_BUTTON)
        #define BUTTON_CLICK() !(buttons & (B_MI|B_RI|B_ST)) // pause/stop button also acts as click until we implement proper pause/stop.
      #else
        #define BUTTON_CLICK()  (buttons & (B_MI|B_RI|B_ST)) // pause/stop button also acts as click until we implement proper pause/stop.
      #endif
    #else
      #if ENABLED(INVERT_CLICK_BUTTON)
        #define BUTTON_CLICK() !(buttons & (B_MI|B_RI))
      #else
        #define BUTTON_CLICK()  (buttons & (B_MI|B_RI))
      #endif
    #endif

    // I2C buttons take too long to read inside an interrupt context and so we read them during lcdui.update
    #define LCD_HAS_SLOW_BUTTONS

  #elif ENABLED(LCD_I2C_PANELOLU2)

    #if !BUTTON_EXISTS(ENC) // Use I2C if not directly connected to a pin

      #define B_I2C_BTN_OFFSET 3 // (the first three bit positions reserved for EN_A, EN_B, EN_C)

      #define B_MI (PANELOLU2_ENCODER_C << B_I2C_BTN_OFFSET) // requires LiquidTWI2 library v1.2.3 or later

      #undef BUTTON_CLICK()
      #if ENABLED(INVERT_CLICK_BUTTON)
        #define BUTTON_CLICK() !(buttons & B_MI)
      #else
        #define BUTTON_CLICK()  (buttons & B_MI)
      #endif

      // I2C buttons take too long to read inside an interrupt context and so we read them during lcdui.update
      #define LCD_HAS_SLOW_BUTTONS

    #endif

  #endif

#else

  // Shift register bits correspond to buttons:
  #define BL_LE 7   // Left
  #define BL_UP 6   // Up
  #define BL_MI 5   // Middle
  #define BL_DW 4   // Down
  #define BL_RI 3   // Right
  #define BL_ST 2   // Red Button
  #define B_LE (_BV(BL_LE))
  #define B_UP (_BV(BL_UP))
  #define B_MI (_BV(BL_MI))
  #define B_DW (_BV(BL_DW))
  #define B_RI (_BV(BL_RI))
  #define B_ST (_BV(BL_ST))

  #if ENABLED(INVERT_CLICK_BUTTON)
    #define BUTTON_CLICK() !(buttons & (B_MI|B_ST))
  #else
    #define BUTTON_CLICK()  (buttons & (B_MI|B_ST))
  #endif

#endif

#ifndef BUTTON_CLICK
  #ifdef EN_C
    #if ENABLED(INVERT_CLICK_BUTTON)
      #define BUTTON_CLICK() !(buttons & EN_C)
    #else
      #define BUTTON_CLICK() (buttons & EN_C)
    #endif
  #else
    #define BUTTON_CLICK() false
  #endif
#endif
