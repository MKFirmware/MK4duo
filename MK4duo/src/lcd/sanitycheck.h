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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _LCD_SANITYCHECK_H_
#define _LCD_SANITYCHECK_H_

/**
 * Make sure only one display is enabled
 *
 * Note: BQ_LCD_SMART_CONTROLLER => REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
 *       REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER => REPRAP_DISCOUNT_SMART_CONTROLLER
 *       SAV_3DGLCD => U8GLIB_SH1106 => ULTIMAKERCONTROLLER
 *       MKS_12864OLED => U8GLIB_SH1106 => ULTIMAKERCONTROLLER
 *       MKS_12864OLED_SSD1306 => U8GLIB_SSD1306 => ULTIMAKERCONTROLLER
 *       miniVIKI => ULTIMAKERCONTROLLER
 *       VIKI2 => ULTIMAKERCONTROLLER
 *       ELB_FULL_GRAPHIC_CONTROLLER => ULTIMAKERCONTROLLER
 *       AZSMZ_12864 => ULTIMAKERCONTROLLER
 *       PANEL_ONE => ULTIMAKERCONTROLLER
 */
static_assert(1 >= 0
  #if ENABLED(ULTIMAKERCONTROLLER) \
      && DISABLED(SAV_3DGLCD) \
      && DISABLED(miniVIKI) \
      && DISABLED(VIKI2) \
      && DISABLED(ELB_FULL_GRAPHIC_CONTROLLER) \
      && DISABLED(AZSMZ_12864) \
      && DISABLED(PANEL_ONE) \
      && DISABLED(MKS_12864OLED) \
      && DISABLED(MKS_12864OLED_SSD1306)
    + 1
  #endif
  #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) \
      && DISABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER) \
      && DISABLED(LCD_FOR_MELZI) \
      && DISABLED(MAKEBOARD_MINI_2_LINE_DISPLAY_1602) \
      && DISABLED(MKS_12864OLED) \
      && DISABLED(MKS_12864OLED_SSD1306)
    + 1
  #endif
  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER) \
      && DISABLED(BQ_LCD_SMART_CONTROLLER)
    + 1
  #endif
  #if ENABLED(LCD_FOR_MELZI)
    + 1
  #endif
  #if ENABLED(MKS_12864OLED)
    + 1
  #endif
  #if ENABLED(MKS_12864OLED_SSD1306)
    + 1
  #endif
  #if ENABLED(MAKEBOARD_MINI_2_LINE_DISPLAY_1602)
    + 1
  #endif
  #if ENABLED(CARTESIO_UI)
    + 1
  #endif
  #if ENABLED(PANEL_ONE)
    + 1
  #endif
  #if ENABLED(MAKRPANEL)
    + 1
  #endif
  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    + 1
  #endif
  #if ENABLED(VIKI2)
    + 1
  #endif
  #if ENABLED(miniVIKI)
    + 1
  #endif
  #if ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    + 1
  #endif
  #if ENABLED(AZSMZ_12864)
    + 1
  #endif
  #if ENABLED(G3D_PANEL)
    + 1
  #endif
  #if ENABLED(MINIPANEL) && DISABLED(MKS_MINI_12864)
    + 1
  #endif
  #if ENABLED(MKS_MINI_12864)
    + 1
  #endif
  #if ENABLED(REPRAPWORLD_KEYPAD) \
      && DISABLED(CARTESIO_UI) \
      && DISABLED(ZONESTAR_LCD)
    + 1
  #endif
  #if ENABLED(RIGIDBOT_PANEL)
    + 1
  #endif
  #if ENABLED(RA_CONTROL_PANEL)
    + 1
  #endif
  #if ENABLED(LCD_SAINSMART_I2C_1602)
    + 1
  #endif
  #if ENABLED(LCD_SAINSMART_I2C_2004)
    + 1
  #endif
  #if ENABLED(LCM1602)
    + 1
  #endif
  #if ENABLED(LCD_I2C_PANELOLU2)
    + 1
  #endif
  #if ENABLED(LCD_I2C_VIKI)
    + 1
  #endif
  #if ENABLED(U8GLIB_SSD1306) && DISABLED(OLED_PANEL_TINYBOY2) && DISABLED(MKS_12864OLED_SSD1306)
    + 1
  #endif
  #if ENABLED(SAV_3DLCD)
    + 1
  #endif
  #if ENABLED(BQ_LCD_SMART_CONTROLLER)
    + 1
  #endif
  #if ENABLED(SAV_3DGLCD)
    + 1
  #endif
  #if ENABLED(OLED_PANEL_TINYBOY2)
    + 1
  #endif
  #if ENABLED(ZONESTAR_LCD)
    + 1
  #endif
  #if ENABLED(ULTI_CONTROLLER)
    + 1
  #endif
  #if ENABLED(NEXTION)
    + 1
  #endif
  , "DEPENDENCY ERROR: Please select no more than one LCD controller option."
);

// Language
#if DISABLED(LCD_LANGUAGE)
  #error "DEPENDENCY ERROR: Missing setting LCD_LANGUAGE."
#endif

// Progress Bar
#if ENABLED(LCD_PROGRESS_BAR)
  #if ENABLED(DOGLCD)
    #error "DEPENDENCY ERROR: LCD_PROGRESS_BAR does not apply to graphical displays."
  #elif ENABLED(FILAMENT_LCD_DISPLAY)
    #error "DEPENDENCY ERROR: LCD_PROGRESS_BAR and FILAMENT_LCD_DISPLAY are not fully compatible. Comment out this line to use both."
  #endif
#endif

// Progress bar
#if ENABLED(ULTIPANEL)
  #if ENABLED(LCD_PROGRESS_BAR)
    #if DISABLED(PROGRESS_BAR_BAR_TIME)
      #error "DEPENDENCY ERROR: Missing setting PROGRESS_BAR_BAR_TIME."
    #endif
    #if DISABLED(PROGRESS_BAR_MSG_TIME)
      #error "DEPENDENCY ERROR: Missing setting PROGRESS_BAR_MSG_TIME."
    #endif
    #if DISABLED(PROGRESS_MSG_EXPIRE)
      #error "DEPENDENCY ERROR: Missing setting PROGRESS_MSG_EXPIRE."
    #endif
  #endif
#endif

// LCD_BED_LEVELING requirements
#if ENABLED(LCD_BED_LEVELING)
  #if DISABLED(NEWPANEL)
    #error "DEPENDENCY ERROR: LCD_BED_LEVELING requires an LCD Normal controller."
  #elif DISABLED(MESH_BED_LEVELING) && !(HAS_ABL && ENABLED(PROBE_MANUALLY))
    #error "DEPENDENCY ERROR: LCD_BED_LEVELING requires MESH_BED_LEVELING or ABL and PROBE_MANUALLY."
  #endif
#endif

// Bootscreen
#if ENABLED(SHOW_BOOTSCREEN)
  #if DISABLED(STRING_SPLASH_LINE1)
    #error "DEPENDENCY ERROR: Missing setting STRING_SPLASH_LINE1."
  #endif
  #if DISABLED(BOOTSCREEN_TIMEOUT)
    #error "DEPENDENCY ERROR: Missing setting BOOTSCREEN_TIMEOUT."
  #endif
#endif

// Encoder rate multipliers
#if ENABLED(ULTIPANEL)
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    #if DISABLED(ENCODER_10X_STEPS_PER_SEC)
      #error "DEPENDENCY ERROR: Missing setting ENCODER_10X_STEPS_PER_SEC."
    #endif
    #if DISABLED(ENCODER_100X_STEPS_PER_SEC)
      #error "DEPENDENCY ERROR: Missing setting ENCODER_100X_STEPS_PER_SEC."
    #endif
  #endif
#endif

// RepRapWorld keypad move step
#if ENABLED(REPRAPWORLD_KEYPAD)
  #if DISABLED(REPRAPWORLD_KEYPAD_MOVE_STEP)
    #error "DEPENDENCY ERROR: Missing setting REPRAPWORLD_KEYPAD_MOVE_STEP."
  #endif
#endif

// Manual feedrate
#if ENABLED(ULTIPANEL) && DISABLED(MANUAL_FEEDRATE)
  #error "DEPENDENCY ERROR: Missing setting MANUAL_FEEDRATE."
#endif

// Required LCD language
#if DISABLED(DOGLCD) && ENABLED(ULTRA_LCD) && DISABLED(DISPLAY_CHARSET_HD44780)
  #error "DEPENDENCY ERROR: You must set DISPLAY_CHARSET_HD44780 to JAPANESE, WESTERN or CYRILLIC for your LCD controller."
#endif

// ULTIPANEL encoder
#if ENABLED(ULTIPANEL) && DISABLED(NEWPANEL) && DISABLED(SR_LCD_2W_NL) && DISABLED(SHIFT_CLK)
  #error "DEPENDENCY ERROR: ULTIPANEL requires some kind of encoder."
#endif
#if ENCODER_PULSES_PER_STEP < 0
  #error "DEPENDENCY ERROR: ENCODER_PULSES_PER_STEP should not be negative, use REVERSE_MENU_DIRECTION instead."
#endif

// Babystepping
#if ENABLED(BABYSTEPPING) && DISABLED(ULTRA_LCD)
  #error "DEPENDENCY ERROR: BABYSTEPPING requires an LCD controller."
#endif

#endif /* _LCD_SANITYCHECK_H_ */
