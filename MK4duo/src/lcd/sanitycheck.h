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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

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
  #if HAS_NEXTION_LCD
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
  #if HAS_GRAPHICAL_LCD
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
  #if !HAS_LCD_MENU
    #error "LCD_BED_LEVELING requires an LCD controller and LCD MENU."
  #elif !(ENABLED(MESH_BED_LEVELING) || OLD_ABL)
    #error "LCD_BED_LEVELING requires MESH_BED_LEVELING or AUTO_BED_LEVELING."
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
#if DISABLED(DOGLCD) && HAS_SPI_LCD && DISABLED(LCD_CHARSET_HD44780)
  #error "DEPENDENCY ERROR: You must set LCD_CHARSET_HD44780 to JAPANESE, WESTERN or CYRILLIC for your LCD controller."
#endif

// ULTIPANEL encoder
#if ENABLED(ULTIPANEL) && DISABLED(NEWPANEL) && DISABLED(SR_LCD_2W_NL) && DISABLED(SHIFT_CLK)
  #error "DEPENDENCY ERROR: ULTIPANEL requires some kind of encoder."
#endif
#if ENCODER_PULSES_PER_STEP < 0
  #error "DEPENDENCY ERROR: ENCODER_PULSES_PER_STEP should not be negative, use REVERSE_MENU_DIRECTION instead."
#endif

// CUSTOM USER MENUS
#if ENABLED(CUSTOM_USER_MENUS)
  #ifdef USER_GCODE_1
    constexpr char _chr1 = USER_GCODE_1[strlen(USER_GCODE_1) - 1];
    static_assert(_chr1 != '\n' && _chr1 != '\r', "USER_GCODE_1 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_2
    constexpr char _chr2 = USER_GCODE_2[strlen(USER_GCODE_2) - 1];
    static_assert(_chr2 != '\n' && _chr2 != '\r', "USER_GCODE_2 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_3
    constexpr char _chr3 = USER_GCODE_3[strlen(USER_GCODE_3) - 1];
    static_assert(_chr3 != '\n' && _chr3 != '\r', "USER_GCODE_3 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_4
    constexpr char _chr4 = USER_GCODE_4[strlen(USER_GCODE_4) - 1];
    static_assert(_chr4 != '\n' && _chr4 != '\r', "USER_GCODE_4 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_5
    constexpr char _chr5 = USER_GCODE_5[strlen(USER_GCODE_5) - 1];
    static_assert(_chr5 != '\n' && _chr5 != '\r', "USER_GCODE_5 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_6
    constexpr char _chr6 = USER_GCODE_6[strlen(USER_GCODE_6) - 1];
    static_assert(_chr6 != '\n' && _chr6 != '\r', "USER_GCODE_6 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_7
    constexpr char _chr7 = USER_GCODE_7[strlen(USER_GCODE_7) - 1];
    static_assert(_chr7 != '\n' && _chr7 != '\r', "USER_GCODE_7 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_8
    constexpr char _chr8 = USER_GCODE_8[strlen(USER_GCODE_8) - 1];
    static_assert(_chr8 != '\n' && _chr8 != '\r', "USER_GCODE_8 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_9
    constexpr char _chr9 = USER_GCODE_9[strlen(USER_GCODE_9) - 1];
    static_assert(_chr9 != '\n' && _chr9 != '\r', "USER_GCODE_9 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_10
    constexpr char _chr10 = USER_GCODE_10[strlen(USER_GCODE_10) - 1];
    static_assert(_chr10 != '\n' && _chr10 != '\r', "USER_GCODE_10 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_11
    constexpr char _chr11 = USER_GCODE_11[strlen(USER_GCODE_11) - 1];
    static_assert(_chr11 != '\n' && _chr11 != '\r', "USER_GCODE_11 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_12
    constexpr char _chr12 = USER_GCODE_12[strlen(USER_GCODE_12) - 1];
    static_assert(_chr12 != '\n' && _chr12 != '\r', "USER_GCODE_12 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_13
    constexpr char _chr13 = USER_GCODE_13[strlen(USER_GCODE_13) - 1];
    static_assert(_chr13 != '\n' && _chr13 != '\r', "USER_GCODE_13 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_14
    constexpr char _chr14 = USER_GCODE_14[strlen(USER_GCODE_14) - 1];
    static_assert(_chr14 != '\n' && _chr14 != '\r', "USER_GCODE_14 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_15
    constexpr char _chr15 = USER_GCODE_15[strlen(USER_GCODE_15) - 1];
    static_assert(_chr15 != '\n' && _chr15 != '\r', "USER_GCODE_15 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_16
    constexpr char _chr16 = USER_GCODE_16[strlen(USER_GCODE_16) - 1];
    static_assert(_chr16 != '\n' && _chr16 != '\r', "USER_GCODE_16 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_17
    constexpr char _chr17 = USER_GCODE_17[strlen(USER_GCODE_17) - 1];
    static_assert(_chr17 != '\n' && _chr17 != '\r', "USER_GCODE_17 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_18
    constexpr char _chr18 = USER_GCODE_18[strlen(USER_GCODE_18) - 1];
    static_assert(_chr18 != '\n' && _chr18 != '\r', "USER_GCODE_18 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_19
    constexpr char _chr19 = USER_GCODE_19[strlen(USER_GCODE_19) - 1];
    static_assert(_chr19 != '\n' && _chr19 != '\r', "USER_GCODE_19 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_20
    constexpr char _chr20 = USER_GCODE_20[strlen(USER_GCODE_20) - 1];
    static_assert(_chr20 != '\n' && _chr20 != '\r', "USER_GCODE_20 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_21
    constexpr char _chr21 = USER_GCODE_21[strlen(USER_GCODE_21) - 1];
    static_assert(_chr21 != '\n' && _chr21 != '\r', "USER_GCODE_21 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_22
    constexpr char _chr22 = USER_GCODE_22[strlen(USER_GCODE_22) - 1];
    static_assert(_chr22 != '\n' && _chr22 != '\r', "USER_GCODE_22 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_23
    constexpr char _chr23 = USER_GCODE_23[strlen(USER_GCODE_23) - 1];
    static_assert(_chr23 != '\n' && _chr23 != '\r', "USER_GCODE_23 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_24
    constexpr char _chr24 = USER_GCODE_24[strlen(USER_GCODE_24) - 1];
    static_assert(_chr24 != '\n' && _chr24 != '\r', "USER_GCODE_24 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_25
    constexpr char _chr25 = USER_GCODE_25[strlen(USER_GCODE_25) - 1];
    static_assert(_chr25 != '\n' && _chr25 != '\r', "USER_GCODE_25 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_26
    constexpr char _chr26 = USER_GCODE_26[strlen(USER_GCODE_26) - 1];
    static_assert(_chr26 != '\n' && _chr26 != '\r', "USER_GCODE_26 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_27
    constexpr char _chr27 = USER_GCODE_27[strlen(USER_GCODE_27) - 1];
    static_assert(_chr27 != '\n' && _chr27 != '\r', "USER_GCODE_27 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_28
    constexpr char _chr28 = USER_GCODE_28[strlen(USER_GCODE_28) - 1];
    static_assert(_chr28 != '\n' && _chr28 != '\r', "USER_GCODE_28 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_29
    constexpr char _chr29 = USER_GCODE_29[strlen(USER_GCODE_29) - 1];
    static_assert(_chr29 != '\n' && _chr29 != '\r', "USER_GCODE_29 cannot have a newline at the end. Please remove it.");
  #endif
  #ifdef USER_GCODE_30
    constexpr char _chr30 = USER_GCODE_30[strlen(USER_GCODE_30) - 1];
    static_assert(_chr30 != '\n' && _chr30 != '\r', "USER_GCODE_30 cannot have a newline at the end. Please remove it.");
  #endif
#endif
