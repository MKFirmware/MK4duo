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
 * conditionals_pre.h
 * Defines that depend on configuration but are not editable.
 */

#if ENABLED(NEXTION)
  #define LCD_WIDTH 25
  #define LCD_HEIGHT 6
  #define LCD_DECIMAL_SMALL_XY
  #undef GAME_BRICKOUT
  #undef GAME_INVADERS
  #undef GAME_SNAKE
  #undef GAME_MAZE
  #undef ENCODER_RATE_MULTIPLIER
  #undef ULTIPANEL_FEEDMULTIPLY
  #undef REVERSE_ENCODER_DIRECTION
  #define REVERSE_MENU_DIRECTION
  #if DISABLED(NEXTION_MAX_MESSAGE_LENGTH)
    #define NEXTION_MAX_MESSAGE_LENGTH 30
  #endif
#endif

#if DISABLED(LCD_TIMEOUT_TO_STATUS)
  #define LCD_TIMEOUT_TO_STATUS 15000
#endif

#if ENABLED(CARTESIO_UI)

  #define DOGLCD
  #define IS_ULTIPANEL

#elif ENABLED(ZONESTAR_LCD)

  #define ADC_KEYPAD
  #define IS_RRW_KEYPAD
  #define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0
  #define ADC_KEY_NUM 8
  #define IS_ULTIPANEL

  // this helps to implement ADC_KEYPAD menus
  #define ENCODER_PULSES_PER_STEP 1
  #define ENCODER_STEPS_PER_MENU_ITEM 1
  #define ENCODER_FEEDRATE_DEADZONE 2
  #define REVERSE_MENU_DIRECTION

#elif ENABLED(RADDS_DISPLAY)

  #define IS_ULTIPANEL
  #define ENCODER_PULSES_PER_STEP 2

#elif ENABLED(ANET_FULL_GRAPHICS_LCD) || ENABLED(BQ_LCD_SMART_CONTROLLER)

  #define IS_RRD_FG_SC

#elif ENABLED(miniVIKI) || ENABLED(VIKI2) || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER) || ENABLED(AZSMZ_12864)

  #define IS_ULTRA_LCD
  #define DOGLCD
  #define IS_ULTIPANEL

  #if ENABLED(miniVIKI)
    #define U8GLIB_ST7565_64128N
  #elif ENABLED(VIKI2)
    #define U8GLIB_ST7565_64128N
  #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    #define U8GLIB_LM6059_AF
    #define SD_DETECT_INVERTED
  #elif ENABLED(AZSMZ_12864)
    #define U8GLIB_ST7565_64128N
  #endif

#elif ENABLED(OLED_PANEL_TINYBOY2)

  #define IS_U8GLIB_SSD1306
  #define IS_ULTIPANEL

#elif ENABLED(RA_CONTROL_PANEL)

  #define LCD_I2C_TYPE_PCA8574
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
  #define IS_ULTIPANEL

#elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #define DOGLCD
  #define U8GLIB_ST7920
  #define IS_ULTIPANEL

#elif ENABLED(WANHAO_D6_OLED)

  #define IS_U8GLIB_SSD1306
  #define IS_ULTIPANEL
  #define LCD_WIDTH 22
  #define LCD_HEIGHT 5
  #define LCD_RESET_PIN       5
  #define SPEAKER

#elif ENABLED(CR10_STOCKDISPLAY)

  #define IS_RRD_FG_SC
  #ifndef ST7920_DELAY_1
    #define ST7920_DELAY_1 DELAY_NS(125)
  #endif
  #ifndef ST7920_DELAY_2
    #define ST7920_DELAY_2 DELAY_NS(125)
  #endif
  #ifndef ST7920_DELAY_3
    #define ST7920_DELAY_3 DELAY_NS(125)
  #endif

#elif ENABLED(MKS_12864OLED)

  #define IS_RRD_SC
  #define U8GLIB_SH1106

#elif ENABLED(MKS_12864OLED_SSD1306)

  #define IS_RRD_SC
  #define IS_U8GLIB_SSD1306

#elif ENABLED(MKS_MINI_12864)

  #define MINIPANEL

#elif ENABLED(FYSETC_MINI_12864)

  #define DOGLCD
  #define IS_ULTIPANEL

#elif ENABLED(ULTI_CONTROLLER)

  #define U8GLIB_SSD1309
  #define LCD_RESET_PIN LCD_PINS_D6 //  This controller need a reset pin
  #define ENCODER_PULSES_PER_STEP 2
  #define ENCODER_STEPS_PER_MENU_ITEM 2

#elif ENABLED(MAKEBOARD_MINI_2_LINE_DISPLAY_1602)

  #define IS_RRD_SC
  #define LCD_WIDTH 16
  #define LCD_HEIGHT 2

#endif

#if ENABLED(IS_RRD_FG_SC)
  #define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
#endif

#if ENABLED(MAKRPANEL) || ENABLED(MINIPANEL)
  #define DOGLCD
  #define IS_ULTIPANEL
  #if ENABLED(MAKRPANEL)
    #define U8GLIB_ST7565_64128N
  #endif
#endif

#if ENABLED(IS_U8GLIB_SSD1306)
  #define U8GLIB_SSD1306
#endif

// 128x64 I2C OLED LCDs - SSD1306/SSD1309/SH1106
#define HAS_SSD1306_OLED_I2C ENABLED(U8GLIB_SSD1306) || ENABLED(U8GLIB_SSD1309) || ENABLED(U8GLIB_SH1106)
#if HAS_SSD1306_OLED_I2C
  #define IS_ULTRA_LCD
  #define DOGLCD
#endif

#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER) || ENABLED(LCD_FOR_MELZI) || ENABLED(SILVER_GATE_GLCD_CONTROLLER)
  #define DOGLCD
  #define U8GLIB_ST7920
  #define IS_RRD_SC
#endif

#if ENABLED(IS_RRD_SC)
  #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

#if ENABLED(ULTIMAKERCONTROLLER) || ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL) || ENABLED(RIGIDBOT_PANEL) || ENABLED(ULTI_CONTROLLER) || ENABLED(PANEL_ONE) || ENABLED(U8GLIB_SH1106)
  #define IS_ULTIPANEL
#endif

/**
 * SPI PANELS
 */

// Einstart OLED has Cardinal nav via pins defined in pins_EINSTART-S.h
#if ENABLED(U8GLIB_SH1106_EINSTART)
  #define DOGLCD
  #define IS_ULTIPANEL
#endif
 
/**
 * I2C PANELS
 */

#if ENABLED(LCD_SAINSMART_I2C_1602) || ENABLED(LCD_SAINSMART_I2C_2004)

  #define LCD_I2C_TYPE_PCF8575
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander

  #if ENABLED(LCD_SAINSMART_I2C_2004)
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif

#elif ENABLED(LCD_I2C_PANELOLU2)

  // PANELOLU2 LCD with status LEDs, separate encoder and click inputs

  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER   // Enable buzzer on LCD (optional)
  #define IS_ULTIPANEL

#elif ENABLED(LCD_I2C_VIKI)

  /**
   * Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
   *
   * This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
   * Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
   * Note: The pause/stop/resume LCD button pin should be connected to the Arduino
   *       BTN_ENC pin (or set BTN_ENC to NoPin if not used)
   */
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER   // Enable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
  #define IS_ULTIPANEL

  #define ENCODER_FEEDRATE_DEADZONE 4

  #define STD_ENCODER_PULSES_PER_STEP 1
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 2

#elif ENABLED(G3D_PANEL)

  #define STD_ENCODER_PULSES_PER_STEP 2
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1

#elif ENABLED(miniVIKI) || ENABLED(VIKI2) \
   || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER) \
   || ENABLED(AZSMZ_12864) \
   || ENABLED(OLED_PANEL_TINYBOY2) \
   || ENABLED(BQ_LCD_SMART_CONTROLLER) \
   || ENABLED(LCD_I2C_PANELOLU2) \
   || ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
  #define STD_ENCODER_PULSES_PER_STEP 4
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
#endif

#ifndef STD_ENCODER_PULSES_PER_STEP
  #define STD_ENCODER_PULSES_PER_STEP 5
#endif
#ifndef STD_ENCODER_STEPS_PER_MENU_ITEM
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
#endif
#ifndef ENCODER_PULSES_PER_STEP
  #define ENCODER_PULSES_PER_STEP STD_ENCODER_PULSES_PER_STEP
#endif
#ifndef ENCODER_STEPS_PER_MENU_ITEM
  #define ENCODER_STEPS_PER_MENU_ITEM STD_ENCODER_STEPS_PER_MENU_ITEM
#endif
#ifndef ENCODER_FEEDRATE_DEADZONE
  #define ENCODER_FEEDRATE_DEADZONE 6
#endif

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection
#if ENABLED(FF_INTERFACEBOARD)
  #define SR_LCD_3W_NL    // Non latching 3 wire shift register
  #define IS_ULTIPANEL
#elif ENABLED(SAV_3DLCD)
  #define SR_LCD_2W_NL    // Non latching 2 wire shift register
  #define IS_ULTIPANEL
#endif

#if ENABLED(IS_ULTIPANEL)
  #define ULTIPANEL
#endif

#if ENABLED(ULTIPANEL)
  #define IS_ULTRA_LCD
  #ifndef NEWPANEL
    #define NEWPANEL
  #endif
#endif

#if ENABLED(IS_ULTRA_LCD)
  #define ULTRA_LCD
#endif

#if ENABLED(IS_RRW_KEYPAD)
  #define REPRAPWORLD_KEYPAD
#endif

// Keypad needs a move step
#if ENABLED(REPRAPWORLD_KEYPAD)
  #define NEWPANEL
  #ifndef REPRAPWORLD_KEYPAD_MOVE_STEP
    #define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0
  #endif
#endif

// Aliases for LCD features
#define HAS_NEXTION_LCD       ENABLED(NEXTION)
#define HAS_SPI_LCD           ENABLED(IS_ULTRA_LCD)
#define HAS_GRAPHICAL_LCD     ENABLED(DOGLCD)
#define HAS_CHARACTER_LCD     (HAS_SPI_LCD && !HAS_GRAPHICAL_LCD)
#define HAS_LCD               (HAS_SPI_LCD || HAS_NEXTION_LCD)
#define HAS_LCD_MENU          ((ENABLED(ULTIPANEL) || ENABLED(NEXTION)) && DISABLED(NO_LCD_MENUS))

#define HAS_ADC_BUTTONS       ENABLED(ADC_KEYPAD)

#define HAS_DIGITAL_BUTTONS   (!HAS_ADC_BUTTONS && ENABLED(NEWPANEL)        \
                              || (BUTTON_EXISTS(EN1) && BUTTON_EXISTS(EN2)) \
                              || BUTTON_EXISTS(ENC) || BUTTON_EXISTS(BACK)  \
                              || BUTTON_EXISTS(UP)  || BUTTON_EXISTS(DWN)   \
                              || BUTTON_EXISTS(LFT) || BUTTON_EXISTS(RT))

#define HAS_SHIFT_ENCODER     (!HAS_ADC_BUTTONS && (ENABLED(IS_RRW_KEYPAD) || (HAS_SPI_LCD && DISABLED(NEWPANEL))))
#define HAS_ENCODER_WHEEL     ((!HAS_ADC_BUTTONS && ENABLED(NEWPANEL)) || (BUTTON_EXISTS(EN1) && BUTTON_EXISTS(EN2)) )
#define HAS_ENCODER_ACTION    (HAS_LCD_MENU || ENABLED(ULTIPANEL_FEEDMULTIPLY))

// I2C buttons must be read in the main thread
#define HAS_SLOW_BUTTONS      (ENABLED(LCD_I2C_VIKI) || ENABLED(LCD_I2C_PANELOLU2))

#define LCD_HAS_DIRECTIONAL_BUTTONS (BUTTON_EXISTS(UP) || BUTTON_EXISTS(DWN) || BUTTON_EXISTS(LFT) || BUTTON_EXISTS(RT))

/**
 * Extruders have some combination of stepper motors and hotends
 * so we separate these concepts into the defines:
 *
 *  EXTRUDERS         - Number of Selectable Tools
 *  HOTENDS           - Number of hotends, whether connected or separate
 *  DRIVER_EXTRUDERS  - Number of driver extruders
 *  E_MANUAL     - Number of E steppers for LCD move options
 *
 */
#if ENABLED(DONDOLO_SINGLE_MOTOR)        // One E stepper, unified E axis, two hotends
  #undef  SINGLENOZZLE
  #undef  EXTRUDERS
  #undef  DRIVER_EXTRUDERS
  #define EXTRUDERS         2
  #define DRIVER_EXTRUDERS  1
  #define E_MANUAL          2
#elif ENABLED(DONDOLO_DUAL_MOTOR)         // Two E stepper, two hotends
  #undef  SINGLENOZZLE
  #undef  EXTRUDERS
  #undef  DRIVER_EXTRUDERS
  #define EXTRUDERS         2
  #define DRIVER_EXTRUDERS  2
  #define E_MANUAL          2
#elif ENABLED(COLOR_MIXING_EXTRUDER)      // Multi-stepper, unified E axis, one hotend
  #define SINGLENOZZLE
  #undef  EXTRUDERS
  #undef  DRIVER_EXTRUDERS
  #define EXTRUDERS         1
  #define DRIVER_EXTRUDERS  MIXING_STEPPERS
  #define E_MANUAL          1
  #define HAS_GRADIENT_MIX  (MIXING_STEPPERS == 2)
#elif ENABLED(PRUSA_MMU2)                 // Multi-Material-Unit V2
  #define SINGLENOZZLE
  #undef  EXTRUDERS
  #undef  DRIVER_EXTRUDERS
  #define EXTRUDERS         5
  #define DRIVER_EXTRUDERS  1
  #define E_MANUAL          5
  #ifndef TOOLCHANGE_ZRAISE
    #define TOOLCHANGE_ZRAISE 0
  #endif
#else
  #undef  DRIVER_EXTRUDERS
  #define DRIVER_EXTRUDERS  EXTRUDERS
  #define E_MANUAL          EXTRUDERS
#endif

/**
 * One hotend, multi-extruder
 */
#if ENABLED(SINGLENOZZLE)
  #undef HOTENDS
  #define HOTENDS           1
  #undef TEMP_SENSOR_1_AS_REDUNDANT
  #undef HOTEND_OFFSET_X
  #undef HOTEND_OFFSET_Y
  #undef HOTEND_OFFSET_Z
  #define HOTEND_OFFSET_X   { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
  #define HOTEND_OFFSET_Y   { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
  #define HOTEND_OFFSET_Z   { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
#else
  #undef HOTENDS
  #define HOTENDS           EXTRUDERS
#endif

/**
 * Multi-extruders support
 */
#define XYZE_N              (uint8_t(E_AXIS) + toolManager.extruder.total)
#define E_INDEX             (uint8_t(E_AXIS) + toolManager.extruder.active)
#define E_INDEX_N(N)        (uint8_t(E_AXIS) + N)

/**
 * The BLTouch Probe emulates a servo probe
 */
#if ENABLED(BLTOUCH)
  #if DISABLED(ENABLE_SERVOS)
    #define ENABLE_SERVOS
  #endif
  #if PROBE_SERVO_NR < 0
    #undef PROBE_SERVO_NR
    #define PROBE_SERVO_NR 0
  #endif
  #if NUM_SERVOS < 1
    #undef NUM_SERVOS
    #define NUM_SERVOS (PROBE_SERVO_NR + 1)
  #endif
  #undef DEACTIVATE_SERVOS_AFTER_MOVE
  #undef SERVO_DEACTIVATION_DELAY
  #define SERVO_DEACTIVATION_DELAY 50
  #if DISABLED(BLTOUCH_DELAY)
    #define BLTOUCH_DELAY 500
  #endif
  #undef PROBE_SERVO_ANGLES
  #define PROBE_SERVO_ANGLES { BLTOUCH_CMD_DEPLOY, BLTOUCH_CMD_STOW }

#endif

// Label Preheat
#ifndef PREHEAT_1_LABEL
  #define PREHEAT_1_LABEL "PLA"
#endif
#ifndef PREHEAT_2_LABEL
  #define PREHEAT_2_LABEL "ABS"
#endif
#ifndef PREHEAT_3_LABEL
  #define PREHEAT_3_LABEL "GUM"
#endif

// Boot Screen small on slim menu
#if ENABLED(SLIM_LCD_MENUS) && ENABLED(BOOTSCREEN_MKLOGO_HIGH)
  #undef BOOTSCREEN_MKLOGO_HIGH
#endif
