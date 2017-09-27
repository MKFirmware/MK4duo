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
 * conditionals_pre.h
 * Defines that depend on configuration but are not editable.
 */

#ifndef _CONDITIONALS_PRE_H_
#define _CONDITIONALS_PRE_H_

  #define LCD_HAS_DIRECTIONAL_BUTTONS (BUTTON_EXISTS(UP) || BUTTON_EXISTS(DWN) || BUTTON_EXISTS(LFT) || BUTTON_EXISTS(RT))

  #if ENABLED(NEXTION)
    #define LCD_HEIGHT 4
  #endif

  #if DISABLED(LCD_TIMEOUT_TO_STATUS)
    #define LCD_TIMEOUT_TO_STATUS 15000
  #endif

  #if ENABLED(SAV_3DGLCD)

    #define U8GLIB_SH1106

  #elif ENABLED(RADDS_DISPLAY)

    #define ENCODER_PULSES_PER_STEP 2
    #define ENCODER_STEPS_PER_MENU_ITEM 1
    #define ULTIPANEL
    #define NEWPANEL

  #elif ENABLED(CARTESIO_UI)

    #define DOGLCD
    #define ULTIPANEL
    #define NEWPANEL
    #define DEFAULT_LCD_CONTRAST 90
    #define LCD_CONTRAST_MIN 60
    #define LCD_CONTRAST_MAX 140

  #elif ENABLED(MAKRPANEL) || ENABLED(MINIPANEL)

    #define DOGLCD
    #define ULTIPANEL
    #define NEWPANEL
    #define DEFAULT_LCD_CONTRAST 17

  #elif ENABLED(ANET_KEYPAD_LCD)

    #define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0
    #define ADC_KEYPAD
    #define ADC_KEY_NUM 8
    #define ULTIPANEL
    #define NEWPANEL
    // this helps to implement ADC_KEYPAD menus
    #define ENCODER_STEPS_PER_MENU_ITEM 1
    #define REVERSE_MENU_DIRECTION

  #elif ENABLED(ANET_FULL_GRAPHICS_LCD)

    #define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

  #elif ENABLED(BQ_LCD_SMART_CONTROLLER)

    #define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

  #elif ENABLED(miniVIKI) || ENABLED(VIKI2) || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

    #define ULTRA_LCD // general LCD support, also 16x2
    #define DOGLCD    // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
    #define ULTIMAKERCONTROLLER // as available from the Ultimaker online store.

    #if ENABLED(miniVIKI)
      #define LCD_CONTRAST_MIN  75
      #define LCD_CONTRAST_MAX 115
      #define DEFAULT_LCD_CONTRAST 95
    #elif ENABLED(VIKI2)
      #define DEFAULT_LCD_CONTRAST 40
    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define LCD_CONTRAST_MIN  90
      #define LCD_CONTRAST_MAX 130
      #define DEFAULT_LCD_CONTRAST 110
      #define U8GLIB_LM6059_AF
      #define SD_DETECT_INVERTED
    #endif

  #elif ENABLED(RA_CONTROL_PANEL)

    #define LCD_I2C_TYPE_PCA8574
    #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
    #define ULTIPANEL
    #define NEWPANEL

  #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    #define DOGLCD
    #define U8GLIB_ST7920
    #define ULTIPANEL
    #define NEWPANEL

  #elif ENABLED(WANHAO_D6_OLED)

    #define U8GLIB_SSD1306
    #define LCD_WIDTH 22
    #define LCD_HEIGHT 5
    #define LCD_CONTRAST_MIN 10
    #define LCD_CONTRAST_MAX 255
    #define DEFAULT_LCD_CONTRAST 100
    #define ULTRA_LCD  // general LCD support, also 16x2
    #define DOGLCD // Support for I2C LCD 128x64
    #define ULTIPANEL

  #endif

  // Generic support for SSD1306 / SH1106 OLED based LCDs.
  #if ENABLED(U8GLIB_SSD1306) || ENABLED(U8GLIB_SH1106)
    #define ULTRA_LCD  //general LCD support, also 16x2
    #define DOGLCD  // Support for I2C LCD 128x64 (Controller SSD1306 / SH1106 graphic Display Family)
  #endif

  #if ENABLED(PANEL_ONE) || ENABLED(U8GLIB_SH1106)
    #define ULTIMAKERCONTROLLER
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define DOGLCD
    #define U8GLIB_ST7920
    #define REPRAP_DISCOUNT_SMART_CONTROLLER
  #endif

  #if ENABLED(ULTIMAKERCONTROLLER)              \
   || ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) \
   || ENABLED(G3D_PANEL)                        \
   || ENABLED(RIGIDBOT_PANEL)
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(REPRAPWORLD_KEYPAD)
    #define NEWPANEL
    #if ENABLED(ULTIPANEL) && DISABLED(REPRAPWORLD_KEYPAD_MOVE_STEP)
      #define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0
    #endif
  #endif

  /**
   * I2C PANELS
   */

  #if ENABLED(LCD_I2C_SAINSMART_YWROBOT)

    // This uses the LiquidCrystal_I2C library ( https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home )
    // Make sure it is placed in the Arduino libraries directory.

    #define LCD_I2C_TYPE_PCF8575
    #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
    #define ULTIPANEL
    #define NEWPANEL
  #elif ENABLED(LCD_I2C_PANELOLU2)

    // PANELOLU2 LCD with status LEDs, separate encoder and click inputs

    #define LCD_I2C_TYPE_MCP23017
    #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
    #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD
    #define ULTIPANEL
    #define NEWPANEL

  #elif ENABLED(LCD_I2C_VIKI)

    /**
     * Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
     *
     * This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
     * Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
     * Note: The pause/stop/resume LCD button pin should be connected to the Arduino
     *       BTN_ENC pin (or set BTN_ENC to -1 if not used)
     */
    #define LCD_I2C_TYPE_MCP23017
    #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
    #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
    #define ULTIPANEL
    #define NEWPANEL

    #define ENCODER_FEEDRATE_DEADZONE 4

    #define STD_ENCODER_PULSES_PER_STEP 1
    #define STD_ENCODER_STEPS_PER_MENU_ITEM 2

  #elif ENABLED(G3D_PANEL)

    #define STD_ENCODER_PULSES_PER_STEP 2
    #define STD_ENCODER_STEPS_PER_MENU_ITEM 1

  #elif ENABLED(miniVIKI) || ENABLED(VIKI2) \
     || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER) \
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

  // Shift register panels
  // ---------------------
  // 2 wire Non-latching LCD SR from:
  // https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection

  #if ENABLED(SAV_3DLCD)
    #define SR_LCD_2W_NL    // Non latching 2 wire shift register
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(DOGLCD) // Change number of lines to match the DOG graphic display
    #if DISABLED(LCD_WIDTH)
      #define LCD_WIDTH 22
    #endif
    #if DISABLED(LCD_HEIGHT)
      #define LCD_HEIGHT 5
    #endif
  #endif

  #if ENABLED(ULTIPANEL)
    #define NEWPANEL  // enable this if you have a click-encoder panel
    #define ULTRA_LCD
    #if DISABLED(LCD_WIDTH)
      #define LCD_WIDTH 20
    #endif
    #if DISABLED(LCD_HEIGHT)
      #define LCD_HEIGHT 4
    #endif
  #else //no panel but just LCD
    #if ENABLED(ULTRA_LCD)
      #if DISABLED(LCD_WIDTH)
        #define LCD_WIDTH 16
      #endif
      #if DISABLED(LCD_HEIGHT)
        #define LCD_HEIGHT 2
      #endif
    #endif
  #endif

  #if ENABLED(DOGLCD)
    /* Custom characters defined in font dogm_font_data_mk4duo_symbols.h / Marlin_symbols.fon */
    // \x00 intentionally skipped to avoid problems in strings
    #define LCD_STR_REFRESH     "\x01"
    #define LCD_STR_FOLDER      "\x02"
    #define LCD_STR_ARROW_RIGHT "\x03"
    #define LCD_STR_UPLEVEL     "\x04"
    #define LCD_STR_CLOCK       "\x05"
    #define LCD_STR_FEEDRATE    "\x06"
    #define LCD_STR_BEDTEMP     "\x07"
    #define LCD_STR_THERMOMETER "\x08"
    #define LCD_STR_DEGREE      "\x09"

    #define LCD_STR_SPECIAL_MAX '\x09'
    // Maximum here is 0x1f because 0x20 is ' ' (space) and the normal charsets begin.
    // Better stay below 0x10 because DISPLAY_CHARSET_HD44780_WESTERN begins here.

    // Symbol characters
    #define LCD_STR_FILAM_DIA   "\xf8"
    #define LCD_STR_FILAM_MUL   "\xa4"
  #else
    // Custom characters defined in the first 8 characters of the LCD
    // Print double-quoted items only as char. They may have 'unexpected' results if used in strings!
    #define LCD_BEDTEMP_CHAR     0x00  // Print only as a char. This will have 'unexpected' results when used in a string!
    #define LCD_DEGREE_CHAR      0x01
    #define LCD_STR_THERMOMETER "\x02" // Still used with string concatenation
    #define LCD_UPLEVEL_CHAR     0x03
    #define LCD_STR_REFRESH     "\x04"
    #define LCD_STR_FOLDER      "\x05"
    #define LCD_FEEDRATE_CHAR    0x06
    #define LCD_CLOCK_CHAR       0x07
    #define LCD_STR_ARROW_RIGHT ">"  /* from the default character set */
  #endif

  /**
   * Default LCD contrast for dogm-like LCD displays
   */
  #if ENABLED(DOGLCD)

    #define HAS_LCD_CONTRAST ( \
        ENABLED(MAKRPANEL) \
     || ENABLED(CARTESIO_UI) \
     || ENABLED(VIKI2) \
     || ENABLED(miniVIKI) \
     || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER) \
     || ENABLED(WANHAO_D6_OLED) \
    )

    #if HAS_LCD_CONTRAST
      #ifndef LCD_CONTRAST_MIN
        #define LCD_CONTRAST_MIN 0
      #endif
      #ifndef LCD_CONTRAST_MAX
        #define LCD_CONTRAST_MAX 63
      #endif
      #ifndef DEFAULT_LCD_CONTRAST
        #define DEFAULT_LCD_CONTRAST 32
      #endif
    #endif
  #else
    #define HAS_LCD_CONTRAST false
  #endif

  // Boot screens
  #if DISABLED(ULTRA_LCD)
    #undef SHOW_BOOTSCREEN
  #elif DISABLED(BOOTSCREEN_TIMEOUT)
    #define BOOTSCREEN_TIMEOUT 2500
  #endif

  #define HAS_LCD         (ENABLED(NEWPANEL) || ENABLED(NEXTION))
  #define HAS_DEBUG_MENU  (ENABLED(LCD_PROGRESS_BAR_TEST))

  /**
   * The BLTouch Probe emulates a servo probe
   */
  #if ENABLED(BLTOUCH)
    #if DISABLED(ENABLE_SERVOS)
      #define ENABLE_SERVOS
    #endif
    #if Z_ENDSTOP_SERVO_NR < 0
      #undef Z_ENDSTOP_SERVO_NR
      #define Z_ENDSTOP_SERVO_NR 0
    #endif
    #if NUM_SERVOS < 1
      #undef NUM_SERVOS
      #define NUM_SERVOS (Z_ENDSTOP_SERVO_NR + 1)
    #endif
    #undef DEACTIVATE_SERVOS_AFTER_MOVE
    #undef SERVO_DEACTIVATION_DELAY
    #define SERVO_DEACTIVATION_DELAY 50
    #if DISABLED(BLTOUCH_DELAY)
      #define BLTOUCH_DELAY 375
    #endif
    #undef Z_ENDSTOP_SERVO_ANGLES
    #define Z_ENDSTOP_SERVO_ANGLES { BLTOUCH_DEPLOY, BLTOUCH_STOW }

    #define BLTOUCH_DEPLOY    10
    #define BLTOUCH_STOW      90
    #define BLTOUCH_SELFTEST 120
    #define BLTOUCH_RESET    160
    #define _TEST_BLTOUCH(P) (READ(P##_PIN) != P##_ENDSTOP_INVERTING)
  #endif

  /**
   * Extruders have some combination of stepper motors and hotends
   * so we separate these concepts into the defines:
   *
   *  EXTRUDERS         - Number of Selectable Tools
   *  HOTENDS           - Number of hotends, whether connected or separate
   *  DRIVER_EXTRUDERS  - Number of driver extruders
   *  TOOL_E_INDEX      - Index to use when getting/setting the tool state
   *
   */
  #if ENABLED(DONDOLO_SINGLE_MOTOR)        // One E stepper, unified E axis, two hotends 
    #undef SINGLENOZZLE
    #undef ADVANCE
    #undef EXTRUDERS
    #undef DRIVER_EXTRUDERS
    #define EXTRUDERS         2
    #define DRIVER_EXTRUDERS  1
    #define TOOL_E_INDEX      0
  #elif ENABLED(DONDOLO_DUAL_MOTOR)         // Two E stepper, two hotends
    #undef SINGLENOZZLE
    #undef ADVANCE
    #undef EXTRUDERS
    #undef DRIVER_EXTRUDERS
    #define EXTRUDERS         2
    #define DRIVER_EXTRUDERS  2
    #define TOOL_E_INDEX      current_block->active_extruder
  #elif ENABLED(COLOR_MIXING_EXTRUDER)      // Multi-stepper, unified E axis, one hotend
    #define SINGLENOZZLE
    #undef EXTRUDERS
    #undef DRIVER_EXTRUDERS
    #define EXTRUDERS         1
    #define DRIVER_EXTRUDERS  MIXING_STEPPERS
    #define TOOL_E_INDEX      0
  #else
    #define TOOL_E_INDEX      current_block->active_extruder
  #endif

  #define TOOL_DE_INDEX       current_block->active_driver

  #if ENABLED(SINGLENOZZLE)                 // One hotend, multi-extruder
    #undef HOTENDS
    #define HOTENDS           1
    #undef TEMP_SENSOR_1_AS_REDUNDANT
    #undef HOTEND_OFFSET_X
    #undef HOTEND_OFFSET_Y
    #undef HOTEND_OFFSET_Z
    #define HOTEND_OFFSET_X   { 0 }
    #define HOTEND_OFFSET_Y   { 0 }
    #define HOTEND_OFFSET_Z   { 0 }
  #else
    #undef HOTENDS
    #define HOTENDS           EXTRUDERS
  #endif

  /**
   * Multi-extruders support
   */
  #if EXTRUDERS > 1
    #define XYZE_N    3 + EXTRUDERS
    #define E_AXIS_N  (E_AXIS + extruder)
    #define E_INDEX   (E_AXIS + tools.active_extruder)
    #define GET_TARGET_EXTRUDER(CMD) if (printer.get_target_tool_from_command(CMD)) return
    #define TARGET_EXTRUDER tools.target_extruder
  #elif EXTRUDERS == 1
    #define XYZE_N    XYZE
    #define E_AXIS_N  E_AXIS
    #define E_INDEX   E_AXIS
    #define GET_TARGET_EXTRUDER(CMD) NOOP
    #define TARGET_EXTRUDER 0
  #elif EXTRUDERS == 0
    #undef PIDTEMP
    #define PIDTEMP false
    #undef FWRETRACT
    #define XYZE_N    XYZ
    #define E_AXIS_N  0
    #define E_INDEX   0
    #define GET_TARGET_EXTRUDER(CMD) NOOP
    #define TARGET_EXTRUDER 0
  #endif

  /**
   * Multi-hotends support
   */
  #if HOTENDS > 1
    #define GET_TARGET_HOTEND(CMD) if (printer.get_target_tool_from_command(CMD)) return
  #else
    #define GET_TARGET_HOTEND(CMD) NOOP
  #endif

  #define HAS_SOFTWARE_ENDSTOPS (ENABLED(MIN_SOFTWARE_ENDSTOPS) || ENABLED(MAX_SOFTWARE_ENDSTOPS))
  #define HAS_RESUME_CONTINUE   (HAS_LCD || ENABLED(EMERGENCY_PARSER))

  // RGB Leds
  #define HAS_NEOPIXEL          (ENABLED(NEOPIXEL_RGB_LED) || ENABLED(NEOPIXEL_RGBW_LED))
  #define HAS_COLOR_LEDS        (ENABLED(BLINKM) || ENABLED(RGB_LED) || ENABLED(RGBW_LED) || ENABLED(PCA9632) || ENABLED(NEOPIXEL_RGB_LED) || ENABLED(NEOPIXEL_RGBW_LED))

#endif /* _CONDITIONALS_PRE_H_ */
