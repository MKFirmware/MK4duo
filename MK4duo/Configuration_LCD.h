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
 * Configuration_LCD.h
 *
 * This configuration file contains all LCD functions.
 *
 * - LCD Language
 * - LCD Character Set
 * - LCD / Controller Selection
 * - LCD Options
 * - User menu items
 *
 */

/*****************************************************************************************
 *********************************** LCD Language ****************************************
 *****************************************************************************************
 *                                                                                       *
 * Here you may choose the language used by MK4duo on the LCD menus,                     *
 * the following list of languages are available:                                        *
 *  en, an, bg, ca, cn, cz, de, el, el_gr, es, eu, fi, fr,                               *
 *  gl, hr, it, jp_kana, nl, pl, pt, pt_br, ru, sk,                                      *
 *  tr, uk, zh_CN, zh_TW                                                                 *
 *                                                                                       *
 * 'en':'English',          'an':'Aragonese', 'bg':'Bulgarian',       'ca':'Catalan',    *
 * 'cn':'Chinese',          'cz':'Czech',     'de':'German',          'el':'Greek',      *
 * 'el_gr':'Greek (Greece)' 'es':'Spanish',   'eu':'Basque-Euskera',  'fi':'Finnish',    *
 * 'fr':'French',           'gl':'Galician',  'hr':'Croatian',        'it':'Italian',    *
 * 'jp_kana':'Japanese',    'nl':'Dutch',     'pl':'Polish',          'pt':'Portuguese', *
 * 'ru':'Russian',          'sk':'Slovak',    'tr':'Turkish',         'uk':'Ukrainian',  *
 * 'pt_br':'Portuguese (Brazilian)',                                                     *
 * 'zh_CN':'Chinese (Simplified)'                                                        *
 * 'zh_TW':'Chinese (Traditional)'                                                       *
 *                                                                                       *
 *****************************************************************************************/
#define LCD_LANGUAGE    en
#define LCD_LANGUAGE_1  NO_LANGUAGE
#define LCD_LANGUAGE_2  NO_LANGUAGE
#define LCD_LANGUAGE_3  NO_LANGUAGE
#define LCD_LANGUAGE_4  NO_LANGUAGE
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** LCD Character Set **************************************
 *****************************************************************************************
 *                                                                                       *
 * Note: This option is NOT applicable to Graphical Displays.                            *
 *                                                                                       *
 * All character-based LCDs provide ASCII plus one of these                              *
 * language extensions:                                                                  *
 *                                                                                       *
 *  - JAPANESE ... the most common                                                       *
 *  - WESTERN  ... with more accented characters                                         *
 *  - CYRILLIC ... for the Russian language                                              *
 *                                                                                       *
 *****************************************************************************************/
#define LCD_CHARSET_HD44780 JAPANESE
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Info Screen Style **************************************
 *****************************************************************************************
 *                                                                                       *
 * Select the style:                                                                     *
 * 0:'Classic', 1:'Prusa'                                                                *
 *                                                                                       *
 *****************************************************************************************/
#define LCD_INFO_SCREEN_STYLE 0
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ******************************   (Character-based LCDs)   *******************************
 *****************************************************************************************/
// RepRapDiscount Smart Controller.
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//
// Note: Usually sold with a white PCB.
//
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

//
// ULTIMAKER Controller.
//
//#define ULTIMAKERCONTROLLER

//
// Ultipanel as seen on Thingiverse.
//
//#define ULTIPANEL

//
// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)
// http://reprap.org/wiki/PanelOne
//
//#define PANEL_ONE

//
// GADGETS3D G3D LCD/SD Controller
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//
// Note: Usually sold with a blue PCB.
//
//#define G3D_PANEL

//
// RigidBot Panel V1.0
// http://www.inventapart.com/
//
//#define RIGIDBOT_PANEL

//
// Makeboard 3D Printer Parts 3D Printer Mini Display 1602 Mini Controller
// https://www.aliexpress.com/item/Micromake-Makeboard-3D-Printer-Parts-3D-Printer-Mini-Display-1602-Mini-Controller-Compatible-with-Ramps-1/32765887917.html
//
//#define MAKEBOARD_MINI_2_LINE_DISPLAY_1602

//
// ANET and Tronxy 20x4 Controller
//
//#define ZONESTAR_LCD            // Requires ADC_KEYPAD_PIN to be assigned to an analog pin.
                                  // This LCD is known to be susceptible to electrical interference
                                  // which scrambles the display.  Pressing any button clears it up.
                                  // This is a LCD2004 display with 5 analog buttons.

//
// Generic 16x2, 16x4, 20x2, or 20x4 character-based LCD.
//
//#define ULTRA_LCD
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ***************************** (I2C and Shift-Register LCDs) *****************************
 *****************************************************************************************/
//
// CONTROLLER TYPE: I2C
//
// Note: These controllers require the installation of Arduino's LiquidCrystal_I2C
// library. For more info: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//

//
// Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
//
//#define RA_CONTROL_PANEL

//
// Sainsmart (YwRobot) LCD Displays
//
// These require F.Malpartida's LiquidCrystal_I2C library
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
//
//#define LCD_SAINSMART_I2C_1602
//#define LCD_SAINSMART_I2C_2004

//
// Generic LCM1602 LCD adapter
//
//#define LCM1602

//
// PANELOLU2 LCD with status LEDs,
// separate encoder and click inputs.
//
// Note: This controller requires Arduino's LiquidTWI2 library v1.2.3 or later.
// For more info: https://github.com/lincomatic/LiquidTWI2
//
// Note: The PANELOLU2 encoder click input can either be directly connected to
// a pin (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
//
//#define LCD_I2C_PANELOLU2

//
// Panucatt VIKI LCD with status LEDs,
// integrated click & L/R/U/D buttons, separate encoder inputs.
//
//#define LCD_I2C_VIKI

// Original RADDS Display from Willy
// http://max3dshop.org/index.php/default/elektronik/radds-lcd-sd-display-with-reset-and-back-buttom.html
//#define RADDS_DISPLAY

//
// CONTROLLER TYPE: Shift register panels
//

//
// 2 wire Non-latching LCD SR from https://goo.gl/aJJ4sH
// LCD configuration: http://reprap.org/wiki/SAV_3D_LCD
//
//#define SAV_3DLCD
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ******************************      (Graphical LCDs)      *******************************
 *****************************************************************************************/
//
// CONTROLLER TYPE: Graphical 128x64 (DOGM)
//
// IMPORTANT: The U8glib library is required for Graphical Display!
//            https://github.com/olikraus/U8glib_Arduino
//

//
// RepRapDiscount FULL GRAPHIC Smart Controller
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

//
// ReprapWorld Graphical LCD
// https://reprapworld.com/?products_details&products_id/1218
//
//#define REPRAPWORLD_GRAPHICAL_LCD

//
// Activate one of these if you have a Panucatt Devices
// Viki 2.0 or mini Viki with Graphic LCD
// http://panucatt.com
//
//#define VIKI2
//#define miniVIKI

//
// MakerLab Mini Panel with graphic
// controller and SD support - http://reprap.org/wiki/Mini_panel
//
//#define MINIPANEL

//
// MaKr3d Makr-Panel with graphic controller and SD support.
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//
//#define MAKRPANEL

//
// Adafruit ST7565 Full Graphic Controller.
// https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/
//
//#define ELB_FULL_GRAPHIC_CONTROLLER

//
// BQ LCD Smart Controller shipped by
// default with the BQ Hephestos 2 and Witbox 2.
//
//#define BQ_LCD_SMART_CONTROLLER

//
// WANHAO D6 SSD1309 OLED full graphics
//
//#define WANHAO_D6_OLED

//
// Cartesio UI
// http://mauk.cc/webshop/cartesio-shop/electronics/user-interface
//
//#define CARTESIO_UI

//
// LCD for Melzi Card with Graphical LCD
//
//#define LCD_FOR_MELZI

//
// SSD1306 OLED full graphics generic display
//
//#define U8GLIB_SSD1306

//
// SAV OLEd LCD module support using either SSD1306 or SH1106 based LCD modules
//
//#define SAV_3DGLCD
//#define U8GLIB_SSD1306
//#define U8GLIB_SH1106

//
// Original Ulticontroller from Ultimaker 2 printer with SSD1309 I2C display and encoder
// https://github.com/Ultimaker/Ultimaker2/tree/master/1249_Ulticontroller_Board_(x1)
//
//#define ULTI_CONTROLLER

//
// TinyBoy2 128x64 OLED / Encoder Panel
//
//#define OLED_PANEL_TINYBOY2

//
// MKS MINI12864 with graphic controller and SD support
// http://reprap.org/wiki/MKS_MINI_12864
//
//#define MKS_MINI_12864

//
// FYSETC variant of the MINI12864 graphic controller with SD support
// https://wiki.fysetc.com/Mini12864_Panel/
//
//#define FYSETC_MINI_12864

//
// Factory display for Creality CR-10
// https://www.aliexpress.com/item/Universal-LCD-12864-3D-Printer-Display-Screen-With-Encoder-For-CR-10-CR-7-Model/32833148327.html
//
// This is RAMPS-compatible using a single 10-pin connector.
// (For CR-10 owners who want to replace the Melzi Creality board but retain the display)
//
//#define CR10_STOCKDISPLAY

//
// ANET and Tronxy Graphical Controller
//
//#define ANET_FULL_GRAPHICS_LCD  // Anet 128x64 full graphics lcd with rotary encoder as used on Anet A6
                                  // A clone of the RepRapDiscount full graphics display but with
                                  // different pins/wiring (see pins_ANET_10.h).

//
// MKS OLED 1.3" 128 × 64 FULL GRAPHICS CONTROLLER
// http://reprap.org/wiki/MKS_12864OLED
//
// Tiny, but very sharp OLED display
//
//#define MKS_12864OLED          // Uses the SH1106 controller (default)
//#define MKS_12864OLED_SSD1306  // Uses the SSD1306 controller

//
// AZSMZ 12864 LCD with SD
// https://www.aliexpress.com/store/product/3D-printer-smart-controller-SMART-RAMPS-OR-RAMPS-1-4-LCD-12864-LCD-control-panel-green/2179173_32213636460.html
//
//#define AZSMZ_12864

//
// Silvergate GLCD controller
// http://github.com/android444/Silvergate
//
//#define SILVER_GATE_GLCD_CONTROLLER
/*****************************************************************************************/


/*****************************************************************************************
 ****************************** LCD / Controller Selection *******************************
 ******************************      Other Controllers     *******************************
 *****************************************************************************************/
//
// Nextion 4.3" - 5" Enanched - 7" Enanched and intelligent HMI panel
//
//#define NEXTION

// Define Serial it use
#define NEXTION_SERIAL 1

// Define Baudrate for Nextion
#define NEXTION_BAUDRATE 57600

// Define max message lenght
#define NEXTION_MAX_MESSAGE_LENGTH 30

// For GFX preview visualization enable NEXTION GFX
//#define NEXTION_GFX

// Define name firmware file for Nextion on SD
#define NEXTION_FIRMWARE_FILE "mk4duo.tft"
/*****************************************************************************************/


/*****************************************************************************************
 ************************************** LCD Options **************************************
 *****************************************************************************************/

#define SHOW_BOOTSCREEN
#define BOOTSCREEN_TIMEOUT  2500
#define BOOTSCREEN_MKLOGO_HIGH                    // Show a hight MK4duo logo on the Boot Screen (disable it saving 399 bytes of flash)
//#define BOOTSCREEN_MKLOGO_ANIMATED              // Animated MK4duo logo. Costs ~‭3260 (or ~940) bytes of PROGMEM.

//
// *** VENDORS PLEASE READ ***
//
// MK4duo allows you to add a custom boot image for Graphical LCDs.
// With this option MK4duo will first show your custom screen followed
// by the standard MK4duo logo with version number and web URL.
// We encourage you to take advantage of this new feature and we also
// respectfully request that you retain the unmodified MK4duo boot screen.
//
// Enable to show the bitmap in MK4duo/src/lcd/custom_bootscreen.h on startup.
//#define SHOW_CUSTOM_BOOTSCREEN

// Custom custom_statusscreen.h files can define:
// - A custom logo image
// - A custom heater bitmap
// - A custom fan bitmap / animation
//
// See the included examples for guidance
// Enable this to show the logo in MK4duo/src/lcd/custom_statusscreen.h on display.
//#define CUSTOM_STATUS_SCREEN_IMAGE

// Additional options for Graphical Displays
//
// Use the optimizations here to improve printing performance,
// which can be adversely affected by graphical display drawing,
// especially when doing several short moves, and when printing
// on DELTA and SCARA machines.
//
// Some of these options may result in the display lagging behind
// controller events, as there is a trade-off between reliable
// printing performance versus fast display updates.

// Enable to save many cycles by drawing a hollow frame on the Info Screen
#define XYZ_HOLLOW_FRAME

// Enable to save many cycles by drawing a hollow frame on Menu Screens
#define MENU_HOLLOW_FRAME

// A bigger font is available for edit items. Costs 3120 bytes of PROGMEM.
// Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.
//#define USE_BIG_EDIT_FONT

// A smaller font may be used on the Info Screen. Costs 2300 bytes of PROGMEM.
// Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.
//#define USE_SMALL_INFOFONT

// Enable this option and reduce the value to optimize screen updates.
// The normal delay is 10µs. Use the lowest value that still gives a reliable display.
//#define DOGM_SPI_DELAY_US 5

// Swap the CW/CCW indicators in the graphics overlay
//#define OVERLAY_GFX_REVERSE

// Status (Info) Screen customizations
// These options may affect code size and screen render time.
// Custom status screens can forcibly override these settings.
//#define STATUS_HOTEND_NUMBERLESS  // Use plain hotend icons instead of numbered ones (with 2+ hotends)
#define STATUS_HOTEND_INVERTED      // Show solid nozzle bitmaps when heating (Requires STATUS_HOTEND_ANIM)
#define STATUS_HOTEND_ANIM          // Use a second bitmap to indicate hotend heating
#define STATUS_BED_ANIM             // Use a second bitmap to indicate bed heating
#define STATUS_CHAMBER_ANIM         // Use a second bitmap to indicate chamber heating
//#define STATUS_ALT_BED_BITMAP     // Use the alternative bed bitmap
//#define STATUS_ALT_FAN_BITMAP     // Use the alternative fan bitmap
//#define STATUS_FAN_FRAMES 3       // :[0,1,2,3,4] Number of fan animation frames
//#define STATUS_HEAT_PERCENT       // Show heating in a progress bar

//
// Game Options
//
//#define GAME_BRICKOUT
//#define GAME_INVADERS
//#define GAME_SNAKE
  
//
// LCD Menu Items
//
// Disable all menus and only display the Status Screen, or
// just remove some extraneous menu items to recover space.
//
//#define NO_LCD_MENUS
//#define SLIM_LCD_MENUS

//
// ENCODER SETTINGS
//
// This option overrides the default number of encoder pulses needed to
// produce one step. Should be increased for high-resolution encoders.
#define ENCODER_PULSES_PER_STEP 5

// Use this option to override the number of step signals required to
// move between next/prev menu items.
#define ENCODER_STEPS_PER_MENU_ITEM 1

//#define LCD_SCREEN_ROT_90    // Rotate screen orientation for graphics display by 90 degree clockwise
//#define LCD_SCREEN_ROT_180   // Rotate screen orientation for graphics display by 180 degree clockwise
//#define LCD_SCREEN_ROT_270   // Rotate screen orientation for graphics display by 270 degree clockwise

//#define INVERT_CLICK_BUTTON           // Option for invert encoder button logic
//#define INVERT_BACK_BUTTON            // Option for invert back button logic if avaible

// Encoder Direction Options
// Test your encoder's behavior first with both options disabled.
//
//  Reversed Value Edit and Menu Nav? Enable REVERSE_ENCODER_DIRECTION.
//  Reversed Menu Navigation only?    Enable REVERSE_MENU_DIRECTION.
//  Reversed Value Editing only?      Enable BOTH options.

// This option reverses the encoder direction everywhere
//  Set this option if CLOCKWISE causes values to DECREASE
//#define REVERSE_ENCODER_DIRECTION

// This option reverses the encoder direction for navigating LCD menus.
//  If CLOCKWISE normally moves DOWN this makes it go UP.
//  If CLOCKWISE normally moves UP this makes it go DOWN.
//#define REVERSE_MENU_DIRECTION

// This option reverses the encoder direction for Select Screen.
//  If CLOCKWISE normally moves LEFT this makes it go RIGHT.
//  If CLOCKWISE normally moves RIGHT this makes it go LEFT.
//#define REVERSE_SELECT_DIRECTION

#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value

// Comment to disable setting feedrate multiplier via encoder
#define ULTIPANEL_FEEDMULTIPLY

// SPEAKER/BUZZER
// If you have a speaker that can produce tones, enable it here.
// By default MK4duo assumes you have a buzzer with a fixed frequency.
//#define SPEAKER

// The duration and frequency for the UI feedback sound.
// Set these to 0 to disable audio feedback in the LCD menus.

// Note: Test audio output with the G-Code:
//  M300 S<frequency Hz> P<duration ms>
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
//#define LCD_FEEDBACK_FREQUENCY_HZ 5000

// Display Voltage Logic Selector on Alligator Board
// 0 -> 3.3 V, 1 -> 5V
#define LCD_ALLIGATOR_VOLTAGE_LEVEL 1

// Include a page of printer information in the LCD Main Menu
#define LCD_INFO_MENU

// Scroll a longer status message into view
//#define STATUS_MESSAGE_SCROLLING

// On the Info Screen, display XY with one decimal place when possible
//#define LCD_DECIMAL_SMALL_XY

// The timeout (in ms) to return to the status screen from sub-menus
//#define LCD_TIMEOUT_TO_STATUS 15000

// LED Control Menu
// Enable this feature to add LED Control to the LCD menu
//#define LED_CONTROL_MENU
//#define LED_COLOR_PRESETS             // Enable the Preset Color menu option
//#define LED_USER_PRESET_STARTUP       // Have the printer display the user preset color on startup
#define LED_USER_PRESET_RED        255  // User defined RED value
#define LED_USER_PRESET_GREEN      255  // User defined GREEN value
#define LED_USER_PRESET_BLUE       255  // User defined BLUE value
#define LED_USER_PRESET_WHITE      255  // User defined WHITE value
#define LED_USER_PRESET_BRIGHTNESS 255  // User defined intensity

// Show a progress bar on HD44780 LCDs for SD printing
//#define LCD_PROGRESS_BAR
// Amount of time (ms) to show the bar
#define PROGRESS_BAR_BAR_TIME 5000U
// Amount of time (ms) to show the status message
#define PROGRESS_BAR_MSG_TIME 1500U
// Amount of time (ms) to retain the status message (0=forever)
#define PROGRESS_MSG_EXPIRE 0
// Uncomment this to show messages for MSG_TIME then hide them
//#define PROGRESS_MSG_ONCE
// Add a menu item to test the progress bar:
//#define LCD_PROGRESS_BAR_TEST
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** User menu items *************************************
 *****************************************************************************************
 *                                                                                       *
 * User defined menu items that execute custom GCode                                     *
 *                                                                                       *
 * You can create 1 - 30 user menu. For 7 - 30 add line coping the format                *
 *****************************************************************************************/
//#define CUSTOM_USER_MENUS

#define USER_SCRIPT_DONE "M117 User Script Done"

#define USER_DESC_1 "Home & ABL"
#define USER_GCODE_1 "G28\nG29"

//#define USER_DESC_2 "Preheat for " PREHEAT_1_LABEL
//#define USER_GCODE_2 "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)

//#define USER_DESC_3 "Preheat for " PREHEAT_2_LABEL
//#define USER_GCODE_3 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)

//#define USER_DESC_4 "Preheat for " PREHEAT_3_LABEL
//#define USER_GCODE_4 "M140 S" STRINGIFY(PREHEAT_3_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_3_TEMP_HOTEND)

//#define USER_DESC_5 "Heat Bed/Home/Level"
//#define USER_GCODE_5 "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nG28\nG29"

//#define USER_DESC_6 "Home & Info"
//#define USER_GCODE_6 "G28\nM503"
/*****************************************************************************************/
