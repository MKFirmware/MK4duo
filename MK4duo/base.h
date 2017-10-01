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

#ifndef _BASE_H_
#define _BASE_H_

#include "Arduino.h"
#include "pins_arduino.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#ifdef __AVR__
  #include <avr/pgmspace.h>
  #include <avr/eeprom.h>
  #include <avr/interrupt.h>
#endif

#include "src/macros.h"
#include "Boards.h"

#include "Configuration_Overall.h"
#include "Configuration_Version.h"

#ifndef CONFIGURATION_OVERALL
  #include "Configuration_Basic.h"
  #include "Configuration_Overall.h"

  #if IS_CARTESIAN
    #include "Configuration_Cartesian.h"
  #elif IS_CORE
    #include "Configuration_Core.h"
  #elif IS_DELTA
    #include "Configuration_Delta.h"
  #elif IS_SCARA
    #include "Configuration_Scara.h"
  #elif IS_MUVE3D
    #include "Configuration_Muve3D.h"
  #endif

  #include "Configuration_Temperature.h"
  #include "Configuration_Feature.h"
  #include "Configuration_Overall.h"
#endif

#if ENABLED(LASER)
  #include "Configuration_Laser.h"
#endif

#if ENABLED(CNCROUTER)
  #include "Configuration_CNCRouter.h"
#endif

#if ENABLED(HAVE_TMCDRIVER) || ENABLED(HAVE_TMC2130) || ENABLED(HAVE_L6470DRIVER)
  #include "Configuration_Motor_Driver.h"
#endif

#include "src/conditionals_pre.h"
#include "src/pins.h"
#include "src/conditionals_post.h"
#include "src/sanitycheck.h"
#include "src/HAL/HAL.h"
#include "src/enum.h"

#if ENABLED(LASER)
  #if ENABLED(LASER_RASTER)
    #include "src/laser/base64/base64.h"
  #endif
  #include "src/laser/laser.h"
#endif

#include "src/tools/tools.h"
#include "src/tools/nozzle.h"
#include "src/fan/fan.h"
#include "src/fan/controllerfan.h"
#include "src/commands/commands.h"
#include "src/mechanics/mechanics.h"
#include "src/eeprom/eeprom.h"
#include "src/printer/printer.h"
#include "src/planner/planner.h"
#include "src/endstop/endstops.h"
#include "src/stepper/stepper.h"
#include "src/heater/heater.h"
#include "src/temperature/temperature.h"

// LCD
#include "src/lcd/language/language.h"
#include "src/lcd/ultralcd.h"
#include "src/lcd/nextion/Nextion_lcd.h"

// SD
#include "src/sd/cardreader.h"

// Utility
#include "src/utility/utility.h"
#include "src/utility/hex_print_routines.h"
#include "src/utility/bezier.h"

// Feature
#include "src/feature/printcounter/duration_t.h"
#include "src/feature/printcounter/printcounter.h"
#include "src/feature/probe/probe.h"
#include "src/feature/bedlevel/bedlevel.h"
#include "src/feature/external_dac/external_dac.h"
#include "src/feature/servo/servo.h"
#include "src/feature/power/power.h"
#include "src/feature/buzzer/buzzer.h"
#include "src/feature/filament/filament.h"
#include "src/feature/fwretract/fwretract.h"
#include "src/feature/advanced_pause/advanced_pause.h"
#include "src/feature/cncrouter/cncrouter.h"
#include "src/feature/mfrc522/mfrc522.h"
#include "src/feature/caselight/caselight.h"
#include "src/feature/flowmeter/flowmeter.h"
#if ENABLED(BLINKM)
  #include "src/feature/rgbled/blinkm.h"
#elif ENABLED(PCA9632)
  #include "src/feature/rgbled/pca9632.h"
#elif HAS_NEOPIXEL
  #include "src/feature/rgbled/Adafruit_NeoPixel.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

#if ENABLED(HAVE_TMCDRIVER)
  #include <SPI.h>
  #include <TMC26XStepper.h>
#endif

#if ENABLED(HAVE_TMC2130DRIVER)
  #include <SPI.h>
  #include <TMC2130Stepper.h>
#endif

#if ENABLED(HAVE_L6470DRIVER)
  #include <SPI.h>
  #include <L6470.h>
#endif

#if ENABLED(ULTRA_LCD)
  #if ENABLED(LCD_I2C_TYPE_PCF8575)
    #include <Wire.h>
    #include <LiquidCrystal_I2C.h>
  #elif ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)
    #include <Wire.h>
    #include <LiquidTWI2.h>
  #elif ENABLED(LCM1602)
    #include <Wire.h>
    #include <LCD.h>
    #include <LiquidCrystal_I2C.h>
  #elif ENABLED(DOGLCD)
    #include <U8glib.h> // library for graphics LCD by Oli Kraus (https://code.google.com/p/u8glib/)
  #else
    #include <LiquidCrystal.h> // library for character LCD
  #endif
#endif

#endif /* _BASE_H_ */
