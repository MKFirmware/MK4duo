/******************************************************************************************
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
 * ----------------------------------------------------------------------------------------
 *
 * MK4duo - Main Include
 * This includes all the other include files in the right order and defines some globals.
 *
 * Alberto Cotronei @MagoKimbra
 * Blog: http://www.marlinkimbra.it
 * WiKI: https://mk4duowiki.altervista.org/wiki/doku.php
 *
 * Licence: GPL
 *
 *******************************************************************************************/

#ifndef _MK4DUO_H_
#define _MK4DUO_H_

#include "Arduino.h"
#include "pins_arduino.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <binary.h>

#ifdef __AVR__
  #include <avr/pgmspace.h>
  #include <avr/eeprom.h>
  #include <avr/interrupt.h>
#endif

#include <SPI.h>

/**
 * Types
 */
typedef uint32_t  millis_t;
typedef int8_t    pin_t;


#include "src/inc/macros.h"
#include "src/inc/driver_types.h"
#include "Boards.h"

/**
 * Configuration settings loading
 */

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
  #include "Configuration_Motor_Driver.h"
  #include "Configuration_Overall.h"
#endif

#if ENABLED(LASER)
  #include "Configuration_Laser.h"
#endif

#if ENABLED(CNCROUTER)
  #include "Configuration_CNCRouter.h"
#endif

/**
 * Modules loading
 */

// Include modules
#include "src/inc/conditionals_pre.h"
#include "src/inc/pins.h"
#include "src/inc/conditionals_post.h"
#include "src/inc/sanitycheck.h"
#include "src/inc/point_t.h"

// HAL modules
#include "src/HAL/HAL.h"

// Watch modules
#include "src/watch/watch.h"
#include "src/watch/stopwatch.h"

// Core modules
#include "src/core/mechanics/mechanics.h"
#include "src/core/tools/tools.h"
#include "src/core/tools/nozzle.h"
#include "src/core/fan/fan.h"
#include "src/core/fan/tachometric.h"
#include "src/core/commands/commands.h"
#include "src/core/eeprom/eeprom.h"
#include "src/core/printer/printer.h"
#include "src/core/planner/planner.h"
#include "src/core/endstop/endstops.h"
#include "src/core/stepper/stepper.h"
#include "src/core/heater/sensor/thermistor.h"
#include "src/core/heater/heater.h"
#include "src/core/temperature/temperature.h"
#include "src/core/printcounter/printcounter.h"

// LCD modules
#include "src/lcd/language/language.h"
#include "src/lcd/ultralcd.h"
#include "src/lcd/nextion/Nextion_lcd.h"

// SD modules
#include "src/sd/cardreader.h"

// Utility modules
#include "src/utility/utility.h"
#include "src/utility/hex_print_routines.h"
#include "src/utility/bezier.h"

// Feature modules
#include "src/feature/emergency_parser/emergency_parser.h"
#include "src/feature/probe/probe.h"
#include "src/feature/bedlevel/bedlevel.h"
#include "src/feature/external_dac/external_dac.h"
#include "src/feature/tmc/tmc.h"
#include "src/feature/servo/servo.h"
#include "src/feature/power/power.h"
#include "src/feature/buzzer/buzzer.h"
#include "src/feature/mixing/mixing.h"
#include "src/feature/filament/filament.h"
#include "src/feature/filamentrunout/filamentrunout.h"
#include "src/feature/fwretract/fwretract.h"
#include "src/feature/advanced_pause/advanced_pause.h"
#include "src/feature/laser/base64/base64.h"
#include "src/feature/laser/laser.h"
#include "src/feature/cncrouter/cncrouter.h"
#include "src/feature/mfrc522/mfrc522.h"
#include "src/feature/flowmeter/flowmeter.h"
#include "src/feature/dhtsensor/dhtsensor.h"
#include "src/feature/rgbled/led.h"
#include "src/feature/caselight/caselight.h"
#include "src/feature/restart/restart.h"

/**
 * External libraries loading
 */

#if HAVE_DRV(TMC26X)
  #include <TMC26XStepper.h>
#endif

#if HAVE_DRV(TMC2130)
  #include <TMC2130Stepper.h>
#endif

#if HAVE_DRV(TMC2208)
  #include <TMC2208Stepper.h>
#endif

#if HAVE_DRV(L6470)
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

#endif /* _MK4DUO_H_ */
