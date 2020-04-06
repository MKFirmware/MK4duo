/******************************************************************************************
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
#pragma once

#include "Arduino.h"
#include "pins_arduino.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __AVR__
  #include <avr/pgmspace.h>
  #include <avr/eeprom.h>
  #include <avr/interrupt.h>
#endif

#include <SPI.h>

/**
 * Macro for start
 */
#include "src/lib/macros.h"

/**
 * Include file
 */
#include "Boards.h"

// Configuration settings loading
#include "Configuration_Overall.h"
#include "Configuration_Version.h"

#if DISABLED(CONFIGURATION_OVERALL)
  #include "Configuration_Basic.h"
  #include "Configuration_Overall.h"

  #if MECH(CARTESIAN)
    #include "Configuration_Cartesian.h"
  #elif IS_CORE
    #include "Configuration_Core.h"
  #elif MECH(DELTA)
    #include "Configuration_Delta.h"
  #elif IS_SCARA
    #include "Configuration_Scara.h"
  #elif IS_MUVE3D
    #include "Configuration_Muve3D.h"
  #endif

  #include "Configuration_Temperature.h"
  #include "Configuration_LCD.h"
  #include "Configuration_Feature.h"
  #include "Configuration_Driver.h"
  #include "Configuration_Overall.h"
#endif

#if ENABLED(LASER)
  #include "Configuration_Laser.h"
#endif

#if ENABLED(CNCROUTER)
  #include "Configuration_CNCRouter.h"
#endif

#include "src/conditionals/conditionals_pre.h"
#include "src/conditionals/conditionals_pins.h"
#include "src/conditionals/conditionals_post.h"

// Lib modules
#include "src/lib/types.h"
#include "src/lib/timer.h"
#include "src/lib/enum.h"
#include "src/lib/restorer.h"
#include "src/lib/circular_queue.h"
#include "src/lib/driver_types.h"
#include "src/lib/duration_t.h"
#include "src/lib/matrix.h"
#include "src/lib/vector_3/vector_3.h"
#include "src/lib/least_squares_fit/least_squares_fit.h"

// Platform modules
#include "src/platform/platform.h"

// Core modules
#include "src/core/hostaction/hostaction.h"
#include "src/core/utility/utility.h"
#include "src/core/watch/watch.h"
#include "src/core/mechanics/mechanics.h"
#include "src/core/toolmanager/toolmanager.h"
#include "src/core/nozzle/nozzle.h"
#include "src/core/fanmanager/fanmanager.h"
#include "src/core/eeprom/eeprom.h"
#include "src/core/printer/printer.h"
#include "src/core/planner/planner.h"
#include "src/core/endstop/endstops.h"
#include "src/core/stepper/stepper.h"
#include "src/core/tempmanager/tempmanager.h"
#include "src/core/printcounter/printcounter.h"
#include "src/core/sdcard/sdcard.h"
#include "src/core/sound/sound.h"

// Command modules
#include "src/commands/commands.h"

// Language modules
#include "src/language/language.h"

// Font modules
#include "src/lcd/fontutils/fontutils.h"

// LcdUI modules
#include "src/lcd/lcdui.h"

// LCD type modules
#include "src/lcd/ultralcd/ultralcd.h"
#include "src/lcd/nextionlcd/nextionlcd.h"

// Menu modules
#include "src/lcd/menu/menu.h"

// Feature modules
#include "src/feature/bezier/bezier.h"
#include "src/feature/digipot/digipot.h"
#include "src/feature/emergency_parser/emergency_parser.h"
#include "src/feature/probe/probe.h"
#include "src/feature/bedlevel/bedlevel.h"
#include "src/feature/babystep/babystep.h"
#include "src/feature/bltouch/bltouch.h"
#include "src/feature/external_dac/external_dac.h"
#include "src/feature/hysteresis/hysteresis.h"
#include "src/feature/power/power.h"
#include "src/feature/mixing/mixing.h"
#include "src/feature/mmu2/mmu2.h"
#include "src/feature/filament/filament.h"
#include "src/feature/fwretract/fwretract.h"
#include "src/feature/flowmeter/flowmeter.h"
#include "src/feature/advanced_pause/advanced_pause.h"
#include "src/feature/filamentrunout/filamentrunout.h"
#include "src/feature/laser/base64/base64.h"
#include "src/feature/laser/laser.h"
#include "src/feature/cncrouter/cncrouter.h"
#include "src/feature/mfrc522/mfrc522.h"
#include "src/feature/pcf8574/pcf8574.h"
#include "src/feature/rgbled/led.h"
#include "src/feature/rgbled/led_events.h"
#include "src/feature/caselight/caselight.h"
#include "src/feature/restart/restart.h"
