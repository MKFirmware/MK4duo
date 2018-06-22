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

/**
 * Require gcc 4.7 or newer (first included with Arduino 1.8.2) for C++11 features.
 */

#ifndef _SANITYCHECK_H_
#define _SANITYCHECK_H_

#if __cplusplus < 201103L
  #error "DEPENDENCY ERROR: MK4duo requires C++11 support (gcc >= 4.7, Arduino IDE >= 1.8.2). Please upgrade your toolchain."
#endif

// Start check
#if DISABLED(SERIAL_PORT)
  #error "DEPENDENCY ERROR: Missing setting SERIAL_PORT."
#endif
#if DISABLED(BAUDRATE)
  #error "DEPENDENCY ERROR: Missing setting BAUDRATE."
#endif
#if DISABLED(MACHINE_UUID)
  #error "DEPENDENCY ERROR: Missing setting MACHINE_UUID."
#endif

// Board
#if DISABLED(MOTHERBOARD)
  #error "DEPENDENCY ERROR: Missing setting MOTHERBOARD."
#endif
#if DISABLED(KNOWN_BOARD)
  #error "DEPENDENCY ERROR: You have to set a valid MOTHERBOARD."
#endif

// Alligatorboard
#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  #if DISABLED(UI_VOLTAGE_LEVEL)
    #error "DEPENDENCY ERROR: Missing setting UI_VOLTAGE_LEVEL."
  #endif
#endif

// Other sanitycheck files
#include "../core/temperature/sanitycheck.h"
#include "../core/heater/sanitycheck.h"
#include "../core/heater/sensor/sanitycheck.h"
#include "../core/fan/sanitycheck.h"
#include "../core/stepper/sanitycheck.h"
#include "../core/tools/sanitycheck.h"
#include "../core/endstop/sanitycheck.h"
#include "../core/mechanics/sanitycheck.h"
#include "../core/mechanics/cartesian/sanitycheck.h"
#include "../core/mechanics/core/sanitycheck.h"
#include "../core/mechanics/delta/sanitycheck.h"
#include "../core/mechanics/scara/sanitycheck.h"

#include "../lcd/sanitycheck.h"
#include "../sd/sanitycheck.h"

#include "../feature/advanced_pause/sanitycheck.h"
#include "../feature/bedlevel/sanitycheck.h"
#include "../feature/caselight/sanitycheck.h"
#include "../feature/cncrouter/sanitycheck.h"
#include "../feature/filament/sanitycheck.h"
#include "../feature/filamentrunout/sanitycheck.h"
#include "../feature/flowmeter/sanitycheck.h"
#include "../feature/fwretract/sanitycheck.h"
#include "../feature/laser/sanitycheck.h"
#include "../feature/mixing/sanitycheck.h"
#include "../feature/power/sanitycheck.h"
#include "../feature/probe/sanitycheck.h"
#include "../feature/restart/sanitycheck.h"
#include "../feature/rgbled/sanitycheck.h"
#include "../feature/servo/sanitycheck.h"
#include "../feature/tmc/sanitycheck.h"

// CONTROLLI ANCORA DA RICOLLOCARE...

#if ENABLED(DOOR_OPEN) && !PIN_EXISTS(DOOR_OPEN)
  #error "DEPENDENCY ERROR: You have to set DOOR_OPEN_PIN to a valid pin if you enable DOOR_OPEN."
#endif

// CHDK
#if ENABLED(CHDK)
  #if DISABLED(CHDK_DELAY)
    #error "DEPENDENCY ERROR: Missing setting CHDK_DELAY."
  #endif
  #if ENABLED(PHOTOGRAPH)
    #error "DEPENDENCY ERROR: CHDK and PHOTOGRAPH are incompatible."
  #endif
  #if !PIN_EXISTS(CHDK)
    #error "DEPENDENCY ERROR: You have to set CHDK_PIN to a valid pin if you enable CHDK."
  #endif
#endif

#if ENABLED(PHOTOGRAPH) && !PIN_EXISTS(PHOTOGRAPH)
  #error "DEPENDENCY ERROR: You have to set PHOTOGRAPH_PIN to a valid pin if you enable PHOTOGRAPH."
#endif

// Buffer
#if !BLOCK_BUFFER_SIZE || !IS_POWER_OF_2(BLOCK_BUFFER_SIZE)
  #error "DEPENDENCY ERROR: BLOCK_BUFFER_SIZE must be a power of 2."
#endif
#if DISABLED(MAX_CMD_SIZE)
  #error "DEPENDENCY ERROR: Missing setting MAX_CMD_SIZE."
#endif
#if DISABLED(BUFSIZE)
  #error "DEPENDENCY ERROR: Missing setting BUFSIZE."
#endif
#if ENABLED(SERIAL_XON_XOFF) && RX_BUFFER_SIZE < 1024
  #error "DEPENDENCY ERROR: For SERIAL_XON_XOFF set RX_BUFFER_SIZE to 1024 or more."
#endif
#if DISABLED(SDSUPPORT) && ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
  #error "DEPENDENCY ERROR: You must enable SDSUPPORT for SERIAL_STATS_MAX_RX_QUEUED."
#endif
#if DISABLED(SDSUPPORT) && ENABLED(SERIAL_STATS_DROPPED_RX)
  #error "DEPENDENCY ERROR: You must enable SDSUPPORT for SERIAL_STATS_DROPPED_RX."
#endif

#endif /* _SANITYCHECK_H_ */
