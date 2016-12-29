/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#ifndef BASE_H
#define BASE_H

#include "Arduino.h"
#include "pins_arduino.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#ifdef ARDUINO_ARCH_AVR
  #include <avr/pgmspace.h>
  #include <avr/eeprom.h>
  #include <avr/interrupt.h>
#endif

#include "src/macros.h"
#include "Boards.h"
#include "src/mechanics.h"

#include "Configuration_Overall.h"
#include "Configuration_Version.h"

#ifndef CONFIGURATION_OVERALL
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
  #elif MECH(MUVE3D)
    #include "Configuration_Muve3D.h"
  #endif

  #include "Configuration_Temperature.h"
  #include "Configuration_Feature.h"
  #include "Configuration_Overall.h"
#endif

#include "src/language/language.h"

#if ENABLED(LASERBEAM)
  #include "Configuration_Laser.h"
#endif

#include "src/conditionals_pre.h"
#include "src/pins.h"
#include "src/conditionals_post.h"
#include "src/sanitycheck.h"
#include "src/HAL/HAL.h"
#include "src/enum.h"

#if HAS(ABL)
  #include "src/planner/vector_3.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_LINEAR)
  #include "src/planner/qr_solve.h"
#elif ENABLED(MESH_BED_LEVELING)
  #include "src/mbl/mesh_bed_leveling.h"
#endif

#if MECH(DELTA)
  #include "src/motion/delta_parameters.h"
#endif

#if ENABLED(LASERBEAM)
  #if ENABLED(LASER_RASTER)
    #include "src/laser/base64/base64.h"
  #endif
  #include "src/laser/laser.h"
#endif
  
#include "src/eeprom/eeprom.h"
#include "src/printcounter/duration_t.h"
#include "src/printcounter/printcounter.h"
#include "src/MK_Main.h"
#include "src/planner/planner.h"
#include "src/endstop/endstops.h"
#include "src/motion/stepper.h"
#include "src/motion/cartesian_correction.h"
#include "src/temperature/temperature.h"
#include "src/sensor/flowmeter.h"
#include "src/lcd/ultralcd.h"
#include "src/lcd/buzzer.h"
#include "src/nextion/Nextion_lcd.h"
#include "src/sd/cardreader.h"
#include "src/servo/servo.h"
#include "src/watchdog/watchdog.h"
#include "src/utility/nozzle.h"
#include "src/utility/blinkm.h"
#include "src/utility/matrix.h"

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  #include "src/alligator/external_dac.h"
#endif

#if HAS(DIGIPOTSS)
  #include <SPI.h>
#endif

#if ENABLED(RFID_MODULE)
  #include "src/mfrc522/MFRC522_serial.h"
#endif

#endif
