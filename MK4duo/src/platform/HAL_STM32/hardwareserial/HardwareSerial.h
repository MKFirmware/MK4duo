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

#ifdef USBCON
  #include <USBSerial.h>
#endif

// SERIAL ports
#if SERIAL_PORT_1 == 0
  #error "Serial port 0 does not exist"
#endif
#if !WITHIN(SERIAL_PORT_1, -1, 6)
  #error "SERIAL_PORT_1 must be from -1 to 6"
#endif
#if SERIAL_PORT_1 == -1
  #define MKSERIAL1 SerialUSB
#elif SERIAL_PORT_1 == 1
  #define MKSERIAL1 Serial1
#elif SERIAL_PORT_1 == 2
  #define MKSERIAL1 Serial2
#elif SERIAL_PORT_1 == 3
  #define MKSERIAL1 Serial3
#elif SERIAL_PORT_1 == 4
  #define MKSERIAL1 Serial4
#elif SERIAL_PORT_1 == 5
  #define MKSERIAL1 Serial5
#elif SERIAL_PORT_1 == 6
  #define MKSERIAL1 Serial6
#endif

#if ENABLED(SERIAL_PORT_2) && SERIAL_PORT_2 >= -1
  #if SERIAL_PORT_2 == 0
    #error "Serial port 0 does not exist"
  #endif
  #if !WITHIN(SERIAL_PORT_2, -1, 6)
    #error "SERIAL_PORT_2 must be from -1 to 6"
  #elif SERIAL_PORT_2 == SERIAL_PORT_1
    #error "SERIAL_PORT_2 must be different than SERIAL_PORT_1"
  #endif
  #if SERIAL_PORT_2 == -1
    #define MKSERIAL2 SerialUSB
  #elif SERIAL_PORT_2 == 1
    #define MKSERIAL2 Serial1
  #elif SERIAL_PORT_2 == 2
    #define MKSERIAL2 Serial2
  #elif SERIAL_PORT_2 == 3
    #define MKSERIAL2 Serial3
  #elif SERIAL_PORT_2 == 4
    #define MKSERIAL2 Serial4
  #elif SERIAL_PORT_2 == 5
    #define MKSERIAL2 Serial5
  #elif SERIAL_PORT_2 == 6
    #define MKSERIAL2 Serial6
  #endif
#endif

#if ENABLED(NEXTION) && NEXTION_SERIAL > 0
  #if NEXTION_SERIAL == 1
    #define nexSerial Serial1
  #elif NEXTION_SERIAL == 2
    #define nexSerial Serial2
  #elif NEXTION_SERIAL == 3
    #define nexSerial Serial3
  #elif NEXTION_SERIAL == 4
    #define nexSerial Serial4
  #elif NEXTION_SERIAL == 5
    #define nexSerial Serial5
  #elif NEXTION_SERIAL == 6
    #define nexSerial Serial6
  #endif
#endif

#if HAS_MMU2 && MMU2_SERIAL > 0
  #if MMU2_SERIAL == 1
    #define mmuSerial Serial1
  #elif MMU2_SERIAL == 2
    #define mmuSerial Serial2
  #elif MMU2_SERIAL == 3
    #define mmuSerial Serial3
  #elif MMU2_SERIAL == 4
    #define mmuSerial Serial4
  #elif MMU2_SERIAL == 5
    #define mmuSerial Serial5
  #elif MMU2_SERIAL == 6
    #define mmuSerial Serial6
  #endif
#endif
