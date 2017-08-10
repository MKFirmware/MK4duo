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

#ifndef _SPI_PINS_AVR_H_
#define _SPI_PINS_AVR_H_

#if ENABLED(__AVR_ATmega1280__) || ENABLED(__AVR_ATmega2560__)
  #define MOSI_PIN            51
  #define MISO_PIN            50
  #define SCK_PIN             52
  #define SS_PIN              53
#elif ENABLED(__AVR_ATmega644P__) || ENABLED(__AVR_ATmega644__) || ENABLED(__AVR_ATmega1284P__)
  #define MOSI_PIN             5
  #define MISO_PIN             6
  #define SCK_PIN              7
  #define SS_PIN               4
#elif ENABLED(__AVR_ATmega32U4__)
  #define MOSI_PIN             2
  #define MISO_PIN             3
  #define SCK_PIN              1
  #define SS_PIN               4
#elif ENABLED(__AVR_AT90USB646__) || ENABLED(__AVR_AT90USB1286__)
  #define MOSI_PIN            22
  #define MISO_PIN            23
  #define SCK_PIN             21
  #define SS_PIN              20
#elif ENABLED(__AVR_ATmega168__) ||ENABLED(__AVR_ATmega168P__) ||ENABLED(__AVR_ATmega328P__)
  #define MOSI_PIN            11
  #define MISO_PIN            12
  #define SCK_PIN             13
  #define SS_PIN              10
#elif ENABLED(__AVR_ATmega1281__)
  #define MOSI_PIN            11
  #define MISO_PIN            12
  #define SCK_PIN             10
  #define SS_PIN              16
#endif

#endif /* _SPI_PINS_AVR_H_ */
