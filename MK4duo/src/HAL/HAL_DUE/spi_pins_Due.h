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

#ifndef _SPI_PINS_DUE_H_
#define _SPI_PINS_DUE_H_

/**
 * Define SPI Pins: SCK, MISO, MOSI, SS
 *
 * Available chip select pins for HW SPI are 4 10 52 77
 */
#if (SDSS == 4) || (SDSS == 10) || (SDSS == 52)|| (SDSS == 59) || (SDSS == 60) || (SDSS == 77)
  #if (SDSS == 4)
    #define SPI_PIN         87
    #define SPI_CHAN        1
  #elif (SDSS == 10)
    #define SPI_PIN         77
    #define SPI_CHAN        0
  #elif (SDSS == 52)
    #define SPI_PIN         86
    #define SPI_CHAN        2
  #elif (SDSS == 59)
    #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD) && MB(ULTRATRONICS)
      #define SOFTWARE_SPI
    #else
      #define SPI_PIN       4
      #define SPI_CHAN      1
    #endif
  #elif (SDSS == 60)
    #define SPI_PIN         60
    #define SPI_CHAN        1
  #else
    #define SPI_PIN         77
    #define SPI_CHAN        0
  #endif
  #define MISO_PIN          74
  #define MOSI_PIN          75
  #define SCK_PIN           76
#else
  #define SOFTWARE_SPI
  #define MOSI_PIN		      51
  #define MISO_PIN		      50
  #define SCK_PIN 		      52
#endif

#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD) && MB(ULTRATRONICS)
  #define SS_PIN            4
#else
  #define SS_PIN            SDSS
#endif

#endif /* _SPI_PINS_DUE_H_ */
