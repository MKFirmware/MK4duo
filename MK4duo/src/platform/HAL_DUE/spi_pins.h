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

/**
 * Define SPI Pins: SCK, MISO, MOSI, SS
 *
 * Available chip select pins for HW SPI are 4 10 52 53 59 60 77
 */
#if (SDSS == 4) || (SDSS == 10) || (SDSS == 52) || (SDSS == 53) || (SDSS == 59) || (SDSS == 60) || (SDSS == 77)
  #define MISO_PIN          74
  #define MOSI_PIN          75
  #define SCK_PIN           76
#else
  #define SOFTWARE_SPI
  #ifndef MISO_PIN
    #define MISO_PIN        50
  #endif
  #ifndef MOSI_PIN
    #define MOSI_PIN        51
  #endif
  #ifndef SCK_PIN
    #define SCK_PIN         52
  #endif
#endif

#define SS_PIN            SDSS
