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

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../MK4duo.h"

// ------------------------
// Public Variables
// ------------------------

static SPISettings spiConfig;

// ------------------------
// Public functions
// ------------------------

#if ENABLED(SOFTWARE_SPI)

// ------------------------
// Software SPI
// ------------------------
#error "Software SPI not supported for STM32. Use Hardware SPI."

#else

// ------------------------
// Hardware SPI
// ------------------------

void HAL::spiBegin() {}

void HAL::spiInit(uint8_t spiRate) {  // Default to slowest rate if not specified)
  uint32_t clock;
  switch (spiRate) {
    case 0: clock = 20000000; break; // 13.9mhz=20000000  6.75mhz=10000000  3.38mhz=5000000  .833mhz=1000000
    case 1: clock =  5000000; break;
    case 2: clock =  2500000; break;
    case 3: clock =  1250000; break;
    case 4: clock =   625000; break;
    case 5: clock =   300000; break;
    default:
      clock = 4000000; // Default from the SPI library
  }
  spiConfig = SPISettings(clock, MSBFIRST, SPI_MODE0);

  SPI.begin();
}

// Write single byte to SPI
void HAL::spiSend(uint8_t nbyte) {
  SPI.beginTransaction(spiConfig);
  SPI.transfer(nbyte);
  SPI.endTransaction();
}

// Read single byte from SPI
uint8_t HAL::spiReceive() {
  SPI.beginTransaction(spiConfig);
  uint8_t returnByte = SPI.transfer(0xFF);
  SPI.endTransaction();
  return returnByte;
}

// Read from SPI into buffer
void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
  if (nbyte == 0) return;
  memset(buf, 0xFF, nbyte);
  SPI.beginTransaction(spiConfig);
  SPI.transfer(buf, nbyte);
  SPI.endTransaction();
}

// Write from buffer to SPI
void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
  uint8_t rxBuf[512];
  SPI.beginTransaction(spiConfig);
  SPI.transfer(token);
  SPI.transfer((uint8_t*)buf, &rxBuf, 512);
  SPI.endTransaction();
}

#endif // ENABLED(SOFTWARE_SPI)

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
