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
 * Software SPI functions originally from Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 *
 * Completely rewritten and tuned by Eduardo José Tagle in 2017/2018
 * in ARM thumb2 inline assembler and tuned for maximum speed and performance
 * allowing SPI clocks of up to 12 Mhz to increase SD card read/write performance
 */

/**
 * Description: HAL for Arduino Due and compatible (SAM3X8E)
 *
 * For ARDUINO_ARCH_SAM
 */

#ifdef ARDUINO_ARCH_SAMD

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../MK4duo.h"

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

#if ENABLED(SOFTWARE_SPI)

  // --------------------------------------------------------------------------
  // software SPI
  // --------------------------------------------------------------------------

  // bitbanging transfer
  // run at ~100KHz (necessary for init)
  uint8_t HAL::spiTransfer(uint8_t b) { // using Mode 0
    for (int bits = 0; bits < 8; bits++) {
      if (b & 0x80) {
        WRITE(MOSI_PIN, HIGH);
      }
      else {
        WRITE(MOSI_PIN, LOW);
      }
      b <<= 1;

      WRITE(SCK_PIN, HIGH);
      delayMicroseconds(5U);

      if (READ(MISO_PIN)) {
        b |= 1;
      }
      WRITE(SCK_PIN, LOW);
      delayMicroseconds(5U);
    }
    return b;
  }

  void HAL::spiBegin(void) {
    SET_OUTPUT(SS_PIN);
    WRITE(SS_PIN, HIGH);
    SET_OUTPUT(SCK_PIN);
    SET_INPUT(MISO_PIN);
    SET_OUTPUT(MOSI_PIN);
  }

  void HAL::spiInit(uint8_t spiRate) {
    UNUSED(spiRate);
    WRITE(SS_PIN, HIGH);
    WRITE(MOSI_PIN, HIGH);
    WRITE(SCK_PIN, LOW);
  }

  uint8_t HAL::spiReceive(void) {
    WRITE(SS_PIN, LOW);
    uint8_t b = spiTransfer(0xFF);
    WRITE(SS_PIN, HIGH);
    return b;
  }

  void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    WRITE(SS_PIN, LOW);
    for (int i = 0; i < nbyte; i++) {
      buf[i] = spiTransfer(0xFF);
    }
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSend(uint8_t b) {
    WRITE(SS_PIN, LOW);
    uint8_t response = spiTransfer(b);
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSend(const uint8_t* buf, size_t n) {
    uint8_t response;
    if (n == 0) return;
    WRITE(SS_PIN, LOW);
    for (uint16_t i = 0; i < n; i++) {
      response = spiTransfer(buf[i]);
    }
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    uint8_t response;

    WRITE(SS_PIN, LOW);
    response = spiTransfer(token);

    for (uint16_t i = 0; i < 512; i++) {
      response = spiTransfer(buf[i]);
    }
    UNUSED(response);
    WRITE(SS_PIN, HIGH);
  }

#else
    #include "SERCOM.h"
    
    #ifndef PERIPH_SPI
        #define PERIPH_SPI           sercom4
        #define PAD_SPI_TX           SPI_PAD_2_SCK_3
        #define PAD_SPI_RX           SERCOM_RX_PAD_0
    #endif // PERIPH_SPI
    
    SPIClass SPIc (&PERIPH_SPI, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX, PAD_SPI_RX);
   
  // --------------------------------------------------------------------------
  // hardware SPI
  // --------------------------------------------------------------------------
  // 8.4 MHz, 4 MHz, 2 MHz, 1 MHz, 0.5 MHz, 0.329 MHz, 0.329 MHz
  int spiSamdBaudrates  [] = { 4200000 , 2100000, 1050000 , 525000, 262500 };
  static bool spiInitMaded = false;

  void HAL::spiBegin() {
    if (!spiInitMaded) {
      SPI.begin();
      spiInit(1);
      spiInitMaded = true;
    }
    SPIc.begin();
    
  }

  void HAL::spiInit(uint8_t spiRate) {
    if (spiInitMaded == false) {
      if (spiRate > 4) spiRate = 1;
        
      SPIc.beginTransaction(SPISettings(spiSamdBaudrates[spiRate], MSBFIRST, SPI_MODE0));
    }
  }

  // Write single byte to SPI
  void HAL::spiSend(byte b) {
    SPIc.transfer(b);
    
  }

  void HAL::spiSend(const uint8_t* buf, size_t n) {
    for (uint16_t i = 0; i < n; i++) {
      SPIc.transfer(buf[i]); 
    }
  }

  void HAL::spiSend(uint32_t chan, byte b) {
  
  }

  void HAL::spiSend(uint32_t chan, const uint8_t* buf, size_t n) {
    
  }

  // Read single byte from SPI
  uint8_t HAL::spiReceive(void) {
    return  SPIc.transfer(0xFF);
  }

  uint8_t HAL::spiReceive(uint32_t chan) {
   
  }

  // Read from SPI into buffer
  void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    for (uint16_t i = 0; i < 512; i++) {
      buf[i]=SPIc.transfer(0xFF); 
    }
  }

  // Write from buffer to SPI
  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    SPIc.transfer( token );

    for (uint16_t i = 0; i < 512; i++) {
      SPIc.transfer(buf[i]); 
    }

  }

#endif // ENABLED(SOFTWARE_SPI)

#endif // ARDUINO_ARCH_SAMD
