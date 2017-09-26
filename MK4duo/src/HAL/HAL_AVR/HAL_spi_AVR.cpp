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
 * Originally from Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 */
 
/**
 * Description: HAL for AVR - SPI functions
 *
 * For __AVR__
 */

#ifdef __AVR__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../../base.h"

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

#if ENABLED(SOFTWARE_SPI)

  #include <SoftSPI.h>

  static SoftSPI<SOFT_SPI_MISO_PIN, SOFT_SPI_MOSI_PIN, SOFT_SPI_SCK_PIN, 0> softSpiBus;

  void HAL::spiBegin() { softSpiBus.begin(); }
  
  /** Set SPI rate */
  void HAL::spiInit(uint8_t spiRate) { UNUSED(spiRate); }

  /** Soft SPI receive byte */
  uint8_t HAL::spiReceive(void) { return softSpiBus.receive(); }

  /** Soft SPI read data */
  uint8_t HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    for (size_t i = 0; i < nbyte; i++) {
      buf[i] = spiReceive();
    }
    return 0;
  }

  /** Soft SPI send byte */
  void HAL::spiSend(uint8_t b) { softSpiBus.send(b); }

  /** Soft SPI send block */
  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    spiSend(token);
    for (uint16_t i = 0; i < 512; i++) {
      spiSend(buf[i]);
    }
  }

#else // HARDWARE_SPI
  
  void HAL::spiBegin(void) {
    #if SS_PIN >= 0
      SET_INPUT(MISO_PIN);
      SET_OUTPUT(MOSI_PIN);
      SET_OUTPUT(SCK_PIN);
      // SS must be in output mode even it is not chip select
      SET_OUTPUT(SS_PIN);
      // set SS high - may be chip select for another SPI device
      WRITE(SS_PIN, HIGH);
    #endif
  }

  // make sure SPCR rate is in expected bits
  #if (SPR0 != 0 || SPR1 != 1)
    #error "unexpected SPCR bits"
  #endif

  /**
   * Initialize hardware SPI
   * Set SCK rate to F_CPU/pow(2, 1 + spiRate) for spiRate [0,6]
   */
  void HAL::spiInit(uint8_t spiRate) {
    uint8_t r = 0;
    for (uint8_t b = 2; spiRate > b && r < 6; b <<= 1, r++);

    SET_OUTPUT(SS_PIN);
    WRITE(SS_PIN, HIGH);
    SET_OUTPUT(SCK_PIN);
    SET_OUTPUT(MOSI_PIN);
    SET_INPUT(MISO_PIN);
    #ifdef PRR
      PRR &= ~(1 << PRSPI);
    #elif ENABLED PRR0
      PRR0 &= ~(1 << PRSPI);
    #endif
    // See avr processor documentation
    SPCR = _BV(SPE) | _BV(MSTR) | (r >> 1);
    SPSR = (r & 1 || r == 6 ? 0 : 1) << SPI2X;
  }

   /** SPI send a byte */
  void HAL::spiSend(byte b) {
    SPDR = b;
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
  }

  void HAL::spiSend(const uint8_t* buf, size_t n) {
    if (n == 0) return;
    SPDR = buf[0];
    if (n > 1) {
      uint8_t b = buf[1];
      size_t i = 2;
      while (1) {
        while (!(SPSR & (1 << SPIF))) {}
        SPDR = b;
        if (i == n) break;
        b = buf[i++];
      }
    }
    while (!(SPSR & (1 << SPIF))) {}
  }

  /** SPI send block  */
  void HAL::spiSendBlock(uint8_t token, const uint8_t* buf) {
    SPDR = token;
    for (uint16_t i = 0; i < 512; i += 2) {
      while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
      SPDR = buf[i];
      while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
      SPDR = buf[i + 1];
    }
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
  }

  /** SPI receive a byte */
  uint8_t HAL::spiReceive(uint8_t send/*=0xFF*/) {
    SPDR = send;
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
    return SPDR;
  }

  /** SPI read data  */
  void HAL::spiReadBlock(uint8_t* buf, uint16_t nbyte) {
    if (nbyte-- == 0) return;
    SPDR = 0XFF;
    for (uint16_t i = 0; i < nbyte; i++) {
      while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
      buf[i] = SPDR;
      SPDR = 0XFF;
    }
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
    buf[nbyte] = SPDR;
  }

#endif  // HARDWARE_SPI

#endif // __AVR__
